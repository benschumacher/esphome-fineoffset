#include "fineoffset.h"
#ifdef USE_SENSOR
#include "sensor/fineoffset_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "text_sensor/fineoffset_text_sensor.h"
#endif

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace fineoffset {

static const char* const TAG = "fineoffset";

FineOffsetState::FineOffsetState(byte packet[5]) {
    sensor_id = ((packet[0] & 0x0F) << 4) + ((packet[1] & 0xF0) >> 4);
    temperature = ((packet[1] & 0xB00000111) << 8) + packet[2];
    humidity = packet[3];
    // make negative
    if (packet[1] & 0xB00001000) {
        temperature = -temperature;
    }
    // state is valid if the contents of the first 4 bytes
    // equals the crc stored in the 5th
    valid = crc8(packet, 4) == packet[4];
}

const char* FineOffsetState::c_str() const {
    std::string data;

    char temperture_buf[7] = {
        0,
    };  // 7 bytes is enough for -100.0 with a '\0'
    snprintf(temperture_buf, sizeof(temperture_buf), "%.1f", (temperature * 0.1f));
    data.clear();
    data += "| Sensor ID:  ";
    data += to_string(sensor_id);
    data += " | humidity: ";
    data += to_string(humidity);
    data += "% | temperature: ";
    data += to_string(temperture_buf);
    data += "°C | ";
    data += to_string(valid ? "OK " : "BAD");

    return data.c_str();
}

// 240,000,000Hz / 40000000 = 5000 interrupts per second, ie. 200us  between interrupts
#define COUNTER_RATE 3200 - 1
// 1 is indicated by 500uS pulse
// wh2_accept from 2 = 400us to 3 = 600us
#define IS_HI_PULSE(interval) (interval >= 400 && interval <= 600)
// 0 is indicated by ~1500us pulse
// wh2_accept from 7 = 1400us to 8 = 1600us
#define IS_LOW_PULSE(interval) (interval >= 1400 && interval <= 1600)
// worst case packet length
// 6 bytes x 8 bits x (1.5 + 1) = 120ms; 120ms = 200us x 600
#define HAS_TIMED_OUT(interval) (interval > 120000)
// we expect 1ms of idle time between pulses
// so if our pulse hasn't arrived by 1.2ms, reset the wh2_packet_state machine
// 6 x 200us = 1.2ms
#define IDLE_HAS_TIMED_OUT(interval) (interval > 1200)
// our expected pulse should arrive after 1ms
// we'll wh2_accept it if it arrives after
// 4 x 200us = 800us
#define IDLE_PERIOD_DONE(interval) (interval >= 800)
// Shorthand for tests
// #define RF_HI (digitalRead(RF_IN) == HIGH)
// #define RF_LOW (digitalRead(RF_IN) == LOW)
#define RF_HI(pin) ((pin).digital_read())
#define RF_LOW(pin) (!(pin).digital_read())

// wh2_flags
#define GOT_PULSE 0x01
#define LOGIC_HI 0x02

FineOffsetStore::FineOffsetStore(FineOffsetComponent* parent)
    : parent_(parent),
      skip_(true),
      sampling_state_{0},
      sample_count_{0},
      was_low_(false),
      wh2_flags_{0},
      wh2_packet_state_{0},
      wh2_timeout_{0},
      wh2_packet_{0},
      cycles_(0),
      packet_no_{0},
      bit_no_{0},
      history_{0} {}

void IRAM_ATTR FineOffsetStore::intr_cb(FineOffsetStore* self) {
    const auto now = millis();

    self->edge_millis_1 = self->edge_millis_2;
    self->edge_millis_1 = now;
    auto interval = self->edge_millis_2 - self->edge_millis_1;
    // self->edge_millis_[1] = self->edge_millis_[2];
    // self->edge_millis_[2] = now;
    // uint64_t interval = self->edge_millis_[2] - self->edge_millis_[1];

    if (self->skip_) {
        self->skip_ = false;
        return;
    }
    if (interval < 200) {
        // Last edge was too short.
        // Skip this edge, and the next too.
        self->skip_ = true;
        return;
    }

    auto pulse = self->edge_millis_1 - self->edge_millis_0;
    self->edge_millis_0 = self->edge_millis_1;
    self->last_interval_ = pulse;

    switch (self->sampling_state_) {
        case 0:  // waiting
            self->wh2_packet_state_ = 0;
            if (RF_HI(self->pin_)) {
                if (self->was_low_) {
                    self->sampling_state_ = 1;
                    self->sample_count_ = 0;
                    self->was_low_ = false;
                }
            } else {
                self->was_low_ = true;
            }
            break;
        case 1:  // acquiring first pulse
            self->sample_count_++;
            // end of first pulse
            // if (RF_LOW(self->pin_)) {
            if (IS_HI_PULSE(pulse)) {
                self->wh2_flags_ = GOT_PULSE | LOGIC_HI;
                self->sampling_state_ = 2;
                self->sample_count_ = 0;
            } else if (IS_LOW_PULSE(pulse)) {
                self->wh2_flags_ = GOT_PULSE;  // logic low
                self->sampling_state_ = 2;
                self->sample_count_ = 0;
            } else {
                self->sampling_state_ = 0;
            }
            // }
            break;
        case 2:  // observe 1ms of idle time
            self->sample_count_++;
            // if (RF_HI(self->pin_)) {
            if (IDLE_HAS_TIMED_OUT(pulse)) {
                self->sampling_state_ = 0;
            } else if (IDLE_PERIOD_DONE(pulse)) {
                self->sampling_state_ = 1;
                self->sample_count_ = 0;
            }
            // }
            break;
    }

    if (self->wh2_timeout_ > 0) {
        self->wh2_timeout_++;
        if (HAS_TIMED_OUT(pulse)) {
            self->wh2_packet_state_ = {0};
            self->wh2_timeout_ = 0;
        }
    }
}

bool FineOffsetStore::accept() {
    // reset if in initial wh2_packet_state
    if (this->wh2_packet_state_ == 0) {
        // should history be 0, does it matter?
        this->history_ = 0xFF;
        this->wh2_packet_state_ = 1;
        // enable wh2_timeout
        this->wh2_timeout_ = 1;
    }  // fall thru to wh2_packet_state one

    // acquire preamble
    if (this->wh2_packet_state_ == 1) {
        // shift history right and store new value
        this->history_ <<= 1;
        // store a 1 if required (right shift along will store a 0)
        if (this->wh2_flags_ & LOGIC_HI) {
            this->history_ |= 0x01;
        }
        // check if we have a valid start of frame
        // xxxxx110
        if ((this->history_ & 0xB00000111) == 0xB00000110) {
            // need to clear packet, and counters
            this->packet_no_ = 0;
            // start at 1 because only need to acquire 7 bits for first packet byte.
            this->bit_no_ = 1;
            this->wh2_packet_[0] = this->wh2_packet_[1] = this->wh2_packet_[2] = this->wh2_packet_[3] =
                this->wh2_packet_[4] = 0;
            // we've acquired the preamble
            this->wh2_packet_state_ = 2;
        }
        return false;
    }

    // acquire packet
    if (this->wh2_packet_state_ == 2) {
        this->wh2_packet_[packet_no_] <<= 1;
        if (this->wh2_flags_ & LOGIC_HI) {
            this->wh2_packet_[this->packet_no_] |= 0x01;
        }

        this->bit_no_++;
        if (this->bit_no_ > 7) {
            this->bit_no_ = 0;
            this->packet_no_++;
        }

        if (this->packet_no_ > 4) {
            this->cycles_++;

            // start the sampling process from scratch
            this->wh2_packet_state_ = 0;
            // clear wh2_timeout
            this->wh2_timeout_ = 0;

            if (state_valid()) {
                this->bad_count_++;
            }
            ESP_LOGD(TAG, "packet_count=%d bad_count=%d cycles=%d", this->packet_no_, this->bad_count_, this->cycles_);
            // Serial.print(spacing, DEC);
            // Serial.print(" | ");
            // Serial.print(average_interval, DEC);
            // Serial.print(" | ");

            for (auto i = 0; i < 5; ++i) {
                ESP_LOGD(TAG, "%d: 0x%x / %d", i, this->wh2_packet_[i], this->wh2_packet_[i]);
            }
            // Serial.print("| Sensor ID: 0x");
            // Serial.print(wh2_sensor_id(), HEX);
            // Serial.print(" | ");
            // Serial.print(wh2_humidity(), DEC);
            // Serial.print("% | ");
            // Serial.print(wh2_temperature(), DEC);
            // Serial.print(" | ");
            // Serial.println((wh2_valid() ? "OK" : "BAD"));

            return true;
        }
    }
    return false;
}

bool FineOffsetStore::state_valid() { return (crc8(this->wh2_packet_, 4) == this->wh2_packet_[4]); }

void FineOffsetStore::record_state() {
    this->cycles_++;
    auto state = FineOffsetState(wh2_packet_);

    if (this->states_.size() == 10) {
        this->states_.pop_front();
    }
    this->states_.push_back(state);

    if (state.valid) {
        this->state_by_sensor_id_.insert({state.sensor_id, state});
        if (this->parent_->is_unknown(state.sensor_id)) {
            this->last_unknown_ = std::unique_ptr<FineOffsetState>(new FineOffsetState(state));
        }
    } else {
        this->last_bad_ = std::unique_ptr<FineOffsetState>(new FineOffsetState(state));
    }

    ESP_LOGD(TAG, "%s", state.c_str());

    this->wh2_flags_ = 0;
}

std::pair<bool, const FineOffsetState&> FineOffsetStore::get_last_state(FineOffsetTextSensorType sensor_type) const {
    switch (sensor_type) {
        case FINEOFFSET_TYPE_LAST:
            if (!this->states_.empty()) {
                return {true, this->states_.back()};
            }
            break;
        case FINEOFFSET_TYPE_LAST_BAD:
            if (this->last_bad_ != nullptr) {
                return {true, *this->last_bad_};
            }
            break;
        case FINEOFFSET_TYPE_UNKNOWN:
            if (this->last_unknown_ != nullptr) {
                return {true, *this->last_unknown_};
            }
            break;
    }

    return {false, FineOffsetState()};
}

void FineOffsetComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Setting up FineOFfset...");
    LOG_PIN("  Input Pin: ", this->pin_);
    LOG_UPDATE_INTERVAL(this);

    for (auto it : this->sensors_) {
        LOG_SENSOR("  ", "Sensor", it.second);
        // ESP_LOGCONFIG("  ", "Sensor '%d':", it.first);
        // it.second->dump_config();
    }

    for (auto sensor : this->text_sensors_) {
        LOG_TEXT_SENSOR("  ", "Text sensor", sensor);
    }
}

void FineOffsetComponent::loop() {
    uint32_t spacing;

    if (this->store_.ready()) {
        ESP_LOGD(TAG, "ready");
        if (this->store_.accept()) {
            ESP_LOGD(TAG, "accept");
            if (!this->store_.state_valid()) {
                return;
            } else {
                this->store_.record_state();
            }
        }
    }
}

void FineOffsetComponent::update() {
    if (!this->store_.skip_) {
        ESP_LOGD(TAG, "sampling_state_=%d sample_count_=%d wh2_flags_=%d  wh2_packet_no_=%d last_millis_=%lu",
                 this->store_.sampling_state_, this->store_.sample_count_, this->store_.wh2_flags_,
                 this->store_.packet_no_, this->store_.last_interval_);
        ESP_LOGD(TAG, "edge_millis_0=%lu", this->store_.edge_millis_0);
        ESP_LOGD(TAG, "edge_millis_1=%lu", this->store_.edge_millis_1);
        ESP_LOGD(TAG, "edge_millis_2=%lu", this->store_.edge_millis_2);
    }

    for (auto it : this->sensors_) {
        auto [found, state] = this->store_.get_state_for_sensor_no(it.first);
        if (found) {
            it.second->publish_temperature(state.temperature * 0.1f);
            it.second->publish_humidity(state.humidity);
        }
    }

    for (auto text_sensor : this->text_sensors_) {
        auto [found, state] = this->store_.get_last_state(text_sensor->get_sensor_type());

        if (found) {
            text_sensor->publish_state(state.c_str());
        }
    }
}

void FineOffsetComponent::register_sensor(uint8_t sensor_no, FineOffsetSensor* obj) {
    obj->set_sensor_no(sensor_no);
    obj->set_parent(this);
    this->sensors_.insert({sensor_no, obj});
}

void FineOffsetComponent::register_text_sensor(FineOffsetTextSensor* obj) {
    obj->set_parent(this);
    this->text_sensors_.push_back(obj);
}

}  // namespace fineoffset
}  // namespace esphome