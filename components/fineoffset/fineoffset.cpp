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
    this->sensor_id = ((packet[0] & 0x0F) << 4) + ((packet[1] & 0xF0) >> 4);
    this->temperature = ((packet[1] & 0b00000111) << 8) + packet[2];
    this->humidity = packet[3];
    // make negative
    if (packet[1] & 0b00001000) {
        this->temperature = -this->temperature;
    }
    // state is valid if the contents of the first 4 bytes
    // equals the crc stored in the 5th
    if (this->sensor_id != 0) {
        this->valid = FineOffsetState::crc8ish(packet, 4) == packet[4];
    }
}

std::string FineOffsetState::str() const {
    std::string data;

    char temperture_buf[7] = {0};  // 7 bytes is enough for -100.0 with a '\0'
    snprintf(temperture_buf, sizeof(temperture_buf), "%.1f", (temperature * 0.1f));
    data.clear();
    data += "| Sensor ID:  ";
    data += to_string(sensor_id);
    data += " | humidity: ";
    data += to_string(humidity);
    data += "% | temperature: ";
    data += temperture_buf;
    data += "°C | ";
    data += (valid ? "OK " : "BAD");

    return data;
}

//--------------------------------------------------------
// 1 is indicated by 500uS pulse
// wh2_accept from 2 = 400us to 3 = 600us
#define IS_HI_PULSE(interval) (interval >= 250 && interval <= 750)
// 0 is indicated by ~1500us pulse
// wh2_accept from 7 = 1400us to 8 = 1600us
#define IS_LOW_PULSE(interval) (interval >= 1200 && interval <= 1750)
// worst case packet length
// 6 bytes x 8 bits =48
#define IDLE_HAS_TIMED_OUT(interval) (interval > 1199)
// our expected pulse should arrive after 1ms
// we'll wh2_accept it if it arrives after
// 4 x 200us = 800us
#define IDLE_PERIOD_DONE(interval) (interval >= 751)

// const auto GOT_PULSE = 0x01;
// const auto LOGIC_HI = 0x02;
#define GOT_PULSE 0x01
#define LOGIC_HI 0x02

FineOffsetStore::FineOffsetStore(FineOffsetComponent* parent) : parent_(parent), wh2_flags_{0}, packet_state_{0} {}

void IRAM_ATTR FineOffsetStore::intr_cb(FineOffsetStore* self) {
    static unsigned long edgeTimeStamp[3] = {0};  // Timestamp of edges
    static bool skip = true;

    // Filter out too short pulses. This method works as a low pass filter.  (borroved from new remote reciever)
    edgeTimeStamp[1] = edgeTimeStamp[2];
    edgeTimeStamp[2] = micros();

    if (skip) {
        skip = false;
        return;
    }

    if (edgeTimeStamp[2] - edgeTimeStamp[1] < 200) {
        // Last edge was too short.
        // Skip this edge, and the next too.
        skip = true;
        return;
    }

    unsigned int pulse = edgeTimeStamp[1] - edgeTimeStamp[0];
    edgeTimeStamp[0] = edgeTimeStamp[1];

    static byte wh2_flags = 0x00;
    static bool wh2_accept_flag = false;
    static byte wh2_packet_state = 0;
    static byte wh2_packet[5] = {0};
    static bool wh2_valid = false;
    static byte sampling_state = 0;
    static byte packet_no = 0;
    static byte bit_no = 0;
    static byte history = 0x01;

    self->packet_state_.store(wh2_packet_state);
    switch (sampling_state) {
        case 0:  // waiting

            if (IS_HI_PULSE(pulse)) {
                wh2_flags = GOT_PULSE | LOGIC_HI;
                sampling_state = 1;
            } else if (IS_LOW_PULSE(pulse)) {
                wh2_flags = GOT_PULSE;  // logic low
            } else {
                sampling_state = 0;
            }
            break;
        case 1:  // observe 1ms of idle time
            if (IDLE_HAS_TIMED_OUT(pulse)) {
                sampling_state = 0;
                wh2_packet_state = 1;
            } else if (IDLE_PERIOD_DONE(pulse)) {
                sampling_state = 0;
            } else {
                sampling_state = 0;
                wh2_packet_state = 1;
            }
            break;
    }

    if (wh2_flags) {
        // acquire preamble
        if (wh2_packet_state == 1) {
            // shift history right and store new value
            history <<= 1;
            // store a 1 if required (right shift along will store a 0)
            if (wh2_flags & LOGIC_HI) {
                history |= 0x01;
            }

            // check if we have a valid start of frame
            // xxxxx110
            if ((history & 0b00000111) == 0b00000110) {
                // need to clear packet, and pulseers
                packet_no = 0;
                // start at 1 becuase only need to acquire 7 bits for first packet byte.
                bit_no = 1;
                wh2_packet[0] = wh2_packet[1] = wh2_packet[2] = wh2_packet[3] = wh2_packet[4] = 0;
                // we've acquired the preamble
                wh2_packet_state = 2;
                history = 0xFF;
            }
            wh2_accept_flag = false;
            self->accept_flag_.store(false);
        }
        // acquire packet
        else if (wh2_packet_state == 2) {
            wh2_packet[packet_no] <<= 1;
            if (wh2_flags & LOGIC_HI) {
                wh2_packet[packet_no] |= 0x01;
            }
            bit_no++;
            if (bit_no > 7) {
                bit_no = 0;
                packet_no++;
            }
            if (packet_no > 4) {
                // start the sampling process from scratch
                wh2_packet_state = 1;
                wh2_accept_flag = true;
                self->accept_flag_.store(true);
            }
        } else {
            wh2_accept_flag = false;
            self->accept_flag_.store(false);
        }

        if (wh2_accept_flag) {
            uint8_t crc = FineOffsetState::crc8ish(wh2_packet, 4);
            self->cycles_++;

            if (crc == wh2_packet[4]) {
                wh2_valid = true;
            } else {
                self->bad_count_++;
                wh2_valid = false;
            }

            FineOffsetState state(wh2_packet);  // Direct value construction - no heap allocation

            // avoid change sensor data during update
            if (wh2_valid == true && self->have_sensor_data_.load() == 0) {
                self->state_obj_ = state;
                self->has_pending_state_ = true;
                self->have_sensor_data_.store(state.sensor_id);
            } else if (!wh2_valid && self->have_sensor_data_.load() == 0) {
                self->state_obj_ = state;
                self->has_pending_state_ = true;
            }

            wh2_accept_flag = false;
            self->accept_flag_.store(false);
        }
        wh2_flags = 0;
    }

    self->wh2_flags_.store(wh2_flags);
    //--------------------------------------------------------
}

bool FineOffsetStore::accept() {
    if (this->have_sensor_data_.load() != 0) {
        return true;
    }
    return false;
}

void FineOffsetStore::record_state() {
    if (!this->has_pending_state_) {
        return;
    }

    FineOffsetState state = this->state_obj_;
    this->has_pending_state_ = false;

    if (state.sensor_id != 0) {
        // Add to circular buffer (replaces std::deque)
        this->states_[this->states_head_] = state;
        this->states_head_ = (this->states_head_ + 1) % MAX_STATES;
        if (this->states_count_ < MAX_STATES) {
            this->states_count_++;
        }

        if (state.valid) {
            // Only store for registered sensors or if sensor discovery is enabled
            if (state.sensor_id < MAX_SENSOR_IDS) {
                this->sensor_states_[state.sensor_id] = state;
                this->sensor_states_valid_[state.sensor_id] = true;
                this->sensor_states_consumed_[state.sensor_id] = false;  // Mark as fresh data
            }

            if (this->parent_->is_unknown_sensor(state.sensor_id)) {
                this->last_unknown_ = state;
                this->has_last_unknown_ = true;
                this->last_unknown_consumed_ = false;  // Mark as fresh data
            }
        } else {
            this->last_bad_ = state;
            this->has_last_bad_ = true;
            this->last_bad_consumed_ = false;  // Mark as fresh data
        }

        ESP_LOGD(TAG, "%s", state.str().c_str());
    }

    this->have_sensor_data_.store(0);
    this->wh2_flags_.store(0);
}

#ifdef USE_TEXT_SENSOR
std::pair<bool, const FineOffsetState> FineOffsetStore::get_last_state(FineOffsetTextSensorType sensor_type) {
    switch (sensor_type) {
        case FINEOFFSET_TYPE_LAST:
            if (this->states_count_ > 0) {
                // Get the most recent state from circular buffer
                uint8_t last_index = (this->states_head_ + MAX_STATES - 1) % MAX_STATES;
                return {true, this->states_[last_index]};
            }
            break;
        case FINEOFFSET_TYPE_LAST_BAD:
            if (this->has_last_bad_) {
                // Mark as consumed for selective clearing
                this->last_bad_consumed_ = true;
                return {true, this->last_bad_};
            }
            break;
        case FINEOFFSET_TYPE_UNKNOWN:
            if (this->has_last_unknown_) {
                // Mark as consumed for selective clearing
                this->last_unknown_consumed_ = true;
                return {true, this->last_unknown_};
            }
            break;
    }

    return {false, FineOffsetState()};
}
#endif

void FineOffsetComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Setting up FineOffset...");
    LOG_PIN("  Input Pin: ", this->pin_);
}

void FineOffsetComponent::setup() {
    Component::setup();
    this->store_.setup(this->pin_);

    // Schedule recurring diagnostic logging for signal processing debugging
    this->schedule_diagnostic_log();
}

void FineOffsetComponent::schedule_diagnostic_log() {
    this->set_timeout("diagnostic_log", 60000, [this]() {
        ESP_LOGV(TAG, "accept_flag_=%s wh2_flags_=%hhu have_sensor_data_=%d packet_state=%hhu cycles_=%u bad_count_=%u",
                 (this->store_.accept_flag_.load() ? "true" : "false"), this->store_.wh2_flags_.load(),
                 this->store_.have_sensor_data_.load(), this->store_.packet_state_.load(), this->store_.cycles_,
                 this->store_.bad_count_);

        // Perform periodic cleanup of consumed sensor data (critical for data freshness)
        this->store_.periodic_cleanup();

        // Reschedule for next interval
        this->schedule_diagnostic_log();
    });
}

void FineOffsetComponent::loop() {
    if (this->store_.ready()) {
        ESP_LOGD(TAG, "ready, core=%d", xPortGetCoreID());
        this->store_.record_state();
    }
}

}  // namespace fineoffset
}  // namespace esphome
