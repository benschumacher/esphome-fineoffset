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

// Protocol timing constants in microseconds
static constexpr uint16_t MIN_HI_PULSE = 250;   // '1' bit minimum
static constexpr uint16_t MAX_HI_PULSE = 750;   // '1' bit maximum
static constexpr uint16_t MIN_LOW_PULSE = 1200; // '0' bit minimum
static constexpr uint16_t MAX_LOW_PULSE = 1750; // '0' bit maximum
static constexpr uint16_t IDLE_TIMEOUT = 2000;  // Gap between packets
static constexpr uint16_t MIN_PULSE_WIDTH = 200; // Minimum valid pulse width for debounce

// Implementation of ISRData constructor
FineOffsetStore::ISRData::ISRData()
    : last_pin_state(false)
    , frame_state(0)
    , packet_ready(false)
    , history(0)
    , bit_position(0)
    , byte_position(0) {
    // Initialize atomic arrays
    for (int i = 0; i < 5; i++) {
        packet_buffer[i].store(0);
    }
    for (int i = 0; i < 3; i++) {
        edge_timestamps[i].store(0);
        edge_rising[i].store(false);
    }
}

FineOffsetState::FineOffsetState(byte packet[5]) {
    this->sensor_id = ((packet[0] & 0x0F) << 4) + ((packet[1] & 0xF0) >> 4);
    this->temperature = ((packet[1] & 0b00000111) << 8) + packet[2];
    this->humidity = packet[3];
    // make negative
    if (packet[1] & 0b00001000) {
        this->temperature = -this->temperature;
    }
    if (this->sensor_id != 0) {
        this->valid = FineOffsetState::crc8ish(packet, 4) == packet[4];
    }
}

uint8_t FineOffsetState::crc8ish(const byte data[], uint8_t len) {
    uint8_t crc = 0;
    uint8_t addr = 0;
    while (len--) {
        uint8_t inbyte = data[addr++];
        for (uint8_t i = 8; i; i--) {
            uint8_t mix = (crc ^ inbyte) & 0x80;
            crc <<= 1;
            if (mix)
                crc ^= 0x31;
            inbyte <<= 1;
        }
    }
    return crc;
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
    data += to_string(valid ? "OK " : "BAD");
    return data;
}

FineOffsetStore::FineOffsetStore(FineOffsetComponent* parent) : parent_(parent) {}

void IRAM_ATTR FineOffsetStore::intr_cb(FineOffsetStore* self) {
    const uint32_t now = micros();
    const bool pin_state = self->pin_.digital_read();
    const bool is_rising = pin_state && !self->isr_data_.last_pin_state.load();
    
    // Update edge timestamps and types
    self->isr_data_.edge_timestamps[0].store(self->isr_data_.edge_timestamps[1].load());
    self->isr_data_.edge_timestamps[1].store(self->isr_data_.edge_timestamps[2].load());
    self->isr_data_.edge_timestamps[2].store(now);
    
    self->isr_data_.edge_rising[0].store(self->isr_data_.edge_rising[1].load());
    self->isr_data_.edge_rising[1].store(self->isr_data_.edge_rising[2].load());
    self->isr_data_.edge_rising[2].store(is_rising);
    
    self->isr_data_.last_pin_state.store(pin_state);

    // Debounce check
    if (now - self->isr_data_.edge_timestamps[1].load() < MIN_PULSE_WIDTH) {
        return;
    }

    // Only process on falling edges
    if (is_rising) {
        return;
    }

    // Calculate high pulse width (time between rising and falling edge)
    const uint32_t pulse_width = now - self->isr_data_.edge_timestamps[1].load();
    
    switch (self->isr_data_.frame_state.load()) {
        case 0: { // Waiting for preamble
            uint8_t history = self->isr_data_.history.load();
            history = (history << 1) & 0x07;
            
            if (pulse_width >= MIN_HI_PULSE && pulse_width <= MAX_HI_PULSE) {
                history |= 0x01;  // Mark as '1'
            }
            
            self->isr_data_.history.store(history);
            
            // Check for preamble pattern (110)
            if ((history & 0x07) == 0x06) {
                // Reset packet buffer
                for (int i = 0; i < 5; i++) {
                    self->isr_data_.packet_buffer[i].store(0);
                }
                self->isr_data_.bit_position.store(0);
                self->isr_data_.byte_position.store(0);
                self->isr_data_.frame_state.store(1);
            }
            break;
        }

        case 1: { // Collecting data
            uint8_t byte_pos = self->isr_data_.byte_position.load();
            uint8_t bit_pos = self->isr_data_.bit_position.load();
            uint8_t current_byte = self->isr_data_.packet_buffer[byte_pos].load();
            
            // Shift left for next bit
            current_byte <<= 1;
            
            // Determine bit value based on pulse width
            if (pulse_width >= MIN_HI_PULSE && pulse_width <= MAX_HI_PULSE) {
                current_byte |= 0x01;  // '1' bit
            } else if (pulse_width >= MIN_LOW_PULSE && pulse_width <= MAX_LOW_PULSE) {
                // '0' bit - nothing to do after shift
            } else {
                // Invalid pulse width - reset state
                self->isr_data_.frame_state.store(0);
                break;
            }
            
            // Store updated byte
            self->isr_data_.packet_buffer[byte_pos].store(current_byte);
            
            // Update bit/byte positions
            bit_pos++;
            if (bit_pos >= 8) {
                bit_pos = 0;
                byte_pos++;
                
                if (byte_pos >= 5) {
                    // Packet complete
                    self->isr_data_.packet_ready.store(true);
                    self->isr_data_.frame_state.store(0);
                    break;
                }
            }
            
            self->isr_data_.bit_position.store(bit_pos);
            self->isr_data_.byte_position.store(byte_pos);
            break;
        }
    }
}

bool FineOffsetStore::accept() {
    return this->isr_data_.packet_ready.load();
}

void FineOffsetStore::record_state() {
    if (!this->isr_data_.packet_ready.load()) {
        return;
    }

    // Copy packet data safely
    byte packet[5];
    for (int i = 0; i < 5; i++) {
        packet[i] = this->isr_data_.packet_buffer[i].load();
    }

    // Create new state from packet
    std::shared_ptr<FineOffsetState> state(new FineOffsetState(packet));

    if (state->sensor_id != 0) {
        if (this->states_.size() == 10) {
            this->states_.pop_front();
        }
        this->states_.push_back(*state);

        if (state->valid) {
            const auto result = this->state_by_sensor_id_.insert({state->sensor_id, *state});
            if (!result.second) {
                result.first->second = *state;
            }
            if (this->parent_->is_unknown(state->sensor_id)) {
                this->last_unknown_ = state;
            }
        } else {
            this->last_bad_ = state;
        }

        ESP_LOGD(TAG, "%s", state->str().c_str());
    }

    // Reset packet ready flag
    this->isr_data_.packet_ready.store(false);
}

std::pair<bool, const FineOffsetState> FineOffsetStore::get_last_state(FineOffsetTextSensorType type) const {
    switch (type) {
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

void FineOffsetComponent::loop() {
    if (this->store_.accept()) {
        ESP_LOGD(TAG, "Processing packet");
        this->store_.record_state();
    }
}

void FineOffsetComponent::update() {
    for (auto& it : this->sensors_) {
        auto state_pair = this->store_.get_state_for_sensor_no(it.first);
        if (state_pair.first) {
            it.second->publish_temperature(state_pair.second.temperature * 0.1f);
            it.second->publish_humidity(state_pair.second.humidity);
        }
    }

    for (auto text_sensor : this->text_sensors_) {
        auto state_pair = this->store_.get_last_state(text_sensor->get_sensor_type());
        if (state_pair.first) {
            if (state_pair.second.str() != text_sensor->state) {
                text_sensor->publish_state(state_pair.second.str());
            }
        }
    }

    this->store_.reset();
}

void FineOffsetComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Setting up FineOFfset...");
    LOG_PIN("  Input Pin: ", this->pin_);
    LOG_UPDATE_INTERVAL(this);

    for (auto& it : this->sensors_) {
        LOG_SENSOR("  ", "Sensor", it.second);
    }

    for (auto sensor : this->text_sensors_) {
        LOG_TEXT_SENSOR("  ", "Text sensor", sensor);
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
