#include "fineoffset.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace fineoffset {

static const char *const TAG = "fineoffset";

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

    char temperture_buf[7] = {0, }; // 7 bytes is enough for -100.0 with a '\0'
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

#define COUNTER_RATE 3200-1 // 16,000,000Hz / 3200 = 5000 interrupts per second, ie. 200us between interrupts
// 1 is indicated by 500uS pulse
// wh2_accept from 2 = 400us to 3 = 600us
#define IS_HI_PULSE(interval)   (interval >= 2 && interval <= 3)
// 0 is indicated by ~1500us pulse
// wh2_accept from 7 = 1400us to 8 = 1600us
#define IS_LOW_PULSE(interval)  (interval >= 7 && interval <= 8)
// worst case packet length
// 6 bytes x 8 bits x (1.5 + 1) = 120ms; 120ms = 200us x 600
#define HAS_TIMED_OUT(interval) (interval > 600)
// we expect 1ms of idle time between pulses
// so if our pulse hasn't arrived by 1.2ms, reset the wh2_packet_state machine
// 6 x 200us = 1.2ms
#define IDLE_HAS_TIMED_OUT(interval) (interval > 6)
// our expected pulse should arrive after 1ms
// we'll wh2_accept it if it arrives after
// 4 x 200us = 800us
#define IDLE_PERIOD_DONE(interval) (interval >= 4)
// Shorthand for tests
//#define RF_HI (digitalRead(RF_IN) == HIGH)
//#define RF_LOW (digitalRead(RF_IN) == LOW)
#define RF_HI(pin) ((pin).digital_read())
#define RF_LOW(pin) (!(pin).digital_read())

// wh2_flags
#define GOT_PULSE 0x01
#define LOGIC_HI  0x02

FineOffsetStore::FineOffsetStore()
    : sampling_state_{0}, sample_count_{0}, was_low_(false), wh2_flags_{0},
      wh2_packet_state_{0}, wh2_timeout_{0},  wh2_packet_{0},
      wh2_calculated_crc_{0}, cycles_(0), packet_no_{0}, bit_no_{0}, history_{0}
{ }

void IRAM_ATTR FineOffsetStore::intr_cb(FineOffsetStore *self) {
    switch (self->sampling_state_) {
    case 0: // waiting
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
    case 1: // acquiring first pulse
        self->sample_count_++;
        // end of first pulse
        if (RF_LOW(self->pin_)) {
            if (IS_HI_PULSE(self->sample_count_)) {
                self->wh2_flags_ = GOT_PULSE | LOGIC_HI;
                self->sampling_state_ = 2;
                self->sample_count_ = 0;        
            } else if (IS_LOW_PULSE(self->sample_count_)) {
                self->wh2_flags_ = GOT_PULSE; // logic low
                self->sampling_state_ = 2;
                self->sample_count_ = 0;
            } else {
                self->sampling_state_ = 0;
            }    
        }   
        break;
    case 2: // observe 1ms of idle time
        self->sample_count_++;
        if (RF_HI(self->pin_)) {
            if (IDLE_HAS_TIMED_OUT(self->sample_count_)) {
                self->sampling_state_ = 0;
            } else if (IDLE_PERIOD_DONE(self->sample_count_)) {
                self->sampling_state_ = 1;
                self->sample_count_ = 0;
            }
        }     
        break;        
    }

    if (self->wh2_timeout_ > 0) {
        self->wh2_timeout_++;
        if (HAS_TIMED_OUT(self->wh2_timeout_)) {
            self->wh2_packet_state_ = {0};
            self->wh2_timeout_ = 0;
        }
    }
}

bool FineOffsetStore::accept() {
    // reset if in initial wh2_packet_state
    if (wh2_packet_state_ == 0) {
        // should history be 0, does it matter?
        history_ = 0xFF;
        wh2_packet_state_ = 1;
        // enable wh2_timeout
        wh2_timeout_ = 1;
    } // fall thru to wh2_packet_state one
    
    // acquire preamble
    if (wh2_packet_state_ == 1) {
        // shift history right and store new value
        history_ <<= 1;
        // store a 1 if required (right shift along will store a 0)
        if (wh2_flags_ & LOGIC_HI) {
            history_ |= 0x01;
        }
        // check if we have a valid start of frame
        // xxxxx110
        if ((history_ & 0xB00000111) == 0xB00000110) {
            // need to clear packet, and counters
            packet_no_ = 0;
            // start at 1 becuase only need to acquire 7 bits for first packet byte.
            bit_no_ = 1;
            wh2_packet_[0] = wh2_packet_[1] = wh2_packet_[2] = wh2_packet_[3] = wh2_packet_[4] = 0;
            // we've acquired the preamble
            wh2_packet_state_ = 2;
        }
        return false;
    }

    // acquire packet
    if (wh2_packet_state_ == 2) {
        wh2_packet_[packet_no_] <<= 1;
        if (wh2_flags_ & LOGIC_HI) {
            wh2_packet_[packet_no_] |= 0x01;
        }

        bit_no_++;
        if (bit_no_ > 7) {
            bit_no_ = 0;
            packet_no_++;
        }

        if (packet_no_ > 4) {
            // start the sampling process from scratch
            wh2_packet_state_ = 0;
            // clear wh2_timeout
            wh2_timeout_ = 0;

            increment_cycles_();
            auto state = FineOffsetState(wh2_packet_);

            if (states_.size() == 10) {
                states_.pop_front();
            }
            states_.push_back(state);
            
            if (state.valid) {
                state_by_sensor_id_.insert({state.sensor_id, state});
            }

            ESP_LOGD(TAG, "%s", state.c_str());

            return true;
        }
    }
    return false;
}

void FineOffsetComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Setting up FineOFfset...");
    LOG_PIN("  Input Pin: ", this->pin_);
    LOG_UPDATE_INTERVAL(this);
}

void FineOffsetComponent::loop() {
    const auto now = millis();
    uint32_t spacing;

    if (store_.ready()) {
        if (store_.accept()) {
            // store_.spacing = store_.now - store_.old;
            // store_.old = store_.now;
        }
    }
}

void FineOffsetComponent::update() {
    
}

} // namespace fineoffset
} // namespace esphome