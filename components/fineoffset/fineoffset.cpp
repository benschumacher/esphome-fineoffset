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
    // Save raw packet for debugging
    for (int i = 0; i < 5; i++) {
        this->raw_packet[i] = packet[i];
    }

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
        this->plausible = this->valid && this->is_plausible();
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
    if (valid && plausible) {
        data += "OK";
    } else if (valid && !plausible) {
        data += "IMPLAUSIBLE";
    } else {
        data += "BAD";
    }

    return data;
}

std::string FineOffsetState::debug_str() const {
    std::string data = this->str();

    // Append raw packet bytes in hex
    char raw_buf[30];
    snprintf(raw_buf, sizeof(raw_buf), " [RAW: %02X %02X %02X %02X %02X]", raw_packet[0], raw_packet[1], raw_packet[2],
             raw_packet[3], raw_packet[4]);
    data += raw_buf;

    // Add CRC info
    uint8_t calculated_crc = crc8ish(raw_packet, 4);
    if (calculated_crc != raw_packet[4]) {
        char crc_buf[40];
        snprintf(crc_buf, sizeof(crc_buf), " CRC calc=%02X recv=%02X", calculated_crc, raw_packet[4]);
        data += crc_buf;
    }

    return data;
}

// WH2 signal timing and decoder constants
namespace signal_timing {
// Pulse timing thresholds (microseconds)
// 1 is indicated by 500µS pulse (wh2_accept from 2=400µs to 3=600µs)
inline constexpr uint32_t HI_PULSE_MIN_US = 250;
inline constexpr uint32_t HI_PULSE_MAX_US = 750;

// 0 is indicated by ~1500µs pulse (wh2_accept from 7=1400µs to 8=1600µs)
inline constexpr uint32_t LOW_PULSE_MIN_US = 1200;
inline constexpr uint32_t LOW_PULSE_MAX_US = 1750;

// Idle period timeouts
inline constexpr uint32_t IDLE_TIMEOUT_US = 1199;  // worst case packet length: 6 bytes x 8 bits = 48
inline constexpr uint32_t IDLE_DONE_US = 751;      // wh2_accept after 4 x 200µs = 800µs

// Type-safe pulse detection functions
constexpr bool is_hi_pulse(uint32_t interval) { return interval >= HI_PULSE_MIN_US && interval <= HI_PULSE_MAX_US; }
constexpr bool is_low_pulse(uint32_t interval) { return interval >= LOW_PULSE_MIN_US && interval <= LOW_PULSE_MAX_US; }
constexpr bool idle_has_timed_out(uint32_t interval) { return interval > IDLE_TIMEOUT_US; }
constexpr bool idle_period_done(uint32_t interval) { return interval >= IDLE_DONE_US; }
}  // namespace signal_timing

namespace decoder_flags {
inline constexpr uint8_t GOT_PULSE = 0x01;
inline constexpr uint8_t LOGIC_HI = 0x02;
}  // namespace decoder_flags

FineOffsetStore::FineOffsetStore(FineOffsetComponent* parent) : parent_(parent), wh2_flags_{0}, packet_state_{0} {}

void IRAM_ATTR FineOffsetStore::intr_cb(FineOffsetStore* self) {
    // NOTE: Static local variables limit this implementation to a single FineOffsetStore instance.
    // Multiple instances would share these statics and interfere with each other.
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

    self->packet_state_.store(wh2_packet_state, std::memory_order_relaxed);
    switch (sampling_state) {
        case 0:  // waiting

            if (signal_timing::is_hi_pulse(pulse)) {
                wh2_flags = decoder_flags::GOT_PULSE | decoder_flags::LOGIC_HI;
                sampling_state = 1;
            } else if (signal_timing::is_low_pulse(pulse)) {
                wh2_flags = decoder_flags::GOT_PULSE;  // logic low
            } else {
                sampling_state = 0;
            }
            break;
        case 1:  // observe 1ms of idle time
            if (signal_timing::idle_has_timed_out(pulse)) {
                sampling_state = 0;
                wh2_packet_state = 1;
            } else if (signal_timing::idle_period_done(pulse)) {
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
            if (wh2_flags & decoder_flags::LOGIC_HI) {
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
            self->accept_flag_.store(false, std::memory_order_relaxed);
        }
        // acquire packet
        else if (wh2_packet_state == 2) {
            wh2_packet[packet_no] <<= 1;
            if (wh2_flags & decoder_flags::LOGIC_HI) {
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
                self->accept_flag_.store(true, std::memory_order_relaxed);
            }
        } else {
            wh2_accept_flag = false;
            self->accept_flag_.store(false, std::memory_order_relaxed);
        }

        if (wh2_accept_flag) {
            uint8_t crc = FineOffsetState::crc8ish(wh2_packet, 4);
            self->cycles_.fetch_add(1, std::memory_order_relaxed);

            if (crc == wh2_packet[4]) {
                wh2_valid = true;
            } else {
                self->bad_count_.fetch_add(1, std::memory_order_relaxed);
                wh2_valid = false;
            }

            FineOffsetState state(wh2_packet);  // Direct value construction - no heap allocation

            // Double-buffered ISR-safe write: write to ISR's buffer, then atomically signal completion
            // avoid change sensor data during update
            if (wh2_valid == true && self->have_sensor_data_.load(std::memory_order_relaxed) == 0) {
                uint8_t buffer_idx = self->isr_buffer_index_.load(std::memory_order_relaxed);
                self->state_buffers_[buffer_idx] = state;  // Safe: main thread uses other buffer
                self->has_pending_state_.store(
                    true, std::memory_order_release);  // Release: ensure state write completes first
                self->have_sensor_data_.store(state.sensor_id, std::memory_order_release);
            } else if (!wh2_valid && self->have_sensor_data_.load(std::memory_order_relaxed) == 0) {
                uint8_t buffer_idx = self->isr_buffer_index_.load(std::memory_order_relaxed);
                self->state_buffers_[buffer_idx] = state;        // Safe: main thread uses other buffer
                self->state_buffers_[buffer_idx].valid = false;  // Ensure marked invalid for debugging
                self->has_pending_state_.store(
                    true, std::memory_order_release);  // Release: ensure state write completes first
            }

            wh2_accept_flag = false;
            self->accept_flag_.store(false, std::memory_order_relaxed);
        }
        wh2_flags = 0;
    }

    self->wh2_flags_.store(wh2_flags, std::memory_order_relaxed);
    //--------------------------------------------------------
}

bool FineOffsetStore::accept() { return this->have_sensor_data_.load(std::memory_order_relaxed) != 0; }

void FineOffsetStore::record_state() {
    if (!this->has_pending_state_.load(std::memory_order_acquire)) {  // Acquire: see all ISR writes
        return;
    }

    // Swap buffers atomically - ISR now writes to the buffer we just read from
    uint8_t read_idx = this->isr_buffer_index_.load(std::memory_order_relaxed);
    uint8_t new_isr_idx = 1 - read_idx;  // Toggle between 0 and 1
    this->isr_buffer_index_.store(new_isr_idx, std::memory_order_relaxed);

    // Now safe to read from old ISR buffer (ISR is writing to the other one)
    FineOffsetState state = this->state_buffers_[read_idx];

    this->has_pending_state_.store(false, std::memory_order_release);

    if (state.sensor_id != 0) {
        // Add to circular buffer (replaces std::deque)
        this->states_[this->states_head_] = state;
        this->states_head_ = (this->states_head_ + 1) % config::MAX_RECENT_STATES;
        if (this->states_count_ < config::MAX_RECENT_STATES) {
            this->states_count_++;
        }

        if (state.valid && state.plausible) {
            // Only store plausible readings for sensor data
            if (state.sensor_id < config::MAX_SENSOR_IDS) {
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
            // Treat CRC-valid but implausible readings as "bad" for debugging
            this->last_bad_ = state;
            this->has_last_bad_ = true;
            this->last_bad_consumed_ = false;  // Mark as fresh data
        }

        if (state.valid && state.plausible) {
            ESP_LOGD(TAG, "%s", state.str().c_str());
        } else {
            // Log invalid/implausible packets with detailed debug info
            ESP_LOGD(TAG, "%s", state.debug_str().c_str());
        }
    }

    this->have_sensor_data_.store(0, std::memory_order_relaxed);
    this->wh2_flags_.store(0, std::memory_order_relaxed);
}

#ifdef USE_TEXT_SENSOR
std::optional<ConsumedStateGuard<FineOffsetState>> FineOffsetStore::get_last_state(
    FineOffsetTextSensorType sensor_type) {
    switch (sensor_type) {
        case FineOffsetTextSensorType::Last:
            if (this->states_count_ > 0) {
                // Get the most recent state from circular buffer (no consumed tracking needed)
                uint8_t last_index = (this->states_head_ + config::MAX_RECENT_STATES - 1) % config::MAX_RECENT_STATES;
                return ConsumedStateGuard<FineOffsetState>(this->states_[last_index], nullptr);
            }
            break;
        case FineOffsetTextSensorType::LastBad:
            if (this->has_last_bad_) {
                return ConsumedStateGuard<FineOffsetState>(this->last_bad_, &this->last_bad_consumed_);
            }
            break;
        case FineOffsetTextSensorType::Unknown:
            if (this->has_last_unknown_) {
                return ConsumedStateGuard<FineOffsetState>(this->last_unknown_, &this->last_unknown_consumed_);
            }
            break;
    }

    return std::nullopt;
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
    this->set_timeout("diagnostic_log", config::DIAGNOSTIC_LOG_INTERVAL_MS, [this]() {
        ESP_LOGV(TAG, "accept_flag_=%s wh2_flags_=%hhu have_sensor_data_=%d packet_state=%hhu cycles_=%u bad_count_=%u",
                 (this->store_.accept_flag_.load(std::memory_order_relaxed) ? "true" : "false"),
                 this->store_.wh2_flags_.load(std::memory_order_relaxed),
                 this->store_.have_sensor_data_.load(std::memory_order_relaxed),
                 this->store_.packet_state_.load(std::memory_order_relaxed),
                 this->store_.cycles_.load(std::memory_order_relaxed),
                 this->store_.bad_count_.load(std::memory_order_relaxed));

        // Perform periodic cleanup of consumed sensor data (critical for data freshness)
        this->store_.periodic_cleanup();

        // Reschedule for next interval
        this->schedule_diagnostic_log();
    });
}

void FineOffsetComponent::loop() {
    if (this->store_.ready()) {
        ESP_LOGV(TAG, "ready, core=%d", xPortGetCoreID());
        this->store_.record_state();
    }
}

}  // namespace fineoffset
}  // namespace esphome
