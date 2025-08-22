#pragma once

#include <esp_types.h>

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
class InternalGPIOPin;
namespace sensor {
class Sensor;
}
namespace text_sensor {
class TextSensor;
}
namespace fineoffset {

// Configuration constants for memory allocation and timing
namespace config {
static constexpr size_t MAX_RECENT_STATES = 10;                // Circular buffer size for recent sensor readings
static constexpr size_t MAX_SENSOR_IDS = 256;                  // Maximum number of sensor IDs (8-bit addressing)
static constexpr uint32_t DIAGNOSTIC_LOG_INTERVAL_MS = 60000;  // Diagnostic logging interval
}  // namespace config

class FineOffsetComponent;

#ifdef USE_TEXT_SENSOR
enum FineOffsetTextSensorType : uint8_t;
#endif

using byte = uint8_t;

#ifdef USE_SENSOR
using FineOffsetChild = Parented<FineOffsetComponent>;
#endif

struct FineOffsetState {
    FineOffsetState() = default;
    FineOffsetState(byte packet[5]);

    std::string str() const;

    uint32_t sensor_id{0};
    int32_t temperature{0};
    uint32_t humidity{0};
    bool valid{false};

    static uint8_t crc8ish(const byte data[], uint8_t len) {
        uint8_t crc = 0;
        uint8_t addr = 0;
        // Indicated changes are from reference CRC-8 function in OneWire library
        while (len--) {
            uint8_t inbyte = data[addr++];
            for (uint8_t i = 8; i; i--) {
                uint8_t mix = (crc ^ inbyte) & 0x80;  // changed from & 0x01
                crc <<= 1;                            // changed from right shift
                if (mix)
                    crc ^= 0x31;  // changed from 0x8C;
                inbyte <<= 1;     // changed from right shift
            }
        }
        return crc;
    }
};

class FineOffsetStore {
   public:
    FineOffsetStore(FineOffsetComponent* parent);

    void setup(InternalGPIOPin* pin) {
        pin->pin_mode(gpio::FLAG_INPUT);
        pin->setup();
        pin->attach_interrupt(&FineOffsetStore::intr_cb, this, gpio::INTERRUPT_ANY_EDGE);
        this->pin_ = pin->to_isr();
    }
    bool accept();
    bool ready() { return (this->have_sensor_data_.load() != 0 || this->has_pending_state_); }
    void record_state();

    std::pair<bool, const FineOffsetState> get_state_for_sensor_no(uint32_t sensor_id) {
        if (sensor_id >= config::MAX_SENSOR_IDS || !this->sensor_states_valid_[sensor_id]) {
            return {false, FineOffsetState()};
        }
        // Mark as consumed for selective clearing
        this->sensor_states_consumed_[sensor_id] = true;
        return {true, this->sensor_states_[sensor_id]};
    }
#ifdef USE_TEXT_SENSOR
    std::pair<bool, const FineOffsetState> get_last_state(FineOffsetTextSensorType type);
#endif
    // Periodic cleanup of consumed data (called every 60s for data freshness)
    void periodic_cleanup() { this->reset(); }

    // Manual reset for full cleanup (for testing/debugging)
    void reset() {
        // Selective clearing: only clear consumed data to maintain data freshness
        for (size_t i = 0; i < config::MAX_SENSOR_IDS; ++i) {
            if (this->sensor_states_consumed_[i]) {
                this->sensor_states_valid_[i] = false;
                this->sensor_states_consumed_[i] = false;
            }
        }

        if (this->last_bad_consumed_) {
            this->has_last_bad_ = false;
            this->last_bad_consumed_ = false;
        }

        if (this->last_unknown_consumed_) {
            this->has_last_unknown_ = false;
            this->last_unknown_consumed_ = false;
        }

        // Always clear the circular buffer as it's not sensor-specific
        this->states_count_ = 0;
        this->states_head_ = 0;
    }

    static void intr_cb(FineOffsetStore* arg);

   protected:
    friend class FineOffsetComponent;

    FineOffsetStore() = delete;
    FineOffsetStore(const FineOffsetStore&) = delete;

    FineOffsetComponent* parent_;
    ISRInternalGPIOPin pin_;

    std::atomic<byte> wh2_flags_{0};
    std::atomic<bool> accept_flag_{false};
    volatile std::atomic<std::uint8_t> have_sensor_data_{0};
    volatile uint32_t cycles_{0};
    volatile uint32_t bad_count_{0};
    std::atomic<byte> packet_state_;

    // Memory-optimized: eliminate heap allocation and fragmentation
    FineOffsetState state_obj_{};
    bool has_pending_state_{false};

    // Fixed-size circular buffer for recent states (replaces std::deque)
    FineOffsetState states_[config::MAX_RECENT_STATES]{};
    uint8_t states_head_{0};
    uint8_t states_count_{0};

    // Fixed-size array for sensor states (replaces std::map) - 8-bit sensor IDs = max 256
    FineOffsetState sensor_states_[config::MAX_SENSOR_IDS]{};
    bool sensor_states_valid_[config::MAX_SENSOR_IDS]{};
    bool sensor_states_consumed_[config::MAX_SENSOR_IDS]{};  // Track if data has been read

    // Value semantics for last bad/unknown (replaces shared_ptr)
    FineOffsetState last_bad_{};
    bool has_last_bad_{false};
    bool last_bad_consumed_{true};  // Start as consumed

    FineOffsetState last_unknown_{};
    bool has_last_unknown_{false};
    bool last_unknown_consumed_{true};  // Start as consumed
};

#ifdef USE_SENSOR
class FineOffsetSensor;
#endif
#ifdef USE_TEXT_SENSOR
class FineOffsetTextSensor;
#endif

class FineOffsetComponent : public Component {
   public:
    FineOffsetComponent() : store_(this) {}

    void set_pin(InternalGPIOPin* pin) { pin_ = pin; }
    void setup() override;
    void loop() override;
    void dump_config() override;

    // Public API for sensors to pull data
    std::optional<FineOffsetState> get_state_for_sensor_no(uint8_t sensor_no) {
        auto [found, state] = this->store_.get_state_for_sensor_no(sensor_no);
        return found ? std::make_optional(state) : std::nullopt;
    }

    // Register known sensor IDs for unknown sensor detection
    void register_known_sensor(uint8_t sensor_no) { this->known_sensor_ids_.insert(sensor_no); }

    bool is_unknown_sensor(uint8_t sensor_no) const {
        return this->known_sensor_ids_.find(sensor_no) == this->known_sensor_ids_.end();
    }

#ifdef USE_TEXT_SENSOR
    std::optional<FineOffsetState> get_last_state(FineOffsetTextSensorType type) {
        auto [found, state] = this->store_.get_last_state(type);
        return found ? std::make_optional(state) : std::nullopt;
    }
#endif

   protected:
    friend class FineOffsetStore;
    FineOffsetStore store_;
    InternalGPIOPin* pin_;
    std::set<uint8_t> known_sensor_ids_;

    void schedule_diagnostic_log();
};

}  // namespace fineoffset
}  // namespace esphome