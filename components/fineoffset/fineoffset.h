#pragma once

#include <esp_types.h>

#include <atomic>
#include <optional>
#include <set>

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

// RAII helper for automatic consumed state tracking
template<typename T> class ConsumedStateGuard {
   public:
    ConsumedStateGuard(const T& data, bool* consumed_flag) : data_(data), consumed_flag_(consumed_flag) {}
    ~ConsumedStateGuard() {
        if (consumed_flag_)
            *consumed_flag_ = true;
    }

    // Move-only semantics
    ConsumedStateGuard(const ConsumedStateGuard&) = delete;
    ConsumedStateGuard& operator=(const ConsumedStateGuard&) = delete;
    ConsumedStateGuard(ConsumedStateGuard&& other) noexcept
        : data_(std::move(other.data_)), consumed_flag_(other.consumed_flag_) {
        other.consumed_flag_ = nullptr;
    }

    const T& get() const { return data_; }

   private:
    T data_;
    bool* consumed_flag_;
};

class FineOffsetComponent;

#ifdef USE_TEXT_SENSOR
enum class FineOffsetTextSensorType : uint8_t;
#endif

using byte = uint8_t;

#ifdef USE_SENSOR
using FineOffsetChild = Parented<FineOffsetComponent>;
#endif

struct FineOffsetState {
    FineOffsetState() = default;
    FineOffsetState(byte packet[5]);

    std::string str() const;
    std::string debug_str() const;  // Detailed debug string with raw packet
    [[nodiscard]] constexpr bool is_plausible() const {
        return humidity <= 100 && temperature >= -400 && temperature <= 800;  // -40.0°C to 80.0°C (in 0.1°C units)
    }

    uint32_t sensor_id{0};
    int32_t temperature{0};
    uint32_t humidity{0};
    bool valid{false};
    bool plausible{false};
    byte raw_packet[5]{0, 0, 0, 0, 0};  // Store raw packet bytes for debugging

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
    [[nodiscard]] bool accept();
    [[nodiscard]] bool ready() {
        return (this->have_sensor_data_.load(std::memory_order_relaxed) != 0 ||
                this->has_pending_state_.load(std::memory_order_relaxed));
    }
    void record_state();

    [[nodiscard]] std::optional<ConsumedStateGuard<FineOffsetState>> get_state_for_sensor_no(uint32_t sensor_id) {
        if (sensor_id >= config::MAX_SENSOR_IDS || !this->sensor_states_valid_[sensor_id]) {
            return std::nullopt;
        }
        return ConsumedStateGuard<FineOffsetState>(this->sensor_states_[sensor_id],
                                                   &this->sensor_states_consumed_[sensor_id]);
    }
#ifdef USE_TEXT_SENSOR
    [[nodiscard]] std::optional<ConsumedStateGuard<FineOffsetState>> get_last_state(FineOffsetTextSensorType type);
#endif
    // Periodic cleanup of consumed data (called every 60s for data freshness)
    void periodic_cleanup() { this->reset(); }

    // Manual reset for full cleanup (for testing/debugging)
    void reset() {
        // Clear consumed data to maintain data freshness
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

        // Clear the circular buffer (not sensor-specific)
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
    std::atomic<uint8_t> have_sensor_data_{0};
    std::atomic<uint32_t> cycles_{0};
    std::atomic<uint32_t> bad_count_{0};
    std::atomic<byte> packet_state_;

    // Double-buffered state for ISR-safe communication (lock-free)
    // ISR writes to state_buffers_[isr_buffer_index_], main thread reads from the other buffer
    FineOffsetState state_buffers_[2]{};
    std::atomic<uint8_t> isr_buffer_index_{0};  // Which buffer ISR writes to (0 or 1)
    std::atomic<bool> has_pending_state_{false};

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
    [[nodiscard]] std::optional<ConsumedStateGuard<FineOffsetState>> get_state_for_sensor_no(uint8_t sensor_no) {
        return this->store_.get_state_for_sensor_no(sensor_no);
    }

    // Register known sensor IDs for unknown sensor detection
    void register_known_sensor(uint8_t sensor_no) { this->known_sensor_ids_.insert(sensor_no); }

    [[nodiscard]] bool is_unknown_sensor(uint8_t sensor_no) const {
        return this->known_sensor_ids_.find(sensor_no) == this->known_sensor_ids_.end();
    }

#ifdef USE_TEXT_SENSOR
    [[nodiscard]] std::optional<ConsumedStateGuard<FineOffsetState>> get_last_state(FineOffsetTextSensorType type) {
        return this->store_.get_last_state(type);
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