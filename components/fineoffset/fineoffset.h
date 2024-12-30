#pragma once

#include <esp_types.h>
#include <atomic>
#include <map>
#include <vector>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace fineoffset {

class FineOffsetComponent;
enum FineOffsetTextSensorType : uint8_t;

using byte = uint8_t;
using FineOffsetChild = Parented<FineOffsetComponent>;

struct FineOffsetState {
    FineOffsetState() = default;
    FineOffsetState(byte packet[5]);

    std::string str() const;

    uint32_t sensor_id{0};
    int32_t temperature{0};
    uint32_t humidity{0};
    bool valid{false};

    static uint8_t crc8ish(const byte data[], uint8_t len);
};

class FineOffsetStore {
public:
    FineOffsetStore(FineOffsetComponent* parent);
    ~FineOffsetStore() = default;

    void setup(InternalGPIOPin* pin) {
        pin->pin_mode(gpio::FLAG_INPUT);
        pin->setup();
        pin->attach_interrupt(&FineOffsetStore::intr_cb, this, gpio::INTERRUPT_ANY_EDGE);
        this->pin_ = pin->to_isr();
    }
    
    bool accept();
    void record_state();
    
    std::pair<bool, const FineOffsetState> get_state_for_sensor_no(uint32_t sensor_id) const {
        auto it = this->state_by_sensor_id_.find(sensor_id);
        if (it == this->state_by_sensor_id_.end()) {
            return {false, FineOffsetState()};
        }
        return {true, it->second};
    }
    
    std::pair<bool, const FineOffsetState> get_last_state(FineOffsetTextSensorType type) const;
    
    void reset() {
        this->states_.clear();
        this->state_by_sensor_id_.clear();
        this->last_bad_.reset();
        this->last_unknown_.reset();
    }

protected:
    friend class FineOffsetComponent;

    // ISR data structure
    struct ISRData {
        // State tracking
        std::atomic<bool> last_pin_state;
        std::atomic<uint8_t> frame_state;  // 0=waiting, 1=data
        std::atomic<bool> packet_ready;
        std::atomic<uint8_t> history;      // For preamble detection
        
        // Packet assembly
        std::atomic<uint8_t> packet_buffer[5];
        std::atomic<uint8_t> bit_position;
        std::atomic<uint8_t> byte_position;
        
        // Edge timing (using separate atomics instead of struct)
        std::atomic<uint32_t> edge_timestamps[3];
        std::atomic<bool> edge_rising[3];
        
        ISRData();  // Constructor declaration
    };

    static void IRAM_ATTR intr_cb(FineOffsetStore* arg);

    FineOffsetStore() = delete;
    FineOffsetStore(const FineOffsetStore&) = delete;

    FineOffsetComponent* parent_;
    ISRInternalGPIOPin pin_;
    ISRData isr_data_;

    // State storage
    std::deque<FineOffsetState> states_;
    std::map<uint32_t, FineOffsetState> state_by_sensor_id_;
    std::shared_ptr<FineOffsetState> last_bad_;
    std::shared_ptr<FineOffsetState> last_unknown_;
};

class FineOffsetSensor;
class FineOffsetTextSensor;

class FineOffsetComponent : public PollingComponent {
public:
    FineOffsetComponent() : store_(this) {}

    void set_pin(InternalGPIOPin* pin) { pin_ = pin; }
    void setup() override {
        PollingComponent::setup();
        this->store_.setup(this->pin_);
    }
    void loop() override;
    void dump_config() override;
    void update() override;

    void register_sensor(uint8_t sensor_no, FineOffsetSensor* obj);
    void register_text_sensor(FineOffsetTextSensor* obj);
    bool is_unknown(uint8_t sensor_no) const { 
        return this->sensors_.find(sensor_no) == this->sensors_.end(); 
    }

protected:
    FineOffsetStore store_;
    InternalGPIOPin* pin_;

    std::map<uint32_t, FineOffsetSensor*> sensors_;
    std::vector<FineOffsetTextSensor*> text_sensors_;
};

}  // namespace fineoffset
}  // namespace esphome
