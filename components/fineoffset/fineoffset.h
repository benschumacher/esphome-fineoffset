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
class InternalGPIOPin;
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
    bool ready() { return (this->have_sensor_data_.load() != 0 || this->state_obj_ != nullptr); }
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

    std::shared_ptr<FineOffsetState> state_obj_{nullptr};

    std::deque<FineOffsetState> states_;
    std::map<uint32_t, FineOffsetState> state_by_sensor_id_;

    std::shared_ptr<FineOffsetState> last_bad_{nullptr};
    std::shared_ptr<FineOffsetState> last_unknown_{nullptr};
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
    bool is_unknown(uint8_t sensor_no) const { return this->sensors_.find(sensor_no) == this->sensors_.end(); }

   protected:
    FineOffsetStore store_;
    InternalGPIOPin* pin_;

    std::map<uint32_t, FineOffsetSensor*> sensors_;
    std::vector<FineOffsetTextSensor*> text_sensors_;
};

}  // namespace fineoffset
}  // namespace esphome