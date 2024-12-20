#pragma once

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

using FineOffsetChild = Parented<FineOffsetComponent>;
using byte = uint8_t;

struct FineOffsetState {
    FineOffsetState() = default;
    FineOffsetState(byte packet[5]);

    const char* c_str() const;

    uint32_t sensor_id;
    int32_t temperature;
    uint32_t humidity;
    bool valid;
};

class FineOffsetStore {
   public:
    FineOffsetStore();
    void setup(InternalGPIOPin* pin) {
        pin->setup();
        this->pin_ = pin->to_isr();
        pin->attach_interrupt(&FineOffsetStore::intr_cb, this, gpio::INTERRUPT_ANY_EDGE);
    }
    bool accept();
    bool ready() { return wh2_flags_ != 0; }

    static void intr_cb(FineOffsetStore* arg);

   protected:
    uint32_t increment_cycles_() { return ++cycles_; }

    ISRInternalGPIOPin pin_;

    // volatile uint32_t edge_timestamps_[3];
    // volatile bool skip_;
    byte sampling_state_;
    byte sample_count_;
    bool was_low_;

    volatile byte wh2_flags_;
    volatile byte wh2_packet_state_;
    volatile int wh2_timeout_;

    byte wh2_packet_[5];
    byte wh2_calculated_crc_;
    uint32_t cycles_;

    byte packet_no_;
    byte bit_no_;
    byte history_;

    std::deque<FineOffsetState> states_;
    std::map<uint32_t, FineOffsetState> state_by_sensor_id_;
};

class FineOffsetSensor;
class FineOffsetTextSensor;

class FineOffsetComponent : public PollingComponent {
   public:
    void set_pin(InternalGPIOPin* pin) { pin_ = pin; }
    void setup() override { this->store_.setup(this->pin_); }
    void loop() override;
    void dump_config() override;
    void update() override;

    void register_sensor(uint8_t sensor_no, FineOffsetSensor* obj);
    void register_text_sensor(FineOffsetTextSensor* obj);

   protected:
    FineOffsetStore store_;
    InternalGPIOPin* pin_;

    std::map<uint32_t, FineOffsetSensor*> sensors_;
    std::vector<FineOffsetTextSensor*> text_sensors_;
};

}  // namespace fineoffset
}  // namespace esphome