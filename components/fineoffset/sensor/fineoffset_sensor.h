#pragma once

#include "../fineoffset.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace fineoffset {

class FineOffsetSensor : public sensor::Sensor, public Component, public FineOffsetChild {
   public:
    void dump_config() override;
    void setup() override;

    uint8_t get_sensor_no() const { return this->sensor_no_; }
    void set_sensor_no(uint8_t sensor_no) { this->sensor_no_ = sensor_no; }

   protected:
    friend FineOffsetComponent;
    uint8_t sensor_no_;
};

}  // namespace fineoffset
}  // namespace esphome
