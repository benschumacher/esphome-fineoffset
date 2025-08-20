#pragma once

#include "../fineoffset.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace fineoffset {

class FineOffsetSensor : public PollingComponent {
   public:
    void dump_config() override;
    void setup() override;
    void update() override;

    uint8_t get_sensor_no() const { return this->sensor_no_; }
    void set_sensor_no(uint8_t sensor_no) { this->sensor_no_ = sensor_no; }

    void set_temperature_sensor(sensor::Sensor* temperature_sensor) { this->temperature_sensor_ = temperature_sensor; }
    void set_humidity_sensor(sensor::Sensor* humidity_sensor) { this->humidity_sensor_ = humidity_sensor; }

    void set_parent(FineOffsetComponent* parent) { this->parent_ = parent; }
    FineOffsetComponent* get_parent() const { return this->parent_; }

   protected:
    uint8_t sensor_no_;
    FineOffsetComponent* parent_{nullptr};

    sensor::Sensor* temperature_sensor_{nullptr};
    sensor::Sensor* humidity_sensor_{nullptr};
};

}  // namespace fineoffset
}  // namespace esphome
