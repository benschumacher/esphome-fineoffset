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

    void set_temperature_sensor(sensor::Sensor* temperature_sensor) { this->temperature_sensor_ = temperature_sensor; }
    void set_humidity_sensor(sensor::Sensor* humidity_sensor) { this->humidity_sensor_ = humidity_sensor; }

    void publish_temperature(float temperature) {
        if (this->temperature_sensor_) {
            this->temperature_sensor_->publish_state(temperature);
        }
    }
    void publish_humidity(float humidity) {
        if (this->humidity_sensor_) {
            this->humidity_sensor_->publish_state(humidity);
        }
    }

   protected:
    friend FineOffsetComponent;

    uint8_t sensor_no_;

    sensor::Sensor* temperature_sensor_{nullptr};
    sensor::Sensor* humidity_sensor_{nullptr};
};

}  // namespace fineoffset
}  // namespace esphome
