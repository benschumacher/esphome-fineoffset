#pragma once

#include "../fineoffset.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace fineoffset {

enum class FineOffsetTextSensorType : uint8_t {
    Last,
    LastBad,
    Unknown,
};

class FineOffsetTextSensor : public text_sensor::TextSensor, public PollingComponent {
   public:
    void setup() override;
    void dump_config() override;
    void update() override;
    void set_sensor_type(FineOffsetTextSensorType sensor_type) { this->sensor_type_ = sensor_type; }
    FineOffsetTextSensorType get_sensor_type() const { return this->sensor_type_; }

    void set_parent(FineOffsetComponent* parent) { this->parent_ = parent; }
    FineOffsetComponent* get_parent() const { return this->parent_; }

   protected:
    FineOffsetTextSensorType sensor_type_;
    FineOffsetComponent* parent_{nullptr};
};

}  // namespace fineoffset
}  // namespace esphome
