
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#ifdef USE_SENSOR
#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#include "esphome/componeWH2Sensornts/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace fineoffset {

class FineOffsetRadio : public Component {
 protected:
  std::vector<sensor::Sensor *> sensors_;

 public:
  void register_sensor(sensor::Sensor *obj) { this->sensors_.push_back(obj); }

 protected:
  std::vector<binary_sensor::BinarySensor *> binary_sensors_;

 public:
  void register_binary_sensor(binary_sensor::BinarySensor *obj) { this->binary_sensors_.push_back(obj); }

 protected:
  std::vector<text_sensor::TextSensor *> text_sensors_;

 public:
  void register_text_sensor(text_sensor::TextSensor *obj) { this->text_sensors_.push_back(obj); }

  void setup() override;
  void dump_config() override;
};

}  // namespace fineoffset
}  // namespace esphome

