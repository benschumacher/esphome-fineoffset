#include "fineoffset_text_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace fineoffset {

static const char *TAG = "fineoffset.text_sensor";

void FineOffsetTextSensor::setup() {
    ESP_LOGCONFIG(TAG, "Setting up FineOffset Text Sensor...");
    // Additional setup code can be added here
}

void FineOffsetTextSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "FineOffset Text Sensor:");
    ESP_LOGCONFIG(TAG, "  Sensor Type: %d", this->sensor_type_);
    // Additional configuration dumping can be added here
}

}  // namespace fineoffset
}  // namespace esphome