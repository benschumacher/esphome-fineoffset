#include "fineoffset_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace fineoffset {

static const char* TAG = "fineoffset.sensor";

void FineOffsetSensor::setup() {
    ESP_LOGCONFIG(TAG, "Setting up FineOffset Sensor...");
    // Initialization code here
}

void FineOffsetSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "FineOffset Sensor:");
    // Configuration dump code here
}

}  // namespace fineoffset
}  // namespace esphome