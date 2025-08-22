#include "fineoffset_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace fineoffset {

static const char* TAG = "fineoffset.sensor";

void FineOffsetSensor::setup() { ESP_LOGCONFIG(TAG, "Setting up FineOffset Sensor..."); }

void FineOffsetSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "FineOffset Sensor:");
    ESP_LOGCONFIG(TAG, "  Sensor No: %d", this->sensor_no_);
    LOG_UPDATE_INTERVAL(this);
}

void FineOffsetSensor::update() {
    if (this->parent_ == nullptr) {
        ESP_LOGW(TAG, "No parent set for sensor %d", this->sensor_no_);
        return;
    }

    auto state_guard = this->parent_->get_state_for_sensor_no(this->sensor_no_);
    if (state_guard.has_value()) {
        const auto& state = state_guard->get();
        if (this->temperature_sensor_) {
            this->temperature_sensor_->publish_state(state.temperature * 0.1f);
        }
        if (this->humidity_sensor_) {
            this->humidity_sensor_->publish_state(state.humidity);
        }
    }
}

}  // namespace fineoffset
}  // namespace esphome