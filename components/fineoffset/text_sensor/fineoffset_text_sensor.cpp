#include "fineoffset_text_sensor.h"
#include "esphome/core/log.h"

namespace esphome {
namespace fineoffset {

static const char* TAG = "fineoffset.text_sensor";

void FineOffsetTextSensor::setup() { ESP_LOGCONFIG(TAG, "Setting up FineOffset Text Sensor..."); }

void FineOffsetTextSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "FineOffset Text Sensor:");
    ESP_LOGCONFIG(TAG, "  Sensor Type: %d", this->sensor_type_);
    LOG_UPDATE_INTERVAL(this);
}

void FineOffsetTextSensor::update() {
    if (this->parent_ == nullptr) {
        ESP_LOGW(TAG, "No parent set for text sensor");
        return;
    }

    auto state_guard = this->parent_->get_last_state(this->sensor_type_);
    if (state_guard.has_value()) {
        const auto& state = state_guard->get();
        std::string state_str = state.str();
        if (state_str != this->state) {
            this->publish_state(state_str);
        }
    }
}

}  // namespace fineoffset
}  // namespace esphome