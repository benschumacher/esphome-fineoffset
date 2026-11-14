#include "fineoffset_sensor.h"

#include <cmath>

#include "esphome/core/log.h"

namespace esphome {
namespace fineoffset {

static const char* const TAG = "fineoffset.sensor";

void FineOffsetSensorNoNumber::control(float value) {
    // HA enforces min/max/step from the schema, but guard anyway: round and clamp to a valid 8-bit ID.
    const long rounded = std::lround(value);
    if (rounded < 1 || rounded > 255) {
        ESP_LOGW(TAG, "Ignoring out-of-range sensor_no %.0f (valid: 1-255)", value);
        this->publish_state(this->parent_->get_sensor_no());  // snap the UI back to the current value
        return;
    }
    this->parent_->apply_sensor_no(static_cast<uint8_t>(rounded), true);
}

void FineOffsetSensor::setup() {
    ESP_LOGCONFIG(TAG, "Setting up FineOffset Sensor...");

    if (this->sensor_no_number_ == nullptr) {
        return;  // No runtime override configured; YAML sensor_no is final.
    }

    // The YAML default is XORed into the preference key, so changing sensor_no in YAML and reflashing
    // discards any stale runtime override instead of silently ignoring the new YAML value.
    this->pref_ = this->sensor_no_number_->make_entity_preference<uint8_t>(this->default_sensor_no_);

    uint8_t stored = 0;
    if (this->pref_.load(&stored) && stored != 0) {
        ESP_LOGI(TAG, "Restored sensor_no %u (YAML default %u)", stored, this->default_sensor_no_);
        this->apply_sensor_no(stored, false);
    }
    this->sensor_no_number_->publish_state(this->sensor_no_);
}

void FineOffsetSensor::apply_sensor_no(uint8_t sensor_no, bool persist) {
    if (sensor_no == 0) {
        ESP_LOGW(TAG, "Ignoring sensor_no 0 (reserved/invalid)");
        return;
    }

    if (sensor_no != this->sensor_no_) {
        ESP_LOGI(TAG, "sensor_no changed %u -> %u", this->sensor_no_, sensor_no);
        if (this->parent_ != nullptr) {
            // NOTE: assumes IDs are not shared between sensor configs; unregistering a shared ID would
            // make the other config's ID look "unknown" in the diagnostic text sensor (cosmetic only).
            this->parent_->unregister_known_sensor(this->sensor_no_);
            this->parent_->register_known_sensor(sensor_no);
        }
        this->sensor_no_ = sensor_no;
    }

    if (persist) {
        this->pref_.save(&this->sensor_no_);
    }
    if (this->sensor_no_number_ != nullptr) {
        this->sensor_no_number_->publish_state(this->sensor_no_);
    }
}

void FineOffsetSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "FineOffset Sensor:");
    ESP_LOGCONFIG(TAG, "  Sensor No: %u (YAML default: %u)", this->sensor_no_, this->default_sensor_no_);
    if (this->sensor_no_number_ != nullptr) {
        LOG_NUMBER("  ", "Sensor No Number", this->sensor_no_number_);
    }
    LOG_UPDATE_INTERVAL(this);
}

void FineOffsetSensor::update() {
    if (this->parent_ == nullptr) {
        ESP_LOGW(TAG, "No parent set for sensor %u", this->sensor_no_);
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
