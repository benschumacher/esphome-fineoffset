#include "fineoffset_sensor.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>

namespace esphome {
namespace fineoffset {

static const char* const TAG = "fineoffset.sensor";

// Unknown-ID packets required before drifting. WH2 sends each reading twice, so 2 == one real transmission
// (or two), and filters out single noise frames that happen to pass the 8-bit CRC.
static constexpr uint8_t DRIFT_MIN_PACKETS = 2;

void FineOffsetConfiguredSensorNoNumber::control(float value) {
    // HA enforces min/max/step from the schema, but guard anyway: round and clamp to a valid 8-bit ID.
    const long rounded = std::lround(value);
    if (rounded < 1 || rounded > 255) {
        ESP_LOGW(TAG, "Ignoring out-of-range sensor_no %.0f (valid: 1-255)", value);
        this->publish_state(this->parent_->get_configured_sensor_no());  // snap the UI back
        return;
    }
    this->parent_->set_configured_sensor_no(static_cast<uint8_t>(rounded));
}

void FineOffsetAcceptDriftButton::press_action() { this->parent_->accept_drift(); }

void FineOffsetSensor::setup() {
    ESP_LOGCONFIG(TAG, "Setting up FineOffset Sensor...");

    // Salting the key with the YAML default means a reflash with a different sensor_no starts clean.
    const uint32_t key =
        fnv1_hash_extend(fnv1_hash_extend(fnv1_hash("fineoffset.sensor_ids"), this->default_sensor_no_), PREF_VERSION);
    this->pref_ =
        global_preferences->make_preference<SavedIds>(key, /*in_flash=*/true);  // survive power loss (ESP8266)

    uint8_t configured = this->default_sensor_no_;
    uint8_t active = this->default_sensor_no_;
    SavedIds stored{};
    if (this->pref_.load(&stored) && stored.configured != 0 && stored.active != 0) {
        this->saved_ = stored;
        configured = stored.configured;
        // A stored drift is only honored if drift is still enabled.
        active = this->drift_enabled_ ? stored.active : stored.configured;
        ESP_LOGI(TAG, "Restored sensor_no configured=%u active=%u (YAML default %u)", configured, active,
                 this->default_sensor_no_);
    }

    if (configured != active) {
        // Snap-back window starts now: any packet from the configured ID after boot reverts the drift.
        this->drifted_at_ms_ = millis();
    }
    this->set_ids_(configured, active);

    if (this->online_binary_sensor_ != nullptr) {
        this->online_binary_sensor_->publish_initial_state(false);
    }
}

void FineOffsetSensor::set_ids_(uint8_t configured, uint8_t active) {
    if (this->parent_ != nullptr) {
        if (this->ids_registered_) {
            this->parent_->unregister_known_sensor(this->configured_);
            this->parent_->unregister_known_sensor(this->active_);
        }
        this->parent_->register_known_sensor(configured);
        this->parent_->register_known_sensor(active);
        this->ids_registered_ = true;
    }

    this->configured_ = configured;
    this->active_ = active;

    // Only write flash when something changed (and never for the plain YAML default on first boot).
    const SavedIds next{configured, active};
    const bool is_default = configured == this->default_sensor_no_ && active == this->default_sensor_no_;
    const bool changed = next.configured != this->saved_.configured || next.active != this->saved_.active;
    if (changed && !(is_default && this->saved_.configured == 0)) {
        this->pref_.save(&next);
        this->saved_ = next;
    }

    this->publish_status_();
}

void FineOffsetSensor::set_configured_sensor_no(uint8_t sensor_no) {
    if (sensor_no == 0) {
        ESP_LOGW(TAG, "Ignoring sensor_no 0 (reserved/invalid)");
        return;
    }
    ESP_LOGI(TAG, "sensor_no set to %u (was configured=%u active=%u)", sensor_no, this->configured_, this->active_);
    this->set_ids_(sensor_no, sensor_no);
}

void FineOffsetSensor::accept_drift() {
    if (!this->is_drifted()) {
        ESP_LOGI(TAG, "Accept drift: not drifted (sensor_no %u), nothing to do", this->configured_);
        return;
    }
    ESP_LOGI(TAG, "Accepted drift: configured sensor_no %u -> %u", this->configured_, this->active_);
    this->set_ids_(this->active_, this->active_);
}

void FineOffsetSensor::evaluate_drift_() {
    if (!this->drift_enabled_ || this->parent_ == nullptr) {
        return;
    }
    const uint32_t now = millis();

    // 1. Snap back: the configured ID was heard after we drifted away from it.
    if (this->is_drifted()) {
        const auto configured_age = this->parent_->last_seen_age_ms(this->configured_);
        if (configured_age.has_value() && *configured_age < now - this->drifted_at_ms_) {
            ESP_LOGI(TAG, "Configured sensor_no %u is back; reverting drift from %u", this->configured_, this->active_);
            this->set_ids_(this->configured_, this->configured_);
            return;
        }
    }

    // 2. Drift: active ID silent for >= timeout (never heard since boot counts as silent since boot)...
    const auto active_age = this->parent_->last_seen_age_ms(this->active_);
    const uint32_t silent_ms = active_age.value_or(now);
    if (silent_ms < this->timeout_ms_) {
        return;
    }

    // ...and a new ID has been heard repeatedly within the last `timeout`. Since silent_ms >= timeout, this
    // also guarantees it was heard *after* the active ID went silent (a neighbor heard earlier won't qualify).
    const auto candidate = this->parent_->latest_unknown();
    if (!candidate.has_value() || candidate->count < DRIFT_MIN_PACKETS || candidate->age_ms >= this->timeout_ms_) {
        return;
    }

    ESP_LOGW(TAG, "sensor_no %u silent for %" PRIu32 "s; drifting to new ID %u (configured stays %u until accepted)",
             this->active_, silent_ms / 1000, candidate->sensor_id, this->configured_);
    if (!this->is_drifted()) {
        this->drifted_at_ms_ = now;
    }
    this->set_ids_(this->configured_, candidate->sensor_id);
}

bool FineOffsetSensor::is_online_() const {
    if (this->parent_ == nullptr) {
        return false;
    }
    const auto age = this->parent_->last_seen_age_ms(this->active_);
    return age.has_value() && *age < this->timeout_ms_;
}

void FineOffsetSensor::publish_status_() {
    if (this->configured_sensor_no_number_ != nullptr) {
        this->configured_sensor_no_number_->publish_state(this->configured_);
    }
    if (this->active_sensor_no_sensor_ != nullptr) {
        this->active_sensor_no_sensor_->publish_state(this->active_);
    }
    if (this->drifted_binary_sensor_ != nullptr) {
        this->drifted_binary_sensor_->publish_state(this->is_drifted());
    }
    if (this->online_binary_sensor_ != nullptr) {
        this->online_binary_sensor_->publish_state(this->is_online_());
    }
}

void FineOffsetSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "FineOffset Sensor:");
    ESP_LOGCONFIG(TAG, "  Sensor No: configured=%u active=%u (YAML default %u)", this->configured_, this->active_,
                  this->default_sensor_no_);
    ESP_LOGCONFIG(TAG, "  Timeout: %" PRIu32 "s, Drift: %s", this->timeout_ms_ / 1000, YESNO(this->drift_enabled_));
    if (this->configured_sensor_no_number_ != nullptr) {
        LOG_NUMBER("  ", "Configured Sensor No", this->configured_sensor_no_number_);
    }
    LOG_SENSOR("  ", "Active Sensor No", this->active_sensor_no_sensor_);
    LOG_BINARY_SENSOR("  ", "Online", this->online_binary_sensor_);
    LOG_BINARY_SENSOR("  ", "Drifted", this->drifted_binary_sensor_);
    LOG_UPDATE_INTERVAL(this);
}

void FineOffsetSensor::update() {
    if (this->parent_ == nullptr) {
        ESP_LOGW(TAG, "No parent set for sensor %u", this->active_);
        return;
    }

    this->evaluate_drift_();

    if (this->online_binary_sensor_ != nullptr) {
        this->online_binary_sensor_->publish_state(this->is_online_());  // deduped by BinarySensor
    }

    auto state_guard = this->parent_->get_state_for_sensor_no(this->active_);
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
