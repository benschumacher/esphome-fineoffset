#pragma once

#include "../fineoffset.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace fineoffset {

class FineOffsetSensor;

/// HA-editable *configured* radio ID. Setting it also makes it the active ID (clears any drift).
class FineOffsetSensorNoNumber : public number::Number, public Parented<FineOffsetSensor> {
   protected:
    void control(float value) override;
};

/// Makes the drifted-to ID the new configured ID.
class FineOffsetAcceptDriftButton : public button::Button, public Parented<FineOffsetSensor> {
   protected:
    void press_action() override;
};

/// One WH2 temperature/humidity sensor.
///
/// WH2 sensors pick a new random ID on every power-up (battery swap). Two IDs are tracked:
///  - configured: from YAML, or overridden via the number entity / accept button. The source of truth.
///  - active:     the ID readings are taken from. Normally == configured.
///
/// With `drift: true`, if the active ID is silent for `timeout` and a new unknown ID has been heard
/// (at least twice, within that window), the sensor follows ("drifts" to) the new ID. If the configured ID
/// is heard again afterwards, it snaps back automatically, so a wrong drift self-corrects. Drift is
/// surfaced via the `drifted` binary sensor until accepted or overridden. Both IDs persist across reboots.
class FineOffsetSensor : public PollingComponent {
   public:
    void dump_config() override;
    void setup() override;
    void update() override;

    // --- Codegen setters ---
    /// YAML sensor_no. Also salts the preference key, so changing it in YAML discards stored runtime IDs.
    void set_sensor_no(uint8_t sensor_no) {
        this->default_sensor_no_ = sensor_no;
        this->configured_ = sensor_no;
        this->active_ = sensor_no;
    }
    void set_timeout(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }
    void set_drift(bool drift) { this->drift_enabled_ = drift; }
    void set_parent(FineOffsetComponent* parent) { this->parent_ = parent; }
    void set_temperature_sensor(sensor::Sensor* s) { this->temperature_sensor_ = s; }
    void set_humidity_sensor(sensor::Sensor* s) { this->humidity_sensor_ = s; }
    void set_sensor_no_number(FineOffsetSensorNoNumber* n) { this->sensor_no_number_ = n; }
    void set_active_sensor_no_sensor(sensor::Sensor* s) { this->active_sensor_no_sensor_ = s; }
    void set_online_binary_sensor(binary_sensor::BinarySensor* b) { this->online_binary_sensor_ = b; }
    void set_drifted_binary_sensor(binary_sensor::BinarySensor* b) { this->drifted_binary_sensor_ = b; }

    // --- Runtime API ---
    /// User override: becomes both configured and active ID (clears drift). Rejects 0.
    void set_configured_sensor_no(uint8_t sensor_no);
    /// Promote the drifted-to active ID to configured. No-op when not drifted.
    void accept_drift();

    [[nodiscard]] uint8_t get_sensor_no() const { return this->active_; }
    [[nodiscard]] uint8_t get_configured_sensor_no() const { return this->configured_; }
    [[nodiscard]] bool is_drifted() const { return this->active_ != this->configured_; }
    [[nodiscard]] FineOffsetComponent* get_parent() const { return this->parent_; }

   protected:
    /// Persisted state. Keep trivially copyable; bump PREF_VERSION if the layout changes.
    struct SavedIds {
        uint8_t configured;
        uint8_t active;
    };
    static constexpr uint32_t PREF_VERSION = 1;

    /// Swap the tracked IDs, fixing known-ID registration, persisting (if changed) and publishing entities.
    void set_ids_(uint8_t configured, uint8_t active);
    void evaluate_drift_();
    [[nodiscard]] bool is_online_() const;
    void publish_status_();

    FineOffsetComponent* parent_{nullptr};

    uint8_t default_sensor_no_{0};
    uint8_t configured_{0};
    uint8_t active_{0};
    bool ids_registered_{false};

    uint32_t timeout_ms_{30 * 60 * 1000};
    bool drift_enabled_{false};
    uint32_t drifted_at_ms_{0};  ///< millis() when drift began (or boot, if restored drifted)

    ESPPreferenceObject pref_;
    SavedIds saved_{0, 0};

    sensor::Sensor* temperature_sensor_{nullptr};
    sensor::Sensor* humidity_sensor_{nullptr};
    FineOffsetSensorNoNumber* sensor_no_number_{nullptr};
    sensor::Sensor* active_sensor_no_sensor_{nullptr};
    binary_sensor::BinarySensor* online_binary_sensor_{nullptr};
    binary_sensor::BinarySensor* drifted_binary_sensor_{nullptr};
};

}  // namespace fineoffset
}  // namespace esphome
