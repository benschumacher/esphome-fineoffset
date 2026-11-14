#pragma once

#include "../fineoffset.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace fineoffset {

class FineOffsetSensor;

/// Optional Home Assistant-editable entity for a FineOffsetSensor's radio ID.
///
/// WH2 sensors pick a new random ID on every power-up (e.g. battery swap). This number lets the ID be
/// changed at runtime without a reflash. All state (preferences, known-ID bookkeeping) is owned by the
/// parent FineOffsetSensor; this class only forwards user input.
class FineOffsetSensorNoNumber : public number::Number, public Parented<FineOffsetSensor> {
   protected:
    void control(float value) override;
};

class FineOffsetSensor : public PollingComponent {
   public:
    void dump_config() override;
    void setup() override;
    void update() override;

    [[nodiscard]] uint8_t get_sensor_no() const { return this->sensor_no_; }

    /// Set the compile-time (YAML) sensor ID. Also used as the default when no runtime override is stored.
    void set_sensor_no(uint8_t sensor_no) {
        this->sensor_no_ = sensor_no;
        this->default_sensor_no_ = sensor_no;
    }

    /// Change the active sensor ID at runtime, update known-ID tracking and (optionally) persist it.
    /// @param sensor_no New radio ID; 0 is rejected because the decoder treats ID 0 as invalid.
    /// @param persist   Save to flash so the value survives reboots.
    void apply_sensor_no(uint8_t sensor_no, bool persist);

    void set_sensor_no_number(FineOffsetSensorNoNumber* number) { this->sensor_no_number_ = number; }

    void set_temperature_sensor(sensor::Sensor* temperature_sensor) { this->temperature_sensor_ = temperature_sensor; }
    void set_humidity_sensor(sensor::Sensor* humidity_sensor) { this->humidity_sensor_ = humidity_sensor; }

    void set_parent(FineOffsetComponent* parent) { this->parent_ = parent; }
    [[nodiscard]] FineOffsetComponent* get_parent() const { return this->parent_; }

   protected:
    uint8_t sensor_no_{0};          ///< Active radio ID
    uint8_t default_sensor_no_{0};  ///< ID from YAML; also keys the stored preference (see setup())
    FineOffsetComponent* parent_{nullptr};

    FineOffsetSensorNoNumber* sensor_no_number_{nullptr};
    ESPPreferenceObject pref_;

    sensor::Sensor* temperature_sensor_{nullptr};
    sensor::Sensor* humidity_sensor_{nullptr};
};

}  // namespace fineoffset
}  // namespace esphome
