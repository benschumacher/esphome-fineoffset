#pragma once

#include "../fineoffset.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace fineoffset {

    class FineOffsetSensor : public sensor::Sensor, public Component {
    public:
        void dump_config() override;
        void setup() override;

    protected:
    };

} // namespace fineoffset
} // namespace esphome
