#include "../fineoffset.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace fineoffset
{
    enum FineOffsetTextSensorType {
        FINEOFFSET_TYPE_LAST,
        FINEOFFSET_TYPE_LAST_BAD,
    };

    class FineOffsetTextSensor : public text_sensor::TextSensor, public Component {
    public:    
        void setup() override;
        void dump_config() override;
        void set_sensor_type(FineOffsetTextSensorType sensor_type) { this->sensor_type_ = sensor_type; }

    protected:
        FineOffsetTextSensorType sensor_type_;        
    };

} // namespace fineoffset    
} // esphome