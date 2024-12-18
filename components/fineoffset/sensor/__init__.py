import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import CONF_ID, UNIT_CELSIUS, UNIT_PERCENT, ICON_THERMOMETER, ICON_WATER_PERCENT, ICON_BATTERY

from .. import fineoffset_ns, FINEOFFSET_CLIENT_SCHEMA, CONF_FINEOFFSET_ID

DEPENDENCIES = ["fineoffset"]

FineOffsetSensor = fineoffset_ns.class_("FineOffsetSensor", sensor.Sensor, cg.Component)

CONF_SERIAL_NO = "serial_no"
CONF_TEMPERATURE = "temperature"
CONF_HUMIDITY = "humidity"
CONF_BATTERY = "battery"

CONFIG_SCHEMA = (
    sensor.sensor_schema(FineOffsetSensor).extend(
        {
            cv.Required(CONF_SERIAL_NO): cv.int_range(0, 255),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER, accuracy_decimals=1,
            ),            
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT, icon=ICON_WATER_PERCENT, accuracy_decimals=1,
            ),            
            cv.Optional(CONF_BATTERY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT, icon=ICON_BATTERY, accuracy_decimals=1,
            ),            
        }
    )
    .extend(FINEOFFSET_CLIENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_FINEOFFSET_ID])
    sensor_no = config[CONF_SERIAL_NO];
    cg.add(paren.register_sensor(sensor_no, var))
