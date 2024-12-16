import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import UNIT_CELCIUS, ICON_THERMOMETER
from . import HUB_CHILD_SCHEMA, CONF_EMPTY_RADIO_ID

DEPENDENCIES = ["fineoffset"]

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS, icon=ICON_THERMOMETER
    )
    .extend(HUB_CHILD_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_EMPTY_SENSOR_HUB_ID])
    var = await sensor.new_sensor(config)

    cg.add(paren.register_sensor(var))
