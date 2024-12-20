import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID

from .. import fineoffset_ns, FINEOFFSET_CLIENT_SCHEMA, CONF_FINEOFFSET_ID

DEPENDENCIES = ["fineoffset"]

FineOffsetTextSensor = fineoffset_ns.class_("FineOffsetTextSensor", text_sensor.TextSensor, cg.Component)
FineOffsetTextSensorTypes = fineoffset_ns.enum("FineOffsetTextSensorTypes")

CONF_TYPE = "type"
TYPES = {
    "last": FineOffsetTextSensorTypes.FINEOFFSET_TYPE_LAST,
    "last_bad": FineOffsetTextSensorTypes.FINEOFFSET_TYPE_LAST_BAD,
}

CONFIG_SCHEMA = (
    text_sensor.text_sensor_schema(FineOffsetTextSensor).extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),            
        }
    )
    .extend(FINEOFFSET_CLIENT_SCHEMA)
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await text_sensor.register_text_sensor(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_sensor_type(config[CONF_TYPE]))

    paren = await cg.get_variable(config[CONF_FINEOFFSET_ID])
    cg.add(paren.register_text_sensor(var))