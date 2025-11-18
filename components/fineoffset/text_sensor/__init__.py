"""Adds text sensors for FineOffset Hub which are mostly used for diagnostic purposes."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

from .. import fineoffset_ns, FINEOFFSET_CLIENT_SCHEMA, CONF_FINEOFFSET_ID

DEPENDENCIES = ["fineoffset"]

FineOffsetTextSensor = fineoffset_ns.class_(
    "FineOffsetTextSensor", text_sensor.TextSensor, cg.PollingComponent
)
FineOffsetTextSensorType = fineoffset_ns.enum("FineOffsetTextSensorType")

CONF_TYPE = "type"
TYPES = {
    "last": FineOffsetTextSensorType.Last,
    "last_bad": FineOffsetTextSensorType.LastBad,
    "unknown": FineOffsetTextSensorType.Unknown,
}

CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(
        FineOffsetTextSensor,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        icon="mdi:information-outline",
    )
    .extend(
        {
            cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(FINEOFFSET_CLIENT_SCHEMA)
)


async def to_code(config):
    """Convert configuration to code."""
    var = cg.new_Pvariable(config[CONF_ID])
    await text_sensor.register_text_sensor(var, config)
    await cg.register_component(var, config)
    cg.add(var.set_sensor_type(config[CONF_TYPE]))

    paren = await cg.get_variable(config[CONF_FINEOFFSET_ID])
    cg.add(var.set_parent(paren))
