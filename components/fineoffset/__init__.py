# import esphome.codegen as cg
# import esphome.config_validation as cv
# from esphome.const import CONF_ID

# empty_component_ns = cg.esphome_ns.namespace("empty_component")
# EmptyComponent = empty_component_ns.class_("EmptyComponent", cg.Component)

# CONFIG_SCHEMA = cv.Schema(
#     {
#         cv.GenerateID(): cv.declare_id(EmptyComponent),
#     }
# ).extend(cv.COMPONENT_SCHEMA)


# async def to_code(config):
#     var = cg.new_Pvariable(config[CONF_ID])
#     await cg.register_component(var, config)


import esphome.codegen as cg
import esphome.config_validation as cv

from esphome import pins
from esphome.const import CONF_ID, CONF_NAME, CONF_PIN

MULTI_CONF = True
AUTO_LOAD = ["sensor"]

CONF_FINEOFFSET_ID = "fineoffset_id"

fineoffset_ns = cg.esphome_ns.namespace('fineoffset')
FineOffset = fineoffset_ns.class_("FineOffsetComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(FineOffset),
        cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,
        cv.Optional(CONF_NAME): cv.string,
    }
).extend(cv.COMPONENT_SCHEMA)

FINEOFFSET_CLIENT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_FINEOFFSET_ID): cv.use_id(FineOffset)
    }
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
