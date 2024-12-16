import esphome.codegen as cg
import esphome.config_validation as cv

from esphome import pins
from esphome.components.gpio import gpio_ns
from esphome.const import CONF_ID, CONF_PIN

GPIOFineoffsetRadio = gpio_ns.class_("GPIOFineoffsetRadio", FineoffsetRadio, cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(GPIOFineoffsetRadio),
        cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
