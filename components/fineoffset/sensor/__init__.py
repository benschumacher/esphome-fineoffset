"""Add support for FineOffset WH2 sensors for temperature and humidity."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number, sensor
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    STATE_CLASS_MEASUREMENT,
    CONF_ID,
    CONF_MODE,
    ENTITY_CATEGORY_CONFIG,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

from .. import fineoffset_ns, FINEOFFSET_CLIENT_SCHEMA, CONF_FINEOFFSET_ID

DEPENDENCIES = ["fineoffset"]
AUTO_LOAD = ["number"]

FineOffsetSensor = fineoffset_ns.class_("FineOffsetSensor", cg.PollingComponent)
FineOffsetSensorNoNumber = fineoffset_ns.class_(
    "FineOffsetSensorNoNumber", number.Number, cg.Parented.template(FineOffsetSensor)
)

CONF_SERIAL_NO = "sensor_no"
CONF_TEMPERATURE = "temperature"
CONF_HUMIDITY = "humidity"
# Optional HA-editable override for sensor_no (WH2 IDs change on battery swap)
CONF_SENSOR_NO_NUMBER = "sensor_no_number"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FineOffsetSensor),
            cv.Required(CONF_SERIAL_NO): cv.int_range(0, 255),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_SENSOR_NO_NUMBER): number.number_schema(
                FineOffsetSensorNoNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:identifier",
            ).extend(
                {
                    cv.Optional(CONF_MODE, default="BOX"): cv.enum(
                        number.NUMBER_MODES, upper=True
                    )
                }
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(FINEOFFSET_CLIENT_SCHEMA)
)


async def to_code(config):
    """Convert configuration to code."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_FINEOFFSET_ID])
    sensor_no = config[CONF_SERIAL_NO]

    # Set up parent relationship and register as known sensor
    cg.add(var.set_sensor_no(sensor_no))
    cg.add(var.set_parent(paren))
    cg.add(paren.register_known_sensor(sensor_no))

    if temperature_sensor_config := config.get(CONF_TEMPERATURE):
        temperature_sensor = cg.new_Pvariable(temperature_sensor_config[CONF_ID])
        await sensor.register_sensor(temperature_sensor, temperature_sensor_config)
        cg.add(var.set_temperature_sensor(temperature_sensor))

    if humidity_sensor_config := config.get(CONF_HUMIDITY):
        humidity_sensor = cg.new_Pvariable(humidity_sensor_config[CONF_ID])
        await sensor.register_sensor(humidity_sensor, humidity_sensor_config)
        cg.add(var.set_humidity_sensor(humidity_sensor))

    if number_config := config.get(CONF_SENSOR_NO_NUMBER):
        # IDs 1-255; 0 is treated as invalid by the decoder
        sensor_no_number = await number.new_number(
            number_config, min_value=1, max_value=255, step=1
        )
        await cg.register_parented(sensor_no_number, var)
        cg.add(var.set_sensor_no_number(sensor_no_number))
