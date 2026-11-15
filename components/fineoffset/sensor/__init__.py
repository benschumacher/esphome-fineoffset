"""Add support for FineOffset WH2 sensors for temperature and humidity."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, button, number, sensor
from esphome.const import (
    CONF_ID,
    CONF_MODE,
    CONF_TIMEOUT,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

from .. import CONF_FINEOFFSET_ID, FINEOFFSET_CLIENT_SCHEMA, fineoffset_ns

DEPENDENCIES = ["fineoffset"]
AUTO_LOAD = ["binary_sensor", "button", "number"]

FineOffsetSensor = fineoffset_ns.class_("FineOffsetSensor", cg.PollingComponent)
FineOffsetConfiguredSensorNoNumber = fineoffset_ns.class_(
    "FineOffsetConfiguredSensorNoNumber",
    number.Number,
    cg.Parented.template(FineOffsetSensor),
)
FineOffsetAcceptDriftButton = fineoffset_ns.class_(
    "FineOffsetAcceptDriftButton",
    button.Button,
    cg.Parented.template(FineOffsetSensor),
)

CONF_SERIAL_NO = "sensor_no"
CONF_TEMPERATURE = "temperature"
CONF_HUMIDITY = "humidity"
# WH2 IDs change on battery swap; see FineOffsetSensor docs in fineoffset_sensor.h
CONF_DRIFT = "drift"
CONF_CONFIGURED_SENSOR_NO = "configured_sensor_no"
CONF_ACTIVE_SENSOR_NO = "active_sensor_no"
CONF_ONLINE = "online"
CONF_DRIFTED = "drifted"
CONF_ACCEPT_DRIFT = "accept_drift"


def _validate_drift(config):
    """Drift-only entities make no sense without drift enabled."""
    if not config[CONF_DRIFT]:
        for key in (CONF_DRIFTED, CONF_ACCEPT_DRIFT):
            if key in config:
                raise cv.Invalid(f"'{key}' requires 'drift: true'", path=[key])
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(FineOffsetSensor),
            cv.Required(CONF_SERIAL_NO): cv.int_range(1, 255),
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
            # Silence threshold for `online` and for drifting
            cv.Optional(
                CONF_TIMEOUT, default="30min"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DRIFT, default=False): cv.boolean,
            cv.Optional(CONF_CONFIGURED_SENSOR_NO): number.number_schema(
                FineOffsetConfiguredSensorNoNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:identifier",
            ).extend(
                {
                    cv.Optional(CONF_MODE, default="BOX"): cv.enum(
                        number.NUMBER_MODES, upper=True
                    )
                }
            ),
            cv.Optional(CONF_ACTIVE_SENSOR_NO): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:identifier",
            ),
            cv.Optional(CONF_ONLINE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_DRIFTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:swap-horizontal",
            ),
            cv.Optional(CONF_ACCEPT_DRIFT): button.button_schema(
                FineOffsetAcceptDriftButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:check-circle-outline",
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(FINEOFFSET_CLIENT_SCHEMA),
    _validate_drift,
)


async def to_code(config):
    """Convert configuration to code."""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    paren = await cg.get_variable(config[CONF_FINEOFFSET_ID])
    # Known-ID registration happens in FineOffsetSensor::setup() (IDs can change at runtime)
    cg.add(var.set_parent(paren))
    cg.add(var.set_sensor_no(config[CONF_SERIAL_NO]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    cg.add(var.set_drift(config[CONF_DRIFT]))

    # Plain entities: (config key, constructor, setter on FineOffsetSensor)
    for key, new_entity, setter in (
        (CONF_TEMPERATURE, sensor.new_sensor, "set_temperature_sensor"),
        (CONF_HUMIDITY, sensor.new_sensor, "set_humidity_sensor"),
        (CONF_ACTIVE_SENSOR_NO, sensor.new_sensor, "set_active_sensor_no_sensor"),
        (CONF_ONLINE, binary_sensor.new_binary_sensor, "set_online_binary_sensor"),
        (CONF_DRIFTED, binary_sensor.new_binary_sensor, "set_drifted_binary_sensor"),
    ):
        if entity_config := config.get(key):
            entity = await new_entity(entity_config)
            cg.add(getattr(var, setter)(entity))

    if number_config := config.get(CONF_CONFIGURED_SENSOR_NO):
        # IDs 1-255; 0 is treated as invalid by the decoder
        configured_number = await number.new_number(
            number_config, min_value=1, max_value=255, step=1
        )
        await cg.register_parented(configured_number, var)
        cg.add(var.set_configured_sensor_no_number(configured_number))

    if accept_config := config.get(CONF_ACCEPT_DRIFT):
        accept_button = await button.new_button(accept_config)
        await cg.register_parented(accept_button, var)
