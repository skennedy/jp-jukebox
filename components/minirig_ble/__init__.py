import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, switch, esp32_ble_tracker
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_MAC_ADDRESS,
    UNIT_PERCENT,
    DEVICE_CLASS_BATTERY,
    STATE_CLASS_MEASUREMENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["esp32_ble_tracker"]
AUTO_LOAD = ["sensor", "switch"]
MULTI_CONF = True

minirig_ble_ns = cg.esphome_ns.namespace("minirig_ble")
MinirigBLEComponent = minirig_ble_ns.class_(
    "MinirigBLEComponent", cg.PollingComponent, esp32_ble_tracker.ESPBTDeviceListener
)

CONF_MINIRIGS = "minirigs"
CONF_BATTERY_SENSOR = "battery_sensor"
CONF_CHARGE_SWITCH = "charge_switch"
CONF_V_MIN = "v_min"
CONF_V_MAX = "v_max"

MINIRIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_MAC_ADDRESS): cv.mac_address,
        cv.Required(CONF_NAME): cv.string,
        cv.Optional(CONF_BATTERY_SENSOR): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_CHARGE_SWITCH): switch.switch_schema(
            MinirigBLEComponent  # placeholder, overridden per instance
        ),
        cv.Optional(CONF_V_MIN, default=10560): cv.int_range(min=8000, max=12000),
        cv.Optional(CONF_V_MAX, default=12582): cv.int_range(min=10000, max=15000),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(MinirigBLEComponent),
        cv.Required(CONF_MINIRIGS): cv.ensure_list(MINIRIG_SCHEMA),
    }
).extend(cv.polling_component_schema("5min"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await esp32_ble_tracker.register_ble_device(var, config)

    for minirig in config[CONF_MINIRIGS]:
        mac = minirig[CONF_MAC_ADDRESS]
        name = minirig[CONF_NAME]
        v_min = minirig[CONF_V_MIN]
        v_max = minirig[CONF_V_MAX]

        sensor_var = None
        if CONF_BATTERY_SENSOR in minirig:
            sensor_var = await sensor.new_sensor(minirig[CONF_BATTERY_SENSOR])

        cg.add(
            var.add_minirig(
                str(mac),
                name,
                sensor_var if sensor_var else cg.nullptr,
                v_min,
                v_max,
            )
        )
