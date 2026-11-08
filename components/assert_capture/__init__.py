"""Records why the firmware aborted (failed assert with file and line, abort()
and its caller, or a task stack overflow) and reports it after the reboot,
in the log and in an optional text sensor. ESP-IDF prints this only on the
serial console; this keeps it where Home Assistant can see it."""

import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["text_sensor"]

CONF_LAST_CRASH = "last_crash"

ns = cg.esphome_ns.namespace("assert_capture")
AssertCapture = ns.class_("AssertCapture", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AssertCapture),
        cv.Optional(CONF_LAST_CRASH): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:alert-octagon"
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for symbol in ("__assert_func", "abort", "vApplicationStackOverflowHook"):
        cg.add_build_flag(f"-Wl,--wrap={symbol}")
    if CONF_LAST_CRASH in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_CRASH])
        cg.add(var.set_text_sensor(sens))
