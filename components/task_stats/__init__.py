"""Logs each FreeRTOS task's share of the CPU at every update, and the time
each core spent idle; optionally publishes the CPU load as a sensor. Turns on
FreeRTOS run-time statistics, which read a timer at every task switch."""

import esphome.codegen as cg
from esphome.components import sensor
from esphome.components.esp32 import add_idf_sdkconfig_option
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
)

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["sensor"]

CONF_TOP = "top"
CONF_LOAD = "load"

ns = cg.esphome_ns.namespace("task_stats")
TaskStats = ns.class_("TaskStats", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TaskStats),
        # Busiest tasks listed in each report.
        cv.Optional(CONF_TOP, default=12): cv.int_range(min=1, max=32),
        # CPU in use over the last update interval, % of all cores.
        cv.Optional(CONF_LOAD): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:cpu-64-bit",
        ),
    }
).extend(cv.polling_component_schema("10s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_top(config[CONF_TOP]))
    add_idf_sdkconfig_option("CONFIG_FREERTOS_USE_TRACE_FACILITY", True)
    add_idf_sdkconfig_option("CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS", True)
    if CONF_LOAD in config:
        sens = await sensor.new_sensor(config[CONF_LOAD])
        cg.add(var.set_load_sensor(sens))
