"""Seek Thermal Compact Pro USB thermal camera (Android model, including Fast
Frame), 320x240. A radiometric source: temperatures from the camera's own
calibration (its factory settings and the tables it sends after each start,
processed as Seek's own app does), with a gain and offset to correct them.
Needs `usb_host:` on an ESP32-P4 (USB 2.0 high speed)."""

import esphome.codegen as cg
from esphome.components import binary_sensor, text_sensor, usb_host
from esphome.components.esp32 import VARIANT_ESP32P4, only_on_variant
from esphome.components.video_controls import (
    button_control,
    new_button_control,
    new_number_control,
    number_control,
)
from esphome.components.video_core import DEVICE_ID_SCHEMA, RadiometricSource, inherit_device_id
import esphome.config_validation as cv
from esphome.const import CONF_OFFSET, DEVICE_CLASS_RUNNING, ENTITY_CATEGORY_DIAGNOSTIC

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["usb_host", "esp32"]
AUTO_LOAD = ["video_core", "video_controls", "binary_sensor", "text_sensor"]
MULTI_CONF = False  # every Compact Pro has the same USB ID

CONF_GAIN = "gain"
CONF_TRANSFERS = "transfers"
CONF_CONTROLS = "controls"
CONF_SHUTTER = "shutter"
CONF_STREAMING = "streaming"
CONF_STATUS = "status"
CONF_LOG_TIMING = "log_timing"
CONF_BAD_COLUMNS = "bad_columns"
CONF_SCENE_FILTER = "scene_filter"

VID = 0x289D
PID = 0x0011  # Compact Pro, Fast Frame too

seek_ns = cg.esphome_ns.namespace("seek_compact_pro")
SeekCompactPro = seek_ns.class_("SeekCompactPro", usb_host.USBClient, RadiometricSource)

CONFIG_SCHEMA = cv.All(
    inherit_device_id("controls.*", CONF_STREAMING, CONF_STATUS),
    usb_host.usb_device_schema(SeekCompactPro, VID, PID).extend(
        {
            **DEVICE_ID_SCHEMA,
            # Temperatures: the camera's approximation times gain, plus offset
            # (degrees C), to match a reference.
            cv.Optional(CONF_GAIN, default=1.0): cv.float_range(min=0.1, max=10.0),
            cv.Optional(CONF_OFFSET, default=0.0): cv.float_range(min=-50.0, max=50.0),
            # Bulk transfers kept queued (13.8 KB of internal RAM each).
            cv.Optional(CONF_TRANSFERS, default=3): cv.int_range(min=1, max=8),
            # Picture columns (0 at the left) filled from their neighbours, like
            # dead pixels: a column that flickers. The camera's own table marks
            # single dead pixels; it does not mark such a column.
            # Seek's default picture filter, as its app runs it: single pixels far
            # off their 3x3 median take the median, offsets are learned from
            # scene motion, and small frame-to-frame changes are smoothed.
            cv.Optional(CONF_SCENE_FILTER, default=True): cv.boolean,
            cv.Optional(CONF_BAD_COLUMNS, default=[]): cv.All(
                cv.ensure_list(cv.int_range(min=0, max=319)), cv.Length(max=8)
            ),
            cv.Optional(CONF_CONTROLS, default={}): cv.Schema(
                {
                    cv.Optional(CONF_GAIN): number_control(),
                    # A temperature difference: no unit conversion in Home Assistant.
                    cv.Optional(CONF_OFFSET): number_control(unit_of_measurement="°C"),
                    # Closes the shutter for a new flat-field frame.
                    cv.Optional(CONF_SHUTTER): button_control(),
                }
            ),
            cv.Optional(CONF_STREAMING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_RUNNING, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:usb"
            ),
            cv.Optional(CONF_LOG_TIMING, default=False): cv.boolean,
        }
    ),
    only_on_variant(supported=[VARIANT_ESP32P4], msg_prefix="The Seek Compact Pro (USB 2.0 high speed)"),
)


async def to_code(config):
    var = await usb_host.register_usb_client(config)
    cg.add(var.set_gain(config[CONF_GAIN]))
    cg.add(var.set_offset(config[CONF_OFFSET]))
    cg.add(var.set_transfers(config[CONF_TRANSFERS]))
    cg.add(var.set_log_timing(config[CONF_LOG_TIMING]))
    cg.add(var.set_scene_filter(config[CONF_SCENE_FILTER]))
    for x in config[CONF_BAD_COLUMNS]:
        cg.add(var.add_bad_column(x))
    if CONF_STREAMING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_STREAMING])
        cg.add(var.set_streaming_sensor(sens))
    if CONF_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATUS])
        cg.add(var.set_status_sensor(sens))

    c = config[CONF_CONTROLS]
    if CONF_GAIN in c:
        await new_number_control(c[CONF_GAIN], var, "set_gain", config[CONF_GAIN], 0.1, 10.0, 0.01)
    if CONF_OFFSET in c:
        await new_number_control(c[CONF_OFFSET], var, "set_offset", config[CONF_OFFSET], -50.0, 50.0, 0.1)
    if CONF_SHUTTER in c:
        await new_button_control(c[CONF_SHUTTER], var, "trigger_shutter")
