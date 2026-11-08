"""HTI-301 (InfiRay T3-317-13) USB thermal camera, 384x288 at 25 fps.

A radiometric source (temperatures, in radiometric mode) and an image source
(the camera's own processed image, in processed mode). Needs `usb_host:` on an
ESP32-P4 (USB 2.0 high speed)."""

import esphome.codegen as cg
from esphome.components import binary_sensor, text_sensor, usb_host
from esphome.components.esp32 import VARIANT_ESP32P4, only_on_variant
from esphome.components.video_controls import (
    button_control,
    new_button_control,
    new_number_control,
    new_select_control,
    number_control,
    select_control,
)
from esphome.components.video_core import ImageSource, RadiometricSource
import esphome.config_validation as cv
from esphome.const import (
    CONF_DISTANCE,
    CONF_HUMIDITY,
    CONF_MODE,
    DEVICE_CLASS_RUNNING,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["usb_host", "esp32"]
AUTO_LOAD = ["video_core", "video_controls", "binary_sensor", "text_sensor"]
MULTI_CONF = False  # every HTI-301 has the same USB ID

CONF_RANGE = "range"
CONF_PALETTE = "palette"
CONF_EMISSIVITY = "emissivity"
CONF_REFLECTED_TEMPERATURE = "reflected_temperature"
CONF_AMBIENT_TEMPERATURE = "ambient_temperature"
CONF_TRANSFERS = "transfers"
CONF_PACKETS_PER_TRANSFER = "packets_per_transfer"
CONF_CONTROLS = "controls"
CONF_CALIBRATE = "calibrate"
CONF_RESTART_USB = "restart_usb"
CONF_STREAMING = "streaming"
CONF_USB_STATUS = "usb_status"
CONF_LOG_TIMING = "log_timing"

VID = 0x1514
PID = 0x0001

MODES = ["radiometric", "processed"]
MODE_LABELS = ["Radiometric", "Processed"]
RANGES = ["normal", "high"]
RANGE_LABELS = ["Normal (-20 to 120 °C)", "High (-20 to 450 °C)"]
PALETTES = ["white_hot", "black_hot", "ironbow", "lava", "rainbow", "iron_gray", "red_hot", "rainbow_2"]
PALETTE_LABELS = ["White hot", "Black hot", "Ironbow", "Lava", "Rainbow", "Iron gray", "Red hot", "Rainbow 2"]

hti301_ns = cg.esphome_ns.namespace("hti301")
HTI301 = hti301_ns.class_("HTI301", usb_host.USBClient, RadiometricSource, ImageSource)
Mode = hti301_ns.enum("Mode", is_class=True)
Range = hti301_ns.enum("Range", is_class=True)

CONFIG_SCHEMA = cv.All(
    usb_host.usb_device_schema(HTI301, VID, PID).extend(
        {
            # radiometric: 14-bit raw values, converted here to temperatures.
            # processed: the camera's own image (palette applied on the camera).
            cv.Optional(CONF_MODE, default="radiometric"): cv.one_of(*MODES, lower=True),
            cv.Optional(CONF_RANGE, default="normal"): cv.one_of(*RANGES, lower=True),
            cv.Optional(CONF_PALETTE, default="white_hot"): cv.one_of(*PALETTES, lower=True),
            # Scene parameters for the temperature conversion. Left out, the
            # values stored in the camera are used.
            cv.Optional(CONF_EMISSIVITY, default=0.95): cv.float_range(min=0.01, max=1.0),
            cv.Optional(CONF_REFLECTED_TEMPERATURE): cv.float_range(min=-40, max=500),  # degrees C
            cv.Optional(CONF_AMBIENT_TEMPERATURE): cv.float_range(min=-40, max=100),  # degrees C
            cv.Optional(CONF_HUMIDITY): cv.percentage,
            cv.Optional(CONF_DISTANCE): cv.float_range(min=0, max=1000),  # metres
            # Isochronous transfers kept queued, and 125 us packets in each:
            # 8 x 16 buffers 16 ms of video (100 KB of internal RAM).
            cv.Optional(CONF_TRANSFERS, default=8): cv.int_range(min=2, max=32),
            cv.Optional(CONF_PACKETS_PER_TRANSFER, default=16): cv.int_range(min=1, max=61),
            cv.Optional(CONF_CONTROLS, default={}): cv.Schema(
                {
                    cv.Optional(CONF_MODE): select_control(),
                    cv.Optional(CONF_RANGE): select_control(),
                    cv.Optional(CONF_PALETTE): select_control(),
                    cv.Optional(CONF_EMISSIVITY): number_control(),
                    cv.Optional(CONF_CALIBRATE): button_control(),
                    # Powers the USB port off and on: everything on it re-enumerates.
                    cv.Optional(CONF_RESTART_USB): button_control(),
                }
            ),
            # On while frames arrive from the camera.
            cv.Optional(CONF_STREAMING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_RUNNING, entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            # While no camera is connected: the USB devices the host sees and
            # the state of every hub port; then the stream's state.
            cv.Optional(CONF_USB_STATUS): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:usb"
            ),
            cv.Optional(CONF_LOG_TIMING, default=False): cv.boolean,
        }
    ),
    only_on_variant(supported=[VARIANT_ESP32P4], msg_prefix="The HTI-301 (USB 2.0 high speed)"),
)


async def to_code(config):
    var = await usb_host.register_usb_client(config)
    mode = MODES.index(config[CONF_MODE])
    rng = RANGES.index(config[CONF_RANGE])
    palette = PALETTES.index(config[CONF_PALETTE])
    cg.add(var.set_mode(Mode.enum(config[CONF_MODE].upper())))
    cg.add(var.set_range(Range.enum(config[CONF_RANGE].upper())))
    cg.add(var.set_palette(palette))
    cg.add(var.set_emissivity(config[CONF_EMISSIVITY]))
    if CONF_REFLECTED_TEMPERATURE in config:
        cg.add(var.set_reflected_temperature(config[CONF_REFLECTED_TEMPERATURE]))
    if CONF_AMBIENT_TEMPERATURE in config:
        cg.add(var.set_ambient_temperature(config[CONF_AMBIENT_TEMPERATURE]))
    if CONF_HUMIDITY in config:
        cg.add(var.set_humidity(config[CONF_HUMIDITY]))
    if CONF_DISTANCE in config:
        cg.add(var.set_distance(config[CONF_DISTANCE]))
    cg.add(var.set_transfers(config[CONF_TRANSFERS]))
    cg.add(var.set_packets_per_transfer(config[CONF_PACKETS_PER_TRANSFER]))
    cg.add(var.set_log_timing(config[CONF_LOG_TIMING]))
    if CONF_STREAMING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_STREAMING])
        cg.add(var.set_streaming_sensor(sens))
    if CONF_USB_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_USB_STATUS])
        cg.add(var.set_status_sensor(sens))

    controls = config[CONF_CONTROLS]
    if CONF_MODE in controls:
        await new_select_control(
            controls[CONF_MODE], var, "set_mode", MODE_LABELS, mode, convert="static_cast<esphome::hti301::Mode>(v)"
        )
    if CONF_RANGE in controls:
        await new_select_control(
            controls[CONF_RANGE], var, "set_range", RANGE_LABELS, rng, convert="static_cast<esphome::hti301::Range>(v)"
        )
    if CONF_PALETTE in controls:
        await new_select_control(controls[CONF_PALETTE], var, "set_palette", PALETTE_LABELS, palette)
    if CONF_EMISSIVITY in controls:
        await new_number_control(
            controls[CONF_EMISSIVITY], var, "set_emissivity", config[CONF_EMISSIVITY], 0.01, 1.0, 0.01
        )
    if CONF_CALIBRATE in controls:
        await new_button_control(controls[CONF_CALIBRATE], var, "calibrate")
    if CONF_RESTART_USB in controls:
        await new_button_control(controls[CONF_RESTART_USB], var, "restart_usb")
