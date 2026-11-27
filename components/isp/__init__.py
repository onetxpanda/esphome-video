"""Image signal processing for a radiometric source: tone mapping onto a
palette, mirroring, upscaling and output pixel format. Output is an image
source (for an overlay, encoder or camera)."""

import esphome.codegen as cg
from esphome.components import text_sensor
from esphome.components.esp32 import get_esp32_variant, include_builtin_idf_component
from esphome.components.esp32.const import VARIANT_ESP32P4
from esphome.components.video_controls import (
    button_control,
    new_button_control,
    new_number_control,
    new_select_control,
    new_switch_control,
    number_control,
    select_control,
    switch_control,
)
from esphome.components.video_core import (
    DEVICE_ID_SCHEMA,
    ImageSource,
    PALETTE_KEYS,
    PaletteTarget,
    RadiometricSink,
    RadiometricSource,
    inherit_device_id,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_FORMAT,
    CONF_ID,
    CONF_SOURCE,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
)
from esphome.helpers import fnv1_hash

CODEOWNERS = ["@onetxpanda"]
MULTI_CONF = True  # e.g. one chain per camera
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["video_core", "video_controls", "text_sensor"]

CONF_SCALE = "scale"
CONF_FIT = "fit"
CONF_ROTATE = "rotate"
CONF_PALETTE = "palette"
CONF_RANGE_MIN = "range_min"
CONF_RANGE_MAX = "range_max"
CONF_MIRROR_X = "mirror_x"
CONF_MIRROR_Y = "mirror_y"
CONF_UPSCALE = "upscale"
CONF_SCALER = "scaler"
CONF_CONTROLS = "controls"
CONF_LOG_TIMING = "log_timing"
CONF_EQUALIZE = "equalize"
CONF_DETAIL = "detail"
CONF_CLEAR_SPOTS = "clear_spots"
CONF_SPOTS = "spots"

FORMATS = {"RGB565_LE": "RGB565_LE", "RGB565_BE": "RGB565_BE", "RGB888": "RGB888", "GRAY8": "GRAY8"}
# Older names, kept working.
PALETTE_ALIASES = {"iron": "ironbow", "gray": "white_hot", "grey": "white_hot"}
UPSCALES = ["nearest", "bilinear", "bicubic"]
UPSCALE_LABELS = ["Nearest", "Bilinear", "Bicubic"]
SCALERS = ["cpu", "ppa"]
ROTATIONS = [0, 90, 180, 270]
ROTATION_LABELS = ["0°", "90°", "180°", "270°"]
SCALER_LABELS = ["CPU", "PPA"]

isp_ns = cg.esphome_ns.namespace("isp")
Isp = isp_ns.class_("Isp", cg.Component, RadiometricSink, ImageSource, PaletteTarget)
Palette = isp_ns.enum("Palette", is_class=True)
Upscale = isp_ns.enum("Upscale", is_class=True)
Scaler = isp_ns.enum("Scaler", is_class=True)
ImageFormat = cg.esphome_ns.namespace("video").enum("ImageFormat", is_class=True)


def _validate(config):
    if config[CONF_RANGE_MIN] >= config[CONF_RANGE_MAX]:
        raise cv.Invalid(f"{CONF_RANGE_MIN} must be below {CONF_RANGE_MAX}")
    return config



def _palette(value):
    value = cv.string_strict(value).lower().replace(" ", "_")
    value = PALETTE_ALIASES.get(value, value)
    if value not in PALETTE_KEYS:
        raise cv.Invalid(f"palette must be one of {', '.join(PALETTE_KEYS)}")
    return value


CONFIG_SCHEMA = cv.All(
    inherit_device_id("controls.*", "spots"),
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Isp),
            **DEVICE_ID_SCHEMA,
            cv.Required(CONF_SOURCE): cv.use_id(RadiometricSource),
            # Output size is the source size times this.
            cv.Optional(CONF_SCALE, default=1): cv.int_range(min=1, max=20),
            # Instead of `scale`: the largest whole scale at which the source
            # (after rotation) fits within this size, worked out for each
            # frame, so one ISP serves sources of different sizes (e.g.
            # 640x480: a 32x24 source 20x, a 160x120 source 4x).
            cv.Optional(CONF_FIT): cv.dimensions,
            # Clockwise, after mirroring.
            cv.Optional(CONF_ROTATE, default=0): cv.one_of(*ROTATIONS, int=True),
            # RGB565_LE is what the ESP32-P4's H.264 encoder and PPA take.
            cv.Optional(CONF_FORMAT, default="RGB565_LE"): cv.enum(FORMATS, upper=True),
            cv.Optional(CONF_PALETTE, default="ironbow"): _palette,
            # Values mapped to the first and last palette colour.
            cv.Optional(CONF_RANGE_MIN, default=15.0): cv.float_,
            cv.Optional(CONF_RANGE_MAX, default=40.0): cv.float_,
            cv.Optional(CONF_MIRROR_X, default=False): cv.boolean,
            cv.Optional(CONF_MIRROR_Y, default=False): cv.boolean,
            # CPU scaling method. The PPA's method is fixed by the hardware.
            cv.Optional(CONF_UPSCALE, default="nearest"): cv.one_of(*UPSCALES, lower=True),
            cv.Optional(CONF_SCALER, default="cpu"): cv.one_of(*SCALERS, lower=True),
            # Percent of histogram equalization mixed into the linear colour
            # map: more colours for the temperatures the scene contains.
            cv.Optional(CONF_EQUALIZE, default=0): cv.float_range(min=0, max=100),
            # Amplification of local detail against a smoothed copy of the
            # frame (sharper edges); 0 = off.
            cv.Optional(CONF_DETAIL, default=0): cv.float_range(min=0, max=4),
            cv.Optional(CONF_CONTROLS, default={}): cv.Schema(
                {
                    # Temperatures in degrees C, so Home Assistant shows and
                    # takes them in its own unit
                    cv.Optional(CONF_RANGE_MIN): number_control(
                        unit_of_measurement=UNIT_CELSIUS, device_class=DEVICE_CLASS_TEMPERATURE
                    ),
                    cv.Optional(CONF_RANGE_MAX): number_control(
                        unit_of_measurement=UNIT_CELSIUS, device_class=DEVICE_CLASS_TEMPERATURE
                    ),
                    cv.Optional(CONF_MIRROR_X): switch_control(),
                    cv.Optional(CONF_MIRROR_Y): switch_control(),
                    cv.Optional(CONF_ROTATE): select_control(),
                    cv.Optional(CONF_UPSCALE): select_control(),
                    cv.Optional(CONF_SCALER): select_control(),
                    cv.Optional(CONF_EQUALIZE): number_control(),
                    cv.Optional(CONF_DETAIL): number_control(),
                    # Removes every spot (spots are placed with toggle_spot,
                    # e.g. from an API action).
                    cv.Optional(CONF_CLEAR_SPOTS): button_control(),
                }
            ),
            # Text sensor listing the spots ("x,y,material,color;..."), e.g.
            # for a dashboard card that places them.
            cv.Optional(CONF_SPOTS): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_LOG_TIMING, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    source = await cg.get_variable(config[CONF_SOURCE])
    cg.add(source.add_radiometric_sink(var))

    if get_esp32_variant() == VARIANT_ESP32P4:
        cg.add_define("USE_ISP_PPA")
        include_builtin_idf_component("esp_driver_ppa")

    cg.add(var.set_scale(config[CONF_SCALE]))
    if CONF_FIT in config:
        cg.add(var.set_fit(config[CONF_FIT][0], config[CONF_FIT][1]))
    cg.add(var.set_rotate(cg.RawExpression(f"esphome::isp::Rotation::R{config[CONF_ROTATE]}")))
    cg.add(var.set_format(ImageFormat.enum(config[CONF_FORMAT])))
    cg.add(var.set_palette(Palette.enum(config[CONF_PALETTE].upper())))
    cg.add(var.set_range_min(config[CONF_RANGE_MIN]))
    cg.add(var.set_range_max(config[CONF_RANGE_MAX]))
    cg.add(var.set_mirror_x(config[CONF_MIRROR_X]))
    cg.add(var.set_mirror_y(config[CONF_MIRROR_Y]))
    cg.add(var.set_upscale(Upscale.enum(config[CONF_UPSCALE].upper())))
    cg.add(var.set_scaler(Scaler.enum(config[CONF_SCALER].upper())))
    cg.add(var.set_log_timing(config[CONF_LOG_TIMING]))
    cg.add(var.set_equalize(config[CONF_EQUALIZE]))
    cg.add(var.set_detail(config[CONF_DETAIL]))

    c = config[CONF_CONTROLS]
    for key in (CONF_RANGE_MIN, CONF_RANGE_MAX):
        if key in c:
            await new_number_control(c[key], var, f"set_{key}", config[key], -40, 300, 0.5)
    for key in (CONF_MIRROR_X, CONF_MIRROR_Y):
        if key in c:
            await new_switch_control(c[key], var, f"set_{key}", config[key])
    if CONF_ROTATE in c:
        await new_select_control(
            c[CONF_ROTATE], var, "set_rotate", ROTATION_LABELS, ROTATIONS.index(config[CONF_ROTATE]),
            convert="static_cast<esphome::isp::Rotation>(v)",
        )
    if CONF_UPSCALE in c:
        await new_select_control(
            c[CONF_UPSCALE], var, "set_upscale", UPSCALE_LABELS, UPSCALES.index(config[CONF_UPSCALE]),
            convert="static_cast<esphome::isp::Upscale>(v)",
        )
    if CONF_EQUALIZE in c:
        await new_number_control(c[CONF_EQUALIZE], var, "set_equalize", config[CONF_EQUALIZE], 0, 100, 5)
    if CONF_DETAIL in c:
        await new_number_control(c[CONF_DETAIL], var, "set_detail", config[CONF_DETAIL], 0, 4, 0.1)
    if CONF_SCALER in c:
        await new_select_control(
            c[CONF_SCALER], var, "set_scaler", SCALER_LABELS, SCALERS.index(config[CONF_SCALER]),
            convert="static_cast<esphome::isp::Scaler>(v)",
        )
    if CONF_CLEAR_SPOTS in c:
        await new_button_control(c[CONF_CLEAR_SPOTS], var, "clear_spots")

    # Spots are kept across reboots, one record per ISP.
    cg.add(var.set_spots_key(fnv1_hash(f"isp_spots_{config[CONF_ID].id}")))
    if CONF_SPOTS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SPOTS])
        cg.add(var.set_spots_text_sensor(sens))
