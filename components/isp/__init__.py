"""Image signal processing for a radiometric source: tone mapping onto a
palette, mirroring, upscaling and output pixel format. Output is an image
source (for an overlay, encoder or camera)."""

import esphome.codegen as cg
from esphome.components.esp32 import get_esp32_variant, include_builtin_idf_component
from esphome.components.esp32.const import VARIANT_ESP32P4
from esphome.components.video_controls import (
    new_number_control,
    new_select_control,
    new_switch_control,
    number_control,
    select_control,
    switch_control,
)
from esphome.components.video_core import ImageSource, RadiometricSink, RadiometricSource
import esphome.config_validation as cv
from esphome.const import CONF_FORMAT, CONF_ID, CONF_SOURCE

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["video_core", "video_controls"]

CONF_SCALE = "scale"
CONF_PALETTE = "palette"
CONF_RANGE_MIN = "range_min"
CONF_RANGE_MAX = "range_max"
CONF_MIRROR_X = "mirror_x"
CONF_MIRROR_Y = "mirror_y"
CONF_UPSCALE = "upscale"
CONF_SCALER = "scaler"
CONF_CONTROLS = "controls"
CONF_LOG_TIMING = "log_timing"

FORMATS = {"RGB565_LE": "RGB565_LE", "RGB565_BE": "RGB565_BE", "RGB888": "RGB888", "GRAY8": "GRAY8"}
PALETTES = ["iron", "gray"]
UPSCALES = ["nearest", "bilinear"]
UPSCALE_LABELS = ["Nearest", "Bilinear"]
SCALERS = ["cpu", "ppa"]
SCALER_LABELS = ["CPU", "PPA"]

isp_ns = cg.esphome_ns.namespace("isp")
Isp = isp_ns.class_("Isp", cg.Component, RadiometricSink, ImageSource)
Palette = isp_ns.enum("Palette", is_class=True)
Upscale = isp_ns.enum("Upscale", is_class=True)
Scaler = isp_ns.enum("Scaler", is_class=True)
ImageFormat = cg.esphome_ns.namespace("video").enum("ImageFormat", is_class=True)


def _validate(config):
    if config[CONF_RANGE_MIN] >= config[CONF_RANGE_MAX]:
        raise cv.Invalid(f"{CONF_RANGE_MIN} must be below {CONF_RANGE_MAX}")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Isp),
            cv.Required(CONF_SOURCE): cv.use_id(RadiometricSource),
            # Output size is the source size times this.
            cv.Optional(CONF_SCALE, default=1): cv.int_range(min=1, max=20),
            # RGB565_LE is what the ESP32-P4's H.264 encoder and PPA take.
            cv.Optional(CONF_FORMAT, default="RGB565_LE"): cv.enum(FORMATS, upper=True),
            cv.Optional(CONF_PALETTE, default="iron"): cv.one_of(*PALETTES, lower=True),
            # Values mapped to the first and last palette colour.
            cv.Optional(CONF_RANGE_MIN, default=15.0): cv.float_,
            cv.Optional(CONF_RANGE_MAX, default=40.0): cv.float_,
            cv.Optional(CONF_MIRROR_X, default=False): cv.boolean,
            cv.Optional(CONF_MIRROR_Y, default=False): cv.boolean,
            # CPU scaling method. The PPA's method is fixed by the hardware.
            cv.Optional(CONF_UPSCALE, default="nearest"): cv.one_of(*UPSCALES, lower=True),
            cv.Optional(CONF_SCALER, default="cpu"): cv.one_of(*SCALERS, lower=True),
            cv.Optional(CONF_CONTROLS, default={}): cv.Schema(
                {
                    cv.Optional(CONF_RANGE_MIN): number_control(),
                    cv.Optional(CONF_RANGE_MAX): number_control(),
                    cv.Optional(CONF_MIRROR_X): switch_control(),
                    cv.Optional(CONF_MIRROR_Y): switch_control(),
                    cv.Optional(CONF_UPSCALE): select_control(),
                    cv.Optional(CONF_SCALER): select_control(),
                }
            ),
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
    cg.add(var.set_format(ImageFormat.enum(config[CONF_FORMAT])))
    cg.add(var.set_palette(Palette.enum(config[CONF_PALETTE].upper())))
    cg.add(var.set_range_min(config[CONF_RANGE_MIN]))
    cg.add(var.set_range_max(config[CONF_RANGE_MAX]))
    cg.add(var.set_mirror_x(config[CONF_MIRROR_X]))
    cg.add(var.set_mirror_y(config[CONF_MIRROR_Y]))
    cg.add(var.set_upscale(Upscale.enum(config[CONF_UPSCALE].upper())))
    cg.add(var.set_scaler(Scaler.enum(config[CONF_SCALER].upper())))
    cg.add(var.set_log_timing(config[CONF_LOG_TIMING]))

    c = config[CONF_CONTROLS]
    for key in (CONF_RANGE_MIN, CONF_RANGE_MAX):
        if key in c:
            await new_number_control(c[key], var, f"set_{key}", config[key], -40, 300, 0.5)
    for key in (CONF_MIRROR_X, CONF_MIRROR_Y):
        if key in c:
            await new_switch_control(c[key], var, f"set_{key}", config[key])
    if CONF_UPSCALE in c:
        await new_select_control(
            c[CONF_UPSCALE], var, "set_upscale", UPSCALE_LABELS, UPSCALES.index(config[CONF_UPSCALE]),
            convert="static_cast<esphome::isp::Upscale>(v)",
        )
    if CONF_SCALER in c:
        await new_select_control(
            c[CONF_SCALER], var, "set_scaler", SCALER_LABELS, SCALERS.index(config[CONF_SCALER]),
            convert="static_cast<esphome::isp::Scaler>(v)",
        )
