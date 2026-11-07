"""ESP32-P4 hardware H.264 encoder.

Takes frames from image sources (isp, overlay, video_test_pattern, ...) and
hands encoded access units to consumers such as rtsp_server. Hardware encoder only: there is no software
fallback.
"""

from pathlib import Path

import esphome.codegen as cg
from esphome.components.esp32 import add_idf_component, only_on_variant
from esphome.components.esp32.const import VARIANT_ESP32P4
from esphome.components.video_core import ImageSource
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_SOURCE, Framework

CODEOWNERS = ["@onetxpanda"]
AUTO_LOAD = ["video_core"]

# Espressif's esp_h264 component, pinned. 1.5.0 adds esp_h264_enc_force_idr.
ESP_H264_VERSION = "1.5.0"

CONF_FPS = "fps"
CONF_GOP = "gop"
CONF_BITRATE = "bitrate"
CONF_QP_MIN = "qp_min"
CONF_QP_MAX = "qp_max"

h264_encoder_ns = cg.esphome_ns.namespace("h264_encoder")
H264Encoder = h264_encoder_ns.class_("H264Encoder", cg.Component)


def _validate(config):
    if config[CONF_QP_MIN] > config[CONF_QP_MAX]:
        raise cv.Invalid(f"{CONF_QP_MIN} must not be greater than {CONF_QP_MAX}")
    ids = [str(s) for s in config[CONF_SOURCE]]
    if len(set(ids)) != len(ids):
        raise cv.Invalid(f"{CONF_SOURCE} lists the same component more than once")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(H264Encoder),
            # One or more image sources. The first is active at boot; a
            # `select` (platform: h264_encoder) switches between them.
            cv.Required(CONF_SOURCE): cv.All(
                cv.ensure_list(cv.use_id(ImageSource)), cv.Length(min=1, max=8)
            ),
            # Frame rate for rate control and the stream's timing info.
            # Default: measured from the active source (the encoder restarts
            # its stream with a keyframe when the rate changes by over 25%).
            cv.Optional(CONF_FPS): cv.int_range(min=1, max=60),
            # Keyframe interval in frames (default: one per second). Smaller
            # values let viewers start sooner, at the cost of bitrate.
            cv.Optional(CONF_GOP): cv.int_range(min=1, max=255),
            # Target bitrate in bit/s. Default: automatic from frame size and fps.
            cv.Optional(CONF_BITRATE): cv.int_range(min=16000, max=40_000_000),
            cv.Optional(CONF_QP_MIN, default=20): cv.int_range(min=0, max=51),
            cv.Optional(CONF_QP_MAX, default=40): cv.int_range(min=0, max=51),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    only_on_variant(supported=[VARIANT_ESP32P4], msg_prefix="The H.264 hardware encoder"),
    cv.only_with_framework(Framework.ESP_IDF),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_fps(config.get(CONF_FPS, 0)))
    cg.add(var.set_gop(config.get(CONF_GOP, 0)))
    cg.add(var.set_bitrate(config.get(CONF_BITRATE, 0)))
    cg.add(var.set_qp(config[CONF_QP_MIN], config[CONF_QP_MAX]))

    # Each source gets its own input, so the encoder knows which is active.
    for source_id in config[CONF_SOURCE]:
        source = await cg.get_variable(source_id)
        cg.add(source.add_image_sink(var.add_source(str(source_id))))

    cg.add_define("USE_H264_ENCODER")
    add_idf_component(name="espressif/esp_h264", ref=ESP_H264_VERSION)
    # Build guard: fails the build unless esp_h264 is compiled with the
    # register definitions for the configured chip revision (see
    # esphome_h264_guard/project_include.cmake). Lives inside this component, so it
    # is pinned by the same ref.
    add_idf_component(name="esphome_h264_guard", path=str(Path(__file__).parent / "esphome_h264_guard"))
