"""Synthetic image source: moving colour bars, a sweeping box and a binary
frame counter. For checking encoders and streams without a camera."""

import esphome.codegen as cg
from esphome.components.video_core import ImageSource
import esphome.config_validation as cv
from esphome.const import CONF_HEIGHT, CONF_ID, CONF_WIDTH

CODEOWNERS = ["@onetxpanda"]
AUTO_LOAD = ["video_core"]

CONF_FPS = "fps"

ns = cg.esphome_ns.namespace("video_test_pattern")
TestPattern = ns.class_("TestPattern", cg.Component, ImageSource)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TestPattern),
        cv.Optional(CONF_WIDTH, default=320): cv.int_range(min=16, max=1920),
        cv.Optional(CONF_HEIGHT, default=240): cv.int_range(min=16, max=2032),
        cv.Optional(CONF_FPS, default=10): cv.int_range(min=1, max=30),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_size(config[CONF_WIDTH], config[CONF_HEIGHT]))
    cg.add(var.set_fps(config[CONF_FPS]))
