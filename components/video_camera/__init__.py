"""An ESPHome camera entity fed by an image source, JPEG-encoded by the
ESP32-P4's hardware encoder while a client wants images."""

import esphome.codegen as cg
from esphome.components.esp32 import include_builtin_idf_component, only_on_variant
from esphome.components.esp32.const import VARIANT_ESP32P4
from esphome.components.video_core import ImageSink, ImageSource
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_SOURCE
from esphome.core.entity_helpers import setup_entity

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["camera", "video_core"]

CONF_QUALITY = "quality"
CONF_SUBSAMPLING = "subsampling"

video_camera_ns = cg.esphome_ns.namespace("video_camera")
Camera = cg.esphome_ns.namespace("camera").class_("Camera", cg.EntityBase, cg.Component)
VideoCamera = video_camera_ns.class_("VideoCamera", Camera, ImageSink)
SUBSAMPLING = {
    "444": cg.RawExpression("JPEG_DOWN_SAMPLING_YUV444"),
    "422": cg.RawExpression("JPEG_DOWN_SAMPLING_YUV422"),
    "420": cg.RawExpression("JPEG_DOWN_SAMPLING_YUV420"),
}

CONFIG_SCHEMA = cv.All(
    cv.ENTITY_BASE_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(VideoCamera),
            cv.Required(CONF_SOURCE): cv.use_id(ImageSource),
            cv.Optional(CONF_QUALITY, default=80): cv.int_range(min=1, max=100),
            # Chroma subsampling for colour images. 444 works at any size that
            # is a multiple of 8; 422 and 420 need multiples of 16.
            cv.Optional(CONF_SUBSAMPLING, default="444"): cv.one_of(*SUBSAMPLING, string=True),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    only_on_variant(supported=[VARIANT_ESP32P4], msg_prefix="The hardware JPEG encoder"),
)


async def to_code(config):
    cg.add_define("USE_CAMERA")
    include_builtin_idf_component("esp_driver_jpeg")
    var = cg.new_Pvariable(config[CONF_ID])
    await setup_entity(var, config, "camera")
    await cg.register_component(var, config)
    source = await cg.get_variable(config[CONF_SOURCE])
    cg.add(source.add_image_sink(var))
    cg.add(var.set_quality(config[CONF_QUALITY]))
    cg.add(var.set_subsampling(SUBSAMPLING[config[CONF_SUBSAMPLING]]))
