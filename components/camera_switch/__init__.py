"""Passes on the radiometric frames of whichever camera is connected, so one
processing chain (radiometric -> isp -> overlay) and one set of its controls
serve every camera. The first camera listed that has sent a frame within
`timeout` is used."""

import esphome.codegen as cg
from esphome.components import text_sensor
from esphome.components.video_core import RadiometricSource
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_NAME, CONF_SOURCE, CONF_TIMEOUT

CODEOWNERS = ["@onetxpanda"]
AUTO_LOAD = ["video_core", "text_sensor"]
MULTI_CONF = True

CONF_CAMERAS = "cameras"
CONF_CAMERA = "camera"

ns = cg.esphome_ns.namespace("camera_switch")
CameraSwitch = ns.class_("CameraSwitch", cg.Component, RadiometricSource)

CAMERA_SCHEMA = cv.Schema(
    {
        # The camera's name, as the `camera` text sensor shows it.
        cv.Required(CONF_NAME): cv.string_strict,
        cv.Required(CONF_SOURCE): cv.use_id(RadiometricSource),
    }
)


def _validate(config):
    names = [c[CONF_NAME] for c in config[CONF_CAMERAS]]
    if len(set(names)) != len(names):
        raise cv.Invalid("camera names must be unique")
    ids = [str(c[CONF_SOURCE]) for c in config[CONF_CAMERAS]]
    if len(set(ids)) != len(ids):
        raise cv.Invalid("cameras lists the same source more than once")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(CameraSwitch),
            # In order of preference: the first one sending frames is used.
            cv.Required(CONF_CAMERAS): cv.All(cv.ensure_list(CAMERA_SCHEMA), cv.Length(min=1, max=8)),
            # A camera that has sent no frame for this long counts as gone.
            # Longer than the slowest camera's frame interval (an MLX90640 at
            # 0.5 Hz in merge mode sends one every 4 s).
            cv.Optional(CONF_TIMEOUT, default="10s"): cv.positive_time_period_milliseconds,
            # Text sensor: the camera in use, or "None".
            cv.Optional(CONF_CAMERA): text_sensor.text_sensor_schema(icon="mdi:camera"),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for cam in config[CONF_CAMERAS]:
        source = await cg.get_variable(cam[CONF_SOURCE])
        cg.add(var.add_camera(cam[CONF_NAME], source))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    if CONF_CAMERA in config:
        sens = await text_sensor.new_text_sensor(config[CONF_CAMERA])
        cg.add(var.set_camera_text_sensor(sens))
