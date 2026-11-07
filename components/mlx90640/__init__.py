"""MLX90640 32x24 thermal sensor: a radiometric source (temperatures, no
image processing). Feed it to a `radiometric` component."""

import esphome.codegen as cg
from esphome.components import i2c
from esphome.components.video_controls import new_select_control, select_control
from esphome.components.video_core import RadiometricSource
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["i2c", "esp32"]
AUTO_LOAD = ["video_core", "video_controls"]
MULTI_CONF = False  # the Melexis I2C driver addresses one sensor

CONF_REFRESH_RATE = "refresh_rate"
CONF_FRAME_MODE = "frame_mode"
CONF_EMISSIVITY = "emissivity"
CONF_CONTROLS = "controls"
CONF_LOG_TIMING = "log_timing"

REFRESH_RATES = ["0.5Hz", "1Hz", "2Hz", "4Hz", "8Hz", "16Hz", "32Hz", "64Hz"]
REFRESH_RATE_LABELS = ["0.5 Hz", "1 Hz", "2 Hz", "4 Hz", "8 Hz", "16 Hz", "32 Hz", "64 Hz"]
FRAME_MODES = ["interpolate", "merge"]
FRAME_MODE_LABELS = ["Interpolate each sub-page", "Merge sub-page pairs"]

mlx90640_ns = cg.esphome_ns.namespace("mlx90640")
MLX90640 = mlx90640_ns.class_("MLX90640", i2c.I2CDevice, cg.Component, RadiometricSource)
FrameMode = mlx90640_ns.enum("FrameMode", is_class=True)


def _refresh_rate(value):
    value = cv.string_strict(value).replace(" ", "")
    if value.lower() in [r.lower() for r in REFRESH_RATES]:
        return [r.lower() for r in REFRESH_RATES].index(value.lower())
    raise cv.Invalid(f"refresh_rate must be one of {', '.join(REFRESH_RATES)}")


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MLX90640),
            # Sub-page rate. A full frame takes two sub-pages.
            cv.Optional(CONF_REFRESH_RATE, default="16Hz"): _refresh_rate,
            cv.Optional(CONF_FRAME_MODE, default="interpolate"): cv.one_of(*FRAME_MODES, lower=True),
            cv.Optional(CONF_EMISSIVITY, default=0.95): cv.float_range(min=0.1, max=1.0),
            cv.Optional(CONF_CONTROLS, default={}): cv.Schema(
                {
                    cv.Optional(CONF_REFRESH_RATE): select_control(),
                    cv.Optional(CONF_FRAME_MODE): select_control(),
                }
            ),
            cv.Optional(CONF_LOG_TIMING, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x33))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    rate = config[CONF_REFRESH_RATE]
    mode = FRAME_MODES.index(config[CONF_FRAME_MODE])
    cg.add(var.set_refresh_rate(rate))
    cg.add(var.set_frame_mode(FrameMode.enum(config[CONF_FRAME_MODE].upper())))
    cg.add(var.set_emissivity(config[CONF_EMISSIVITY]))
    cg.add(var.set_log_timing(config[CONF_LOG_TIMING]))

    controls = config[CONF_CONTROLS]
    if CONF_REFRESH_RATE in controls:
        await new_select_control(
            controls[CONF_REFRESH_RATE], var, "set_refresh_rate", REFRESH_RATE_LABELS, rate
        )
    if CONF_FRAME_MODE in controls:
        await new_select_control(
            controls[CONF_FRAME_MODE],
            var,
            "set_frame_mode",
            FRAME_MODE_LABELS,
            mode,
            convert="static_cast<esphome::mlx90640::FrameMode>(v)",
        )
