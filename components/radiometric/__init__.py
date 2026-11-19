"""Temperature-domain processing of a radiometric source: invalid-pixel
repair, outlier filter, motion-adaptive temporal filter, edge-preserving
smoothing, median filter, and statistics with their sensors. Output is a
radiometric source again (e.g. for `isp`)."""

import esphome.codegen as cg
from esphome.components import sensor
from esphome.components.video_controls import (
    new_number_control,
    new_switch_control,
    number_control,
    switch_control,
)
from esphome.components.video_core import (
    DEVICE_ID_SCHEMA,
    RadiometricSink,
    RadiometricSource,
    inherit_device_id,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_SOURCE,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

CODEOWNERS = ["@onetxpanda"]
MULTI_CONF = True  # e.g. one chain per camera
AUTO_LOAD = ["video_core", "video_controls", "sensor"]

CONF_TEMPORAL_FRAMES = "temporal_frames"
CONF_MOTION_THRESHOLD = "motion_threshold"
CONF_EDGE_SMOOTHING = "edge_smoothing"
CONF_MEDIAN_FILTER = "median_filter"
CONF_OUTLIER_LEVEL = "outlier_level"
CONF_CONTROLS = "controls"
CONF_MIN = "min"
CONF_MAX = "max"
CONF_MEAN = "mean"
CONF_MEDIAN = "median"
CONF_CENTER = "center"
CONF_LOG_TIMING = "log_timing"
CONF_VALID_MIN = "valid_min"
CONF_VALID_MAX = "valid_max"
SENSORS = [CONF_MIN, CONF_MAX, CONF_MEAN, CONF_MEDIAN, CONF_CENTER]

radiometric_ns = cg.esphome_ns.namespace("radiometric")
Radiometric = radiometric_ns.class_(
    "Radiometric", cg.PollingComponent, RadiometricSink, RadiometricSource
)

_TEMP = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=2,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)


def _validate(config):
    if config[CONF_VALID_MIN] >= config[CONF_VALID_MAX]:
        raise cv.Invalid(f"{CONF_VALID_MIN} must be below {CONF_VALID_MAX}")
    return config


CONFIG_SCHEMA = cv.All(inherit_device_id("controls.*", *SENSORS), cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Radiometric),
        **DEVICE_ID_SCHEMA,
        cv.Required(CONF_SOURCE): cv.use_id(RadiometricSource),
        # Temporal filter: still pixels are averaged over about this many
        # frames; 1 = off. Pixels whose neighbourhood changes by more than
        # motion_threshold (degrees) are treated as moving and follow at once.
        cv.Optional(CONF_TEMPORAL_FRAMES, default=1): cv.int_range(min=1, max=16),
        cv.Optional(CONF_MOTION_THRESHOLD, default=0.5): cv.float_range(min=0.05, max=10),
        # Edge-preserving (bilateral) smoothing: neighbours within about this
        # many degrees are averaged, edges are kept; 0 = off.
        cv.Optional(CONF_EDGE_SMOOTHING, default=0.0): cv.float_range(min=0, max=10),
        cv.Optional(CONF_MEDIAN_FILTER, default=False): cv.boolean,
        # A pixel differing from both row neighbours by at least this many
        # degrees becomes their average; 0 = off.
        cv.Optional(CONF_OUTLIER_LEVEL, default=10.0): cv.float_range(min=0, max=100),
        cv.Optional(CONF_CONTROLS, default={}): cv.Schema(
            {
                cv.Optional(CONF_TEMPORAL_FRAMES): number_control(),
                cv.Optional(CONF_MOTION_THRESHOLD): number_control(),
                cv.Optional(CONF_EDGE_SMOOTHING): number_control(),
                cv.Optional(CONF_MEDIAN_FILTER): switch_control(),
                cv.Optional(CONF_OUTLIER_LEVEL): number_control(),
            }
        ),
        **{cv.Optional(k): _TEMP for k in SENSORS},
        cv.Optional(CONF_LOG_TIMING, default=False): cv.boolean,
        # Readings outside this range are invalid and repaired from their
        # neighbours. The defaults suit the MLX90640 (-40..300 C).
        cv.Optional(CONF_VALID_MIN, default=-40.0): cv.float_range(min=-273, max=2000),
        cv.Optional(CONF_VALID_MAX, default=300.0): cv.float_range(min=-273, max=2000),
    }
).extend(cv.polling_component_schema("1s")), _validate)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    source = await cg.get_variable(config[CONF_SOURCE])
    cg.add(source.add_radiometric_sink(var))

    cg.add(var.set_temporal_frames(config[CONF_TEMPORAL_FRAMES]))
    cg.add(var.set_motion_threshold(config[CONF_MOTION_THRESHOLD]))
    cg.add(var.set_edge_smoothing(config[CONF_EDGE_SMOOTHING]))
    cg.add(var.set_median_filter(config[CONF_MEDIAN_FILTER]))
    cg.add(var.set_outlier_level(config[CONF_OUTLIER_LEVEL]))
    cg.add(var.set_log_timing(config[CONF_LOG_TIMING]))
    cg.add(var.set_valid_range(config[CONF_VALID_MIN], config[CONF_VALID_MAX]))
    for key in SENSORS:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))

    controls = config[CONF_CONTROLS]
    if CONF_TEMPORAL_FRAMES in controls:
        await new_number_control(
            controls[CONF_TEMPORAL_FRAMES], var, "set_temporal_frames",
            config[CONF_TEMPORAL_FRAMES], 1, 16, 1,
        )
    if CONF_MOTION_THRESHOLD in controls:
        await new_number_control(
            controls[CONF_MOTION_THRESHOLD], var, "set_motion_threshold", config[CONF_MOTION_THRESHOLD], 0.05, 10, 0.05
        )
    if CONF_EDGE_SMOOTHING in controls:
        await new_number_control(
            controls[CONF_EDGE_SMOOTHING], var, "set_edge_smoothing", config[CONF_EDGE_SMOOTHING], 0, 10, 0.1
        )
    if CONF_MEDIAN_FILTER in controls:
        await new_switch_control(controls[CONF_MEDIAN_FILTER], var, "set_median_filter", config[CONF_MEDIAN_FILTER])
    if CONF_OUTLIER_LEVEL in controls:
        await new_number_control(
            controls[CONF_OUTLIER_LEVEL], var, "set_outlier_level", config[CONF_OUTLIER_LEVEL], 0, 100, 0.5
        )
