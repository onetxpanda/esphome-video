"""Draws an image source's annotations (e.g. min/max/centre temperatures from
a `radiometric` component, carried through the `isp`) as reticles with value
labels. Draws into the frames in place: it must be its source's only
consumer; attach other consumers to the overlay instead."""

import esphome.codegen as cg
from esphome.components.video_controls import (
    new_select_control,
    new_switch_control,
    select_control,
    switch_control,
)
from esphome.components.video_core import ImageSink, ImageSource
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_SOURCE

CODEOWNERS = ["@onetxpanda"]
MULTI_CONF = True  # e.g. one chain per camera
AUTO_LOAD = ["video_core", "video_controls"]

CONF_MARKS = "marks"
CONF_HOLD = "hold"
CONF_ENABLED = "enabled"
CONF_UNIT = "unit"
CONF_CONTROLS = "controls"

KINDS = ["min", "max", "center"]
UNITS = ["C", "F"]
UNIT_LABELS = ["°C", "°F"]

overlay_ns = cg.esphome_ns.namespace("overlay")
Overlay = overlay_ns.class_("Overlay", cg.Component, ImageSink, ImageSource)
Unit = overlay_ns.enum("Unit", is_class=True)
AnnotationKind = cg.esphome_ns.namespace("video").enum("AnnotationKind", is_class=True)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Overlay),
        cv.Required(CONF_SOURCE): cv.use_id(ImageSource),
        cv.Optional(CONF_MARKS, default=KINDS): cv.ensure_list(cv.one_of(*KINDS, lower=True)),
        # How often a mark's position and value are refreshed (together).
        cv.Optional(CONF_HOLD, default="1s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_ENABLED, default=True): cv.boolean,
        cv.Optional(CONF_UNIT, default="C"): cv.one_of(*UNITS, upper=True),
        cv.Optional(CONF_CONTROLS, default={}): cv.Schema(
            {
                cv.Optional(CONF_ENABLED): switch_control(),
                cv.Optional(CONF_UNIT): select_control(),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    source = await cg.get_variable(config[CONF_SOURCE])
    cg.add(source.add_image_sink(var))
    for k in dict.fromkeys(config[CONF_MARKS]):
        cg.add(var.add_kind(AnnotationKind.enum(k.upper())))
    cg.add(var.set_hold_ms(config[CONF_HOLD].total_milliseconds))
    cg.add(var.set_enabled(config[CONF_ENABLED]))
    unit = UNITS.index(config[CONF_UNIT])
    cg.add(var.set_unit(Unit.enum("FAHRENHEIT" if unit else "CELSIUS")))
    c = config[CONF_CONTROLS]
    if CONF_ENABLED in c:
        await new_switch_control(c[CONF_ENABLED], var, "set_enabled", config[CONF_ENABLED])
    if CONF_UNIT in c:
        await new_select_control(
            c[CONF_UNIT], var, "set_unit", UNIT_LABELS, unit, convert="static_cast<esphome::overlay::Unit>(v)"
        )
