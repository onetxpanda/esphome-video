"""Runtime option entities (number, switch, select) for esphome-video
components. Not configured directly: a component's `controls:` block creates
them through the helpers below."""

import esphome.codegen as cg
from esphome.components import number, select, switch
import esphome.config_validation as cv
from esphome.const import CONF_RESTORE_VALUE

CODEOWNERS = ["@onetxpanda"]
AUTO_LOAD = ["number", "select", "switch"]

ns = cg.esphome_ns.namespace("video_controls")
ControlNumber = ns.class_("ControlNumber", number.Number, cg.Component)
ControlSwitch = ns.class_("ControlSwitch", switch.Switch, cg.Component)
ControlSelect = ns.class_("ControlSelect", select.Select, cg.Component)


def number_control(**kwargs):
    return (
        number.number_schema(ControlNumber, **kwargs)
        .extend({cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean})
        .extend(cv.COMPONENT_SCHEMA)
    )


def switch_control():
    return switch.switch_schema(ControlSwitch, default_restore_mode="RESTORE_DEFAULT_OFF").extend(
        cv.COMPONENT_SCHEMA
    )


def select_control():
    return select.select_schema(ControlSelect).extend(
        {cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean}
    ).extend(cv.COMPONENT_SCHEMA)


def _setter(parent, method, arg_type, convert):
    # A lambda calling parent->method(convert(v)); `parent` is a global pointer.
    return cg.RawExpression(f"[]({arg_type} v) {{ {parent}->{method}({convert}); }}")


async def new_number_control(conf, parent, method, initial, min_value, max_value, step, convert="v"):
    var = await number.new_number(conf, min_value=min_value, max_value=max_value, step=step)
    await cg.register_component(var, conf)
    cg.add(var.set_apply(_setter(parent, method, "float", convert)))
    cg.add(var.set_initial(initial))
    cg.add(var.set_restore(conf[CONF_RESTORE_VALUE]))
    return var


async def new_switch_control(conf, parent, method, initial):
    var = await switch.new_switch(conf)
    await cg.register_component(var, conf)
    cg.add(var.set_apply(_setter(parent, method, "bool", "v")))
    cg.add(var.set_initial(initial))
    return var


async def new_select_control(conf, parent, method, labels, initial_index, convert="v"):
    var = await select.new_select(conf, options=labels)
    await cg.register_component(var, conf)
    cg.add(var.set_apply(_setter(parent, method, "size_t", convert)))
    cg.add(var.set_initial(initial_index))
    cg.add(var.set_restore(conf[CONF_RESTORE_VALUE]))
    return var
