"""Chooses which of several image sources reaches the consumers after it
(encoder, camera), with a select entity, so they all show the same thing. An
entry may list several sources; frames from any of them pass while that
entry is chosen (e.g. a camera's two outputs, only one of which produces
frames at a time)."""

import esphome.codegen as cg
from esphome.components import select
from esphome.components.video_core import ImageSource
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_NAME, CONF_RESTORE_VALUE, CONF_SOURCE

CODEOWNERS = ["@onetxpanda"]
AUTO_LOAD = ["video_core", "select"]
MULTI_CONF = True

CONF_SOURCES = "sources"
CONF_SELECT = "select"

ns = cg.esphome_ns.namespace("video_switch")
VideoSwitch = ns.class_("VideoSwitch", cg.Component, ImageSource)
SwitchSelect = ns.class_("SwitchSelect", select.Select, cg.Component)

ENTRY_SCHEMA = cv.Schema(
    {
        # The entry's option in the select.
        cv.Required(CONF_NAME): cv.string_strict,
        cv.Required(CONF_SOURCE): cv.All(cv.ensure_list(cv.use_id(ImageSource)), cv.Length(min=1, max=4)),
    }
)


def _validate(config):
    names = [e[CONF_NAME] for e in config[CONF_SOURCES]]
    if len(set(names)) != len(names):
        raise cv.Invalid("source names must be unique")
    ids = [str(s) for e in config[CONF_SOURCES] for s in e[CONF_SOURCE]]
    if len(set(ids)) != len(ids):
        raise cv.Invalid("sources lists the same component more than once")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(VideoSwitch),
            cv.Required(CONF_SOURCES): cv.All(cv.ensure_list(ENTRY_SCHEMA), cv.Length(min=1, max=16)),
            cv.Optional(CONF_SELECT): select.select_schema(SwitchSelect, icon="mdi:video-switch")
            .extend({cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean})
            .extend(cv.COMPONENT_SCHEMA),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for index, entry in enumerate(config[CONF_SOURCES]):
        cg.add(var.add_entry(entry[CONF_NAME]))
        for source_id in entry[CONF_SOURCE]:
            source = await cg.get_variable(source_id)
            cg.add(var.add_source(source, index))
    if CONF_SELECT in config:
        conf = config[CONF_SELECT]
        sel = await select.new_select(conf, options=[e[CONF_NAME] for e in config[CONF_SOURCES]])
        await cg.register_component(sel, conf)
        cg.add(sel.set_switch(var))
        cg.add(sel.set_restore(conf[CONF_RESTORE_VALUE]))
