"""Select entity that switches an h264_encoder between its sources."""

import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import CONF_OPTIONS, CONF_RESTORE_VALUE, CONF_SOURCE
from esphome.core import CORE
import esphome.final_validate as fv

from .. import H264Encoder, h264_encoder_ns

DEPENDENCIES = ["h264_encoder"]

CONF_H264_ENCODER_ID = "h264_encoder_id"

H264SourceSelect = h264_encoder_ns.class_("H264SourceSelect", select.Select, cg.Component)

CONFIG_SCHEMA = (
    select.select_schema(H264SourceSelect, icon="mdi:video-switch")
    .extend(
        {
            cv.GenerateID(CONF_H264_ENCODER_ID): cv.use_id(H264Encoder),
            # Display names, one per encoder source in the same order.
            # Default: the source component IDs.
            cv.Optional(CONF_OPTIONS): cv.All(cv.ensure_list(cv.string_strict), cv.Length(min=1)),
            cv.Optional(CONF_RESTORE_VALUE, default=True): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


def _final_validate(config):
    encoder = fv.full_config.get().get("h264_encoder")
    if encoder is None:
        return config
    sources = encoder[CONF_SOURCE]
    options = config.get(CONF_OPTIONS)
    if options is not None:
        if len(options) != len(sources):
            raise cv.Invalid(
                f"{CONF_OPTIONS} must give one name per encoder source "
                f"({len(sources)}: {', '.join(str(s) for s in sources)}), in the same order"
            )
        if len(set(options)) != len(options):
            raise cv.Invalid(f"{CONF_OPTIONS} must be unique")
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    sources = CORE.config["h264_encoder"][CONF_SOURCE]
    options = config.get(CONF_OPTIONS) or [str(s) for s in sources]
    var = await select.new_select(config, options=options)
    await cg.register_component(var, config)
    encoder = await cg.get_variable(config[CONF_H264_ENCODER_ID])
    cg.add(var.set_encoder(encoder))
    cg.add(var.set_restore(config[CONF_RESTORE_VALUE]))
