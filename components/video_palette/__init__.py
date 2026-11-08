"""One select entity choosing the false-colour palette of several stages at
once: ISPs (their lookup table) and cameras that colour their own image
(e.g. the HTI-301 in processed mode). It restores its last value on boot;
with nothing saved it applies `palette`."""

import esphome.codegen as cg
from esphome.components import select
from esphome.components.video_controls import select_control
from esphome.components.video_core import PALETTE_KEYS, PALETTE_LABELS, PaletteTarget
import esphome.config_validation as cv
from esphome.const import CONF_RESTORE_VALUE

CODEOWNERS = ["@onetxpanda"]
AUTO_LOAD = ["video_core", "video_controls", "select"]
MULTI_CONF = True

CONF_TARGETS = "targets"
CONF_PALETTE = "palette"


def _validate(config):
    ids = [str(t) for t in config[CONF_TARGETS]]
    if len(set(ids)) != len(ids):
        raise cv.Invalid("targets lists the same component more than once")
    return config


CONFIG_SCHEMA = cv.All(
    select_control().extend(
        {
            # ISPs and cameras whose palette this select sets.
            cv.Required(CONF_TARGETS): cv.All(cv.ensure_list(cv.use_id(PaletteTarget)), cv.Length(min=1)),
            cv.Optional(CONF_PALETTE, default="ironbow"): cv.one_of(*PALETTE_KEYS, lower=True),
        }
    ),
    _validate,
)


async def to_code(config):
    targets = [await cg.get_variable(t) for t in config[CONF_TARGETS]]
    var = await select.new_select(config, options=PALETTE_LABELS)
    await cg.register_component(var, config)
    body = " ".join(f"{t}->set_palette_index(static_cast<uint8_t>(v));" for t in targets)
    cg.add(var.set_apply(cg.RawExpression(f"[](size_t v) {{ {body} }}")))
    cg.add(var.set_initial(PALETTE_KEYS.index(config[CONF_PALETTE])))
    cg.add(var.set_restore(config[CONF_RESTORE_VALUE]))
