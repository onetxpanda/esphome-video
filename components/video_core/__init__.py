"""Frame types, buffer pools and source/sink interfaces shared by the
esphome-video components. Loaded automatically by the components that use it."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_DEVICE_ID

CODEOWNERS = ["@onetxpanda"]

# A component's `device_id`: the ESPHome sub-device (`esphome: devices:`) its
# entities belong to, so they need no name prefix to tell them apart from
# another chain's. An entity can still name its own `device_id`.
DEVICE_ID_SCHEMA = {cv.Optional(CONF_DEVICE_ID): cv.sub_device_id}


def inherit_device_id(*paths):
    """A validator to run before a component's schema: gives each of the
    component's entities that has no `device_id` the component's own.
    `paths` locate the entity configs ("controls.*": every entry of
    `controls`). It runs first because ESPHome checks entity names for
    duplicates per device while it validates each entity."""

    def apply(node, keys, device):
        if not isinstance(node, dict):
            return
        if not keys:
            node.setdefault(CONF_DEVICE_ID, device)
            return
        key, rest = keys[0], keys[1:]
        children = list(node.values()) if key == "*" else [node[key]] if key in node else []
        for child in children:
            apply(child, rest, device)

    def validator(config):
        device = config.get(CONF_DEVICE_ID) if isinstance(config, dict) else None
        if device is not None:
            for path in paths:
                apply(config, path.split("."), device)
        return config

    return validator

video_ns = cg.esphome_ns.namespace("video")
ImageSource = video_ns.class_("ImageSource")
ImageSink = video_ns.class_("ImageSink")
RadiometricSource = video_ns.class_("RadiometricSource")
RadiometricSink = video_ns.class_("RadiometricSink")
PaletteTarget = video_ns.class_("PaletteTarget")

# Palette keys (YAML) and names (select options), in the order of
# PALETTE_NAMES in frame.h.
PALETTE_KEYS = ["white_hot", "black_hot", "ironbow", "lava", "rainbow", "iron_gray", "red_hot", "rainbow_2"]
PALETTE_LABELS = ["White hot", "Black hot", "Ironbow", "Lava", "Rainbow", "Iron gray", "Red hot", "Rainbow 2"]
