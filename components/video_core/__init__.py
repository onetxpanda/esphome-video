"""Frame types, buffer pools and source/sink interfaces shared by the
esphome-video components. Loaded automatically by the components that use it."""

import esphome.codegen as cg

CODEOWNERS = ["@onetxpanda"]

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
