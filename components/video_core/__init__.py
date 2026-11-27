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


def usb_camera_protections():
    """What every USB camera component needs on the ESP32-P4, called from its
    to_code(). Without it, frame processing starves the USB host: a camera's
    transfers run late or are lost, and with a 25 frames/s camera feeding the
    H.264 encoder the USB task spun on a lock until the task watchdog reset
    the chip."""
    from pathlib import Path

    from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option
    from esphome.core import HexInt

    # Give the USB controller's receive FIFO the most room (3.3 KB rather than
    # 2.5 KB), so incoming packets wait there, not get dropped, while its DMA
    # is held up.
    add_idf_sdkconfig_option("CONFIG_USB_HOST_HW_BUFFER_BIAS_IN", True)
    add_idf_sdkconfig_option("CONFIG_USB_HOST_HW_BUFFER_BIAS_BALANCED", False)
    # Cache write-backs before DMA (H.264, JPEG, PPA) run with interrupts off
    # and hold a lock the USB driver takes; a whole frame's (614 KB at
    # 640x480) lasts far longer than the 125 us the USB interrupt has to start
    # the next transfer. In 8 KB pieces, each is short.
    add_idf_sdkconfig_option("CONFIG_ESP_MM_CACHE_MSYNC_C2M_CHUNKED_OPS", True)
    add_idf_sdkconfig_option("CONFIG_ESP_MM_CACHE_MSYNC_C2M_CHUNKED_OPS_MAX_LEN", HexInt(0x2000))
    # The USB interrupt path in IRAM, so processing cannot make it late.
    add_idf_component(name="usb_camera_iram", path=str(Path(__file__).parent / "usb_camera_iram"))
