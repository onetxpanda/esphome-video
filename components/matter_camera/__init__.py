"""Matter camera: live video from the H.264 encoder over WebRTC.

Adds a Matter camera endpoint (device type 0x0142) to the matter component.
A controller (Home Assistant 2026.11+, Apple Home, ...) negotiates a WebRTC
session through Matter and receives the encoder's stream. The cluster and
signaling code comes from Espressif's esp-matter camera example (standalone
mode); the WebRTC stack is Espressif's port of Amazon's KVS WebRTC SDK.
"""

from pathlib import Path
import shutil
import subprocess

import esphome.codegen as cg
from esphome.components.esp32 import (
    add_idf_component,
    add_idf_sdkconfig_option,
    include_builtin_idf_component,
    require_mbedtls_ecp,
    require_mbedtls_peer_cert,
    require_mbedtls_tls,
    require_mbedtls_tls_server,
)
from esphome.components import socket
from esphome.components.h264_encoder import H264Encoder
from esphome.components.matter.endpoints import require_clusters
from esphome.components.matter.types import MatterComponent
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.core import CORE

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["matter", "h264_encoder", "network", "psram"]

CONF_ENCODER = "encoder"
CONF_ENDPOINT = "endpoint"
CONF_MATTER_ID = "matter_id"

# Espressif's ESP-IDF port of the Amazon KVS WebRTC SDK, pinned. Only two of
# its submodules are needed: the SDK itself and usrsctp.
KVS_REPO = "https://github.com/espressif/esp-port-for-amazon-kvs-sdk.git"
KVS_COMMIT = "038f59a1caa396e4e12ab66c125e6ba0a8e48e1d"
KVS_SUBMODULES = ("amazon-kinesis-video-streams-webrtc-sdk-c", "components/esp_usrsctp/usrsctp")
# The KVS components the camera code uses directly; the rest come in through
# their manifests.
KVS_COMPONENTS = ("app_webrtc", "kvs_webrtc", "esp_webrtc_utils", "kvs_utils", "media_stream")
# Manifest dependencies dropped from the local copy: esp_hosted and its bridge
# (ESPHome's esp32_hosted owns the co-processor), AWS signaling transports
# (Matter carries the signaling) and the MQTT client of KVS's console tools.
KVS_DROPPED_DEPENDENCIES = {
    "esp_webrtc_utils": ("espressif/esp_hosted", "webrtc_bridge", "espressif/mqtt"),
    "app_webrtc": ("webrtc_bridge",),
    "kvs_webrtc": ("libwebsockets", "espressif/esp_websocket_client"),
}

matter_camera_ns = cg.esphome_ns.namespace("matter_camera")
MatterCamera = matter_camera_ns.class_("MatterCamera", cg.Component)


def _require_camera_clusters(config):
    require_clusters(
        "CONFIG_SUPPORT_CAMERA_AV_STREAM_MANAGEMENT_CLUSTER",
        "CONFIG_SUPPORT_WEB_RTC_TRANSPORT_PROVIDER_CLUSTER",
    )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MatterCamera),
            cv.GenerateID(CONF_MATTER_ID): cv.use_id(MatterComponent),
            cv.Required(CONF_ENCODER): cv.use_id(H264Encoder),
            cv.Optional(CONF_ENDPOINT, default=1): cv.int_range(min=1, max=65534),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _require_camera_clusters,
    # ICE: a UDP socket per local address and viewer (up to two viewers).
    socket.consume_sockets(10, "matter_camera", socket.SocketType.UDP),
)


# ---- media_stream replacement ------------------------------------------------
# KVS's media_stream drives its own camera, ISP and encoders. Only its encoded
# frame bus (video_sink) and rate controller are used here; frames come from
# the h264_encoder component.

_MEDIA_STREAM_CMAKE = """\
idf_component_register(
    SRCS "src/video_rate_ctrl.c" "src/video_sink.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_webrtc_utils esp_timer pthread)
"""

_MEDIA_STREAM_MANIFEST = """\
dependencies:
  esp_webrtc_utils:
    path: ../esp_webrtc_utils
    version: "*"
"""

_MEDIA_STREAM_KCONFIG = """\
menu "Media Stream Configuration"
    config VIDEO_SINK_MAX
        int "Maximum number of encoded video sinks"
        range 1 8
        default 4
    config VIDEO_SINK_SLOW_WARN_US
        int "Slow sink warning threshold (us)"
        range 100 100000
        default 5000
    config VIDEO_RATE_CTRL_MAX
        int "Maximum number of video rate controllers"
        range 1 8
        default 4
    config VIDEO_RATE_CTRL_FPS_FLOOR
        int "Rate control: lowest frame rate the ladder may schedule"
        range 1 30
        default 12
    config VIDEO_RATE_CTRL_MIN_MBPS_PER_PX
        int "Rate control: bitrate floor in milli-bits per pixel per second"
        range 0 2000
        default 280
endmenu
"""


# esp_webrtc_utils without its console commands (esp_cli, wifi_cli), which need
# the console and MQTT components.
_WEBRTC_UTILS_CMAKE = """\
idf_component_register(
    SRCS "src/message_utils.c" "src/webrtc_mem_utils.c" "src/esp_work_queue.c"
         "src/message_queue.c" "src/esp_webrtc_time.c" "src/flash_wrapper.c"
    REQUIRES esp_common freertos nvs_flash pthread esp_timer esp_wifi lwip vfs
    INCLUDE_DIRS "include")
"""


def _git(*args: str, cwd: Path) -> None:
    subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True, text=True)


def _local_kvs() -> Path:
    """Fetch the KVS port once, apply its SDK patches, and adapt it for
    ESPHome. Returns the checkout's root."""
    from esphome import yaml_util

    base = Path(CORE.data_dir) / "matter_camera" / f"kvs-{KVS_COMMIT[:12]}"
    ready = base / ".esphome-local-copy"
    if ready.is_file():
        return base

    shutil.rmtree(base, ignore_errors=True)
    base.mkdir(parents=True)
    try:
        _git("init", "-q", cwd=base)
        _git("remote", "add", "origin", KVS_REPO, cwd=base)
        _git("fetch", "-q", "--depth", "1", "origin", KVS_COMMIT, cwd=base)
        _git("checkout", "-q", "FETCH_HEAD", cwd=base)
        _git("submodule", "update", "--init", "--depth", "1", *KVS_SUBMODULES, cwd=base)
        sdk = base / KVS_SUBMODULES[0]
        for patch in sorted((base / "patches").glob("[0-9]*.patch")):
            _git("apply", str(patch), cwd=sdk)
    except subprocess.CalledProcessError as e:
        raise cv.Invalid(f"Could not fetch the KVS WebRTC SDK: {e.stderr or e}") from e

    components = base / "components"
    for name, dropped in KVS_DROPPED_DEPENDENCIES.items():
        manifest = components / name / "idf_component.yml"
        data = yaml_util.load_yaml(manifest)
        deps = data.get("dependencies", {})
        for dep in dropped:
            deps.pop(dep, None)
        manifest.write_text(yaml_util.dump(data))

    (components / "esp_webrtc_utils" / "CMakeLists.txt").write_text(_WEBRTC_UTILS_CMAKE)
    media = components / "media_stream"
    (media / "CMakeLists.txt").write_text(_MEDIA_STREAM_CMAKE)
    (media / "idf_component.yml").write_text(_MEDIA_STREAM_MANIFEST)
    (media / "Kconfig.projbuild").write_text(_MEDIA_STREAM_KCONFIG)

    ready.touch()
    return base


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    encoder = await cg.get_variable(config[CONF_ENCODER])
    cg.add(var.set_encoder(encoder))
    cg.add(var.set_endpoint_id(config[CONF_ENDPOINT]))
    matter = await cg.get_variable(config[CONF_MATTER_ID])
    cg.add(var.set_matter(matter))
    cg.add(
        matter.register_endpoint(
            config[CONF_ENDPOINT],
            cg.RawExpression("esphome::matter_camera::MatterCamera::build_endpoint"),
        )
    )

    kvs = _local_kvs()
    for name in KVS_COMPONENTS:
        add_idf_component(name=name, path=str(kvs / "components" / name))
    # The SDK patches are applied above; KVS's own CMake step would redo them
    # with `git am`.
    add_idf_sdkconfig_option("CONFIG_KVS_AUTO_APPLY_PATCHES", False)

    # KVS's components list the legacy "driver" component.
    include_builtin_idf_component("driver")

    # WebRTC media is DTLS-SRTP; the camera can take either DTLS role. KVS
    # talks to mbedTLS directly (its esp-tls path is only for TURN over TLS).
    add_idf_sdkconfig_option("CONFIG_USE_ESP_TLS_FOR_KVS", False)
    require_mbedtls_tls()
    require_mbedtls_tls_server()
    require_mbedtls_peer_cert()
    require_mbedtls_ecp()
    # A DTLS session's buffers are tens of KB: keep them in PSRAM.
    add_idf_sdkconfig_option("CONFIG_MBEDTLS_EXTERNAL_MEM_ALLOC", True)
    add_idf_sdkconfig_option("CONFIG_MBEDTLS_SSL_PROTO_DTLS", True)
    add_idf_sdkconfig_option("CONFIG_MBEDTLS_SSL_DTLS_SRTP", True)
