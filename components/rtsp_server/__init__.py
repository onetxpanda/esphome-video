"""RTSP server for an h264_encoder stream (RTP over TCP interleaved).

Serves rtsp://<device>:<port>/<path>. Clients that try UDP first (ffmpeg)
are told 461 Unsupported Transport and retry over TCP; go2rtc uses TCP.
"""

import re

import esphome.codegen as cg
from esphome.components import socket
from esphome.components.h264_encoder import H264Encoder
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PATH, CONF_PORT
from esphome.core import CORE

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["network", "h264_encoder"]
AUTO_LOAD = ["socket"]

CONF_ENCODER = "encoder"
CONF_MAX_CLIENTS = "max_clients"
CONF_BUFFER_SIZE = "buffer_size"
CONF_SESSION_NAME = "session_name"

rtsp_server_ns = cg.esphome_ns.namespace("rtsp_server")
RtspServer = rtsp_server_ns.class_("RtspServer", cg.Component)

_PATH_RE = r"^[A-Za-z0-9._~-]+(/[A-Za-z0-9._~-]+)*$"


def _validate_path(value):
    value = cv.string_strict(value).strip("/")
    if "trackID=" in value:
        raise cv.Invalid("path must not contain 'trackID=' (the server adds the track itself)")
    if not re.fullmatch(_PATH_RE, value):
        raise cv.Invalid("path may contain only letters, digits, '.', '_', '~', '-' and '/'")
    return value


def _consume_sockets(config):
    # One listening socket plus one per client.
    socket.consume_sockets(1, "rtsp_server", socket.SocketType.TCP_LISTEN)(config)
    return socket.consume_sockets(config[CONF_MAX_CLIENTS], "rtsp_server")(config)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RtspServer),
            cv.GenerateID(CONF_ENCODER): cv.use_id(H264Encoder),
            cv.Optional(CONF_PORT, default=554): cv.port,
            cv.Optional(CONF_PATH, default="stream"): _validate_path,
            cv.Optional(CONF_SESSION_NAME): cv.string_strict,
            cv.Optional(CONF_MAX_CLIENTS, default=2): cv.int_range(min=1, max=4),
            # Per-client send backlog. Beyond it whole frames are dropped and the
            # client resumes at the next keyframe.
            cv.Optional(CONF_BUFFER_SIZE, default="128kB"): cv.All(
                cv.validate_bytes, cv.int_range(min=16 * 1024, max=4 * 1024 * 1024)
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _consume_sockets,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    encoder = await cg.get_variable(config[CONF_ENCODER])
    cg.add(var.set_encoder(encoder))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_path(config[CONF_PATH]))
    cg.add(var.set_session_name(config.get(CONF_SESSION_NAME, CORE.friendly_name or CORE.name)))
    cg.add(var.set_max_clients(config[CONF_MAX_CLIENTS]))
    cg.add(var.set_buffer_size(config[CONF_BUFFER_SIZE]))
