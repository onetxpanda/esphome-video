"""Thermal Master P1 USB thermal camera (USB 3474:45C2), 160x120 at 25 frames/s.
A radiometric source (the camera's temperatures, 1/64 K, with a gain and
offset to correct them) and an image source (the camera's own processed
picture). Its settings, actions and readings, the command set as the vendor
app's library has it, are optional entities. Needs `usb_host:` on an
ESP32-P4 (USB 2.0 high speed)."""

import esphome.codegen as cg
from esphome.components import binary_sensor, button, number, select, sensor, switch, text_sensor, usb_host
from esphome.components.esp32 import VARIANT_ESP32P4, only_on_variant
from esphome.components.video_controls import (
    button_control,
    new_number_control,
    number_control,
)
from esphome.components.video_core import (
    DEVICE_ID_SCHEMA,
    ImageSource,
    RadiometricSource,
    inherit_device_id,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_OFFSET,
    CONF_UPDATE_INTERVAL,
    CONF_X,
    CONF_Y,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_PROBLEM,
    DEVICE_CLASS_RUNNING,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_SECOND,
)

CODEOWNERS = ["@onetxpanda"]
DEPENDENCIES = ["usb_host", "esp32"]
AUTO_LOAD = [
    "video_core",
    "video_controls",
    "binary_sensor",
    "button",
    "number",
    "select",
    "sensor",
    "switch",
    "text_sensor",
]
MULTI_CONF = False  # every P1 has the same USB ID

CONF_GAIN = "gain"
CONF_TRANSFERS = "transfers"
CONF_CONTROLS = "controls"
CONF_SETTINGS = "settings"
CONF_MAINTENANCE = "maintenance"
CONF_STREAMING = "streaming"
CONF_STATUS = "status"
CONF_LOG_TIMING = "log_timing"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_CAMERA_POINT = "camera_point"
CONF_CAMERA_MAX = "camera_max_temperature"
CONF_CAMERA_MIN = "camera_min_temperature"
CONF_PALETTES = "palettes"
CONF_RANGE_IN_USE = "range_in_use"
CONF_SUN_DETECTED = "sun_detected"
CONF_SHUTTER_COUNTDOWN = "shutter_countdown"
CONF_VL_RESULT = "vl_result"

VID = 0x3474
PID = 0x45C2  # P1

tm_ns = cg.esphome_ns.namespace("thermal_master_p1")
ThermalMasterP1 = tm_ns.class_("ThermalMasterP1", usb_host.USBClient, RadiometricSource, ImageSource)
SettingNumber = tm_ns.class_("SettingNumber", number.Number)
SettingPartNumber = tm_ns.class_("SettingPartNumber", number.Number)
SettingSelect = tm_ns.class_("SettingSelect", select.Select)
SettingSwitch = tm_ns.class_("SettingSwitch", switch.Switch)
Setting = tm_ns.enum("Setting", is_class=True)
Action = tm_ns.enum("Action", is_class=True)
Reading = tm_ns.enum("Reading", is_class=True)
Info = tm_ns.enum("Info", is_class=True)
Param = tm_ns.enum("Param", is_class=True)
Custom = tm_ns.enum("Custom", is_class=True)

RANGE_AUTO = 2  # thermal_master_p1.h


def _sel(*options):
    """Select options: (key, label, camera value)."""
    return ("select", options)


def _num(lo, hi, step, scale=1, offset=0, unit=None, temperature=False):
    """A number: shown = camera value / scale + offset."""
    return ("number", (lo, hi, step, scale, offset, unit, temperature))


def _pair(parts, lo, hi, signed=False, unit=None, temperature=False):
    """Two values one command sets together, each its own number."""
    return ("pair", (parts, lo, hi, signed, unit, temperature))


_SWITCH = ("switch", None)
_LEVEL = _num(0, 100, 1)
_WORD = _num(0, 65535, 1)

# The camera's settings (tm_core.h Setting), in the units shown in Home
# Assistant; scale and offset turn them into the camera's values.
SETTINGS = {
    "range": (
        "RANGE",
        _sel(
            ("high", "High (-4 to 302 °F)", 1),
            ("low", "Low (302 to 1112 °F)", 0),
            ("auto", "Automatic", RANGE_AUTO),
        ),
    ),
    # The camera's palette index; the names are the `palettes` text.
    "palette": ("PALETTE", _num(0, 255, 1)),
    "brightness": ("BRIGHTNESS", _LEVEL),
    "contrast": ("CONTRAST", _LEVEL),
    "global_contrast": ("GLOBAL_CONTRAST", _LEVEL),
    "detail_enhance": ("DETAIL_ENHANCE", _LEVEL),
    "noise_reduction": ("NOISE_REDUCTION", _LEVEL),
    "spatial_noise_reduction": ("SPATIAL_NOISE_REDUCTION", _LEVEL),
    "temporal_noise_reduction": ("TEMPORAL_NOISE_REDUCTION", _LEVEL),
    "gamma": ("GAMMA", _LEVEL),
    "edge_enhance": ("EDGE_ENHANCE", _sel(("off", "Off", 0), ("level_1", "Level 1", 1), ("level_2", "Level 2", 2))),
    "roi_level": ("ROI_LEVEL", _sel(("off", "Off", 0), ("third", "A third", 1), ("half", "A half", 2))),
    "profession_mode": ("PROFESSION_MODE", _sel(("normal", "Normal", 0), ("professional", "Professional", 1))),
    "mirror_flip": (
        "MIRROR_FLIP",
        _sel(("none", "None", 0), ("mirror", "Mirror", 1), ("flip", "Flip", 2), ("both", "Mirror and flip", 3)),
    ),
    "isothermal_mode": ("ISOTHERMAL_MODE", _sel(*[(str(i), str(i), i) for i in range(6)])),
    "isothermal_limit": (
        "ISOTHERMAL_LIMIT",
        _pair(("low", "high"), -70, 650, signed=True, unit=UNIT_CELSIUS, temperature=True),
    ),
    "isothermal": ("ISOTHERMAL", _SWITCH),
    "sun_detect": ("SUN_DETECT", _SWITCH),
    "sun_detect_pixel_ratio": ("SUN_DETECT_PIXEL_RATIO", _pair(("min", "max"), 0, 100)),
    "sun_detect_roundness": ("SUN_DETECT_ROUNDNESS", _pair(("min", "max"), 0, 100)),
    "overexposure_protection": (
        "OVEREXPOSURE_PROTECTION",
        _sel(("off", "Off", 0), ("on", "On", 1), ("mode_2", "Mode 2", 2)),
    ),
    "auto_shutter": ("AUTO_SHUTTER", _SWITCH),
    "auto_shutter_threshold": ("AUTO_SHUTTER_THRESHOLD", _WORD),
    "auto_shutter_min_interval": ("AUTO_SHUTTER_MIN_INTERVAL", _WORD),
    "auto_shutter_max_interval": ("AUTO_SHUTTER_MAX_INTERVAL", _num(5, 65535, 1)),
    "shutter_open": ("SHUTTER_OPEN", _SWITCH),
    "env_correction": ("ENV_CORRECTION", _SWITCH),
    "emissivity": ("EMISSIVITY", _num(0.001, 1.0, 0.001, scale=16384)),
    # Whole kelvin, C + 273.
    "ambient": ("AMBIENT", _num(-40, 100, 1, offset=-273, unit=UNIT_CELSIUS, temperature=True)),
    "reflected": ("REFLECTED", _num(-40, 100, 1, offset=-273, unit=UNIT_CELSIUS, temperature=True)),
    "transmittance": ("TRANSMITTANCE", _num(0.001, 1.0, 0.001, scale=16384)),
    "stream_mid_mode": (
        "STREAM_MID_MODE",
        _sel(("picture", "Picture", 0), ("tpd", "TPD", 1), ("lce", "LCE", 2), ("ir", "IR", 3), ("snr", "SNR", 4)),
    ),
    "palette_switch": ("PALETTE_SWITCH", _SWITCH),
    "frame_temp_display": ("FRAME_TEMP_DISPLAY", _SWITCH),
}
# Maintenance settings: only under `maintenance:`.
MAINTENANCE_SETTINGS = {"cursor": ("CURSOR", _SWITCH), "nuc_bypass": ("NUC_BYPASS", _SWITCH)}

# Buttons (tm_core.h Action). The heartbeat is `heartbeat_interval`.
ACTIONS = {
    "shutter": "SHUTTER",
    "shutter_ooc": "SHUTTER_OOC",
    "shutter_both": "SHUTTER_BOTH",
    "ffc_update": "FFC_UPDATE",
    "ffc_without_shutter": "FFC_WITHOUT_SHUTTER",
}
MAINTENANCE_ACTIONS = {
    k: k.upper()
    for k in [
        "reboot",
        "save_settings",
        "restore_settings",
        "reset_to_rom",
        "reset_to_bootloader",
        "save_defaults",
        "restore_calibration",
        "ooc_save",
        "k_calibrate_low",
        "k_calibrate_high",
        "k_calculate",
        "k_cancel",
        "k_clear",
        "k_save",
        "k_restore",
        "dpc_auto",
        "dpc_cancel",
        "dpc_clear",
        "dpc_save",
        "dpc_restore",
        "rmcover_auto",
        "rmcover_cancel",
        "rmcover_clear",
        "rmcover_save",
        "rmcover_restore",
        "tpd1_cancel",
        "tpd1_clear",
        "tpd1_save",
        "tpd_restore",
        "tpd2_cancel",
        "tpd2_clear",
        "tpd2_save",
        "tpd2_restore",
        "crg_save",
    ]
}
# Maintenance commands with values: their numbers and buttons.
MAINTENANCE_PARAMS = {
    "cursor_x": ("CURSOR_X", 0, 159, 1),
    "cursor_y": ("CURSOR_Y", 0, 119, 1),
    # The library passes the reference unscaled, as a whole number; its unit is not given.
    "tpd_reference": ("TPD_REFERENCE", -32768, 32767, 1),
    "vl_register": ("VL_REGISTER", 0, 16777215, 1),
    "vl_value": ("VL_VALUE", 0, 16777215, 1),
    "crg_new_value": ("CRG_VALUE", 0, 16777215, 1),
}
MAINTENANCE_CUSTOM = {
    "cursor_move": "CURSOR_MOVE",
    "dpc_mark": "DPC_MARK",
    "dpc_unmark": "DPC_UNMARK",
    "tpd1_recalibrate": "TPD1_RECALIBRATE",
    "tpd2_point1": "TPD2_POINT1",
    "tpd2_point2": "TPD2_POINT2",
    "vl_read": "VL_READ",
    "vl_write": "VL_WRITE",
    "crg_write": "CRG_WRITE",
}

_TEMPERATURE = dict(
    unit_of_measurement=UNIT_CELSIUS,
    device_class=DEVICE_CLASS_TEMPERATURE,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
_DIAG = dict(entity_category=ENTITY_CATEGORY_DIAGNOSTIC)
# Readings (tm_core.h Reading): numeric ones are sensors, the others text.
SENSOR_READINGS = {
    "device_temperature": ("DEVICE_TEMPERATURE", {**_TEMPERATURE, **_DIAG}),
    "powered_time": (
        "POWERED_TIME",
        dict(unit_of_measurement=UNIT_SECOND, device_class=DEVICE_CLASS_DURATION, accuracy_decimals=0, **_DIAG),
    ),
    "crg_value": ("CRG_VALUE", dict(accuracy_decimals=0, **_DIAG)),
    "sensor_vtemp": ("SENSOR_VTEMP", dict(accuracy_decimals=0, **_DIAG)),
    "camera_point_temperature": ("CAMERA_POINT", _TEMPERATURE),
}
TEXT_READINGS = {
    "device_status": "DEVICE_STATUS",
    "shutter_status": "SHUTTER_STATUS",
    "stream_source": "STREAM_SOURCE",
    "digital_video_output": "DIGITAL_VIDEO_OUTPUT",
    "system_data_flag": "SYSTEM_DATA_FLAG",
    "cursor_position": "CURSOR_POSITION",
}
# Device information, read once a start (tm_core.h Info).
INFO_TEXTS = {
    "model": "NAME",
    "firmware": "FIRMWARE",
    "customer_id": "CUSTOMER_ID",
    "vendor_id": "VENDOR_ID",
    "product_id": "PRODUCT_ID",
    "part_number": "PART_NUMBER",
    "serial": "SERIAL",
    "isp_version": "ISP_VERSION",
    "abstract_version": "ABSTRACT_VERSION",
    "system_version": "SYSTEM_VERSION",
    "calibration_version": "CALIBRATION_VERSION",
    "data_version": "DATA_VERSION",
    "package_version": "PACKAGE_VERSION",
    "sensor_type": "SENSOR_TYPE",
    "sensor_id": "SENSOR_ID",
}
OTHER_TEXTS = {CONF_PALETTES: "set_palettes_text", CONF_RANGE_IN_USE: "set_range_text"}


def _number_kw(unit, temperature):
    kw = {"entity_category": ENTITY_CATEGORY_CONFIG}
    if unit is not None:
        kw["unit_of_measurement"] = unit
    if temperature:
        kw["device_class"] = DEVICE_CLASS_TEMPERATURE
    return kw


def _setting_controls(key, kind, extra):
    """The entity keys a setting has, with their schemas."""
    if kind == "select":
        return {key: select.select_schema(SettingSelect, entity_category=ENTITY_CATEGORY_CONFIG)}
    if kind == "switch":
        return {
            key: switch.switch_schema(
                SettingSwitch, entity_category=ENTITY_CATEGORY_CONFIG, default_restore_mode="DISABLED"
            )
        }
    if kind == "pair":
        parts, _, _, _, unit, temperature = extra
        return {f"{key}_{p}": number.number_schema(SettingPartNumber, **_number_kw(unit, temperature)) for p in parts}
    _, _, _, _, _, unit, temperature = extra
    return {key: number.number_schema(SettingNumber, **_number_kw(unit, temperature))}


def _pack_pair(signed, first, second):
    if signed:
        v = (first & 0xFFFF) | ((second & 0xFFFF) << 16)
        return v - (1 << 32) if v >= (1 << 31) else v
    return (first & 0xFF) | ((second & 0xFF) << 8)


def _setting_value(kind, extra):
    """Validator for a `settings:` value, giving the camera's value."""
    if kind == "select":
        values = {k: v for k, _, v in extra}
        return cv.All(cv.string, cv.one_of(*values, lower=True), lambda k: values[k])
    if kind == "switch":
        return cv.All(cv.boolean, lambda b: 1 if b else 0)
    if kind == "pair":
        parts, lo, hi, signed, _, _ = extra
        return cv.All(
            cv.Schema({cv.Required(p): cv.int_range(min=lo, max=hi) for p in parts}),
            lambda d: _pack_pair(signed, d[parts[0]], d[parts[1]]),
        )
    lo, hi, _, scale, offset, _, _ = extra
    return cv.All(cv.float_range(min=lo, max=hi), lambda v: int(round((v - offset) * scale)))


def _controls(table):
    out = {}
    for key, (_, (kind, extra)) in table.items():
        out.update(_setting_controls(key, kind, extra))
    return out


CONTROLS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_GAIN): number_control(),
        # A temperature difference: no unit conversion in Home Assistant.
        cv.Optional(CONF_OFFSET): number_control(unit_of_measurement="°C"),
        **{cv.Optional(k): button_control() for k in ACTIONS},
        **{cv.Optional(k): s for k, s in _controls(SETTINGS).items()},
    }
)
MAINTENANCE_SCHEMA = cv.Schema(
    {
        **{cv.Optional(k): button_control() for k in MAINTENANCE_ACTIONS},
        **{cv.Optional(k): button_control() for k in MAINTENANCE_CUSTOM},
        **{cv.Optional(k): number_control(entity_category=ENTITY_CATEGORY_CONFIG) for k in MAINTENANCE_PARAMS},
        **{cv.Optional(k): s for k, s in _controls(MAINTENANCE_SETTINGS).items()},
        cv.Optional(CONF_VL_RESULT): sensor.sensor_schema(accuracy_decimals=0, **_DIAG),
    }
)

CONFIG_SCHEMA = cv.All(
    inherit_device_id(
        "controls.*",
        "maintenance.*",
        CONF_STREAMING,
        CONF_STATUS,
        CONF_CAMERA_MAX,
        CONF_CAMERA_MIN,
        CONF_SUN_DETECTED,
        CONF_SHUTTER_COUNTDOWN,
        *SENSOR_READINGS,
        *TEXT_READINGS,
        *INFO_TEXTS,
        *OTHER_TEXTS,
    ),
    usb_host.usb_device_schema(ThermalMasterP1, VID, PID).extend(
        {
            **DEVICE_ID_SCHEMA,
            # Temperatures: the camera's times gain, plus offset (degrees C),
            # to match a reference.
            cv.Optional(CONF_GAIN, default=1.0): cv.float_range(min=0.1, max=10.0),
            cv.Optional(CONF_OFFSET, default=0.0): cv.float_range(min=-50.0, max=50.0),
            # Bulk transfers kept queued (8 KB of internal RAM each).
            cv.Optional(CONF_TRANSFERS, default=4): cv.int_range(min=2, max=8),
            # Camera settings written at every start (then read back). Auto
            # shutter and sun detection are on unless set here.
            cv.Optional(CONF_SETTINGS, default={}): cv.Schema(
                {cv.Optional(k): _setting_value(*v[1]) for k, v in SETTINGS.items()}
            ),
            # Readings are polled this often (the device information once a start).
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.positive_time_period_milliseconds,
            # The library's keepalive; off unless set (the vendor app sends none to a P1).
            cv.Optional(CONF_HEARTBEAT_INTERVAL): cv.positive_time_period_milliseconds,
            # The point camera_point_temperature reads (the camera's own measurement); the centre by default.
            cv.Optional(CONF_CAMERA_POINT): cv.Schema(
                {cv.Required(CONF_X): cv.int_range(min=0, max=159), cv.Required(CONF_Y): cv.int_range(min=0, max=119)}
            ),
            cv.Optional(CONF_CONTROLS, default={}): CONTROLS_SCHEMA,
            # Commands that change or wipe what the camera keeps (its stored
            # settings, calibration, dead pixels, firmware mode): entities only
            # when listed here.
            cv.Optional(CONF_MAINTENANCE, default={}): MAINTENANCE_SCHEMA,
            cv.Optional(CONF_STREAMING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_RUNNING, **_DIAG
            ),
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(icon="mdi:usb", **_DIAG),
            # The camera's flag for the sun in view (with sun detection on).
            cv.Optional(CONF_SUN_DETECTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_PROBLEM, icon="mdi:white-balance-sunny"
            ),
            # The metadata rows' shutter countdown (unit not given).
            cv.Optional(CONF_SHUTTER_COUNTDOWN): sensor.sensor_schema(accuracy_decimals=0, **_DIAG),
            cv.Optional(CONF_CAMERA_MAX): sensor.sensor_schema(**_TEMPERATURE),
            cv.Optional(CONF_CAMERA_MIN): sensor.sensor_schema(**_TEMPERATURE),
            **{cv.Optional(k): sensor.sensor_schema(**kw) for k, (_, kw) in SENSOR_READINGS.items()},
            **{
                cv.Optional(k): text_sensor.text_sensor_schema(**_DIAG)
                for k in [*TEXT_READINGS, *INFO_TEXTS, *OTHER_TEXTS]
            },
            cv.Optional(CONF_LOG_TIMING, default=False): cv.boolean,
        }
    ),
    only_on_variant(supported=[VARIANT_ESP32P4], msg_prefix="The Thermal Master P1 (USB 2.0 high speed)"),
)


async def _new_setting_entities(conf, var, key, enum_name, kind, extra):
    s = getattr(Setting, enum_name)
    if kind == "select":
        if key not in conf:
            return
        ent = await select.new_select(conf[key], options=[label for _, label, _ in extra])
        cg.add(ent.set_parent(var, s, [v for _, _, v in extra]))
        cg.add(var.add_setting_entity(s, ent))
    elif kind == "switch":
        if key not in conf:
            return
        ent = await switch.new_switch(conf[key])
        cg.add(ent.set_parent(var, s))
        cg.add(var.add_setting_entity(s, ent))
    elif kind == "pair":
        parts, lo, hi, _, _, _ = extra
        for i, p in enumerate(parts):
            k = f"{key}_{p}"
            if k not in conf:
                continue
            ent = await number.new_number(conf[k], min_value=lo, max_value=hi, step=1)
            cg.add(ent.set_parent(var, s, i))
            cg.add(var.add_setting_entity(s, ent))
    else:
        if key not in conf:
            return
        lo, hi, step, scale, offset, _, _ = extra
        ent = await number.new_number(conf[key], min_value=lo, max_value=hi, step=step)
        cg.add(ent.set_parent(var, s, float(scale), float(offset)))
        cg.add(var.add_setting_entity(s, ent))


async def _new_button(conf, call):
    btn = await button.new_button(conf)
    await cg.register_component(btn, conf)
    cg.add(btn.set_apply(cg.RawExpression(f"[]() {{ {call}; }}")))


async def to_code(config):
    var = await usb_host.register_usb_client(config)
    cg.add(var.set_gain(config[CONF_GAIN]))
    cg.add(var.set_offset(config[CONF_OFFSET]))
    cg.add(var.set_transfers(config[CONF_TRANSFERS]))
    cg.add(var.set_log_timing(config[CONF_LOG_TIMING]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    if CONF_HEARTBEAT_INTERVAL in config:
        cg.add(var.set_heartbeat_interval(config[CONF_HEARTBEAT_INTERVAL]))
    for key, value in config[CONF_SETTINGS].items():
        cg.add(var.set_initial_setting(getattr(Setting, SETTINGS[key][0]), value))
    if CONF_CAMERA_POINT in config:
        cg.add(var.set_point(config[CONF_CAMERA_POINT][CONF_X], config[CONF_CAMERA_POINT][CONF_Y]))

    if CONF_STREAMING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_STREAMING])
        cg.add(var.set_streaming_sensor(sens))
    if CONF_SUN_DETECTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_SUN_DETECTED])
        cg.add(var.set_sun_sensor(sens))
    if CONF_STATUS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_STATUS])
        cg.add(var.set_status_sensor(sens))
    for key, method in (
        (CONF_CAMERA_MAX, "set_camera_max_sensor"),
        (CONF_CAMERA_MIN, "set_camera_min_sensor"),
        (CONF_SHUTTER_COUNTDOWN, "set_shutter_countdown_sensor"),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, method)(sens))
    for key, (enum_name, _) in SENSOR_READINGS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(var.set_reading_sensor(getattr(Reading, enum_name), sens))
    for key, enum_name in TEXT_READINGS.items():
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(var.set_reading_text(getattr(Reading, enum_name), sens))
    for key, enum_name in INFO_TEXTS.items():
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(var.set_info_text(getattr(Info, enum_name), sens))
    for key, method in OTHER_TEXTS.items():
        if key in config:
            sens = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(var, method)(sens))

    c = config[CONF_CONTROLS]
    if CONF_GAIN in c:
        await new_number_control(c[CONF_GAIN], var, "set_gain", config[CONF_GAIN], 0.1, 10.0, 0.01)
    if CONF_OFFSET in c:
        await new_number_control(c[CONF_OFFSET], var, "set_offset", config[CONF_OFFSET], -50.0, 50.0, 0.1)
    for key, enum_name in ACTIONS.items():
        if key in c:
            await _new_button(c[key], f"{var}->press(esphome::thermal_master_p1::Action::{enum_name})")
    for key, (enum_name, (kind, extra)) in SETTINGS.items():
        await _new_setting_entities(c, var, key, enum_name, kind, extra)

    m = config[CONF_MAINTENANCE]
    for key, enum_name in MAINTENANCE_ACTIONS.items():
        if key in m:
            await _new_button(m[key], f"{var}->press(esphome::thermal_master_p1::Action::{enum_name})")
    for key, enum_name in MAINTENANCE_CUSTOM.items():
        if key in m:
            await _new_button(m[key], f"{var}->press_custom(esphome::thermal_master_p1::Custom::{enum_name})")
    for key, (enum_name, lo, hi, step) in MAINTENANCE_PARAMS.items():
        if key in m:
            await new_number_control(
                m[key], var, "set_param", lo, lo, hi, step, convert=f"esphome::thermal_master_p1::Param::{enum_name}, v"
            )
    for key, (enum_name, (kind, extra)) in MAINTENANCE_SETTINGS.items():
        await _new_setting_entities(m, var, key, enum_name, kind, extra)
    if CONF_VL_RESULT in m:
        sens = await sensor.new_sensor(m[CONF_VL_RESULT])
        cg.add(var.set_vl_result_sensor(sens))
