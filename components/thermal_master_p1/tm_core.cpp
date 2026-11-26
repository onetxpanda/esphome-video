#include "tm_core.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace thermal_master_p1 {

uint16_t crc16_xmodem(const uint8_t *data, size_t n) {
  uint16_t crc = 0;
  for (size_t i = 0; i < n; i++) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

// ---------------------------------------------------------------- commands

void Command::seal_() {
  const size_t n = this->is_long() ? 22 : 16;
  const uint16_t crc = crc16_xmodem(this->bytes.data(), n);
  this->bytes[n] = static_cast<uint8_t>(crc);
  this->bytes[n + 1] = static_cast<uint8_t>(crc >> 8);
}

uint16_t Command::response_bytes() const {
  const size_t at = this->is_long() ? 16 : 12;
  return static_cast<uint16_t>(this->bytes[at] | (this->bytes[at + 1] << 8));
}

Command &Command::u8(size_t at, uint8_t v) {
  this->bytes[at] = v;
  this->seal_();
  return *this;
}

Command &Command::u16(size_t at, uint16_t v) {
  this->bytes[at] = static_cast<uint8_t>(v);
  this->bytes[at + 1] = static_cast<uint8_t>(v >> 8);
  this->seal_();
  return *this;
}

Command &Command::u32(size_t at, uint32_t v) {
  for (int i = 0; i < 4; i++)
    this->bytes[at + i] = static_cast<uint8_t>(v >> (8 * i));
  this->seal_();
  return *this;
}

Command &Command::response(uint16_t n) { return this->u16(this->is_long() ? 16 : 12, n); }

Command vdcmd(uint32_t id, uint16_t response_bytes) {
  Command c;
  c.size = (id & 0xFF) == 0x03 ? LONG_HEADER : STANDARD_HEADER;
  c.bytes[0] = static_cast<uint8_t>(id);
  c.bytes[1] = static_cast<uint8_t>(id >> 8);
  c.bytes[2] = static_cast<uint8_t>(id >> 16);
  c.response(response_bytes);
  return c;
}

Command make_command(uint16_t type, uint16_t param, uint16_t reg, uint16_t response_bytes) {
  Command c;
  c.bytes[0] = static_cast<uint8_t>(type >> 8);
  c.bytes[1] = static_cast<uint8_t>(type);
  c.bytes[2] = static_cast<uint8_t>(param);
  c.bytes[3] = static_cast<uint8_t>(param >> 8);
  c.u16(4, reg);
  c.response(response_bytes);
  return c;
}

// The time the library allows the slow commands.
static constexpr uint32_t RANGE_BUDGET_MS = 22000;
static constexpr uint32_t SHUTTER_BUDGET_MS = 4000;
static constexpr uint32_t RECAL_BUDGET_MS = DEFAULT_BUDGET_MS + 5000;

Command command(Cmd c) {
  switch (c) {
    case Cmd::READ_MODEL:
      return info_command(Info::NAME);
    case Cmd::READ_FIRMWARE:
      return info_command(Info::FIRMWARE);
    case Cmd::READ_SYSTEM_VERSION:
      return info_command(Info::SYSTEM_VERSION);
    case Cmd::READ_RANGE:
      return make_command(0x012F, 0x81, 0, 1);
    case Cmd::RANGE_HIGH:
      return make_command(0x012F, 0x41, 1, 0).budget(RANGE_BUDGET_MS);
    case Cmd::SHUTTER:
      return make_command(0x0136, 0x43, 0, 0).budget(SHUTTER_BUDGET_MS);
  }
  return Command{};
}

// ---------------------------------------------------------------- settings

// libircmd020's commands (Thermal Master 2.3.11), for device type 15.
static const SettingInfo SETTINGS[static_cast<size_t>(Setting::COUNT)] = {
    {"range", 0x412F01, Place::B4, 0, 0x812F01, 0, 0, 0, 1, RANGE_BUDGET_MS, false},
    {"palette", 0x450310, Place::B5, 0, 0x850310, 0, 0, 0, 255, 0, false},
    {"brightness", 0x470410, Place::B4, 0, 0x870410, 1, 0, 0, 100, 0, false},
    {"contrast", 0x480410, Place::B4, 0, 0x880410, 1, 0, 0, 100, 0, false},
    {"global_contrast", 0x4A0410, Place::B4, 0, 0x8A0410, 1, 0, 0, 100, 0, false},
    {"detail_enhance", 0x450410, Place::B4, 0, 0x850410, 1, 0, 0, 100, 0, false},
    {"noise_reduction", 0x440410, Place::B4, 0, 0x840410, 1, 0, 0, 100, 0, false},
    {"spatial_noise_reduction", 0x4B0410, Place::B4, 0, 0x8B0410, 1, 0, 0, 100, 0, false},
    {"temporal_noise_reduction", 0x4C0410, Place::B4, 0, 0x8C0410, 1, 0, 0, 100, 0, false},
    {"gamma", 0x4D0410, Place::B4, 0, 0x8D0410, 1, 0, 0, 100, 0, false},
    {"edge_enhance", 0x4E0410, Place::B4, 0, 0x8E0410, 0, 0, 0, 2, 0, false},
    {"roi_level", 0x460410, Place::B4, 0, 0x860410, 1, 0, 0, 2, 0, false},
    {"profession_mode", 0x500410, Place::B4, 0, 0, 0, 0, 0, 1, 0, false},
    {"mirror_flip", 0x431010, Place::B4, 0, 0x831010, 0, 0, 0, 3, 0, false},
    {"isothermal_mode", 0x460310, Place::B5, 0, 0x860310, 0, 0, 0, 5, 0, false},
    {"isothermal_limit", 0x470310, Place::S16_PAIR_B8, 0, 0x870310, 0, 0, -70, 650, 0, false},
    {"isothermal", 0x480310, Place::B5, 0, 0x880310, 0, 0, 0, 1, DEFAULT_BUDGET_MS + 5000, false},
    {"sun_detect", 0x4B0310, Place::B4, 0, 0x8B0310, 0, 0, 0, 1, 0, false},
    {"sun_detect_pixel_ratio", 0x490310, Place::PAIR_B4, 0, 0x890310, 0, 0, 0, 100, 0, false},
    {"sun_detect_roundness", 0x4A0310, Place::PAIR_B4, 0, 0x8A0310, 0, 0, 0, 100, 0, false},
    {"overexposure_protection", 0x4C0310, Place::B4, 0, 0x8D0310, 0, 0, 0, 2, 0, false},
    {"auto_shutter", 0x410210, Place::B4, 0, 0x810210, 0, 0, 0, 1, 0, false},
    {"auto_shutter_threshold", 0x420210, Place::U16_B5, 0, 0x820210, 0, 1, 0, 65535, 0, false},
    {"auto_shutter_min_interval", 0x420210, Place::U16_B5, 1, 0x820210, 1, 1, 0, 65535, 0, false},
    {"auto_shutter_max_interval", 0x420210, Place::U16_B5, 2, 0x820210, 2, 1, 5, 65535, 0, false},
    {"shutter_open", 0x440210, Place::B4, 0, 0x830210, 0, 0, 0, 1, 0, false},
    {"env_correction", 0x460710, Place::B4, 0, 0x860710, 0, 0, 0, 1, 0, false},
    {"emissivity", 0x440710, Place::U16_B4, 0, 0x840710, 0, 0, 1, 16384, 0, false},
    {"ambient", 0x430710, Place::U16_B4, 0, 0x830710, 0, 0, 233, 373, 0, false},
    {"reflected", 0x420710, Place::U16_B4, 0, 0x820710, 0, 0, 233, 373, 0, false},
    {"transmittance", 0x450710, Place::U16_B4, 0, 0x850710, 0, 0, 1, 16384, 0, false},
    {"stream_mid_mode", 0x451010, Place::B4, 0, 0x851010, 0, 0, 0, 4, 0, false},
    {"palette_switch", 0x423501, Place::B5, 0, 0, 0, 0, 0, 1, 0, false},
    {"frame_temp_display", 0x510203, Place::B4, 0, 0, 0, 0, 0, 1, 0, false},
    {"cursor", 0x571110, Place::B4, 0, 0x811110, 0, 0, 0, 1, 0, true},
    {"nuc_bypass", 0x482601, Place::B4, 0, 0, 0, 0, 0, 1, 0, true},
};

const SettingInfo &setting_info(Setting s) { return SETTINGS[static_cast<size_t>(s)]; }

bool setting_is_pair(Setting s) {
  const Place p = setting_info(s).place;
  return p == Place::PAIR_B4 || p == Place::S16_PAIR_B8;
}

uint16_t setting_read_bytes(Setting s) {
  switch (setting_info(s).place) {
    case Place::B4:
    case Place::B5:
      return 1;
    case Place::U16_B4:
    case Place::U16_B5:
    case Place::PAIR_B4:
      return 2;
    case Place::S16_PAIR_B8:
      return 4;
  }
  return 1;
}

int32_t pack_pair(Setting s, int32_t first, int32_t second) {
  if (setting_info(s).place == Place::S16_PAIR_B8)
    return static_cast<int32_t>(static_cast<uint32_t>(static_cast<uint16_t>(first)) |
                                (static_cast<uint32_t>(static_cast<uint16_t>(second)) << 16));
  return (first & 0xFF) | ((second & 0xFF) << 8);
}

int32_t pair_part(Setting s, int32_t v, int part) {
  const uint32_t u = static_cast<uint32_t>(v);
  if (setting_info(s).place == Place::S16_PAIR_B8)
    return static_cast<int16_t>(part == 0 ? (u & 0xFFFF) : (u >> 16));
  return part == 0 ? (u & 0xFF) : ((u >> 8) & 0xFF);
}

int32_t setting_clamp(Setting s, int32_t v) {
  const SettingInfo &i = setting_info(s);
  auto clamp = [&i](int32_t x) { return std::max(i.min, std::min(i.max, x)); };
  if (setting_is_pair(s))
    return pack_pair(s, clamp(pair_part(s, v, 0)), clamp(pair_part(s, v, 1)));
  return clamp(v);
}

Command setting_write(Setting s, int32_t v) {
  const SettingInfo &i = setting_info(s);
  v = setting_clamp(s, v);
  Command c = vdcmd(i.set);
  switch (i.place) {
    case Place::B4:
      c.u8(4, static_cast<uint8_t>(v));
      break;
    case Place::B5:
      c.u8(5, static_cast<uint8_t>(v));
      break;
    case Place::U16_B4:
      c.u16(4, static_cast<uint16_t>(v));
      break;
    case Place::U16_B5:
      c.u8(4, i.arg).u16(5, static_cast<uint16_t>(v));
      break;
    case Place::PAIR_B4:
      c.u8(4, static_cast<uint8_t>(pair_part(s, v, 0))).u8(5, static_cast<uint8_t>(pair_part(s, v, 1)));
      break;
    case Place::S16_PAIR_B8:
      c.u16(8, static_cast<uint16_t>(pair_part(s, v, 0))).u16(10, static_cast<uint16_t>(pair_part(s, v, 1)));
      break;
  }
  if (i.budget_ms != 0)
    c.budget(i.budget_ms);
  return c;
}

Command setting_read(Setting s) {
  const SettingInfo &i = setting_info(s);
  return vdcmd(i.get, setting_read_bytes(s)).u8(4, i.get_b4).u8(5, i.get_b5);
}

int32_t setting_value(Setting s, const uint8_t *resp, size_t n) {
  if (setting_info(s).get == 0 || n < setting_read_bytes(s))
    return UNKNOWN;
  switch (setting_info(s).place) {
    case Place::B4:
    case Place::B5:
      return resp[0];
    case Place::U16_B4:
    case Place::U16_B5:
      return resp[0] | (resp[1] << 8);
    case Place::PAIR_B4:
      return pack_pair(s, resp[0], resp[1]);
    case Place::S16_PAIR_B8:
      return pack_pair(s, static_cast<int16_t>(resp[0] | (resp[1] << 8)),
                       static_cast<int16_t>(resp[2] | (resp[3] << 8)));
  }
  return UNKNOWN;
}

bool has_attributes(Setting s) {
  switch (s) {
    case Setting::BRIGHTNESS:
    case Setting::CONTRAST:
    case Setting::GLOBAL_CONTRAST:
    case Setting::NOISE_REDUCTION:
    case Setting::AUTO_SHUTTER_THRESHOLD:
    case Setting::AUTO_SHUTTER_MIN_INTERVAL:
    case Setting::AUTO_SHUTTER_MAX_INTERVAL:
      return true;
    default:
      return false;
  }
}

Command attribute_read(Setting s) {
  const SettingInfo &i = setting_info(s);
  Command c = vdcmd(i.get, 6);
  if (i.place == Place::U16_B5)
    return c.u8(4, i.arg).u8(5, 2);  // the parameter's type, then "attribute"
  return c.u8(4, 2);
}

bool parse_attributes(const uint8_t *resp, size_t n, Attributes *a) {
  if (n < 6)
    return false;
  a->max = static_cast<uint16_t>(resp[0] | (resp[1] << 8));
  a->min = static_cast<uint16_t>(resp[2] | (resp[3] << 8));
  a->step = static_cast<uint16_t>(resp[4] | (resp[5] << 8));
  return true;
}

// ---------------------------------------------------------------- actions

struct ActionInfo {
  const char *name;
  uint32_t id;
  uint8_t b4;
  bool maintenance;
  bool no_status;
  uint32_t budget_ms;
};
static const ActionInfo ACTIONS[static_cast<size_t>(Action::COUNT)] = {
    {"shutter", 0x433601, 0, false, false, SHUTTER_BUDGET_MS},
    {"shutter_ooc", 0x433601, 1, false, false, SHUTTER_BUDGET_MS},
    {"shutter_both", 0x433601, 2, false, false, SHUTTER_BUDGET_MS},
    {"ffc_update", 0x430210, 0, false, false, 0},
    {"ffc_without_shutter", 0x521001, 0, false, false, 0},
    {"heartbeat_start", 0x531010, 0, false, false, 0},
    {"heartbeat", 0x541010, 0, false, false, 0},
    {"reboot", 0x410101, 0, true, true, 0},
    {"save_settings", 0x511010, 0, true, false, 0},
    {"restore_settings", 0x521010, 0, true, true, 0},
    {"reset_to_rom", 0x420101, 0, true, true, 0},
    {"reset_to_bootloader", 0x470101, 0, true, true, 0},
    {"save_defaults", 0x480201, 0, true, false, 0},
    {"restore_calibration", 0x420610, 0, true, false, 0},
    {"ooc_save", 0x490F01, 0, true, false, 0},
    {"k_calibrate_low", 0x411110, 0, true, false, 0},
    {"k_calibrate_high", 0x411110, 1, true, false, 0},
    {"k_calculate", 0x411110, 2, true, false, 0},
    {"k_cancel", 0x421110, 0, true, false, 0},
    {"k_clear", 0x441110, 0, true, false, 0},
    {"k_save", 0x431110, 0, true, false, 0},
    {"k_restore", 0x451110, 0, true, false, 0},
    {"dpc_auto", 0x511110, 0, true, false, DEFAULT_BUDGET_MS + 10000},
    {"dpc_cancel", 0x531110, 0, true, false, 0},
    {"dpc_clear", 0x551110, 0, true, false, 0},
    {"dpc_save", 0x541110, 0, true, false, 0},
    {"dpc_restore", 0x561110, 0, true, false, 0},
    {"rmcover_auto", 0x611110, 0, true, false, DEFAULT_BUDGET_MS + 30000},
    {"rmcover_cancel", 0x621110, 0, true, false, 0},
    {"rmcover_clear", 0x641110, 0, true, false, 0},
    {"rmcover_save", 0x631110, 0, true, false, 0},
    {"rmcover_restore", 0x651110, 0, true, false, 0},
    {"tpd1_cancel", 0x721110, 0, true, false, 0},
    {"tpd1_clear", 0x741110, 0, true, false, 0},
    {"tpd1_save", 0x731110, 0, true, false, 0},
    {"tpd_restore", 0x751110, 0, true, false, 0},
    {"tpd2_cancel", 0x771110, 0, true, false, 0},
    {"tpd2_clear", 0x791110, 0, true, false, 0},
    {"tpd2_save", 0x781110, 0, true, false, 0},
    {"tpd2_restore", 0x7A1110, 0, true, false, 0},
    {"crg_save", 0x681010, 0, true, false, 0},
};

const char *action_name(Action a) { return ACTIONS[static_cast<size_t>(a)].name; }
bool action_is_maintenance(Action a) { return ACTIONS[static_cast<size_t>(a)].maintenance; }

Command action_command(Action a) {
  const ActionInfo &i = ACTIONS[static_cast<size_t>(a)];
  Command c = vdcmd(i.id).u8(4, i.b4);
  if (i.no_status)
    c.without_status();
  if (i.budget_ms != 0)
    c.budget(i.budget_ms);
  return c;
}

Command cursor_position(uint16_t x, uint16_t y) { return vdcmd(0x581110).u16(4, x).u16(6, y); }
Command mark_dead_pixel(uint16_t x, uint16_t y) { return vdcmd(0x521110).u16(4, x).u16(6, y); }
Command unmark_dead_pixel(uint16_t x, uint16_t y) { return vdcmd(0x521110).u8(3, 1).u16(4, x).u16(6, y); }
Command tpd_one_point(int16_t reference) {
  return vdcmd(0x711110).u16(4, static_cast<uint16_t>(reference)).budget(RECAL_BUDGET_MS);
}
Command tpd_two_point(uint8_t point, int16_t reference) {
  return vdcmd(0x761110).u8(3, 3).u16(4, static_cast<uint16_t>(reference)).u8(8, point).budget(RECAL_BUDGET_MS);
}
Command vl_write(uint32_t reg, uint32_t v) { return vdcmd(0x5C1010).u32(4, reg).u32(8, v); }
Command vl_read(uint32_t reg) { return vdcmd(0x9C1010, 1).u32(4, reg); }
Command crg_write(uint32_t v) { return vdcmd(0x661010).u32(4, v); }

// ---------------------------------------------------------------- readings

Command reading_command(Reading r, uint16_t x, uint16_t y) {
  switch (r) {
    case Reading::DEVICE_TEMPERATURE:
      return vdcmd(0x911010, 2);
    case Reading::POWERED_TIME:
      return vdcmd(0x931010, 4);
    case Reading::DEVICE_STATUS:
      return vdcmd(0x820101, 1);
    case Reading::SHUTTER_STATUS:
      return vdcmd(0x830210, 1);
    case Reading::CRG_VALUE:
      return vdcmd(0xA61010, 1);
    case Reading::SENSOR_VTEMP:
      return vdcmd(0x860F01, 2);  // b4 = 0: the IR sensor's VTEMP
    case Reading::STREAM_SOURCE:
      return vdcmd(0x812C01, 1);
    case Reading::DIGITAL_VIDEO_OUTPUT:
      return vdcmd(0x861010, 3);
    case Reading::SYSTEM_DATA_FLAG:
      return vdcmd(0x812110, 2);
    case Reading::CURSOR_POSITION:
      return vdcmd(0x821110, 4);
    case Reading::CAMERA_POINT:
      return vdcmd(0x820203, 2).u16(4, x).u16(6, y);
    case Reading::CAMERA_FRAME:
      return vdcmd(0x810203, 12);
    case Reading::GAIN_SWITCH_STATE:
      return vdcmd(0x830101, 1);
    case Reading::COUNT:
      break;
  }
  return Command{};
}

const char *device_status_name(uint8_t raw) {
  switch (raw) {
    case 0:
    case 1:
    case 5:
      return "Start-up";
    case 2:
      return "Preview";
    case 6:
      return "Video out";
    case 3:
      return "Firmware update";
    default:
      return "Error";
  }
}

const char *shutter_status_name(uint8_t raw) {
  switch (raw) {
    case 0:
      return "Closed";
    case 1:
      return "Open";
    default:
      return "Unknown";
  }
}

const char *stream_source_name(uint8_t raw) {
  switch (raw) {
    case 0x00:
      return "IR";
    case 0x01:
      return "KBC";
    case 0x03:
      return "TNR";
    case 0x04:
      return "HBC/DPC";
    case 0x05:
      return "VBC";
    case 0x0A:
      return "SNR";
    case 0x13:
      return "DDE";
    case 0x10:
      return "AGC";
    case 0x15:
      return "Gamma";
    case 0x19:
      return "TPD";
    case 0x17:
    case 0x1A:
      return "Mirror";
    case 0x16:
      return "Picture";
    default:
      return "Unknown";
  }
}

const char *video_format_name(uint8_t v) {
  static const char *const NAMES[] = {"USB", "DVP", "BT656", "MIPI", "BT656 interlaced", "NTSC", "PAL", "SPI"};
  return v < 8 ? NAMES[v] : "Unknown";
}

bool parse_frame_info(const uint8_t *resp, size_t n, FrameInfo *f) {
  if (n < 12)
    return false;
  auto u16 = [resp](size_t i) { return static_cast<uint16_t>(resp[i] | (resp[i + 1] << 8)); };
  // The library returns {T(b2-3), T(b0-1), point(b8-11), point(b4-7)}: two
  // temperatures, each with its point. Which is the maximum is not named; the
  // larger is.
  const float a = raw_to_celsius(u16(2)), b = raw_to_celsius(u16(0));
  const bool a_max = a >= b;
  f->max_c = a_max ? a : b;
  f->min_c = a_max ? b : a;
  f->max_x = a_max ? u16(8) : u16(4);
  f->max_y = a_max ? u16(10) : u16(6);
  f->min_x = a_max ? u16(4) : u16(8);
  f->min_y = a_max ? u16(6) : u16(10);
  return true;
}

Command info_command(Info i) {
  static const uint16_t BYTES[] = {0, 30, 12, 10, 16, 16, 64, 64, 64, 64, 64, 64, 64, 50, 64, 64};
  const uint8_t t = static_cast<uint8_t>(i);
  return make_command(0x0101, 0x81, t, t < 16 ? BYTES[t] : 64);
}

Command palette_count_command() { return vdcmd(0x833201, 1).u8(5, 0xFF); }
Command palette_name_command(uint8_t index) { return vdcmd(0x833201, 10).u8(5, index); }

// ---------------------------------------------------------------- markers

Marker parse_marker(const uint8_t *p) {
  Marker m;
  m.length = p[0];
  m.sync = p[1];
  std::memcpy(&m.frame, p + 2, 4);
  std::memcpy(&m.time, p + 6, 4);
  std::memcpy(&m.phase, p + 10, 2);
  return m;
}

bool is_start_marker(const Marker &m) { return m.length == MARKER_BYTES && (m.sync & 0xFE) == 0x8C; }
bool is_end_marker(const Marker &m) { return m.length == MARKER_BYTES && (m.sync & 0xFE) == 0x8E; }

void Assembler::reset() {
  this->bytes_ = 0;
  this->overflow_ = false;
  this->have_phase_ = false;
}

void Assembler::count_skips_(uint16_t phase) {
  if (this->have_phase_) {
    const uint16_t d = static_cast<uint16_t>(phase - this->last_phase_) & 2047;
    if (d != 0) {
      if (this->step_ == 0 || d < this->step_)
        this->step_ = d;
      if (d >= this->step_ + this->step_ / 2)
        this->skipped_ += (d + this->step_ / 2) / this->step_ - 1;
    }
  }
  this->have_phase_ = true;
  this->last_phase_ = phase;
}

Assembler::Result Assembler::feed(const uint8_t *data, size_t n, uint8_t *dst) {
  if (n == MARKER_BYTES) {
    // An end marker: a frame is whole only if it comes just after one's rows.
    const size_t had = this->bytes_;
    const bool overflow = this->overflow_;
    this->bytes_ = 0;
    this->overflow_ = false;
    if (overflow)
      return Result::PART;  // the long frame was reported when it overflowed
    if (had != BODY_BYTES)
      return had == 0 ? Result::PART : Result::SHORT;
    if (dst == nullptr)
      return Result::NO_BUFFER;
    std::memcpy(dst + BODY_BYTES, data, n);
    const Marker start = parse_marker(dst), end = parse_marker(dst + BODY_BYTES);
    if (!is_start_marker(start) || !is_end_marker(end) || start.frame != end.frame ||
        (start.sync & 1) != (end.sync & 1))
      return Result::BAD_MARKER;
    this->count_skips_(start.phase);
    return Result::FRAME;
  }
  if (this->overflow_)
    return Result::PART;
  if (this->bytes_ + n > BODY_BYTES) {
    this->bytes_ = 0;
    this->overflow_ = true;
    return Result::LONG;
  }
  if (dst != nullptr)
    std::memcpy(dst + this->bytes_, data, n);
  this->bytes_ += n;
  return Result::PART;
}

// ---------------------------------------------------------------- temperatures

uint32_t decode(const uint8_t *frame, float gain, float offset, float *temps, video::FrameStats *stats,
                bool low_range) {
  float first = NAN;
  if (low_range) {
    first = LOW_RANGE_FLOOR_C * gain + offset;
  } else {
    // A value for pixels without a reading at the top left: the first reading.
    for (int i = 0; i < PIXELS && std::isnan(first); i++) {
      const uint16_t r = raw_temperature(frame, i % W, i / W);
      if (r >= MIN_VALID_RAW)
        first = raw_to_celsius(r) * gain + offset;
    }
    if (std::isnan(first)) {
      if (stats != nullptr)
        stats->valid = false;
      return PIXELS;
    }
  }
  uint32_t bad = 0;
  float lo = INFINITY, hi = -INFINITY;
  double sum = 0;
  int lo_i = 0, hi_i = 0;
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      const int i = y * W + x;
      const uint16_t r = raw_temperature(frame, x, y);
      float t;
      if (low_range) {
        if (r < LOW_RANGE_FLOOR_RAW) {
          bad++;
          t = first;
        } else {
          t = raw_to_celsius(r) * gain + offset;
        }
      } else if (r >= MIN_VALID_RAW) {
        t = raw_to_celsius(r) * gain + offset;
      } else {
        bad++;
        t = x > 0 ? temps[i - 1] : (y > 0 ? temps[i - W] : first);
      }
      temps[i] = t;
      sum += t;
      if (t < lo) {
        lo = t;
        lo_i = i;
      }
      if (t > hi) {
        hi = t;
        hi_i = i;
      }
    }
  }
  if (stats != nullptr) {
    stats->valid = std::isfinite(lo) && std::isfinite(hi);
    stats->min = lo;
    stats->max = hi;
    stats->mean = static_cast<float>(sum / PIXELS);
    stats->min_x = static_cast<uint16_t>(lo_i % W);
    stats->min_y = static_cast<uint16_t>(lo_i / W);
    stats->max_x = static_cast<uint16_t>(hi_i % W);
    stats->max_y = static_cast<uint16_t>(hi_i / W);
  }
  return bad;
}

static inline uint8_t clamp8(int v) { return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); }

void picture_to_rgb565(const uint8_t *frame, uint8_t *dst, size_t dst_stride) {
  // Full-range BT.601, 8.8 fixed point: R = Y + 1.402 V', G = Y - 0.344 U' - 0.714 V', B = Y + 1.772 U'.
  for (int y = 0; y < H; y++) {
    const uint8_t *s = frame + MARKER_BYTES + static_cast<size_t>(y) * W * 2;
    uint8_t *d = dst + static_cast<size_t>(y) * dst_stride;
    for (int x = 0; x + 1 < W; x += 2, s += 4) {
      const int u = s[1] - 128, v = s[3] - 128;
      const int rv = (359 * v + 128) >> 8;
      const int guv = (88 * u + 183 * v + 128) >> 8;
      const int bu = (454 * u + 128) >> 8;
      for (int k = 0; k < 2; k++) {
        const int yy = s[k * 2];
        const uint8_t r = clamp8(yy + rv), g = clamp8(yy - guv), b = clamp8(yy + bu);
        const uint16_t c = static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        *d++ = static_cast<uint8_t>(c);
        *d++ = static_cast<uint8_t>(c >> 8);
      }
    }
  }
}

InfoRows parse_info_rows(const uint8_t *frame) {
  const uint8_t *p = info_bytes(frame);
  InfoRows r;
  r.ffc_countdown = p[42];
  r.sun_protect = p[82];
  r.hw_sun_protect = p[138];
  r.frame_count = static_cast<uint16_t>(p[128] | (p[129] << 8));
  r.frame_count2 = static_cast<uint16_t>(p[360] | (p[361] << 8));
  return r;
}

// ---------------------------------------------------------------- automatic range

AutoRange::Vote AutoRange::vote(const uint8_t *frame) {
  uint32_t under = 0, over = 0;
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      if (raw_temperature(frame, x, y) < THRESHOLD_RAW)
        under++;
      else
        over++;
    }
  if (static_cast<float>(under) > 0.95f * PIXELS)
    return HIGH;
  if (static_cast<float>(over) > 0.10f * PIXELS)
    return LOW;
  return NONE;
}

void AutoRange::reset() {
  this->low_votes_ = this->high_votes_ = 0;
  this->waiting_ = false;
  this->waited_ = 0;
}

int AutoRange::feed(const uint8_t *frame, int current) {
  if (this->waiting_) {
    if (this->waited_++ >= WAIT_FRAMES) {
      this->waiting_ = false;
      this->waited_ = 0;
    }
    return -1;
  }
  switch (vote(frame)) {
    case LOW:
      this->low_votes_++;
      this->high_votes_ = 0;
      break;
    case HIGH:
      this->high_votes_++;
      this->low_votes_ = 0;
      break;
    case NONE:
      this->low_votes_ = this->high_votes_ = 0;
      break;
  }
  if (this->low_votes_ <= SWITCH_FRAMES && this->high_votes_ <= SWITCH_FRAMES)
    return -1;
  const int want = this->low_votes_ > SWITCH_FRAMES ? 0 : 1;
  this->low_votes_ = this->high_votes_ = 0;
  this->waiting_ = true;
  this->waited_ = 0;
  return want != current ? want : -1;
}

}  // namespace thermal_master_p1
}  // namespace esphome
