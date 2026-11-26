#pragma once

// Thermal Master P1 (USB 3474:45C2), 160 x 120: the commands, the frame
// markers, frame assembly from bulk transfers, temperatures, the metadata
// rows, and the vendor's automatic range switching. No ESP-IDF here, so the
// unit tests build it on a PC.
//
// Sources:
// - The vendor's Android app, Thermal Master 2.3.11 (com.thermalmaster.p2telephoto),
//   decompiled: its command library (libircmd020), its USB stream code
//   (libomniircamera020) and its Java. The command set, how each command's
//   values are placed, the status handshake and the time each command may
//   take are the library's. The library puts the P1 in its device type 15
//   with the P3.
// - The open drivers that ran on a P1 (jvdillon/p3-ir-camera, Apache-2.0, from
//   captures of the vendor's Windows tool; Jozo132/Open-TM-P1): the start-up
//   sequence and the frame layout as seen on the wire.
//
// A frame, as the camera sends it on bulk endpoint 0x81:
//   12-byte start marker (a UVC payload header)
//   rows 0..119     the camera's own picture, YUYV
//   rows 120..121   metadata (640 bytes; see InfoRows)
//   rows 122..241   temperatures, uint16 little-endian, 1/64 kelvin
//   12-byte end marker, always a transfer of its own
// The start marker and the 77,440 bytes of rows end with a short packet, so a
// frame completes as a 12-byte transfer that comes just when start marker and
// rows are in. A 12-byte transfer anywhere else is the end of a frame joined
// part way (at start-up, or after the longer frame a shutter cycle makes).

#include <array>
#include <climits>
#include <cstddef>
#include <cstdint>

#include "../video_core/frame.h"

namespace esphome {
namespace thermal_master_p1 {

static constexpr uint16_t VID = 0x3474;
static constexpr uint16_t PID_P1 = 0x45C2;

static constexpr int W = 160, H = 120;
static constexpr int PIXELS = W * H;
static constexpr int ROWS = 2 * H + 2;  // picture, 2 metadata rows, temperatures
static constexpr int META_ROW = H;
static constexpr int TEMP_ROW = H + 2;
static constexpr size_t MARKER_BYTES = 12;
static constexpr size_t ROWS_BYTES = 2 * static_cast<size_t>(ROWS) * W;  // 77,440
static constexpr size_t BODY_BYTES = MARKER_BYTES + ROWS_BYTES;          // what comes before the end marker
static constexpr size_t FRAME_BYTES = BODY_BYTES + MARKER_BYTES;         // 77,464
static constexpr size_t INFO_BYTES = 2 * 2 * static_cast<size_t>(W);    // the metadata rows, 640
static constexpr float RAW_PER_KELVIN = 64.0f;
// In the high range (-20..150 C), below 200 K (-73 C) a value is not a
// reading. Such pixels take a neighbour's value and are counted.
static constexpr uint16_t MIN_VALID_RAW = 200 * 64;
// In the low range (the vendor app's 150..600 C), values under 150 C are not
// measurements (the app shows "<150 C"; at room temperature they read about
// 57 K). Such pixels read 150 C and are counted.
static constexpr uint16_t LOW_RANGE_FLOOR_RAW = 27082;  // 150 C: (150 + 273.15) * 64
static constexpr float LOW_RANGE_FLOOR_C = 150.0f;

// ---------------------------------------------------------------- commands
// Sent with a vendor control request (0x41, 0x20). The vendor's id 0xAABBCC is
// bytes b0 = 0xCC, b1 = 0xBB (the group), b2 = 0xAA (the command; bit 7: a
// data phase follows). b0 picks the header: 0x01 or 0x10 an 18-byte one
// (response length at b12-13, CRC-16/XMODEM of b0..15 at b16-17), 0x03 a
// 24-byte one (the camera's temperature measurements: length at b16-17, CRC
// of b0..21 at b22-23). Values go in b3..b11 as each command has them.
//
// The handshake (the vendor library's): the command, then
//   a write  the 1-byte status (0xC1, 0x22) until it is not 1 (busy), about
//            every millisecond for up to `budget_ms`; it must end at 3.
//   a read   the status, which must be 2; the response (0xC1, 0x21), exactly
//            the length the command gives; the status again, which must be 3.
//   a reset  nothing (the camera restarts).
static constexpr size_t STANDARD_HEADER = 18;
static constexpr size_t LONG_HEADER = 24;
static constexpr uint32_t DEFAULT_BUDGET_MS = 2000;
static constexpr uint8_t STATUS_BUSY = 1, STATUS_READY = 2, STATUS_DONE = 3;

struct Command {
  std::array<uint8_t, LONG_HEADER> bytes{};
  uint8_t size{STANDARD_HEADER};
  bool no_status{false};                  // sent without a status read: the camera resets
  uint32_t budget_ms{DEFAULT_BUDGET_MS};  // how long the camera may take to finish it

  const uint8_t *data() const { return this->bytes.data(); }
  uint8_t operator[](size_t i) const { return this->bytes[i]; }
  // The same bytes on the wire.
  bool operator==(const Command &o) const { return this->size == o.size && this->bytes == o.bytes; }
  bool operator!=(const Command &o) const { return !(*this == o); }
  bool is_long() const { return this->size == LONG_HEADER; }
  uint16_t response_bytes() const;

  // Values; each keeps the CRC right.
  Command &u8(size_t at, uint8_t v);
  Command &u16(size_t at, uint16_t v);
  Command &u32(size_t at, uint32_t v);
  Command &response(uint16_t n);
  Command &budget(uint32_t ms) {
    this->budget_ms = ms;
    return *this;
  }
  Command &without_status() {
    this->no_status = true;
    return *this;
  }

 protected:
  void seal_();
};

uint16_t crc16_xmodem(const uint8_t *data, size_t n);
// The vendor's id with a response length; the header from the id.
Command vdcmd(uint32_t id, uint16_t response_bytes = 0);
// As the captures write them: type (b0-1, big-endian), parameter (b2-3),
// register (b4-5), response length.
Command make_command(uint16_t type, uint16_t param, uint16_t reg, uint16_t response_bytes);

// The start-up sequence's commands, as the vendor's Windows tool sends them.
enum class Cmd : uint8_t {
  READ_MODEL,           // device information 1 (the name), 30 bytes: "P1"
  READ_FIRMWARE,        // device information 2, 12 bytes
  READ_SYSTEM_VERSION,  // device information 10, 64 bytes
  READ_RANGE,           // 1 byte: 1 the high range. The Windows tool sends it as part of the start (the
                        // open drivers' "start stream", with b4 = 1); the vendor's library has it as the
                        // range read, with b4 = 0.
  RANGE_HIGH,           // the -20..150 C range
  SHUTTER,              // a shutter cycle, the offset (B) updated: the app's shutter button
};
Command command(Cmd c);

// ---------------------------------------------------------------- settings
// Written, and read back where the camera has a read for them.
static constexpr int32_t UNKNOWN = INT32_MIN;

enum class Setting : uint8_t {
  RANGE,                      // 0 low (150..600 C), 1 high (-20..150 C)
  PALETTE,                    // the camera's palette index (its own picture); names from palette_name_command
  BRIGHTNESS,                 // 0..100 (the camera's own picture)
  CONTRAST,                   // 0..100
  GLOBAL_CONTRAST,            // 0..100 (the vendor app's contrast slider)
  DETAIL_ENHANCE,             // 0..100
  NOISE_REDUCTION,            // 0..100
  SPATIAL_NOISE_REDUCTION,    // 0..100
  TEMPORAL_NOISE_REDUCTION,   // 0..100
  GAMMA,                      // 0..100
  EDGE_ENHANCE,               // 0..2
  ROI_LEVEL,                  // 0 off, 1 a third, 2 a half
  PROFESSION_MODE,            // 0 normal, 1 professional; no read (the library's read is edge enhancement's)
  MIRROR_FLIP,                // 0 none, 1 mirror, 2 flip, 3 both
  ISOTHERMAL_MODE,            // 0..5
  ISOTHERMAL_LIMIT,           // low and high, whole degrees C (a pair)
  ISOTHERMAL,                 // switch
  SUN_DETECT,                 // switch: the camera flags the sun (metadata byte 138)
  SUN_DETECT_PIXEL_RATIO,     // min and max, 0..100 (a pair)
  SUN_DETECT_ROUNDNESS,       // min and max, 0..100 (a pair)
  OVEREXPOSURE_PROTECTION,    // 0..2 (the library also calls this command "all FFC functions":
                              // 0 available, 1 unavailable, 2 cancel)
  AUTO_SHUTTER,               // switch: the camera's own shutter cycles
  AUTO_SHUTTER_THRESHOLD,     // the camera's auto shutter parameters (units not given)
  AUTO_SHUTTER_MIN_INTERVAL,
  AUTO_SHUTTER_MAX_INTERVAL,  // more than 4
  SHUTTER_OPEN,               // 0 close the shutter, 1 open it; reads 0 closed, 1 open, 2 unknown
  ENV_CORRECTION,             // switch: the camera's environment correction
  EMISSIVITY,                 // 1..16384 = 1/16384..1
  AMBIENT,                    // TA, kelvin (C + 273), 233..373
  REFLECTED,                  // TU, kelvin (C + 273), 233..373
  TRANSMITTANCE,              // TAU, 1..16384 = 1/16384..1
  STREAM_MID_MODE,            // 0 picture, 1 TPD, 2 LCE, 3 IR, 4 SNR
  PALETTE_SWITCH,             // the library's palette switch; no read
  FRAME_TEMP_DISPLAY,         // the camera's own frame temperature display; no read
  CURSOR,                     // switch: the calibration cursor (maintenance)
  NUC_BYPASS,                 // switch: the non-uniformity correction bypassed (maintenance); no read
  COUNT
};

// Where a setting's value goes.
enum class Place : uint8_t {
  B4,           // one byte at b4
  B5,           // one byte at b5
  U16_B4,       // 16 bits at b4-5
  U16_B5,       // 16 bits at b5-6, after `arg` at b4
  PAIR_B4,      // two bytes at b4 and b5; reads 2 bytes in the same order
  S16_PAIR_B8,  // two signed 16-bit values at b8-9 and b10-11; reads 4 bytes in the same order
};

struct SettingInfo {
  const char *name;
  uint32_t set;
  Place place;
  uint8_t arg;      // U16_B5: the byte at b4
  uint32_t get;     // 0: no read
  uint8_t get_b4;   // the read's b4 (1: "current", for the picture levels and the auto shutter parameters' type)
  uint8_t get_b5;   // the read's b5
  int32_t min, max; // for a pair, each value's
  uint32_t budget_ms;
  bool maintenance;
};
const SettingInfo &setting_info(Setting s);
bool setting_is_pair(Setting s);
uint16_t setting_read_bytes(Setting s);
// A pair's two values in one int32: first | second << 8 (bytes) or << 16 (16 bits).
int32_t pack_pair(Setting s, int32_t first, int32_t second);
int32_t pair_part(Setting s, int32_t v, int part);
// `v` clamped (each value of a pair) to what the setting takes.
int32_t setting_clamp(Setting s, int32_t v);
Command setting_write(Setting s, int32_t v);
Command setting_read(Setting s);
// The value in a read's response, or UNKNOWN.
int32_t setting_value(Setting s, const uint8_t *resp, size_t n);
// The camera's limits for a setting (max, min, step), where it has them: the
// picture levels and the auto shutter parameters.
bool has_attributes(Setting s);
Command attribute_read(Setting s);
struct Attributes {
  uint16_t max, min, step;
};
bool parse_attributes(const uint8_t *resp, size_t n, Attributes *a);

// ---------------------------------------------------------------- actions
// Commands with no value and no response.
enum class Action : uint8_t {
  SHUTTER,              // a shutter cycle; the offset (B) updated (the vendor app's shutter button)
  SHUTTER_OOC,          // a shutter cycle; the OOC updated
  SHUTTER_BOTH,         // a shutter cycle; both updated
  FFC_UPDATE,           // the library's flat-field update
  FFC_WITHOUT_SHUTTER,  // a flat-field correction without the shutter
  HEARTBEAT_START,      // the library's keepalive (the same bytes as its "stream permission off")
  HEARTBEAT,
  // Maintenance: they change or wipe what the camera keeps.
  REBOOT,               // the vendor app's reboot (also "reset to normal" from the bootloader); no status
  SAVE_SETTINGS,        // the camera's settings into its flash
  RESTORE_SETTINGS,     // its stored defaults; no status
  RESET_TO_ROM,         // no status
  RESET_TO_BOOTLOADER,  // the firmware update mode; no status
  SAVE_DEFAULTS,        // the library's "save default parameters"
  RESTORE_CALIBRATION,  // all recalibration data restored (the library's second calibration reset)
  OOC_SAVE,             // the OOC data into flash
  K_CALIBRATE_LOW,      // K (pixel gain) calibration: a low-temperature object in view
  K_CALIBRATE_HIGH,     // a high-temperature object in view
  K_CALCULATE,
  K_CANCEL,
  K_CLEAR,
  K_SAVE,
  K_RESTORE,
  DPC_AUTO,             // dead-pixel calibration
  DPC_CANCEL,
  DPC_CLEAR,
  DPC_SAVE,
  DPC_RESTORE,
  RMCOVER_AUTO,         // the library's "rmcover" calibration
  RMCOVER_CANCEL,
  RMCOVER_CLEAR,
  RMCOVER_SAVE,
  RMCOVER_RESTORE,
  TPD1_CANCEL,          // temperature (TPD) recalibration, one and two points
  TPD1_CLEAR,
  TPD1_SAVE,
  TPD_RESTORE,
  TPD2_CANCEL,
  TPD2_CLEAR,
  TPD2_SAVE,
  TPD2_RESTORE,
  CRG_SAVE,
  COUNT
};
const char *action_name(Action a);
bool action_is_maintenance(Action a);
Command action_command(Action a);

// Commands with their own values (maintenance).
Command cursor_position(uint16_t x, uint16_t y);
Command mark_dead_pixel(uint16_t x, uint16_t y);    // the pixel under (x, y) is dead
Command unmark_dead_pixel(uint16_t x, uint16_t y);  // it is not
Command tpd_one_point(int16_t reference);           // the library passes the value unscaled, as a whole number
Command tpd_two_point(uint8_t point, int16_t reference);  // point 0 or 1
Command vl_write(uint32_t reg, uint32_t v);         // the library's "VL parameter"
Command vl_read(uint32_t reg);                      // 1 byte
Command crg_write(uint32_t v);

// ---------------------------------------------------------------- readings
enum class Reading : uint8_t {
  DEVICE_TEMPERATURE,    // 2 bytes, 0.01 C
  POWERED_TIME,          // 4 bytes
  DEVICE_STATUS,         // 1 byte, device_status_name
  SHUTTER_STATUS,        // 1 byte, shutter_status_name
  CRG_VALUE,             // 1 byte
  SENSOR_VTEMP,          // 2 bytes: the sensor's VTEMP, raw
  STREAM_SOURCE,         // 1 byte, stream_source_name
  DIGITAL_VIDEO_OUTPUT,  // 3 bytes: on, format, frames/s
  SYSTEM_DATA_FLAG,      // 2 bytes: the system configuration's update flag
  CURSOR_POSITION,       // 4 bytes: x, y
  CAMERA_POINT,          // 2 bytes: the camera's temperature at a point, 1/64 K
  CAMERA_FRAME,          // 12 bytes: the camera's frame minimum and maximum, parse_frame_info
  GAIN_SWITCH_STATE,     // 1 byte: 2 while a range change is under way
  COUNT
};
// `x`, `y`: CAMERA_POINT's point.
Command reading_command(Reading r, uint16_t x = W / 2, uint16_t y = H / 2);
const char *device_status_name(uint8_t raw);
const char *shutter_status_name(uint8_t raw);
const char *stream_source_name(uint8_t raw);
const char *video_format_name(uint8_t v);

struct FrameInfo {
  float max_c, min_c;
  uint16_t max_x, max_y, min_x, min_y;
};
bool parse_frame_info(const uint8_t *resp, size_t n, FrameInfo *f);

// Device information: a string per type.
enum class Info : uint8_t {
  NAME = 1,
  FIRMWARE,
  CUSTOMER_ID,
  VENDOR_ID,
  PRODUCT_ID,
  PART_NUMBER,
  SERIAL,
  ISP_VERSION,
  ABSTRACT_VERSION,
  SYSTEM_VERSION,
  CALIBRATION_VERSION,
  DATA_VERSION,
  PACKAGE_VERSION,
  SENSOR_TYPE,
  SENSOR_ID,
  LAST = SENSOR_ID
};
Command info_command(Info i);

// The camera's palettes: how many, and each one's name (10 bytes).
Command palette_count_command();
Command palette_name_command(uint8_t index);

// ---------------------------------------------------------------- markers
struct Marker {
  uint8_t length{0};
  uint8_t sync{0};    // start 0x8C/0x8D, end 0x8E/0x8F (alternating by frame)
  uint32_t frame{0};  // the same in a frame's start and end marker
  uint32_t time{0};
  uint16_t phase{0};  // steps by about 40 a frame, wraps at 2048
};
Marker parse_marker(const uint8_t *p);
bool is_start_marker(const Marker &m);
bool is_end_marker(const Marker &m);

// ---------------------------------------------------------------- assembly
// Fed every completed bulk transfer, in order. Copies the bytes into `dst`
// (FRAME_BYTES, or null to follow the stream without keeping it) and says
// when a whole, well-marked frame is there.
class Assembler {
 public:
  enum class Result : uint8_t {
    PART,       // more to come
    FRAME,      // dst holds a frame; the next transfer starts a new one
    NO_BUFFER,  // a frame went by with no buffer to keep it
    SHORT,      // an end marker before the frame was whole (joined part way, or bytes lost)
    LONG,       // more bytes than a frame (the frame after a shutter cycle); waiting for an end marker
    BAD_MARKER  // the right size, but the markers do not match
  };
  Result feed(const uint8_t *data, size_t n, uint8_t *dst);
  void reset();
  size_t pending() const { return this->bytes_; }

  // Frames the camera made that never came, from the markers' phase.
  uint32_t camera_skipped() const { return this->skipped_; }

 protected:
  void count_skips_(uint16_t phase);
  size_t bytes_{0};
  bool overflow_{false};
  bool have_phase_{false};
  uint16_t last_phase_{0};
  uint16_t step_{0};  // smallest phase step seen: one frame
  uint32_t skipped_{0};
};

// ---------------------------------------------------------------- temperatures
// Pixel (x, y) of a whole frame's temperature rows, raw.
inline uint16_t raw_temperature(const uint8_t *frame, int x, int y) {
  const uint8_t *p = frame + MARKER_BYTES + 2 * (static_cast<size_t>(TEMP_ROW + y) * W + x);
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
// Word i (0 .. 2 * W - 1) of the metadata rows.
inline uint16_t metadata_word(const uint8_t *frame, int i) {
  const uint8_t *p = frame + MARKER_BYTES + 2 * (static_cast<size_t>(META_ROW) * W + i);
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}
inline const uint8_t *info_bytes(const uint8_t *frame) {
  return frame + MARKER_BYTES + 2 * static_cast<size_t>(META_ROW) * W;
}
inline float raw_to_celsius(uint16_t raw) { return raw / RAW_PER_KELVIN - 273.15f; }

// The frame's temperatures in degrees C, row-major, times gain plus offset.
// `low_range`: the camera is in its low range. Returns how many pixels held
// no reading (high range: they take a neighbour's value) or were under the
// low range (they read 150 C); in the high range PIXELS means none held a
// reading and `temps` is not usable.
uint32_t decode(const uint8_t *frame, float gain, float offset, float *temps, video::FrameStats *stats,
                bool low_range = false);

// The camera's own picture (rows 0..119): YUYV 4:2:2 (Y0 U Y1 V). To RGB565,
// low byte first, full range.
void picture_to_rgb565(const uint8_t *frame, uint8_t *dst, size_t dst_stride);

// The metadata rows as the vendor's Java reads them (which of them the P1
// fills is not known).
struct InfoRows {
  uint8_t ffc_countdown;    // byte 42
  uint8_t sun_protect;      // byte 82
  uint8_t hw_sun_protect;   // byte 138: 1 when the camera sees the sun (with sun detection on)
  uint16_t frame_count;     // bytes 128-129
  uint16_t frame_count2;    // bytes 360-361
};
InfoRows parse_info_rows(const uint8_t *frame);

// ---------------------------------------------------------------- automatic range
// The vendor library's automatic range switching (it runs on the host, on
// every frame's raw temperatures): a frame with more than 95 % of its pixels
// under 170 C votes for the high range; else one with more than 10 % at or
// over 170 C votes for the low range. After more than 125 frames in a row with
// the same vote, the range is checked and changed if it differs; then 176
// frames are not looked at (the library's 175, counted as it counts them),
// whether it changed or not.
class AutoRange {
 public:
  static constexpr uint16_t THRESHOLD_RAW = 28361;  // 170 C
  static constexpr uint32_t SWITCH_FRAMES = 125;
  static constexpr uint32_t WAIT_FRAMES = 175;
  enum Vote : uint8_t { NONE = 0, LOW = 1, HIGH = 2 };
  static Vote vote(const uint8_t *frame);
  // The range to change to (0 low, 1 high), or -1. `current`: the camera's
  // range (0 or 1; -1 not known: the range the votes want is set).
  int feed(const uint8_t *frame, int current);
  void reset();

 protected:
  uint32_t low_votes_{0}, high_votes_{0};
  bool waiting_{false};
  uint32_t waited_{0};
};

}  // namespace thermal_master_p1
}  // namespace esphome
