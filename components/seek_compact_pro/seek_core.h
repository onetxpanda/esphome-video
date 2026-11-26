#pragma once

// Seek Thermal Compact Pro (sensor PIR324): frame format, calibration and the
// conversion to temperatures, as Seek's own library (libseekip, in the Seek
// Thermal app 2.3.4) does it for this camera. No ESPHome or ESP-IDF
// dependency; unit-tested on a host.
//
// The camera sends 342 x 260 little-endian 16-bit words per frame:
//   row 0         word 0 1401 (every frame), 1 a 16-bit counter, 2 what the
//                 frame is, 5 the chip temperature in counts, 8..11 the time
//                 since the camera started (ms, 64-bit, high word first),
//                 12..15 the same in microseconds when not zero
//   row 2         a reference row: the mean of its first N pixels (N from
//                 the factory settings) follows the sensor's temperature
//   columns 0..323, rows 4..257
//                 the pixels that see the scene; the picture is the 320 x 240
//                 at column 2, row 12 (the camera's geometry record)
// Frame ids: 4 the per-pixel response table (gain; 0 marks a bad pixel),
// 9 a 16384-entry table every raw value goes through first, 14 a
// temperature correction table, 8 the start-up shutter frame, 7 offsets for
// the camera's second range (raw values from 15000 up), 1 a shutter frame, 6
// a frame just before each shutter frame, 3 an image; 5 is not used for this
// camera. The tables arrive once after each start, in that order.
//
// An image pixel: raw -> table 9 -> minus the last shutter frame (through
// table 9; plus the second-range offset for raw values from 15000 up) -> times
// its gain (16384 x the table-4 mean over its table-4 value) / 16384, on a
// pedestal of 3000. Bad pixels, and configured bad columns, take the mean of
// their neighbours. Temperature (factory thermography version 3): with s the
// smoothed row-2 mean less the factory reference, x = k0 + k1 x (pixel -
// 3000), and seven coefficients each a quadratic in s, T = D + C x + A x^2 +
// B x^3 (or G + F x + E x^2 from a threshold on); then the frame-14 table's
// correction (piecewise linear in T, along s); then the warm-up transient
// a x exp(-t / tau) is taken off. A `gain` and `offset` apply last.
//
// Before the temperature conversion, Seek's default picture filter runs on
// the pixels (unless turned off): its scene-based correction (single pixels
// more than 15 counts off their 3x3 median take the median every frame; with
// enough scene motion it also learns each pixel's offset, decaying 127/128
// a frame without), then its temporal filter (differences under 6 sigma from
// the last output are cored). A shutter frame stops the next picture from
// learning and restarts the temporal filter.

#include <cstddef>
#include <cstdint>
#include <cmath>

#include "../video_core/frame.h"

namespace esphome {
namespace seek_compact_pro {

static constexpr uint16_t RAW_W = 342, RAW_H = 260;
static constexpr uint16_t IMG_W = 320, IMG_H = 240;
static constexpr uint16_t IMG_ROW0 = 12;    // the picture's first row
static constexpr uint16_t IMG_COL0 = 2;     // and first column
static constexpr size_t RAW_WORDS = static_cast<size_t>(RAW_W) * RAW_H;
static constexpr size_t RAW_BYTES = RAW_WORDS * 2;
static constexpr size_t IMG_PIXELS = static_cast<size_t>(IMG_W) * IMG_H;
// The camera sends a frame as 20-row pieces of 13680 bytes.
static constexpr size_t CHUNK_BYTES = 13680;
static constexpr uint16_t FRAME_MAGIC = 1401;
// The factory settings block, as read at start-up.
static constexpr size_t FACTORY_BYTES = 5120;
// Bad columns that can be configured.
static constexpr size_t MAX_BAD_COLUMNS = 8;
// Raw values from this on are in the camera's second range.
static constexpr uint16_t SECOND_RANGE = 15000;
static constexpr int32_t PEDESTAL = 3000;
static constexpr uint8_t MAX_ADJUST_TABLES = 3;

enum FrameId : uint16_t {
  FRAME_SHUTTER = 1,
  FRAME_IMAGE = 3,
  FRAME_GAIN = 4,
  FRAME_PRE_SHUTTER = 6,
  FRAME_RANGE2 = 7,
  FRAME_START_SHUTTER = 8,
  FRAME_LUT = 9,
  FRAME_ADJUST = 14,
};
// The first frame after a start.
static constexpr uint16_t FRAME_FIRST = FRAME_GAIN;

inline uint16_t frame_magic(const uint16_t *raw) { return raw[0]; }
inline uint16_t frame_counter(const uint16_t *raw) { return raw[1]; }
inline uint16_t frame_id(const uint16_t *raw) { return raw[2]; }
inline uint16_t image_px(const uint16_t *raw, size_t i) {
  return raw[(IMG_ROW0 + i / IMG_W) * static_cast<size_t>(RAW_W) + IMG_COL0 + i % IMG_W];
}
// Microseconds since the camera started (row 0, words 8..15).
uint64_t frame_uptime_us(const uint16_t *raw);
// Whether the Processor uses frames of this id.
inline bool frame_used(uint16_t id) {
  return id == FRAME_SHUTTER || id == FRAME_IMAGE || id == FRAME_GAIN || id == FRAME_PRE_SHUTTER ||
         id == FRAME_RANGE2 || id == FRAME_START_SHUTTER || id == FRAME_LUT || id == FRAME_ADJUST;
}

// The factory settings block's thermography (byte offsets into the block).
struct Factory {
  bool valid{false};
  int32_t version{0};        // 0x60; only 3 is supported
  float sensor_ref{0};       // 0x64: row-2 counts the coefficients are centred on
  uint16_t ref_pixels{320};  // 0x48: row-2 pixels averaged
  int16_t range2_offset{0};  // 0x52, added to second-range offsets (version < 5)
  bool lut_shift{true};      // 0x43C != 1: table 9 halved below 15000, doubled from it
  float k0{0}, k1{0};        // 0x98, 0x9C: x = k0 + k1 x (pixel - 3000)
  float threshold{0};        // 0xF4: x from here on uses G, F, E
  float coef[7][3]{};        // B, A, C, D, E, F, G at 0xA0..0xF0: (s^2, s, 1) each
  float warm_a{6.7f}, warm_tau{95.0f};  // warm-up transient (C, seconds)
  uint8_t filter_sigma{4};   // from 0x1008: the temporal filter's sigma, counts
};
// False (and `out` not valid) unless the block is marked valid (0x02 =
// 0xF731), is for this camera (0x4E = 1401) and its thermography is version 3.
bool parse_factory(const uint8_t *block, size_t len, Factory *out);

// The frame-14 correction tables (Seek's "ThermAdjust", format 3).
class ThermAdjust {
 public:
  // From a frame-14 frame: up to three tables at payload offsets 0, 0x2200
  // and 0x4400 (payload: rows 4 on, 324 words a row). False when out of memory.
  bool load(const uint16_t *raw);
  bool loaded() const { return this->loaded_; }
  uint8_t tables() const { return this->count_; }
  // Per frame: the node grid at the sensor value s (16.16 counts).
  void begin_frame(int64_t s_q16);
  // t less every table's correction, in slot order.
  float apply(float t) const;
  void reset() {
    this->loaded_ = false;
    this->count_ = 0;
  }

 protected:
  struct Table {
    uint8_t ns{0}, nt{0}, fb{5}, slot{0};
    int16_t axis[32]{};
    video::LargeBuffer<int16_t> node, value;  // 64 x 32 each, 1/2^fb C
    // per frame
    float val[64]{};
    int64_t x0[64]{}, slope[64]{};
    video::LargeBuffer<uint8_t> index;  // 65536: temperature (1/2^fb C, + 0x8000) -> node
  };
  bool parse_(const uint8_t *p, Table &t);
  void begin_(Table &t, int64_t s_q16);
  float apply_(const Table &t, float v) const;
  Table t_[MAX_ADJUST_TABLES];
  uint8_t order_[MAX_ADJUST_TABLES]{};
  uint8_t count_{0};
  bool loaded_{false};
};

// Seek's default picture filter (its sbnuc, then alpf), on the 16-bit pixels.
class SceneFilter {
 public:
  void set_sigma(uint8_t sigma);
  // In place; `raw` is the frame the pixels came from (values from 15000 up
  // are a range of their own for the median). False when out of memory.
  bool process(uint16_t *img, const uint16_t *raw);
  // A new shutter offset: the next picture does not learn, the temporal filter restarts.
  void shutter();
  void reset();
  uint32_t updates() const { return this->updates_; }
  uint32_t replaced() const { return this->replaced_; }  // in the last picture

 protected:
  void median_(const uint16_t *x, const uint8_t *flag, uint16_t *out);
  video::LargeBuffer<int16_t> corr_;      // per pixel, 1/128 count
  video::LargeBuffer<uint16_t> prev_, med_, tmp_, alpf_;
  video::LargeBuffer<uint8_t> flag_;
  bool have_prev_{false}, skip_{false}, sigma_set_{false};
  uint32_t updates_{0}, replaced_{0};
  uint8_t n_{24};
  uint16_t table_[64]{};
};

// The per-pixel table (frame 4): bad pixels and each pixel's gain.
class PixelTable {
 public:
  // Picture columns (0..IMG_W-1) bad on top of the table's own; takes effect
  // at the next build(). False when out of range or the list is full.
  bool add_bad_column(uint16_t x);
  // False when out of memory. count() pixels are bad afterwards.
  bool build(const uint16_t *raw);
  // Gives each bad pixel of `img` (IMG_W x IMG_H) the mean of its good (or
  // already filled) 4-neighbours, in an order where each has one.
  void fill(uint16_t *img) const;
  bool alive(size_t i) const { return this->ready_ && this->alive_.data()[i]; }
  // Per pixel: 16384 x the table's mean over its value (0 for a bad pixel).
  const uint16_t *gain() const { return this->gain_.data(); }
  size_t count() const { return this->count_; }
  bool ready() const { return this->ready_; }
  void reset() {
    this->ready_ = false;
    this->count_ = 0;
  }

 protected:
  video::LargeBuffer<uint8_t> alive_;
  video::LargeBuffer<uint16_t> gain_;
  video::LargeBuffer<uint32_t> order_;  // pixel indices, filled in this order
  uint16_t bad_columns_[MAX_BAD_COLUMNS]{};
  size_t num_bad_columns_{0};
  size_t count_{0};
  bool ready_{false};
};

class Processor {
 public:
  enum class Result : uint8_t { IGNORED, TABLE, SHUTTER, IMAGE, NOT_READY, NO_MEMORY };
  // See PixelTable::add_bad_column; before the first frame.
  bool add_bad_column(uint16_t x) { return this->table_.add_bad_column(x); }
  // The factory settings block (FACTORY_BYTES); false when it is not one
  // this calibration supports (then there are no temperatures).
  bool set_factory(const uint8_t *block, size_t len);
  const Factory &factory() const { return this->factory_; }
  // Seek's default picture filter (on unless turned off).
  void set_scene_filter(bool on) { this->scene_on_ = on; }
  bool scene_filter() const { return this->scene_on_; }
  uint32_t scene_updates() const { return this->scene_.updates(); }
  uint32_t scene_replaced() const { return this->scene_.replaced(); }
  // Correction of the camera's temperatures: t' = t * gain + offset.
  void set_correction(float gain, float offset) {
    this->gain_ = gain;
    this->offset_ = offset;
  }
  // One raw frame (RAW_WORDS). For an image, `temps` (IMG_PIXELS) receives
  // degrees C, and `stats` its statistics.
  Result feed(const uint16_t *raw, float *temps, video::FrameStats *stats = nullptr);
  // The tables (frames 4, 9 and 14) have come since the last reset().
  bool has_table() const { return this->table_.ready() && this->lut_ready_ && this->adjust_.loaded(); }
  bool ready() const { return this->has_table() && this->have_shutter_; }
  size_t dead_pixels() const { return this->table_.count(); }
  uint8_t adjust_tables() const { return this->adjust_.tables(); }
  // The sensor value s (row-2 mean less the factory reference), counts.
  float sensor_counts() const { return static_cast<float>(this->s_q16_) / 65536.0f; }
  float warm_up_c() const { return this->warm_; }  // taken off the last image
  void reset();

 protected:
  void sensor_(const uint16_t *raw);
  void through_lut_(const uint16_t *raw, uint16_t *out) const;

  Factory factory_;
  PixelTable table_;
  ThermAdjust adjust_;
  SceneFilter scene_;
  bool scene_on_{true};
  video::LargeBuffer<uint16_t> lut_;      // 16384
  video::LargeBuffer<uint16_t> shutter_;  // the last shutter frame, through table 9
  video::LargeBuffer<uint16_t> start_;    // the start-up shutter frame, through table 9
  video::LargeBuffer<int32_t> range2_;    // second-range offsets
  video::LargeBuffer<uint16_t> image_;
  bool lut_ready_{false}, have_shutter_{false}, have_start_{false}, have_range2_{false};
  int64_t row2_q16_{0};  // smoothed row-2 mean, 16.16
  int64_t s_q16_{0};
  float warm_{0};
  float gain_{1.0f}, offset_{0.0f};
};

}  // namespace seek_compact_pro
}  // namespace esphome
