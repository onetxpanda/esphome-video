#pragma once

// HTI-301 (InfiRay T3-317-13) thermal camera: the parts with no ESPHome or
// ESP-IDF dependency, unit-tested on a host.
//
//   UVC descriptors   finds the interfaces, frame size and isochronous endpoint
//   probe/commit      the UVC streaming-parameter block
//   UvcAssembler      isochronous payload packets -> whole frames
//   metadata          the camera's calibration rows below the image
//   TemperatureTable  14-bit raw values -> degrees C (the ht301_hacklib method)
//   yuyv_to_rgb565    the camera's own processed image -> RGB565
//
// A frame is width x (image rows + 4) 16-bit words. In radiometric mode the
// image rows hold 14-bit raw values; in processed mode they hold YUYV 4:2:2
// (full range). The 4 extra rows hold metadata in radiometric mode.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "../video_core/frame.h"

namespace esphome {
namespace hti301 {

static constexpr uint16_t VID = 0x1514;
static constexpr uint16_t PID = 0x0001;
static constexpr uint16_t METADATA_ROWS = 4;
static constexpr uint16_t RAW_MASK = 0x3FFF;  // 14-bit radiometric values
static constexpr size_t TABLE_SIZE = 16384;

// Commands, written to the camera terminal's Zoom (Absolute) control.
enum Command : uint16_t {
  CMD_CALIBRATE = 0x8000,         // shutter / flat-field correction
  CMD_RADIOMETRIC_MODE = 0x8004,  // 14-bit raw values + metadata
  CMD_PROCESSED_MODE = 0x8005,    // the camera's own YUYV image
  CMD_RANGE_NORMAL = 0x8020,      // -20..120 C
  CMD_RANGE_HIGH = 0x8021,        // -20..450 C
  CMD_PALETTE_FIRST = 0x8800,     // + palette index, processed mode
};
// 0xFF01 and 0x80FA switch the camera to a firmware-update mode (it
// re-enumerates as 1514:FFFF); 0x80FE/0x80FF write its flash. Never sent.

// Palettes (CMD_PALETTE_FIRST + index) are in the order of video::PALETTE_NAMES.

// ---------------------------------------------------------------- descriptors

struct StreamAlt {
  uint8_t alt{0};
  uint8_t endpoint{0};      // bEndpointAddress (IN)
  uint16_t packet_size{0};  // bytes per transaction (wMaxPacketSize bits 0..10)
  uint8_t transactions{1};  // per (micro)frame (1 + bits 11..12)
  uint8_t interval{1};      // bInterval
};

struct UvcLayout {
  bool ok{false};
  uint16_t uvc_version{0};  // bcdUVC
  uint8_t vc_interface{0xFF};
  uint8_t vs_interface{0xFF};
  uint8_t format_index{0};  // first uncompressed format
  uint8_t frame_index{0};   // its first frame
  uint16_t width{0};
  uint16_t height{0};  // including the metadata rows
  uint32_t max_frame_size{0};
  uint32_t frame_interval{0};  // 100 ns units
  static constexpr uint8_t MAX_ALTS = 8;
  StreamAlt alts[MAX_ALTS];
  uint8_t alt_count{0};
  // Probe/commit block length for this UVC version.
  size_t probe_size() const { return this->uvc_version >= 0x0110 ? 34 : 26; }
  // The streaming alternate setting with the smallest bandwidth that carries
  // `payload` bytes per service interval in one transaction, or nullptr.
  const StreamAlt *alt_for(uint32_t payload) const;
};

// Parses a configuration descriptor (wTotalLength bytes).
UvcLayout parse_descriptors(const uint8_t *cfg, size_t len);

// ---------------------------------------------------------------- probe/commit

struct Probe {
  uint16_t hint{1};  // dwFrameInterval fixed
  uint8_t format_index{1};
  uint8_t frame_index{1};
  uint32_t frame_interval{0};
  uint32_t max_frame_size{0};
  uint32_t max_payload{0};
};
// Writes `len` (26 or 34) bytes.
void encode_probe(const Probe &p, uint8_t *buf, size_t len);
// False if `len` is too short.
bool decode_probe(const uint8_t *buf, size_t len, Probe *out);

// ---------------------------------------------------------------- assembly

// Assembles UVC isochronous payloads (UVC 1.5 section 2.4.3.3: header length,
// flags, optional PTS/SCR) into frames of `frame_size` bytes. A frame ends at
// EOF, at a change of the frame-ID bit, or when it reaches frame_size. Only a
// frame of exactly frame_size bytes, with no packet lost, is complete.
class UvcAssembler {
 public:
  static constexpr uint8_t FLAG_FID = 0x01;
  static constexpr uint8_t FLAG_EOF = 0x02;
  static constexpr uint8_t FLAG_ERR = 0x40;

  void set_frame_size(size_t n) {
    this->frame_size_ = n;
    this->reset();
  }
  size_t frame_size() const { return this->frame_size_; }
  void reset() {
    this->offset_ = 0;
    this->have_fid_ = false;
    this->damaged_ = false;
  }
  // A packet was lost (transfer error, skipped service interval, bad header):
  // the frame being assembled cannot be complete.
  void lost() { this->damaged_ = true; }

  // Feeds one packet. `buf` (capacity `cap`) receives the frame being
  // assembled; it may be null, and the frame is then dropped. When a frame
  // ends, finish(bool complete, size_t bytes) is called and must leave `buf`
  // pointing at the buffer for the next frame (the same one when incomplete).
  template<typename Finish> void feed(const uint8_t *p, size_t len, uint8_t *&buf, size_t cap, Finish &&finish) {
    if (len == 0)
      return;  // an empty service interval: normal
    const uint8_t hlen = p[0];
    if (hlen < 2 || hlen > len) {
      this->lost();
      return;
    }
    const uint8_t flags = p[1];
    if (flags & FLAG_ERR) {
      this->lost();
      return;
    }
    const bool fid = flags & FLAG_FID;
    if (this->have_fid_ && fid != this->fid_ && this->offset_ != 0)
      this->end_(finish);
    this->fid_ = fid;
    this->have_fid_ = true;
    const size_t n = len - hlen;
    if (n != 0) {
      if (buf != nullptr && this->offset_ + n <= cap && this->offset_ + n <= this->frame_size_) {
        std::memcpy(buf + this->offset_, p + hlen, n);
      } else {
        this->damaged_ = true;
      }
      this->offset_ += n;
    }
    if (this->offset_ != 0 && ((flags & FLAG_EOF) || this->offset_ >= this->frame_size_))
      this->end_(finish);
  }

 protected:
  template<typename Finish> void end_(Finish &finish) {
    const bool complete = !this->damaged_ && this->offset_ == this->frame_size_;
    const size_t bytes = this->offset_;
    this->offset_ = 0;
    this->damaged_ = false;
    finish(complete, bytes);
  }

  size_t frame_size_{0};
  size_t offset_{0};
  bool fid_{false};
  bool have_fid_{false};
  bool damaged_{false};
};

// ---------------------------------------------------------------- metadata

struct Metadata {
  bool valid{false};  // calibration words present and usable
  float fpa_temp{0};  // detector, degrees C
  float shutter_temp{0};
  float core_temp{0};
  uint16_t cal_00{0};
  float cal_01{0}, cal_02{0}, cal_03{0}, cal_04{0}, cal_05{0};
  // The environment stored in the camera (its "user area"); present only on
  // 384- and 640-wide models.
  bool has_environment{false};
  float correction{0}, reflected{0}, ambient{0}, humidity{0}, emissivity{0}, distance{0};
  // The camera's own hottest/coldest pixel and centre, raw.
  uint16_t max_x{0}, max_y{0}, max_raw{0}, min_x{0}, min_y{0}, min_raw{0}, center_raw{0};
};

// `frame` is width * (image_rows + METADATA_ROWS) words.
Metadata parse_metadata(const uint16_t *frame, uint16_t width, uint16_t image_rows);

// ---------------------------------------------------------------- temperatures

// Scene parameters for the conversion. A field that is NaN takes the value
// stored in the camera, or the default below when the camera has none or an
// implausible one.
struct Environment {
  float emissivity;   // 0.01..1
  float reflected;    // degrees C
  float ambient;      // degrees C
  float humidity;     // 0..1
  float distance;     // metres
  float correction;   // degrees added to every pixel
};
static constexpr Environment DEFAULT_ENVIRONMENT{0.95f, 25.0f, 25.0f, 0.5f, 1.0f, 0.0f};

// The environment actually used: `overrides`, else the camera's, else the defaults.
Environment resolve_environment(const Environment &overrides, const Metadata &m);

class TemperatureTable {
 public:
  // Takes the calibration from `m` and the scene from `env`. False if the
  // calibration cannot be used (e.g. cal_01 == 0) or the table (64 KB, in
  // PSRAM on the device) cannot be allocated.
  bool prepare(const Metadata &m, const Environment &env);
  // The temperature for one raw value; NaN where the calibration has none.
  float temperature(uint16_t raw) const;
  // Fills the table for raw values lo..hi (inclusive, at most TABLE_SIZE - 1).
  void build(uint16_t lo, uint16_t hi);
  // The table, indexed by raw value: allocated by prepare(), filled by build().
  const float *values() { return this->table_.data(); }

 protected:
  video::LargeBuffer<float> table_;
  float cal_a_{0}, cal_b_{0}, cal_c_{0}, cal_d_{0}, cal_01_{1};
  float numerator_sub_{0}, denominator_{1}, ambient_{0}, distance_{0}, correction_{0};
  int table_offset_{0};
};

// Converts width x height raw words (masked to 14 bits) to temperatures
// through a table built over their range: the camera's reported min/max,
// extended for any pixel outside it. Fills `stats` (if given) in the same
// pass; it is invalid if any pixel has no finite temperature. Returns false
// if the calibration is unusable.
bool raw_to_temperatures(const uint16_t *raw, uint16_t width, uint16_t height, const Metadata &m,
                         const Environment &env, TemperatureTable &table, float *out,
                         video::FrameStats *stats = nullptr);

// ---------------------------------------------------------------- processed image

// Full-range YUYV 4:2:2 (Y0 U Y1 V) to RGB565, low byte first. `width` must be even.
void yuyv_to_rgb565(const uint8_t *src, uint16_t width, uint16_t height, uint8_t *dst, size_t dst_stride);

}  // namespace hti301
}  // namespace esphome
