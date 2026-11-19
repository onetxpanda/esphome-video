#pragma once

// Image-domain processing: tone mapping (a value range onto a 256-entry
// palette), mirroring, scaling and output pixel format. The CPU path here has
// no ESPHome or ESP-IDF dependency and is unit-tested on a host; the P4's PPA
// backend lives in isp.cpp.
//
// Tone mapping happens once at input resolution (into an 8-bit index image);
// scaling then works on indices, so cost per output pixel is a few integer
// operations and a palette lookup.
//
// Tone mapping is linear over the range by default. Two options change it:
// - equalize mixes in histogram equalization (with a plateau limit and
//   smoothed over frames), giving more colours to the temperatures the scene
//   actually contains;
// - detail amplifies local differences against a smoothed copy of the frame
//   (base/detail separation), which sharpens edges and small features.

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "../video_core/frame.h"

namespace esphome {
namespace isp {

enum class Upscale : uint8_t { NEAREST = 0, BILINEAR = 1, BICUBIC = 2 };

struct Rgb {
  uint8_t r, g, b;
};

struct Options {
  float range_min{15.0f};  // input value mapped to palette index 0
  float range_max{40.0f};  // input value mapped to palette index 255
  bool mirror_x{false};
  bool mirror_y{false};
  uint8_t scale{1};  // output = input * scale, 1..20
  Upscale upscale{Upscale::NEAREST};
  video::ImageFormat format{video::ImageFormat::RGB565_LE};
  float equalize{0.0f};  // 0 = linear .. 1 = fully histogram-equalized
  float detail{0.0f};    // detail gain; 0 = off
};

// Whether tone mapping is the plain linear map (no equalization, no detail).
inline bool linear_tone(const Options &o) { return !(o.equalize > 0.0f) && !(o.detail > 0.0f); }

// Maps an input pixel's centre to output pixel coordinates under `o`
// (mirroring and scaling), for overlays.
struct Point {
  int x, y;
};
Point map_point(const Options &o, uint16_t in_w, uint16_t in_h, float x, float y);

class CpuIsp {
 public:
  void set_palette(const std::array<Rgb, 256> &palette);
  const std::array<Rgb, 256> &palette() const { return this->palette_; }

  // Tone-maps `in` (in_w x in_h values) to the index image (mirroring applied).
  // False when the index buffer cannot be allocated. Equalization keeps its
  // curve between calls, so frames of one stream should go through one CpuIsp.
  bool tone_map(const float *in, uint16_t in_w, uint16_t in_h, const Options &o);
  const uint8_t *indices() const { return this->idx_.data(); }

  // Writes the tone-mapped image at indices() to `dst` (stride bytes per
  // row), scaled by o.scale, in o.format.
  void render(const Options &o, uint8_t *dst, uint32_t stride) const;

  // Colours an index image (w x h, w bytes per row), such as a hardware
  // scaler's output from indices(), into `dst` (stride bytes per row) in
  // `format`.
  void lookup(const uint8_t *idx, uint16_t w, uint16_t h, video::ImageFormat format, uint8_t *dst,
              uint32_t stride) const;

  // Unscaled output in one pass: tone-maps `in` (mirroring applied) straight
  // to `dst` in o.format, as tone_map() then render() would at scale 1,
  // without the index image.
  void map_direct(const float *in, uint16_t in_w, uint16_t in_h, const Options &o, uint8_t *dst,
                  uint32_t stride) const;

  uint16_t width() const { return this->w_; }
  uint16_t height() const { return this->h_; }

 protected:
  void write_px_(uint8_t *p, uint8_t idx, video::ImageFormat f) const;
  template<video::ImageFormat F> void bilinear_(int s, uint8_t *dst, uint32_t stride) const;
  template<video::ImageFormat F> void bicubic_(int s, uint8_t *dst, uint32_t stride) const;
  // The tone curve at CURVE_BINS + 1 evenly spaced points over the range.
  void update_curve_(const float *in, size_t n, const Options &o);
  template<video::ImageFormat F>
  void lookup_(const uint8_t *idx, uint16_t w, uint16_t h, uint8_t *dst, uint32_t stride) const;
  template<video::ImageFormat F>
  void map_direct_(const float *in, uint16_t in_w, uint16_t in_h, const Options &o, uint8_t *dst,
                   uint32_t stride) const;
  std::array<Rgb, 256> palette_{};
  std::array<uint16_t, 256> pal565_le_{};  // palette as RGB565 values
  video::LargeBuffer<uint8_t> idx_;
  uint16_t w_{0}, h_{0};

 public:
  static constexpr int CURVE_BINS = 64;

 protected:
  std::array<float, CURVE_BINS + 1> curve_{};
  bool curve_valid_{false};
  float curve_lo_{0}, curve_hi_{0};
  video::LargeBuffer<float> base_;  // the smoothed frame, for detail
};

}  // namespace isp
}  // namespace esphome
