#pragma once

// Image-domain processing: tone mapping (a value range onto a 256-entry
// palette), mirroring, scaling and output pixel format. The CPU path here has
// no ESPHome or ESP-IDF dependency and is unit-tested on a host; the P4's PPA
// backend lives in isp.cpp.
//
// Tone mapping happens once at input resolution (into an 8-bit index image);
// scaling then works on indices, so cost per output pixel is a few integer
// operations and a palette lookup.

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "../video_core/frame.h"

namespace esphome {
namespace isp {

enum class Upscale : uint8_t { NEAREST = 0, BILINEAR = 1 };

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
};

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
  void tone_map(const float *in, uint16_t in_w, uint16_t in_h, const Options &o);
  const uint8_t *indices() const { return this->idx_.data(); }

  // Writes the tone-mapped image at indices() to `dst` (stride bytes per
  // row), scaled by o.scale, in o.format.
  void render(const Options &o, uint8_t *dst, uint32_t stride) const;

  // Small colour image at input resolution (for a hardware scaler).
  void colorize(const Options &o, uint8_t *dst, uint32_t stride) const;

  uint16_t width() const { return this->w_; }
  uint16_t height() const { return this->h_; }

 protected:
  void write_px_(uint8_t *p, uint8_t idx, video::ImageFormat f) const;
  template<video::ImageFormat F> void bilinear_(int s, uint8_t *dst, uint32_t stride) const;
  std::array<Rgb, 256> palette_{};
  std::array<uint16_t, 256> pal565_le_{};  // palette as RGB565 values
  std::vector<uint8_t> idx_;
  uint16_t w_{0}, h_{0};
};

}  // namespace isp
}  // namespace esphome
