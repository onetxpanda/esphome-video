#include "isp_core.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace isp {

using video::ImageFormat;

Point map_point(const Options &o, uint16_t in_w, uint16_t in_h, float x, float y) {
  const float mx = o.mirror_x ? (in_w - 1) - x : x;
  const float my = o.mirror_y ? (in_h - 1) - y : y;
  const int s = std::max<int>(o.scale, 1);
  return Point{static_cast<int>(mx * s + s / 2), static_cast<int>(my * s + s / 2)};
}

void CpuIsp::set_palette(const std::array<Rgb, 256> &palette) {
  this->palette_ = palette;
  for (int i = 0; i < 256; i++) {
    const Rgb &c = palette[i];
    this->pal565_le_[i] = static_cast<uint16_t>(((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3));
  }
}

// The palette index for value t: (t - lo) * k, rounded and clamped; NaN is 0.
// Always inlined: ESPHome builds for size, which would otherwise make this a
// call per pixel.
static inline __attribute__((always_inline)) uint8_t to_index(float t, float lo, float k) {
  const float v = (t - lo) * k;
  if (!(v > 0.0f))  // also NaN
    return 0;
  if (v >= 255.0f)
    return 255;
  return static_cast<uint8_t>(v + 0.5f);
}

static inline float index_scale(const Options &o) { return 255.0f / std::max(o.range_max - o.range_min, 1e-3f); }

// The loops below keep every option in a local: they store through byte
// pointers, which may alias anything, so a member or reference would be
// reloaded for every pixel.
bool CpuIsp::tone_map(const float *in, uint16_t in_w, uint16_t in_h, const Options &o) {
  if (!this->idx_.ensure(static_cast<size_t>(in_w) * in_h)) {
    this->w_ = this->h_ = 0;
    return false;
  }
  this->w_ = in_w;
  this->h_ = in_h;
  const float lo = o.range_min, k = index_scale(o);
  const int step = o.mirror_x ? -1 : 1;
  const bool my = o.mirror_y;
  uint8_t *idx = this->idx_.data();
  for (int y = 0; y < in_h; y++) {
    const float *src = in + static_cast<size_t>(my ? in_h - 1 - y : y) * in_w + (step < 0 ? in_w - 1 : 0);
    uint8_t *dst = idx + static_cast<size_t>(y) * in_w;
    for (int x = 0; x < in_w; x++, src += step)
      dst[x] = to_index(*src, lo, k);
  }
  return true;
}

void CpuIsp::map_direct(const float *in, uint16_t in_w, uint16_t in_h, const Options &o, uint8_t *dst,
                        uint32_t stride) const {
  if (in == nullptr || dst == nullptr)
    return;
  switch (o.format) {
    case ImageFormat::RGB565_LE:
      this->map_direct_<ImageFormat::RGB565_LE>(in, in_w, in_h, o, dst, stride);
      break;
    case ImageFormat::RGB565_BE:
      this->map_direct_<ImageFormat::RGB565_BE>(in, in_w, in_h, o, dst, stride);
      break;
    case ImageFormat::RGB888:
      this->map_direct_<ImageFormat::RGB888>(in, in_w, in_h, o, dst, stride);
      break;
    case ImageFormat::GRAY8:
      this->map_direct_<ImageFormat::GRAY8>(in, in_w, in_h, o, dst, stride);
      break;
  }
}

template<ImageFormat F>
void CpuIsp::map_direct_(const float *in, uint16_t in_w, uint16_t in_h, const Options &o, uint8_t *dst,
                         uint32_t stride) const {
  const float lo = o.range_min, k = index_scale(o);
  const int step = o.mirror_x ? -1 : 1;
  const bool my = o.mirror_y;
  const uint16_t *pal565 = this->pal565_le_.data();
  const Rgb *pal = this->palette_.data();
  for (int y = 0; y < in_h; y++) {
    const float *src = in + static_cast<size_t>(my ? in_h - 1 - y : y) * in_w + (step < 0 ? in_w - 1 : 0);
    uint8_t *p = dst + static_cast<size_t>(y) * stride;
    for (int x = 0; x < in_w; x++, src += step) {
      const uint8_t i = to_index(*src, lo, k);
      if (F == ImageFormat::RGB565_LE) {
        const uint16_t v = pal565[i];
        p[0] = static_cast<uint8_t>(v & 0xFF);
        p[1] = static_cast<uint8_t>(v >> 8);
        p += 2;
      } else if (F == ImageFormat::RGB565_BE) {
        const uint16_t v = pal565[i];
        p[0] = static_cast<uint8_t>(v >> 8);
        p[1] = static_cast<uint8_t>(v & 0xFF);
        p += 2;
      } else if (F == ImageFormat::RGB888) {
        p[0] = pal[i].r;
        p[1] = pal[i].g;
        p[2] = pal[i].b;
        p += 3;
      } else {
        *p++ = i;
      }
    }
  }
}

inline void CpuIsp::write_px_(uint8_t *p, uint8_t idx, ImageFormat f) const {
  switch (f) {
    case ImageFormat::RGB565_LE: {
      const uint16_t v = this->pal565_le_[idx];
      p[0] = static_cast<uint8_t>(v & 0xFF);
      p[1] = static_cast<uint8_t>(v >> 8);
      break;
    }
    case ImageFormat::RGB565_BE: {
      const uint16_t v = this->pal565_le_[idx];
      p[0] = static_cast<uint8_t>(v >> 8);
      p[1] = static_cast<uint8_t>(v & 0xFF);
      break;
    }
    case ImageFormat::RGB888: {
      const Rgb &c = this->palette_[idx];
      p[0] = c.r;
      p[1] = c.g;
      p[2] = c.b;
      break;
    }
    case ImageFormat::GRAY8:
      p[0] = idx;
      break;
  }
}

void CpuIsp::colorize(const Options &o, uint8_t *dst, uint32_t stride) const {
  const size_t bpp = video::bytes_per_pixel(o.format);
  for (int y = 0; y < this->h_; y++)
    for (int x = 0; x < this->w_; x++)
      this->write_px_(dst + y * stride + x * bpp, this->idx_[y * this->w_ + x], o.format);
}

void CpuIsp::render(const Options &o, uint8_t *dst, uint32_t stride) const {
  const int s = std::min(std::max<int>(o.scale, 1), 20);
  const int ow = this->w_ * s;
  const size_t bpp = video::bytes_per_pixel(o.format);
  const size_t row_bytes = static_cast<size_t>(ow) * bpp;
  if (this->w_ == 0 || this->h_ == 0 || dst == nullptr)
    return;

  if (o.upscale == Upscale::NEAREST || s == 1) {
    // Build each output row once, then copy it to the rest of its block.
    for (int y = 0; y < this->h_; y++) {
      uint8_t *row = dst + static_cast<size_t>(y) * s * stride;
      uint8_t *p = row;
      const uint8_t *in = &this->idx_[y * this->w_];
      for (int x = 0; x < this->w_; x++) {
        this->write_px_(p, in[x], o.format);
        for (int k = 1; k < s; k++)
          std::memcpy(p + k * bpp, p, bpp);
        p += s * bpp;
      }
      for (int k = 1; k < s; k++)
        std::memcpy(row + k * stride, row, row_bytes);
    }
    return;
  }

  switch (o.format) {
    case ImageFormat::RGB565_LE:
      this->bilinear_<ImageFormat::RGB565_LE>(s, dst, stride);
      break;
    case ImageFormat::RGB565_BE:
      this->bilinear_<ImageFormat::RGB565_BE>(s, dst, stride);
      break;
    case ImageFormat::RGB888:
      this->bilinear_<ImageFormat::RGB888>(s, dst, stride);
      break;
    case ImageFormat::GRAY8:
      this->bilinear_<ImageFormat::GRAY8>(s, dst, stride);
      break;
  }
}

// Bilinear on the index field, sampling at pixel centres. Per output row the
// two source rows are blended once at input width; each output pixel is then
// a 1-D interpolation of that blended row plus a palette lookup.
template<ImageFormat F> void CpuIsp::bilinear_(int s, uint8_t *dst, uint32_t stride) const {
  const int ow = this->w_ * s, oh = this->h_ * s;
  constexpr size_t bpp = F == ImageFormat::RGB888 ? 3 : (F == ImageFormat::GRAY8 ? 1 : 2);
  // Horizontal sample positions (in 1/256 of an input pixel) for every output column.
  std::vector<uint16_t> x0(ow), x1(ow), fx(ow);
  for (int ox = 0; ox < ow; ox++) {
    int sx = ((2 * ox + 1) * 256) / (2 * s) - 128;  // (ox + 0.5) / s - 0.5
    sx = std::min(std::max(sx, 0), (this->w_ - 1) * 256);
    x0[ox] = static_cast<uint16_t>(sx >> 8);
    x1[ox] = static_cast<uint16_t>(std::min((sx >> 8) + 1, this->w_ - 1));
    fx[ox] = static_cast<uint16_t>(sx & 0xFF);
  }
  std::vector<uint32_t> blended(this->w_);  // index * 256, after the vertical blend
  for (int oy = 0; oy < oh; oy++) {
    int sy = ((2 * oy + 1) * 256) / (2 * s) - 128;
    sy = std::min(std::max(sy, 0), (this->h_ - 1) * 256);
    const int y0 = sy >> 8, y1 = std::min(y0 + 1, this->h_ - 1);
    const uint32_t fy = static_cast<uint32_t>(sy & 0xFF);
    const uint8_t *r0 = &this->idx_[y0 * this->w_];
    const uint8_t *r1 = &this->idx_[y1 * this->w_];
    for (int x = 0; x < this->w_; x++)
      blended[x] = r0[x] * (256 - fy) + r1[x] * fy;
    uint8_t *p = dst + static_cast<size_t>(oy) * stride;
    for (int ox = 0; ox < ow; ox++, p += bpp) {
      const uint32_t v = (blended[x0[ox]] * (256 - fx[ox]) + blended[x1[ox]] * fx[ox] + (1u << 15)) >> 16;
      const uint8_t idx = static_cast<uint8_t>(v);
      if constexpr (F == ImageFormat::RGB565_LE) {
        const uint16_t c = this->pal565_le_[idx];
        p[0] = static_cast<uint8_t>(c & 0xFF);
        p[1] = static_cast<uint8_t>(c >> 8);
      } else if constexpr (F == ImageFormat::RGB565_BE) {
        const uint16_t c = this->pal565_le_[idx];
        p[0] = static_cast<uint8_t>(c >> 8);
        p[1] = static_cast<uint8_t>(c & 0xFF);
      } else if constexpr (F == ImageFormat::RGB888) {
        const Rgb &c = this->palette_[idx];
        p[0] = c.r;
        p[1] = c.g;
        p[2] = c.b;
      } else {
        p[0] = idx;
      }
    }
  }
}

}  // namespace isp
}  // namespace esphome
