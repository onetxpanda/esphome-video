#include "isp_core.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
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

static inline float clamp01(float f) { return f > 0.0f ? (f < 1.0f ? f : 1.0f) : 0.0f; }  // NaN: 0

Point spot_point(uint16_t out_w, uint16_t out_h, float fx, float fy) {
  // Rounded, so a fraction that is a whole pixel in exact arithmetic is that pixel.
  auto at = [](float f, int n) { return std::max(0, std::min(static_cast<int>(clamp01(f) * n + 0.5f), n - 1)); };
  return Point{at(fx, out_w), at(fy, out_h)};
}

float spot_value(const float *in, uint16_t in_w, uint16_t in_h, const Options &o, float fx, float fy) {
  if (in == nullptr || in_w == 0 || in_h == 0)
    return 0.0f;
  // Input pixel i spans [i, i + 1) of the width; its centre is at i + 0.5.
  float u = clamp01(fx) * in_w - 0.5f, v = clamp01(fy) * in_h - 0.5f;
  if (o.mirror_x)
    u = (in_w - 1) - u;
  if (o.mirror_y)
    v = (in_h - 1) - v;
  u = std::min(std::max(u, 0.0f), static_cast<float>(in_w - 1));
  v = std::min(std::max(v, 0.0f), static_cast<float>(in_h - 1));
  const int x0 = static_cast<int>(u), y0 = static_cast<int>(v);
  const int x1 = std::min(x0 + 1, in_w - 1), y1 = std::min(y0 + 1, in_h - 1);
  const float ax = u - x0, ay = v - y0;
  const float top = in[y0 * in_w + x0] * (1 - ax) + in[y0 * in_w + x1] * ax;
  const float bottom = in[y1 * in_w + x0] * (1 - ax) + in[y1 * in_w + x1] * ax;
  return top * (1 - ay) + bottom * ay;
}

const Material MATERIALS[] = {
    {"default", "Default", 0.0f},
    {"tortoise_shell", "Tortoise shell", 0.96f},
    {"skin", "Skin", 0.98f},
    {"black_plastic", "Black plastic", 0.95f},
    {"plastic", "Plastic", 0.92f},
    {"rubber", "Rubber", 0.95f},
    {"wood", "Wood", 0.90f},
    {"paper", "Paper, cardboard", 0.93f},
    {"fabric", "Fabric", 0.95f},
    {"water", "Water", 0.96f},
    {"glass", "Glass", 0.92f},
    {"ceramic", "Ceramic tile", 0.92f},
    {"concrete", "Concrete", 0.94f},
    {"brick", "Brick", 0.93f},
    {"soil", "Soil", 0.94f},
    {"sand", "Sand", 0.90f},
    {"plants", "Plants", 0.96f},
    {"paint", "Painted surface", 0.93f},
    {"anodized", "Aluminum, anodized", 0.77f},
    {"aluminum", "Aluminum, bare", 0.10f},
    {"stainless_steel", "Stainless steel", 0.16f},
};
const uint8_t MATERIAL_COUNT = sizeof(MATERIALS) / sizeof(MATERIALS[0]);

static bool same_name(const std::string &a, const char *b) {
  size_t i = 0;
  for (; i < a.size() && b[i] != '\0'; i++)
    if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i])))
      return false;
  return i == a.size() && b[i] == '\0';
}

uint8_t material_index(const std::string &name) {
  for (uint8_t i = 0; i < MATERIAL_COUNT; i++)
    if (same_name(name, MATERIALS[i].key) || same_name(name, MATERIALS[i].label))
      return i;
  return 0;
}

float correct_emissivity(float t, float e_source, float e, float reflected) {
  if (!(e > 0.0f && e <= 1.0f) || !(e_source > 0.0f && e_source <= 1.0f) || !std::isfinite(reflected) || e == e_source)
    return t;
  // The radiance the source saw: e_source * T^4 + (1 - e_source) * R^4 for
  // its temperature T, and e * To^4 + (1 - e) * R^4 for the surface's.
  constexpr double ZERO_C = 273.15;
  const double ts = static_cast<double>(t) + ZERO_C, r = static_cast<double>(reflected) + ZERO_C;
  const double ts4 = ts * ts * ts * ts, r4 = r * r * r * r;
  const double to4 = (e_source * ts4 + (e - e_source) * r4) / e;
  // Below zero: the source saw less than the background reflects off such a
  // surface; the coldest it can be.
  return to4 > 0 ? static_cast<float>(std::sqrt(std::sqrt(to4)) - ZERO_C) : static_cast<float>(-ZERO_C);
}

std::string format_spots(const Spot *spots, uint8_t count) {
  std::string s;
  char buf[48];
  for (uint8_t i = 0; i < count; i++) {
    const uint8_t m = spots[i].material < MATERIAL_COUNT ? spots[i].material : 0;
    std::snprintf(buf, sizeof(buf), "%s%.1f,%.1f,%s,%u", i ? ";" : "", spots[i].x * 100.0f, spots[i].y * 100.0f,
                  MATERIALS[m].key, spots[i].color);
    s += buf;
  }
  return s;
}

uint8_t free_spot_color(const Spot *spots, uint8_t count) {
  uint32_t used = 0;
  for (uint8_t i = 0; i < count; i++)
    used |= 1u << (spots[i].color & 31);
  for (uint8_t c = 0; c < MAX_SPOTS; c++)
    if (!(used & (1u << c)))
      return c;
  return MAX_SPOTS;
}

bool toggle_spot(Spot *spots, uint8_t &count, float fx, float fy, float aspect, uint8_t material) {
  fx = clamp01(fx);
  fy = clamp01(fy);
  int nearest = -1;
  float best = SPOT_HIT_RADIUS * SPOT_HIT_RADIUS;
  for (uint8_t i = 0; i < count; i++) {
    // Distances in image heights, so the radius is round on screen.
    const float dx = (spots[i].x - fx) * aspect, dy = spots[i].y - fy;
    const float d = dx * dx + dy * dy;
    if (d <= best) {
      best = d;
      nearest = i;
    }
  }
  if (nearest >= 0) {
    for (uint8_t i = nearest; i + 1 < count; i++)
      spots[i] = spots[i + 1];
    count--;
    return true;
  }
  if (count >= MAX_SPOTS)
    return false;
  const uint8_t color = free_spot_color(spots, count);
  spots[count] = Spot{fx, fy, material < MATERIAL_COUNT ? material : static_cast<uint8_t>(0), color};
  count++;
  return true;
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
// Branch-free: much of a scene often lies outside the colour range, which
// makes clamping branches unpredictable. fmax returns the other operand for
// NaN, so NaN becomes 0; on the P4 fmin and fmax are single instructions.
static inline __attribute__((always_inline)) uint8_t to_index(float t, float lo, float k) {
  const float v = std::fmin(std::fmax((t - lo) * k, 0.0f), 255.0f);
  return static_cast<uint8_t>(v + 0.5f);
}

static inline float index_scale(const Options &o) { return 255.0f / std::max(o.range_max - o.range_min, 1e-3f); }

// The loops below keep every option in a local: they store through byte
// pointers, which may alias anything, so a member or reference would be
// reloaded for every pixel.
bool CpuIsp::tone_map(const float *in, uint16_t in_w, uint16_t in_h, const Options &o) {
  const size_t n = static_cast<size_t>(in_w) * in_h;
  if (!this->idx_.ensure(n)) {
    this->w_ = this->h_ = 0;
    return false;
  }
  this->w_ = in_w;
  this->h_ = in_h;
  const int step = o.mirror_x ? -1 : 1;
  const bool my = o.mirror_y;
  uint8_t *idx = this->idx_.data();
  if (linear_tone(o)) {
    this->curve_valid_ = false;  // equalization starts afresh when turned on
    const float lo = o.range_min, k = index_scale(o);
    for (int y = 0; y < in_h; y++) {
      const float *src = in + static_cast<size_t>(my ? in_h - 1 - y : y) * in_w + (step < 0 ? in_w - 1 : 0);
      uint8_t *dst = idx + static_cast<size_t>(y) * in_w;
      for (int x = 0; x < in_w; x++, src += step)
        dst[x] = to_index(*src, lo, k);
    }
    return true;
  }

  this->update_curve_(in, n, o);
  // The curve at value t: piecewise linear between its points; NaN maps to 0.
  const float lo = o.range_min;
  const float kb = static_cast<float>(CURVE_BINS) / std::max(o.range_max - o.range_min, 1e-3f);
  const float *curve = this->curve_.data();
  auto f = [lo, kb, curve](float t) {
    const float pos = std::fmin(std::fmax((t - lo) * kb, 0.0f), static_cast<float>(CURVE_BINS));
    const int i = std::min(static_cast<int>(pos), CURVE_BINS - 1);
    return curve[i] + (pos - i) * (curve[i + 1] - curve[i]);
  };

  // Detail: the frame smoothed with a 3x3 [1 2 1] kernel (edges replicated)
  // is the base; each pixel's difference from it is amplified by 1 + detail.
  const float gain = o.detail > 0.0f ? 1.0f + o.detail : 1.0f;
  const float *base = nullptr;
  if (o.detail > 0.0f && this->base_.ensure(n)) {
    float *b = this->base_.data();
    for (int y = 0; y < in_h; y++) {
      const int y0 = std::max(y - 1, 0), y1 = std::min(y + 1, in_h - 1);
      for (int x = 0; x < in_w; x++) {
        const int x0 = std::max(x - 1, 0), x1 = std::min(x + 1, in_w - 1);
        auto at = [in, in_w](int yy, int xx) { return in[static_cast<size_t>(yy) * in_w + xx]; };
        b[static_cast<size_t>(y) * in_w + x] =
            (at(y0, x0) + 2 * at(y0, x) + at(y0, x1) + 2 * at(y, x0) + 4 * at(y, x) + 2 * at(y, x1) + at(y1, x0) +
             2 * at(y1, x) + at(y1, x1)) *
            (1.0f / 16.0f);
      }
    }
    base = b;
  }

  for (int y = 0; y < in_h; y++) {
    const size_t srow = static_cast<size_t>(my ? in_h - 1 - y : y) * in_w;
    uint8_t *dst = idx + static_cast<size_t>(y) * in_w;
    for (int x = 0; x < in_w; x++) {
      const size_t i = srow + (step < 0 ? in_w - 1 - x : x);
      const float t = in[i];
      float v;
      if (!std::isfinite(t)) {
        v = 0.0f;
      } else if (base != nullptr && std::isfinite(base[i])) {
        const float fb = f(base[i]);
        v = fb + gain * (f(t) - fb);
      } else {
        v = f(t);
      }
      dst[x] = static_cast<uint8_t>(std::fmin(std::fmax(v, 0.0f), 255.0f) + 0.5f);
    }
  }
  return true;
}

// Histogram equalization with a plateau: a histogram of the in-range values
// over CURVE_BINS bins, each bin capped at PLATEAU times the mean so a large
// uniform background cannot claim most of the palette (and turn its noise
// into colour), then its cumulative sum as the curve. `equalize` mixes it
// with the linear curve. Successive curves are blended (a quarter of the new
// one per frame) so the colours do not flicker with noise.
void CpuIsp::update_curve_(const float *in, size_t n, const Options &o) {
  constexpr float PLATEAU = 3.0f;
  constexpr float BLEND = 0.25f;
  const float lo = o.range_min, hi = o.range_max;
  const float kb = static_cast<float>(CURVE_BINS) / std::max(hi - lo, 1e-3f);
  std::array<float, CURVE_BINS + 1> next;
  for (int i = 0; i <= CURVE_BINS; i++)
    next[i] = 255.0f * i / CURVE_BINS;  // linear
  const float e = std::fmin(std::fmax(o.equalize, 0.0f), 1.0f);
  if (e > 0.0f) {
    std::array<uint32_t, CURVE_BINS> hist{};
    uint32_t count = 0;
    for (size_t i = 0; i < n; i++) {
      const float pos = (in[i] - lo) * kb;
      if (pos >= 0.0f && pos <= static_cast<float>(CURVE_BINS)) {  // false for NaN
        hist[std::min(static_cast<int>(pos), CURVE_BINS - 1)]++;
        count++;
      }
    }
    if (count > 0) {
      const float cap = std::max(1.0f, PLATEAU * count / CURVE_BINS);
      std::array<float, CURVE_BINS + 1> cum;
      cum[0] = 0.0f;
      for (int i = 0; i < CURVE_BINS; i++)
        cum[i + 1] = cum[i] + std::fmin(static_cast<float>(hist[i]), cap);
      const float total = cum[CURVE_BINS];
      for (int i = 0; i <= CURVE_BINS; i++)
        next[i] = (1.0f - e) * next[i] + e * 255.0f * cum[i] / total;
    }
  }
  if (this->curve_valid_ && this->curve_lo_ == lo && this->curve_hi_ == hi) {
    for (int i = 0; i <= CURVE_BINS; i++)
      this->curve_[i] += BLEND * (next[i] - this->curve_[i]);
  } else {
    this->curve_ = next;
    this->curve_valid_ = true;
    this->curve_lo_ = lo;
    this->curve_hi_ = hi;
  }
}

// The per-pixel loop is built for speed (the firmware is built for size,
// which neither schedules nor unrolls it).
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O2")
#endif
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
#pragma GCC unroll 4
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
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

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

// Built for speed, like map_direct(): it runs once per output pixel.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O2")
#endif
void CpuIsp::lookup(const uint8_t *idx, uint16_t w, uint16_t h, ImageFormat format, uint8_t *dst,
                    uint32_t stride) const {
  if (idx == nullptr || dst == nullptr)
    return;
  switch (format) {
    case ImageFormat::RGB565_LE:
      this->lookup_<ImageFormat::RGB565_LE>(idx, w, h, dst, stride);
      break;
    case ImageFormat::RGB565_BE:
      this->lookup_<ImageFormat::RGB565_BE>(idx, w, h, dst, stride);
      break;
    case ImageFormat::RGB888:
      this->lookup_<ImageFormat::RGB888>(idx, w, h, dst, stride);
      break;
    case ImageFormat::GRAY8:
      this->lookup_<ImageFormat::GRAY8>(idx, w, h, dst, stride);
      break;
  }
}

template<ImageFormat F>
void CpuIsp::lookup_(const uint8_t *idx, uint16_t w, uint16_t h, uint8_t *dst, uint32_t stride) const {
  const uint16_t *pal = this->pal565_le_.data();
  const Rgb *rgb = this->palette_.data();
  for (int y = 0; y < h; y++) {
    const uint8_t *src = idx + static_cast<size_t>(y) * w;
    uint8_t *p = dst + static_cast<size_t>(y) * stride;
#pragma GCC unroll 4
    for (int x = 0; x < w; x++) {
      const uint8_t i = src[x];
      if constexpr (F == ImageFormat::RGB565_LE) {
        const uint16_t c = pal[i];
        p[2 * x] = static_cast<uint8_t>(c & 0xFF);
        p[2 * x + 1] = static_cast<uint8_t>(c >> 8);
      } else if constexpr (F == ImageFormat::RGB565_BE) {
        const uint16_t c = pal[i];
        p[2 * x] = static_cast<uint8_t>(c >> 8);
        p[2 * x + 1] = static_cast<uint8_t>(c & 0xFF);
      } else if constexpr (F == ImageFormat::RGB888) {
        const Rgb &c = rgb[i];
        p[3 * x] = c.r;
        p[3 * x + 1] = c.g;
        p[3 * x + 2] = c.b;
      } else {
        p[x] = i;
      }
    }
  }
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

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

  if (o.upscale == Upscale::BICUBIC) {
    switch (o.format) {
      case ImageFormat::RGB565_LE:
        this->bicubic_<ImageFormat::RGB565_LE>(s, dst, stride);
        break;
      case ImageFormat::RGB565_BE:
        this->bicubic_<ImageFormat::RGB565_BE>(s, dst, stride);
        break;
      case ImageFormat::RGB888:
        this->bicubic_<ImageFormat::RGB888>(s, dst, stride);
        break;
      case ImageFormat::GRAY8:
        this->bicubic_<ImageFormat::GRAY8>(s, dst, stride);
        break;
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

// Bicubic (Catmull-Rom) on the index field, sampling at pixel centres like
// bilinear_(). Output pixel q * s + p (input pixel q, phase p) samples at
// q + (p + 0.5) / s - 0.5, so the taps and weights depend only on the phase:
// they are worked out once per phase (at most 20). Per output row the four
// source rows are blended once at input width, then each output pixel is a
// four-tap blend of that row plus a palette lookup. Catmull-Rom overshoots a
// little at edges (a slight sharpening); the result is clamped to the palette.
// Built for speed, like lookup(): it runs once per output pixel.
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O2")
#endif
template<ImageFormat F> void CpuIsp::bicubic_(int s, uint8_t *dst, uint32_t stride) const {
  const int w = this->w_, h = this->h_;
  constexpr size_t bpp = F == ImageFormat::RGB888 ? 3 : (F == ImageFormat::GRAY8 ? 1 : 2);
  int off[20];      // first tap relative to q
  int32_t wt[20][4];  // weights * 256, summing to 256
  for (int p = 0; p < s; p++) {
    const float pos = (p + 0.5f) / s - 0.5f;  // in (-0.5, 0.5)
    const int fl = pos < 0.0f ? -1 : 0;
    const float t = pos - fl, t2 = t * t, t3 = t2 * t;
    const float c[4] = {(-t3 + 2 * t2 - t) * 0.5f, (3 * t3 - 5 * t2 + 2) * 0.5f, (-3 * t3 + 4 * t2 + t) * 0.5f,
                        (t3 - t2) * 0.5f};
    off[p] = fl - 1;
    int32_t sum = 0;
    for (int k = 0; k < 4; k++) {
      wt[p][k] = static_cast<int32_t>(std::lround(c[k] * 256.0f));
      sum += wt[p][k];
    }
    wt[p][1] += 256 - sum;
  }
  std::vector<int32_t> blended(w);  // index * 256, after the vertical blend
  auto clampi = [](int v, int n) { return v < 0 ? 0 : (v >= n ? n - 1 : v); };
  for (int qy = 0; qy < h; qy++) {
    for (int py = 0; py < s; py++) {
      const uint8_t *r[4];
      for (int k = 0; k < 4; k++)
        r[k] = &this->idx_[clampi(qy + off[py] + k, h) * w];
      const int32_t *wy = wt[py];
      for (int x = 0; x < w; x++)
        blended[x] = wy[0] * r[0][x] + wy[1] * r[1][x] + wy[2] * r[2][x] + wy[3] * r[3][x];
      uint8_t *p = dst + static_cast<size_t>(qy * s + py) * stride;
      for (int qx = 0; qx < w; qx++) {
        for (int px = 0; px < s; px++, p += bpp) {
          const int b = qx + off[px];
          const int32_t *wx = wt[px];
          int32_t v = wx[0] * blended[clampi(b, w)] + wx[1] * blended[clampi(b + 1, w)] +
                      wx[2] * blended[clampi(b + 2, w)] + wx[3] * blended[clampi(b + 3, w)];
          v = v < 0 ? 0 : (v > (255 << 16) ? (255 << 16) : v);
          const uint8_t idx = static_cast<uint8_t>((v + (1 << 15)) >> 16);
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
  }
}

#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

}  // namespace isp
}  // namespace esphome
