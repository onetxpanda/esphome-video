#include "radiometric_core.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace radiometric {

void Processor::set_options(const Options &o) {
  Options n = o;
  n.temporal_frames = std::min<uint8_t>(std::max<uint8_t>(n.temporal_frames, 1), MAX_TEMPORAL);
  n.motion_threshold = std::max(n.motion_threshold, 0.01f);
  n.edge_smoothing = std::max(n.edge_smoothing, 0.0f);
  if (n.temporal_frames == 1)
    this->state_valid_ = false;  // turned off: start afresh when turned on again
  if (n.edge_smoothing > 0.0f && n.edge_smoothing != this->opt_.edge_smoothing) {
    // Gaussian of the temperature difference, out to 3 sigma (sigma = edge_smoothing).
    for (int k = 0; k < 64; k++) {
      const float d = 3.0f * k / 63.0f;
      this->range_lut_[k] = std::exp(-0.5f * d * d);
    }
    this->range_lut_scale_ = 63.0f / (3.0f * n.edge_smoothing);
  }
  this->opt_ = n;
}

bool Processor::analyze(const float *in, uint16_t width, uint16_t height) {
  const size_t n = static_cast<size_t>(width) * height;
  if (n == 0 || in == nullptr || (this->opt_.median_stat && !this->tmp_.ensure(n)))
    return false;
  const float lo = this->opt_.valid_min, hi = this->opt_.valid_max;
  size_t mi = 0, ma = 0;
  double sum = 0;
  for (int r = 0; r < height; r++) {
    const size_t row = static_cast<size_t>(r) * width;
    float row_sum = 0.0f;
    for (size_t i = row; i < row + width; i++) {
      const float v = in[i];
      if (!(std::isfinite(v) && v >= lo && v <= hi))
        return false;
      if (v < in[mi])
        mi = i;
      if (v > in[ma])
        ma = i;
      row_sum += v;
    }
    sum += row_sum;
  }
  Stats s;
  s.valid = true;
  s.min = in[mi];
  s.max = in[ma];
  s.min_x = static_cast<uint16_t>(mi % width);
  s.min_y = static_cast<uint16_t>(mi / width);
  s.max_x = static_cast<uint16_t>(ma % width);
  s.max_y = static_cast<uint16_t>(ma / width);
  s.mean = static_cast<float>(sum / n);
  if (this->opt_.median_stat) {
    std::memcpy(this->tmp_.data(), in, n * sizeof(float));
    float *t = this->tmp_.data();
    std::nth_element(t, t + n / 2, t + n);
    const float upper = t[n / 2];
    s.median = n % 2 == 0 ? (*std::max_element(t, t + n / 2) + upper) / 2.0f : upper;
  } else {
    s.median = NAN;
  }
  s.center = in[(height / 2) * width + width / 2];
  this->stats_ = s;
  return true;
}

bool Processor::adopt(const video::FrameStats &fs, const float *in, uint16_t width, uint16_t height) {
  if (!fs.valid || this->opt_.median_stat || in == nullptr || width == 0 || height == 0)
    return false;
  // Every pixel lies between the min and the max.
  if (!(fs.min >= this->opt_.valid_min && fs.max <= this->opt_.valid_max))
    return false;
  Stats s;
  s.valid = true;
  s.min = fs.min;
  s.max = fs.max;
  s.mean = fs.mean;
  s.min_x = fs.min_x;
  s.min_y = fs.min_y;
  s.max_x = fs.max_x;
  s.max_y = fs.max_y;
  s.median = NAN;
  s.center = in[(height / 2) * width + width / 2];
  this->stats_ = s;
  return true;
}

bool Processor::process(const float *in, float *out, uint16_t width, uint16_t height) {
  const size_t n = static_cast<size_t>(width) * height;
  if (n == 0 || in == nullptr || out == nullptr)
    return false;
  if (width != this->w_ || height != this->h_) {
    this->w_ = width;
    this->h_ = height;
    this->state_.reset();
    this->state_valid_ = false;
  }
  if (!this->work_.ensure(n) || !this->tmp_.ensure(n)) {
    this->stats_.valid = false;
    return false;
  }
  float *w = this->work_.data();
  std::memcpy(w, in, n * sizeof(float));
  const float lo = this->opt_.valid_min, hi = this->opt_.valid_max;
  auto valid_temp = [lo, hi](float t) { return std::isfinite(t) && t >= lo && t <= hi; };

  // 1. Invalid pixels -> mean of valid 3x3 neighbours, read from the input
  // (`in` may alias `out`, which is written only at the end).
  const float *src = in;
  for (int r = 0; r < height; r++) {
    for (int c = 0; c < width; c++) {
      if (valid_temp(src[r * width + c]))
        continue;
      float sum = 0.0f;
      int k = 0;
      for (int dr = -1; dr <= 1; dr++)
        for (int dc = -1; dc <= 1; dc++) {
          const int rr = r + dr, cc = c + dc;
          if ((dr || dc) && rr >= 0 && rr < height && cc >= 0 && cc < width && valid_temp(src[rr * width + cc])) {
            sum += src[rr * width + cc];
            k++;
          }
        }
      w[r * width + c] = k ? sum / k : 20.0f;
    }
  }

  // 2. Isolated-pixel filter along rows.
  if (this->opt_.outlier_level > 0.0f) {
    const float level = this->opt_.outlier_level;
    for (size_t i = 1; i + 1 < n; i++)
      if (std::fabs(w[i] - w[i - 1]) >= level && std::fabs(w[i] - w[i + 1]) >= level)
        w[i] = (w[i - 1] + w[i + 1]) / 2.0f;
  }

  // 3. Temporal filter, then edge-preserving smoothing.
  if (this->opt_.temporal_frames > 1)
    this->temporal_(w, n);
  if (this->opt_.edge_smoothing > 0.0f)
    this->bilateral_(w);

  // 4. 3x3 median, edges replicated.
  if (this->opt_.median_filter) {
    std::memcpy(this->tmp_.data(), w, n * sizeof(float));
    const float *med = this->tmp_.data();  // the frame before this filter
    float win[9];
    for (int r = 0; r < height; r++)
      for (int c = 0; c < width; c++) {
        int k = 0;
        for (int dr = -1; dr <= 1; dr++) {
          const int rr = std::min(std::max(r + dr, 0), height - 1);
          for (int dc = -1; dc <= 1; dc++)
            win[k++] = med[rr * width + std::min(std::max(c + dc, 0), width - 1)];
        }
        std::nth_element(win, win + 4, win + 9);
        w[r * width + c] = win[4];
      }
  }

  // 5. Statistics.
  Stats s;
  s.valid = true;
  size_t mi = 0, ma = 0;
  // Rows summed in float, rows added in double: the ESP32-P4's FPU is single
  // precision, so a double add per pixel would be emulated in software.
  double sum = 0;
  for (int r = 0; r < height; r++) {
    const size_t row = static_cast<size_t>(r) * width;
    float row_sum = 0.0f;
    for (size_t i = row; i < row + width; i++) {
      if (w[i] < w[mi])
        mi = i;
      if (w[i] > w[ma])
        ma = i;
      row_sum += w[i];
    }
    sum += row_sum;
  }
  s.min = w[mi];
  s.max = w[ma];
  s.min_x = static_cast<uint16_t>(mi % width);
  s.min_y = static_cast<uint16_t>(mi / width);
  s.max_x = static_cast<uint16_t>(ma % width);
  s.max_y = static_cast<uint16_t>(ma / width);
  s.mean = static_cast<float>(sum / n);
  if (this->opt_.median_stat) {
    std::memcpy(this->tmp_.data(), w, n * sizeof(float));
    float *t = this->tmp_.data();
    std::nth_element(t, t + n / 2, t + n);
    const float upper = t[n / 2];
    if (n % 2 == 0) {
      const float lower = *std::max_element(t, t + n / 2);
      s.median = (lower + upper) / 2.0f;
    } else {
      s.median = upper;
    }
  } else {
    s.median = NAN;
  }
  s.center = w[(height / 2) * width + width / 2];
  this->stats_ = s;

  if (out != w)
    std::memcpy(out, w, n * sizeof(float));
  return true;
}

// Recursive filter with motion detection. Each pixel's output moves towards
// the new reading by a = 1/temporal_frames while its 3x3 neighbourhood
// changed by no more than motion_threshold on average (noise), and by up to
// a = 1 as the change reaches twice that (movement), so still areas are
// averaged over many frames while moving edges are not smeared. The
// neighbourhood keeps single noisy pixels from passing as movement.
void Processor::temporal_(float *w, size_t n) {
  if (!this->state_.ensure(n)) {
    this->state_valid_ = false;  // out of memory: no filtering this frame
    return;
  }
  float *s = this->state_.data();
  if (!this->state_valid_) {
    std::memcpy(s, w, n * sizeof(float));
    this->state_valid_ = true;
    return;
  }
  const int width = this->w_, height = this->h_;
  float *d = this->tmp_.data();
  for (size_t i = 0; i < n; i++)
    d[i] = std::fabs(w[i] - s[i]);
  const float a_min = 1.0f / this->opt_.temporal_frames;
  const float t = this->opt_.motion_threshold, inv_t = 1.0f / t;
  for (int r = 0; r < height; r++) {
    const int r0 = std::max(r - 1, 0), r1 = std::min(r + 1, height - 1);
    for (int c = 0; c < width; c++) {
      const int c0 = std::max(c - 1, 0), c1 = std::min(c + 1, width - 1);
      float sum = 0.0f;
      for (int rr = r0; rr <= r1; rr++)
        for (int cc = c0; cc <= c1; cc++)
          sum += d[rr * width + cc];
      const float change = sum / static_cast<float>((r1 - r0 + 1) * (c1 - c0 + 1));
      const float m = std::fmin(std::fmax((change - t) * inv_t, 0.0f), 1.0f);
      const float a = a_min + (1.0f - a_min) * m;
      const size_t i = static_cast<size_t>(r) * width + c;
      s[i] += a * (w[i] - s[i]);
    }
  }
  std::memcpy(w, s, n * sizeof(float));
}

// 5x5 bilateral filter: neighbours are weighted by distance (Gaussian,
// sigma 1 pixel) and by temperature difference (Gaussian, sigma
// edge_smoothing, nothing beyond 3 sigma), so noise is averaged away while
// edges between warmer and cooler areas stay sharp.
void Processor::bilateral_(float *w) {
  const int width = this->w_, height = this->h_;
  const size_t n = static_cast<size_t>(width) * height;
  std::memcpy(this->tmp_.data(), w, n * sizeof(float));
  const float *src = this->tmp_.data();
  static const float SPATIAL[3][3] = {
      {1.0f, 0.60653066f, 0.13533528f},        // exp(-(dx^2+dy^2)/2) for |dy| = 0
      {0.60653066f, 0.36787944f, 0.08208500f},  // |dy| = 1
      {0.13533528f, 0.08208500f, 0.01831564f},  // |dy| = 2
  };
  const float k = this->range_lut_scale_;
  for (int r = 0; r < height; r++) {
    for (int c = 0; c < width; c++) {
      const float v = src[r * width + c];
      float sum = 0.0f, wsum = 0.0f;
      for (int dr = -2; dr <= 2; dr++) {
        const int rr = r + dr;
        if (rr < 0 || rr >= height)
          continue;
        for (int dc = -2; dc <= 2; dc++) {
          const int cc = c + dc;
          if (cc < 0 || cc >= width)
            continue;
          const float u = src[rr * width + cc];
          const float q = std::fabs(u - v) * k;
          if (!(q < 64.0f))
            continue;
          const float wt = SPATIAL[std::abs(dr)][std::abs(dc)] * this->range_lut_[static_cast<int>(q)];
          sum += wt * u;
          wsum += wt;
        }
      }
      w[r * width + c] = sum / wsum;  // wsum >= 1: the centre pixel weighs 1
    }
  }
}

}  // namespace radiometric
}  // namespace esphome
