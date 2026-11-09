#include "radiometric_core.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace radiometric {



void Processor::set_options(const Options &o) {
  Options n = o;
  n.temporal_frames = std::min<uint8_t>(std::max<uint8_t>(n.temporal_frames, 1), MAX_TEMPORAL);
  if (n.temporal_frames != this->opt_.temporal_frames) {
    this->history_count_ = 0;  // start the average afresh at the new depth
    this->history_next_ = 0;
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

bool Processor::process(const float *in, float *out, uint16_t width, uint16_t height) {
  const size_t n = static_cast<size_t>(width) * height;
  if (n == 0 || in == nullptr || out == nullptr)
    return false;
  if (width != this->w_ || height != this->h_) {
    this->w_ = width;
    this->h_ = height;
    this->history_.reset();
    this->history_count_ = this->history_next_ = 0;
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

  // 3. Temporal average.
  const uint8_t depth = this->opt_.temporal_frames;
  if (depth > 1 && this->history_.size() != n * MAX_TEMPORAL) {
    this->history_count_ = this->history_next_ = 0;
    if (!this->history_.ensure(n * MAX_TEMPORAL))
      this->history_.reset();  // out of memory: no averaging this frame
  }
  if (depth > 1 && this->history_.size() == n * MAX_TEMPORAL) {
    std::memcpy(&this->history_[static_cast<size_t>(this->history_next_) * n], w, n * sizeof(float));
    this->history_next_ = static_cast<uint8_t>((this->history_next_ + 1) % depth);
    if (this->history_count_ < depth)
      this->history_count_++;
    for (size_t i = 0; i < n; i++) {
      float sum = 0.0f;
      for (uint8_t k = 0; k < this->history_count_; k++)
        sum += this->history_[k * n + i];
      w[i] = sum / this->history_count_;
    }
  }

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

}  // namespace radiometric
}  // namespace esphome
