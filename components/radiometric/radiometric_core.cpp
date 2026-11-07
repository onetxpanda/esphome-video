#include "radiometric_core.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace esphome {
namespace radiometric {

static inline bool valid_temp(float t) { return std::isfinite(t) && t >= MIN_VALID_C && t <= MAX_VALID_C; }

void Processor::set_options(const Options &o) {
  Options n = o;
  n.temporal_frames = std::min<uint8_t>(std::max<uint8_t>(n.temporal_frames, 1), MAX_TEMPORAL);
  if (n.temporal_frames != this->opt_.temporal_frames) {
    this->history_count_ = 0;  // start the average afresh at the new depth
    this->history_next_ = 0;
  }
  this->opt_ = n;
}

void Processor::process(const float *in, float *out, uint16_t width, uint16_t height) {
  const size_t n = static_cast<size_t>(width) * height;
  if (n == 0 || in == nullptr || out == nullptr)
    return;
  if (width != this->w_ || height != this->h_) {
    this->w_ = width;
    this->h_ = height;
    this->history_.assign(n * MAX_TEMPORAL, 0.0f);
    this->history_count_ = this->history_next_ = 0;
    this->work_.resize(n);
    this->tmp_.resize(n);
  }
  float *w = this->work_.data();
  std::memcpy(w, in, n * sizeof(float));

  // 1. Invalid pixels -> mean of valid 3x3 neighbours.
  std::memcpy(this->tmp_.data(), w, n * sizeof(float));
  const float *src = this->tmp_.data();
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
  if (depth > 1) {
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
    float win[9];
    for (int r = 0; r < height; r++)
      for (int c = 0; c < width; c++) {
        int k = 0;
        for (int dr = -1; dr <= 1; dr++) {
          const int rr = std::min(std::max(r + dr, 0), height - 1);
          for (int dc = -1; dc <= 1; dc++)
            win[k++] = src[rr * width + std::min(std::max(c + dc, 0), width - 1)];
        }
        std::nth_element(win, win + 4, win + 9);
        w[r * width + c] = win[4];
      }
  }

  // 5. Statistics.
  Stats s;
  s.valid = true;
  size_t mi = 0, ma = 0;
  double sum = 0;
  for (size_t i = 0; i < n; i++) {
    if (w[i] < w[mi])
      mi = i;
    if (w[i] > w[ma])
      ma = i;
    sum += w[i];
  }
  s.min = w[mi];
  s.max = w[ma];
  s.min_x = static_cast<uint16_t>(mi % width);
  s.min_y = static_cast<uint16_t>(mi / width);
  s.max_x = static_cast<uint16_t>(ma % width);
  s.max_y = static_cast<uint16_t>(ma / width);
  s.mean = static_cast<float>(sum / n);
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
  s.center = w[(height / 2) * width + width / 2];
  this->stats_ = s;

  if (out != w)
    std::memcpy(out, w, n * sizeof(float));
}

}  // namespace radiometric
}  // namespace esphome
