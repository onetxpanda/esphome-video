#pragma once

// Temperature-domain processing of radiometric frames: invalid-pixel repair,
// isolated-outlier filter, temporal average, 3x3 median, and statistics
// (min/max with positions, mean, median, centre). Positions are in sensor
// coordinates; geometry (mirroring, scaling) belongs to the ISP.
//
// No ESPHome or ESP-IDF dependency; unit-tested on a host.

#include <cstddef>
#include <cstdint>

#include "../video_core/frame.h"

namespace esphome {
namespace radiometric {

static constexpr float MIN_VALID_C = -40.0f;
static constexpr float MAX_VALID_C = 300.0f;

struct Options {
  uint8_t temporal_frames{1};  // 1 = off; at most MAX_TEMPORAL
  bool median_filter{false};
  float outlier_level{10.0f};  // degrees; <= 0 disables
  // Readings outside this range (or not finite) are invalid and repaired
  // from their neighbours; the defaults suit the MLX90640.
  float valid_min{MIN_VALID_C};
  float valid_max{MAX_VALID_C};
  // Whether to compute the median statistic (a full sort-like pass); NaN when not.
  bool median_stat{true};
};

struct Stats {
  bool valid{false};
  float min{0}, max{0}, mean{0}, median{0}, center{0};  // degrees C
  uint16_t min_x{0}, min_y{0}, max_x{0}, max_y{0};      // sensor coordinates
};

static constexpr uint8_t MAX_TEMPORAL = 8;

class Processor {
 public:
  void set_options(const Options &o);
  const Options &options() const { return this->opt_; }

  // Processes `in` (width*height temperatures) into `out` (same size; may
  // alias `in`) and updates stats(). A size change restarts the history.
  // False (stats invalid, `out` untouched) when its working memory cannot be
  // allocated.
  bool process(const float *in, float *out, uint16_t width, uint16_t height);
  const Stats &stats() const { return this->stats_; }

  // True when a filter changes pixels (outlier filter, averaging or median).
  bool filters_active() const {
    return this->opt_.outlier_level > 0.0f || this->opt_.temporal_frames > 1 || this->opt_.median_filter;
  }
  // With no filter active: the statistics process() would compute, from one
  // read-only pass over `in`, so the frame itself can be passed on as it is.
  // False (stats untouched) if a pixel is invalid and needs process() to
  // repair it, or memory for the median is missing.
  bool analyze(const float *in, uint16_t width, uint16_t height);

 protected:
  Options opt_;
  uint16_t w_{0}, h_{0};
  video::LargeBuffer<float> history_;  // MAX_TEMPORAL frames, only while averaging
  uint8_t history_count_{0}, history_next_{0};
  video::LargeBuffer<float> work_, tmp_;
  Stats stats_;
};

}  // namespace radiometric
}  // namespace esphome
