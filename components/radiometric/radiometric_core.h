#pragma once

// Temperature-domain processing of radiometric frames: invalid-pixel repair,
// isolated-outlier filter, motion-adaptive temporal filter, edge-preserving
// (bilateral) smoothing, 3x3 median, and statistics (min/max with positions,
// mean, median, centre). Positions are in sensor coordinates; geometry
// (mirroring, scaling) belongs to the ISP.
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
  // Temporal filter: a still pixel is averaged over about this many frames;
  // a pixel whose neighbourhood changed by more than motion_threshold takes
  // the new reading at once (no smearing). 1 = off; at most MAX_TEMPORAL.
  uint8_t temporal_frames{1};
  float motion_threshold{0.5f};  // degrees: mean 3x3 change treated as noise
  // Edge-preserving smoothing: neighbours within about this many degrees are
  // averaged, larger differences (edges) are kept. 0 = off.
  float edge_smoothing{0.0f};
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

static constexpr uint8_t MAX_TEMPORAL = 16;

// Sensors alone need one frame per `period_ms`. Whether one is due at `now`
// when the last was taken at `last` (0: none yet). Correct across millis() wrap.
inline bool sensor_frame_due(uint32_t now, uint32_t last, uint32_t period_ms) {
  return last == 0 || now - last >= period_ms;
}

class Processor {
 public:
  void set_options(const Options &o);
  const Options &options() const { return this->opt_; }

  // Processes `in` (width*height temperatures) into `out` (same size; may
  // alias `in`) and updates stats(). A size change restarts the temporal filter.
  // False (stats invalid, `out` untouched) when its working memory cannot be
  // allocated.
  bool process(const float *in, float *out, uint16_t width, uint16_t height);
  const Stats &stats() const { return this->stats_; }

  // True when a filter changes pixels (outlier, temporal, smoothing or median).
  bool filters_active() const {
    return this->opt_.outlier_level > 0.0f || this->opt_.temporal_frames > 1 || this->opt_.median_filter ||
           this->opt_.edge_smoothing > 0.0f;
  }
  // With no filter active: the statistics process() would compute, from one
  // read-only pass over `in`, so the frame itself can be passed on as it is.
  // False (stats untouched) if a pixel is invalid and needs process() to
  // repair it, or memory for the median is missing.
  bool analyze(const float *in, uint16_t width, uint16_t height);
  // The same as analyze() from statistics the source computed with the frame,
  // without reading it (only the centre pixel). False (stats untouched) if
  // they cannot stand in: not valid, a median is wanted, or a pixel lies
  // outside valid_min..valid_max.
  bool adopt(const video::FrameStats &fs, const float *in, uint16_t width, uint16_t height);

 protected:
  void temporal_(float *w, size_t n);
  void bilateral_(float *w);

  Options opt_;
  uint16_t w_{0}, h_{0};
  video::LargeBuffer<float> state_;  // the temporal filter's output, only while it is on
  bool state_valid_{false};
  float range_lut_[64]{};  // bilateral weight by temperature difference
  float range_lut_scale_{0.0f};
  video::LargeBuffer<float> work_, tmp_;
  Stats stats_;
};

}  // namespace radiometric
}  // namespace esphome
