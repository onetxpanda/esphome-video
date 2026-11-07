#pragma once

// Temperature-domain processing of radiometric frames: invalid-pixel repair,
// isolated-outlier filter, temporal average, 3x3 median, and statistics
// (min/max with positions, mean, median, centre). Positions are in sensor
// coordinates; geometry (mirroring, scaling) belongs to the ISP.
//
// No ESPHome or ESP-IDF dependency; unit-tested on a host.

#include <cstddef>
#include <cstdint>
#include <vector>

namespace esphome {
namespace radiometric {

struct Options {
  uint8_t temporal_frames{1};  // 1 = off; at most MAX_TEMPORAL
  bool median_filter{false};
  float outlier_level{10.0f};  // degrees; <= 0 disables
};

struct Stats {
  bool valid{false};
  float min{0}, max{0}, mean{0}, median{0}, center{0};  // degrees C
  uint16_t min_x{0}, min_y{0}, max_x{0}, max_y{0};      // sensor coordinates
};

static constexpr uint8_t MAX_TEMPORAL = 8;
static constexpr float MIN_VALID_C = -40.0f;
static constexpr float MAX_VALID_C = 300.0f;

class Processor {
 public:
  void set_options(const Options &o);
  const Options &options() const { return this->opt_; }

  // Processes `in` (width*height temperatures) into `out` (same size; may
  // alias `in`) and updates stats(). A size change restarts the history.
  void process(const float *in, float *out, uint16_t width, uint16_t height);
  const Stats &stats() const { return this->stats_; }

 protected:
  Options opt_;
  uint16_t w_{0}, h_{0};
  std::vector<float> history_;  // MAX_TEMPORAL frames
  uint8_t history_count_{0}, history_next_{0};
  std::vector<float> work_, tmp_;
  Stats stats_;
};

}  // namespace radiometric
}  // namespace esphome
