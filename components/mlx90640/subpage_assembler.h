#pragma once

// Builds MLX90640 frames from sub-pages. The sensor measures half of its
// pixels per sub-page (chess or interleaved pattern); a frame is either each
// sub-page with its other half interpolated from it, or a pair of sub-pages
// merged. No ESPHome or ESP-IDF dependency; unit-tested on a host.

#include <array>
#include <cstdint>

namespace esphome {
namespace mlx90640 {

static constexpr int SENSOR_COLS = 32;
static constexpr int SENSOR_ROWS = 24;
static constexpr int SENSOR_PIXELS = SENSOR_COLS * SENSOR_ROWS;

enum class FrameMode : uint8_t {
  INTERPOLATE = 0,  // a frame per sub-page; the unmeasured half is interpolated
  MERGE = 1,        // a frame per sub-page pair, measured pixels only
};

class SubpageAssembler {
 public:
  // Restarts assembly.
  void set_mode(FrameMode m);
  FrameMode mode() const { return this->mode_; }

  // `temps`: 768 temperatures in which this sub-page's pixels are fresh.
  // `chess`: pixel (r,c) belongs to sub-page (r+c)&1, else to sub-page r&1.
  // Returns true when frame() holds a new frame.
  bool add(const float *temps, int subpage, bool chess);
  const float *frame() const { return this->frame_.data(); }

 protected:
  FrameMode mode_{FrameMode::INTERPOLATE};
  std::array<float, SENSOR_PIXELS> frame_{};
  uint8_t seen_{0};
};

}  // namespace mlx90640
}  // namespace esphome
