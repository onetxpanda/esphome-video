#pragma once

// Which camera a camera_switch passes on: the first one in its list that has
// sent a frame within the timeout. No ESPHome dependency (unit-tested).

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace camera_switch {

static constexpr int NO_CAMERA = -1;

// last_ms[i]: when camera i last sent a frame; seen[i]: whether it ever has.
// With none sending, the choice stays `current` (the picture last shown).
inline int choose_camera(const uint32_t *last_ms, const bool *seen, size_t count, uint32_t now_ms, uint32_t timeout_ms,
                         int current) {
  for (size_t i = 0; i < count; i++)
    if (seen[i] && now_ms - last_ms[i] <= timeout_ms)
      return static_cast<int>(i);
  return current;
}

}  // namespace camera_switch
}  // namespace esphome
