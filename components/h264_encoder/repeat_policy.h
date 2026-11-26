#pragma once

#include <cstdint>

namespace esphome {
namespace h264_encoder {

// Keeps the stream at `min_fps` or more when the source is slower: the
// encoder encodes the last source frame again (a repeat) whenever no frame
// has gone out for 1/min_fps s. Viewers such as Home Assistant's stream
// worker and FFmpeg need several frames before they report timestamps, which
// takes half a minute from a source delivering one frame every 4 s.
//
// Repeats stop MAX_REPEAT_MS after the last source frame, so a source that
// has stopped shows as a stopped stream, not as a frozen picture.
struct RepeatPolicy {
  static constexpr uint32_t MAX_REPEAT_MS = 10000;

  uint8_t min_fps{0};  // 0 = no repeats

  bool enabled() const { return this->min_fps != 0; }
  uint32_t period_ms() const { return this->enabled() ? 1000u / this->min_fps : 0; }

  // A source slower than min_fps (or not yet measured) has its frames copied,
  // so the last one is still there to repeat. `interval_x16` is the mean
  // frame interval in 1/16 ms (0 = not measured).
  bool source_slow(uint32_t interval_x16) const {
    if (!this->enabled())
      return false;
    if (interval_x16 == 0)
      return true;
    // 16000 / interval_x16 < min_fps, without division.
    return 16000u < static_cast<uint32_t>(this->min_fps) * interval_x16;
  }

  // Milliseconds until a repeat is due, given when a frame last went out.
  uint32_t wait_ms(uint32_t now_ms, uint32_t last_out_ms) const {
    const uint32_t since = now_ms - last_out_ms, period = this->period_ms();
    return since >= period ? 0 : period - since;
  }

  // Whether the last source frame is recent enough to repeat.
  bool may_repeat(uint32_t now_ms, uint32_t last_source_ms) const {
    return this->enabled() && now_ms - last_source_ms <= MAX_REPEAT_MS;
  }

  // The frame rate the stream declares: the source's, but not below min_fps.
  uint8_t declared_fps(uint8_t source_fps) const { return source_fps < this->min_fps ? this->min_fps : source_fps; }
};

// Stream timestamps never go backwards or repeat: a source frame captured
// before a repeat went out is stamped just after that repeat.
inline uint32_t next_timestamp(uint32_t candidate_ms, uint32_t last_ms, bool have_last) {
  if (have_last && static_cast<int32_t>(candidate_ms - last_ms) <= 0)
    return last_ms + 1;
  return candidate_ms;
}

}  // namespace h264_encoder
}  // namespace esphome
