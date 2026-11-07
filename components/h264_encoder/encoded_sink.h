#pragma once

// Receives the H.264 encoder's output: Annex-B access units, with SPS/PPS
// before every IDR. Called from the encoder's task; `data` is only valid
// during the call.

#include <cstddef>
#include <cstdint>

namespace esphome {
namespace h264_encoder {

class EncodedFrameSink {
 public:
  virtual ~EncodedFrameSink() = default;
  virtual void on_encoded_frame(const uint8_t *data, size_t length, uint32_t timestamp_ms, bool keyframe) = 0;
  // False while this sink has no consumers; the encoder idles when no sink wants frames.
  virtual bool wants_frames() const = 0;
};

}  // namespace h264_encoder
}  // namespace esphome
