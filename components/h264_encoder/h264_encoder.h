#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"

#include <atomic>
#include <mutex>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../video_core/frame.h"
#include "encoded_sink.h"
#include "esp_h264_enc_single_hw.h"

namespace esphome {
namespace h264_encoder {

class H264Encoder;

// The encoder's input for one source. Each configured source gets its own,
// so the encoder knows where a frame came from; only the active source's
// frames are encoded, and inactive sources are told nobody wants frames.
class SourceInput : public video::ImageSink {
 public:
  SourceInput(H264Encoder *encoder, uint8_t index) : encoder_(encoder), index_(index) {}
  void on_image_frame(const video::ImageFrame &frame) override;
  bool wants_frames() const override;
  void on_source_change() override;

 protected:
  H264Encoder *encoder_;
  uint8_t index_;
};

// ESP32-P4 hardware H.264 encoder. Takes image frames from one of its
// sources (on the source's task) and encodes them on its own task. A frame
// already in the encoder's layout (RGB565 little-endian, both sizes a
// multiple of 16, packed rows) is encoded from the source's buffer, held
// until the hardware is done; other frames are converted into a DMA buffer.
// One frame is in flight at a time; frames arriving meanwhile are dropped.
// Nothing is encoded while no EncodedFrameSink wants frames.
class H264Encoder : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // 0 = from the measured source frame rate.
  void set_fps(uint8_t fps) { this->fps_ = fps; }
  // 0 = one keyframe per second.
  void set_gop(uint8_t gop) { this->gop_ = gop; }
  void set_bitrate(uint32_t bitrate) { this->bitrate_ = bitrate; }
  void set_qp(uint8_t qp_min, uint8_t qp_max) {
    this->qp_min_ = qp_min;
    this->qp_max_ = qp_max;
  }

  void add_encoded_sink(EncodedFrameSink *sink) { this->sinks_.push_back(sink); }
  // The next encoded frame will be a keyframe (IDR). Thread-safe.
  void request_keyframe() { this->keyframe_requested_ = true; }

  // Registers a source; returns the sink to hand to that source.
  video::ImageSink *add_source(const char *name) {
    auto *input = new SourceInput(this, static_cast<uint8_t>(this->source_names_.size()));  // NOLINT
    this->source_names_.push_back(name);
    return input;
  }
  size_t source_count() const { return this->source_names_.size(); }
  // Size of the stream being encoded; 0 until the encoder has opened.
  uint16_t get_encoded_width() const { return this->enc_w_; }
  uint16_t get_encoded_height() const { return this->enc_h_; }
  uint8_t get_active_source() const { return this->active_source_; }
  // Switches the encoded source. The next frame is a keyframe; a different
  // size or format reconfigures the encoder. Thread-safe.
  void set_active_source(uint8_t index);
  // The source at `index` now shows different content: if it is the one
  // being encoded, start afresh as on a source switch.
  void source_changed(uint8_t index);

  // Called by SourceInput, on the source's task.
  void handle_frame(uint8_t source, const video::ImageFrame &frame);
  bool source_wanted(uint8_t source) const { return source == this->active_source_ && this->consumers_want_frames_(); }

 protected:
  bool consumers_want_frames_() const;
  static void task_entry_(void *arg);
  void task_loop_();
  bool prepare_buffers_(uint16_t width, uint16_t height, esp_h264_raw_format_t type, bool convert);
  void free_buffers_();
  bool convert_(const video::ImageFrame &frame);
  bool open_encoder_();
  void close_encoder_();
  uint8_t target_fps_() const;
  void note_interval_(uint32_t ts);

  // Configuration
  uint8_t fps_{0};
  uint8_t gop_{0};
  uint32_t bitrate_{0};  // 0 = automatic
  uint8_t qp_min_{20};
  uint8_t qp_max_{40};

  std::vector<EncodedFrameSink *> sinks_;
  std::vector<const char *> source_names_;
  std::atomic<uint8_t> active_source_{0};
  TaskHandle_t task_{nullptr};

  // Hand-over between the source's task and the encoder task. While busy_
  // is set the encoder task owns the fields below.
  std::atomic<bool> busy_{false};
  std::atomic<bool> keyframe_requested_{false};
  std::atomic<bool> was_active_{false};
  video::FrameRef held_;        // zero-copy input, released after encoding
  const uint8_t *in_ptr_{nullptr};
  uint32_t in_len_{0};
  uint8_t *in_buf_{nullptr};    // conversion buffer
  uint32_t in_cap_{0};
  uint8_t *out_buf_{nullptr};
  uint32_t out_cap_{0};
  uint16_t width_{0};
  uint16_t height_{0};
  uint16_t aligned_w_{0};
  uint16_t aligned_h_{0};
  esp_h264_raw_format_t type_{ESP_H264_RAW_FMT_RGB565_LE};
  uint32_t frame_ts_{0};

  // Source frame rate, measured on the source's task.
  std::atomic<uint32_t> interval_x16_{0};  // mean frame interval, 1/16 ms
  uint32_t last_ts_{0};
  bool have_last_ts_{false};

  // Owned by the encoder task.
  esp_h264_enc_handle_t enc_{nullptr};
  esp_h264_enc_param_hw_handle_t param_{nullptr};
  uint16_t enc_w_{0};
  uint16_t enc_h_{0};
  esp_h264_raw_format_t enc_type_{ESP_H264_RAW_FMT_RGB565_LE};
  uint8_t enc_fps_{0};
  uint32_t enc_opened_ms_{0};
  bool drifting_{false};  // measured rate off from the declared one since drift_since_ms_
  uint32_t drift_since_ms_{0};

  // Statistics
  std::atomic<uint32_t> encoded_frames_{0};
  std::atomic<uint32_t> dropped_frames_{0};
  std::atomic<uint32_t> encode_errors_{0};
  std::atomic<uint32_t> encoded_bytes_{0};
  std::atomic<uint32_t> keyframes_{0};
  std::atomic<uint32_t> zero_copy_{0};
  uint32_t last_stats_ms_{0};
  std::atomic<bool> format_error_logged_{false};
};

}  // namespace h264_encoder
}  // namespace esphome
