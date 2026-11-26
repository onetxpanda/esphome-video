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
#include "repeat_policy.h"
#include "slot_exchange.h"

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
// sources (on the source's task) and encodes them on its own task. Frames
// pass through three slots (SlotExchange), so neither task waits for the
// other: the encoder encodes its slot while the source fills another. A
// frame already in the encoder's layout (RGB565 little-endian, both sizes a
// multiple of 16, packed rows) from a fast source is held, not copied, until
// the hardware is done; other frames are converted into the slot's buffer.
// If the encoder is still busy when a newer frame is ready, the older ready
// frame is dropped. Nothing is encoded while no EncodedFrameSink wants
// frames, unless always_on is set. A source slower than min_fps has its
// frames copied, and the last one is encoded again to keep the stream at
// min_fps (see RepeatPolicy).
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
  // 0 = no repeats: the stream runs at the source's rate however slow.
  void set_min_fps(uint8_t min_fps) { this->repeat_.min_fps = min_fps; }
  // Encode even while no EncodedFrameSink wants frames.
  void set_always_on(bool always_on) { this->always_on_ = always_on; }
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
  // One frame on its way to the encoder.
  struct Slot {
    uint8_t *buf{nullptr};  // conversion buffer, kept for reuse
    uint32_t cap{0};
    const uint8_t *data{nullptr};  // what to encode: buf, or a held source buffer
    uint32_t len{0};
    video::FrameRef held;  // the source's buffer, when not copied
    uint16_t width{0};
    uint16_t height{0};
    esp_h264_raw_format_t type{ESP_H264_RAW_FMT_RGB565_LE};
    uint32_t timestamp_ms{0};
    bool copied{false};  // data is buf: can be encoded again (repeated)
  };
  static uint32_t raw_size_(uint16_t width, uint16_t height, esp_h264_raw_format_t type);
  static void convert_(const video::ImageFrame &frame, uint8_t *dst);
  bool prepare_output_(const Slot &slot);
  bool open_encoder_(const Slot &slot);
  void close_encoder_();
  uint8_t target_fps_() const;
  void note_interval_(uint32_t ts);
  bool repeat_ready_() const;

  // Configuration
  uint8_t fps_{0};
  uint8_t gop_{0};
  uint32_t bitrate_{0};  // 0 = automatic
  uint8_t qp_min_{20};
  uint8_t qp_max_{40};
  RepeatPolicy repeat_;
  bool always_on_{false};

  std::vector<EncodedFrameSink *> sinks_;
  std::vector<const char *> source_names_;
  std::atomic<uint8_t> active_source_{0};
  TaskHandle_t task_{nullptr};

  // Hand-over between the source's task and the encoder task: a slot is
  // touched only by the side SlotExchange gives it to.
  Slot slots_[SlotExchange::SLOTS];
  SlotExchange exchange_;
  std::atomic<bool> keyframe_requested_{false};
  std::atomic<bool> was_active_{false};
  // Cleared on a source switch or when nobody wants frames: the encoder's
  // slot is not repeated until a new frame arrives.
  std::atomic<bool> repeatable_{false};
  std::atomic<uint32_t> last_source_ms_{0};

  // Source frame rate, measured on the source's task.
  std::atomic<uint32_t> interval_x16_{0};  // mean frame interval, 1/16 ms
  uint32_t last_ts_{0};
  bool have_last_ts_{false};

  // Owned by the encoder task.
  int front_{SlotExchange::NONE};  // the encoder's slot
  uint8_t *out_buf_{nullptr};
  uint32_t out_cap_{0};
  esp_h264_enc_handle_t enc_{nullptr};
  esp_h264_enc_param_hw_handle_t param_{nullptr};
  uint16_t enc_w_{0};
  uint16_t enc_h_{0};
  esp_h264_raw_format_t enc_type_{ESP_H264_RAW_FMT_RGB565_LE};
  uint8_t enc_fps_{0};
  uint32_t enc_opened_ms_{0};
  bool drifting_{false};  // measured rate off from the declared one since drift_since_ms_
  uint32_t drift_since_ms_{0};
  uint32_t last_out_ms_{0};  // when the last frame went out
  uint32_t last_out_ts_{0};  // and its stream timestamp
  bool have_out_ts_{false};

  // Statistics
  std::atomic<uint32_t> encoded_frames_{0};
  std::atomic<uint32_t> dropped_frames_{0};
  std::atomic<uint32_t> encode_errors_{0};
  std::atomic<uint32_t> encoded_bytes_{0};
  std::atomic<uint32_t> keyframes_{0};
  std::atomic<uint32_t> zero_copy_{0};
  std::atomic<uint32_t> repeats_{0};
  uint32_t last_stats_ms_{0};
  std::atomic<bool> format_error_logged_{false};
};

}  // namespace h264_encoder
}  // namespace esphome
