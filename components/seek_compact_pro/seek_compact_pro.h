#pragma once

// Seek Thermal Compact Pro USB thermal camera (Android model, including Fast
// Frame), 320 x 240. A radiometric source: temperatures from the camera's own
// slope and factory reference temperature (see seek_core.h).
//
// Threads:
//   main loop   connection, the start-up sequence (vendor control requests,
//               factory table), shutter, watchdog, statistics
//   USB task    bulk completions: assembles each frame (13 pieces of 13680
//               bytes), asks for the next one at once (the camera stops for
//               good if asked late), hands the frame to the processing task
//   processing  shutter correction, dead pixels, temperatures; runs the sinks

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/usb_host/usb_host.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include "../video_core/frame.h"
#include "seek_core.h"

namespace esphome {
namespace seek_compact_pro {

class SeekCompactPro : public usb_host::USBClient, public video::RadiometricSource {
 public:
  SeekCompactPro(uint16_t vid, uint16_t pid) : usb_host::USBClient(vid, pid) {}
  void setup() override;
  void loop() override;
  void dump_config() override;

  void add_radiometric_sink(video::RadiometricSink *sink) override { this->sinks_.add(sink); }

  // Thread-safe. Temperatures are t * gain + offset (offset in degrees C).
  void set_gain(float v) { this->gain_ = v; }
  void set_offset(float v) { this->offset_ = v; }
  // Before setup: a picture column (0..319) filled from its neighbours, like a
  // dead pixel (one that flickers, say). At most MAX_BAD_COLUMNS.
  void add_bad_column(uint16_t x) { this->processor_.add_bad_column(x); }
  // Before setup: Seek's default picture filter (scene-based correction, temporal filter).
  void set_scene_filter(bool on) { this->processor_.set_scene_filter(on); }
  // Main loop: closes the shutter for a new flat-field frame.
  void trigger_shutter();
  void set_transfers(uint8_t n) { this->num_transfers_ = n; }
  void set_log_timing(bool v) { this->log_timing_ = v; }
#ifdef USE_BINARY_SENSOR
  void set_streaming_sensor(binary_sensor::BinarySensor *s) { this->streaming_sensor_ = s; }
#endif
#ifdef USE_TEXT_SENSOR
  void set_status_sensor(text_sensor::TextSensor *s) { this->status_sensor_ = s; }
#endif

 protected:
  enum class Stage : uint8_t { IDLE, INIT, STREAM, BACKOFF };
  enum : uint8_t { CTRL_IDLE, CTRL_PENDING, CTRL_OK, CTRL_FAILED };
  // One request of the start-up sequence. `in`: a read of `length` bytes,
  // kept under `keep` (KEEP_*).
  struct Step {
    uint8_t request;
    bool in;
    std::vector<uint8_t> data;
    uint16_t length;
    uint8_t keep;
    bool optional;  // a refusal is not a failure
  };
  enum : uint8_t { KEEP_NONE, KEEP_FIRMWARE, KEEP_CHIP_ID, KEEP_FIRMWARE_LONG, KEEP_TABLE };

  void on_connected() override;
  void on_disconnected() override;

  void build_steps_();
  void run_init_();
  void goto_stage_(Stage s, uint32_t delay_ms);
  void fail_(const char *what);
  bool send_control_(bool in, uint8_t request, const std::vector<uint8_t> &data, uint16_t length);
  bool start_transfers_();
  void stop_stream_();
  bool request_frame_();
  void send_credit_();  // any thread: START_GET_IMAGE_TRANSFER for one frame

  // USB task.
  static void transfer_done_(usb_transfer_t *t);
  void on_transfer_(usb_transfer_t *t);

  // Processing task.
  static void task_entry_(void *arg);
  void task_loop_();
  void process_(const video::FrameRef &raw, uint32_t ts);

  void set_status_(const std::string &s);
  void publish_streaming_(bool on);
  void report_(uint32_t now);

  video::SinkList<video::RadiometricSink> sinks_;

  // Configuration and options.
  uint8_t num_transfers_{3};
  bool log_timing_{false};
  std::atomic<float> gain_{1.0f}, offset_{0.0f};
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *streaming_sensor_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *status_sensor_{nullptr};
#endif

  // Start-up (main loop).
  Stage stage_{Stage::IDLE};
  uint32_t stage_at_{0};
  std::vector<Step> steps_;
  size_t step_{0};
  bool step_sent_{false};
  uint8_t failures_{0};
  bool shutter_want_{false};
  std::vector<uint8_t> table_;           // the factory table, as read
  std::string firmware_, chip_id_;       // as read, for the log
  Factory factory_;                      // the factory block's thermography (for the log)
  bool interface_claimed_{false};

  // Control transfer in flight (one at a time); completions of abandoned ones are ignored.
  std::atomic<uint8_t> ctrl_state_{CTRL_IDLE};
  std::atomic<uint32_t> ctrl_gen_{0};
  uint8_t ctrl_resp_[64]{};
  size_t ctrl_resp_len_{0};
  uint32_t ctrl_sent_ms_{0};

  // Streaming.
  std::vector<usb_transfer_t *> transfers_;
  std::atomic<bool> streaming_{false};
  std::atomic<int> inflight_{0};
  std::atomic<bool> frame_done_{false};   // USB task: a frame completed, ask for the next
  std::atomic<bool> discard_{false};      // main loop: throw away a partly assembled frame
  std::atomic<bool> table_lost_{false};   // processing: a frame came before the pixel table
  uint32_t requested_ms_{0};              // main loop: when the frame now coming was asked for
  uint8_t missed_{0};                     // frames asked for in a row that did not come
  std::atomic<uint32_t> last_frame_ms_{0};

  // USB task only: the frame being assembled.
  video::FrameRef assembling_;
  size_t asm_bytes_{0};

  std::unique_ptr<video::FramePool> raw_pool_, rad_pool_;

  // USB task -> processing task. Frames are processed in order; images may be
  // dropped when processing falls behind, shutter and first frames never.
  std::mutex mailbox_mutex_;
  // Tables, shutter and other frames processing uses, in order (an image waits
  // in its own slot and may be replaced by a newer one).
  static constexpr uint8_t SPECIAL_SLOTS = 4;
  video::FrameRef pending_image_, pending_special_[SPECIAL_SLOTS];
  uint32_t pending_image_ms_{0}, pending_special_ms_[SPECIAL_SLOTS]{};
  uint8_t special_head_{0}, special_count_{0};
  bool factory_pending_{false};
  std::vector<uint8_t> factory_for_task_;
  bool reset_pending_{false};
  TaskHandle_t task_{nullptr};

  // Processing task only.
  Processor processor_;
  uint32_t sequence_{0};
  std::atomic<uint32_t> dead_pixels_{0};
  std::atomic<float> sensor_{0}, warm_up_{0};

  // Statistics (any thread writes, main loop reads and resets).
  std::atomic<uint32_t> frames_{0}, images_{0}, shutters_{0}, dropped_{0}, errors_{0}, resyncs_{0}, process_us_{0},
      no_buffer_{0}, torn_{0}, credit_errors_{0};
  uint32_t last_report_ms_{0};
  std::string status_;
  bool streaming_published_{false}, streaming_known_{false};
};

}  // namespace seek_compact_pro
}  // namespace esphome
