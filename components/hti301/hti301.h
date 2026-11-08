#pragma once

// HTI-301 USB thermal camera (UVC, isochronous, USB 2.0 high speed).
//
// Threads:
//   main loop   connection, stream set-up (probe/commit, alternate setting),
//               camera commands, watchdog, statistics
//   USB task    (the usb_host client's) isochronous completions: assembles
//               frames and passes each complete one to the processing task
//   processing  converts a frame (radiometric: raw -> degrees C; processed:
//               YUYV -> RGB565) and runs the sinks on it
//
// Commands go through the camera terminal's Zoom (Absolute) control. The
// camera does not report its mode, so on every stream start the configured
// mode, range and palette are sent, and frames are only passed on once the
// mode command has been acknowledged.

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
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
#include "hti301_core.h"

namespace esphome {
namespace hti301 {

enum class Mode : uint8_t { RADIOMETRIC = 0, PROCESSED = 1 };
enum class Range : uint8_t { NORMAL = 0, HIGH = 1 };

class HTI301 : public usb_host::USBClient, public video::RadiometricSource, public video::ImageSource {
 public:
  HTI301(uint16_t vid, uint16_t pid) : usb_host::USBClient(vid, pid) {}
  void setup() override;
  void loop() override;
  void dump_config() override;

  void add_radiometric_sink(video::RadiometricSink *sink) override { this->rad_sinks_.add(sink); }
  void add_image_sink(video::ImageSink *sink) override { this->img_sinks_.add(sink); }

  // Main loop only. Sent to the camera while it streams.
  void set_mode(Mode m) { this->mode_want_ = static_cast<uint8_t>(m); }
  void set_range(Range r) { this->range_want_ = static_cast<uint8_t>(r); }
  void set_palette(uint8_t p) { this->palette_want_ = p < PALETTE_COUNT ? p : 0; }
  // Triggers the shutter calibration; ignored while the camera is not streaming.
  void calibrate();

  // Scene parameters for radiometric mode; NaN takes the camera's stored value.
  void set_emissivity(float v) { this->env_emissivity_ = v; }
  void set_reflected_temperature(float v) { this->env_reflected_ = v; }
  void set_ambient_temperature(float v) { this->env_ambient_ = v; }
  void set_humidity(float v) { this->env_humidity_ = v; }
  void set_distance(float v) { this->env_distance_ = v; }

  void set_transfers(uint8_t n) { this->num_transfers_ = n; }
  void set_packets_per_transfer(uint8_t n) { this->packets_per_transfer_ = n; }
  void set_log_timing(bool v) { this->log_timing_ = v; }
#ifdef USE_BINARY_SENSOR
  void set_streaming_sensor(binary_sensor::BinarySensor *s) { this->streaming_sensor_ = s; }
#endif
#ifdef USE_TEXT_SENSOR
  // What is on the USB bus while no camera is connected, then the stream's state.
  void set_status_sensor(text_sensor::TextSensor *s) { this->status_sensor_ = s; }
#endif

 protected:
  enum class Stage : uint8_t {
    IDLE,        // no camera
    IDLE_ALT,    // SET_INTERFACE alternate setting 0
    PROBE,       // SET_CUR VS_PROBE
    PROBE_READ,  // GET_CUR VS_PROBE
    COMMIT,      // SET_CUR VS_COMMIT
    SELECT_ALT,  // claim the streaming interface, SET_INTERFACE alternate setting n
    START,       // submit the isochronous transfers
    STREAMING,
    BACKOFF,     // a step failed; start again later
  };
  enum : uint8_t { CTRL_IDLE, CTRL_PENDING, CTRL_OK, CTRL_FAILED };
  enum class Cmd : uint8_t { NONE, MODE, RANGE, PALETTE, CALIBRATE };
  static constexpr uint8_t UNKNOWN = 0xFF;

  void on_connected() override;
  void on_disconnected() override;

  // Stream set-up (main loop).
  void run_stage_();
  bool send_stage_request_();
  void stage_succeeded_(uint32_t now);
  void fail_stage_(const char *what);
  void goto_stage_(Stage s, uint32_t delay_ms);
  bool start_transfers_();
  void stop_stream_();
  bool send_control_(uint8_t type, uint8_t request, uint16_t value, uint16_t index, std::vector<uint8_t> data);

  // Camera commands (main loop).
  void run_commands_();
  bool send_command_(Cmd kind, uint8_t value, uint16_t code);
  void command_succeeded_(Cmd kind, uint8_t value);
  void reset_camera_state_();
  static const char *cmd_name_(Cmd kind);

  // USB task.
  static void transfer_done_(usb_transfer_t *t);
  void on_transfer_(usb_transfer_t *t);
  void frame_assembled_(bool complete);

  // Processing task.
  static void task_entry_(void *arg);
  void task_loop_();
  void process_radiometric_(const video::FrameRef &raw, uint32_t ts);
  void process_image_(const video::FrameRef &raw, uint32_t ts);

  // USB bus diagnosis while no camera is connected (main loop).
  void scan_bus_(uint32_t now);
  std::string describe_hub_(usb_device_handle_t dev);
  int diag_request_(usb_device_handle_t dev, uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                    uint16_t length, uint8_t *out);
  static void diag_done_(usb_transfer_t *t);
  void set_status_(const std::string &s);

  void report_(uint32_t now);
  void publish_streaming_(bool on);
  static const char *stage_name_(Stage s);

  video::SinkList<video::RadiometricSink> rad_sinks_;
  video::SinkList<video::ImageSink> img_sinks_;

  // Configuration.
  uint8_t num_transfers_{8};
  uint8_t packets_per_transfer_{16};
  bool log_timing_{false};
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *streaming_sensor_{nullptr};
#endif

  // Wanted vs acknowledged camera state (main loop).
  uint8_t mode_want_{0}, range_want_{0}, palette_want_{0};
  uint8_t mode_set_{UNKNOWN}, range_set_{UNKNOWN}, palette_set_{UNKNOWN};
  bool calibrate_want_{false};
  Cmd cmd_kind_{Cmd::NONE};  // command in flight
  uint8_t cmd_value_{0};
  uint8_t cmd_failures_{0};
  uint32_t cmd_retry_ms_{0};
  uint8_t gave_up_[5]{UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN};  // per Cmd: value the camera kept rejecting

  std::atomic<float> env_emissivity_{0.95f}, env_reflected_{NAN}, env_ambient_{NAN}, env_humidity_{NAN},
      env_distance_{NAN};

  // Stream set-up (main loop).
  UvcLayout layout_;
  Stage stage_{Stage::IDLE};
  uint32_t stage_at_{0};
  bool stage_sent_{false};
  uint8_t failures_{0};
  Probe probe_;
  uint8_t probe_buf_[34]{};
  const StreamAlt *alt_{nullptr};
  bool vs_claimed_{false};
  uint16_t width_{0}, image_rows_{0};
  size_t frame_bytes_{0};
  std::vector<usb_transfer_t *> transfers_;
  uint32_t stream_start_ms_{0};

  // Control transfer in flight (one at a time). A completion whose
  // generation is not current belongs to an abandoned request.
  std::atomic<uint8_t> ctrl_state_{CTRL_IDLE};
  std::atomic<uint32_t> ctrl_gen_{0};
  uint8_t ctrl_resp_[34]{};
  size_t ctrl_resp_len_{0};
  uint32_t ctrl_sent_ms_{0};
  bool ctrl_slow_logged_{false};

  // Shared with the USB task.
  std::atomic<bool> streaming_{false};
  std::atomic<int> inflight_{0};
  std::atomic<uint8_t> out_mode_{UNKNOWN};  // mode of frames now arriving; UNKNOWN = drop them
  std::atomic<uint8_t> settle_{0};          // complete frames still to drop after a mode/range change
  std::atomic<uint32_t> last_frame_ms_{0};

  // USB task only (while transfers are in flight).
  UvcAssembler assembler_;
  video::FrameRef assembling_;
  uint8_t *asm_buf_{nullptr};
  size_t asm_cap_{0};

  // Frames: raw (as received), radiometric (degrees C), image (RGB565).
  std::unique_ptr<video::FramePool> raw_pool_, rad_pool_, img_pool_;

  // USB task -> processing task: the newest complete frame.
  std::mutex mailbox_mutex_;
  video::FrameRef pending_;
  uint8_t pending_mode_{0};
  uint32_t pending_ms_{0};
  TaskHandle_t task_{nullptr};

  // Processing task only.
  TemperatureTable table_;
  uint32_t sequence_{0};

  // Statistics (any thread writes, main loop reads and resets).
  std::atomic<uint32_t> frames_{0}, incomplete_{0}, settled_{0}, replaced_{0}, packets_lost_{0},
      transfer_errors_{0}, no_buffer_{0}, bad_calibration_{0}, process_us_{0}, processed_{0};
  // Bus diagnosis: one control transfer of our own, waited for on the main loop.
  usb_transfer_t *diag_xfer_{nullptr};
  SemaphoreHandle_t diag_sem_{nullptr};
  std::atomic<bool> diag_busy_{false};
  usb_device_handle_t diag_dev_{nullptr};  // left open while a timed-out request is still in flight
  uint32_t next_scan_ms_{0};
  std::string status_;
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *status_sensor_{nullptr};
#endif

  uint32_t last_report_ms_{0};
  uint32_t restarts_{0};
  bool streaming_published_{false};
  bool streaming_state_known_{false};
};

}  // namespace hti301
}  // namespace esphome
