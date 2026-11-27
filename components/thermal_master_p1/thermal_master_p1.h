#pragma once

// Thermal Master P1 USB thermal camera, 160 x 120 at 25 frames/s. Every frame
// carries two pictures:
//   radiometric  the camera's temperatures (1/64 K), corrected here by a gain
//                and offset
//   image        the camera's own processed picture (its palette, brightness,
//                contrast, noise reduction ...), YUYV, as RGB565
// The camera's settings, actions and readings (its command set as the vendor
// app's library has it) are entities. It does its own shutter cycles (about
// every 90 s) unless told not to.
//
// Threads:
//   main loop   connection, the start-up sequence (vendor control requests
//               with their delays), the command queue (settings, actions,
//               readings, range changes), watchdog, statistics
//   USB task    bulk completions: assembles each frame and hands it on
//   processing  temperatures, the picture, the metadata rows and the
//               automatic range's votes; runs the sinks

#include <atomic>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/usb_host/usb_host.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include "../video_core/frame.h"
#include "tm_core.h"

namespace esphome {
namespace thermal_master_p1 {

class ThermalMasterP1;

// The range setting's value for the vendor's automatic range switching (it
// runs here, not in the camera).
static constexpr int32_t RANGE_AUTO = 2;

// A camera setting's entity: a change goes to the camera, and the entity
// shows what the camera then reports (or, with no read, what it took).
class CameraSetting {
 public:
  virtual ~CameraSetting() = default;
  virtual void show(int32_t raw) = 0;  // the camera's value; UNKNOWN: not known
};

class SettingNumber : public number::Number, public CameraSetting {
 public:
  // Shown = camera value / scale + offset.
  void set_parent(ThermalMasterP1 *p, Setting s, float scale, float offset) {
    this->parent_ = p;
    this->setting_ = s;
    this->scale_ = scale;
    this->offset_ = offset;
  }
  void show(int32_t raw) override { this->publish_state(raw == UNKNOWN ? NAN : raw / this->scale_ + this->offset_); }

 protected:
  void control(float value) override;
  ThermalMasterP1 *parent_{nullptr};
  Setting setting_{Setting::BRIGHTNESS};
  float scale_{1}, offset_{0};
};

// One of a pair setting's two values.
class SettingPartNumber : public number::Number, public CameraSetting {
 public:
  void set_parent(ThermalMasterP1 *p, Setting s, int part) {
    this->parent_ = p;
    this->setting_ = s;
    this->part_ = part;
  }
  void show(int32_t raw) override {
    this->publish_state(raw == UNKNOWN ? NAN : static_cast<float>(pair_part(this->setting_, raw, this->part_)));
  }

 protected:
  void control(float value) override;
  ThermalMasterP1 *parent_{nullptr};
  Setting setting_{Setting::ISOTHERMAL_LIMIT};
  int part_{0};
};

class SettingSelect : public select::Select, public CameraSetting {
 public:
  void set_parent(ThermalMasterP1 *p, Setting s, std::vector<int32_t> values) {
    this->parent_ = p;
    this->setting_ = s;
    this->values_ = std::move(values);
  }
  void show(int32_t raw) override;

 protected:
  void control(size_t index) override;
  ThermalMasterP1 *parent_{nullptr};
  Setting setting_{Setting::MIRROR_FLIP};
  std::vector<int32_t> values_;
};

class SettingSwitch : public switch_::Switch, public CameraSetting {
 public:
  void set_parent(ThermalMasterP1 *p, Setting s) {
    this->parent_ = p;
    this->setting_ = s;
  }
  void show(int32_t raw) override;

 protected:
  void write_state(bool state) override;
  ThermalMasterP1 *parent_{nullptr};
  Setting setting_{Setting::ISOTHERMAL};
};

// The values maintenance commands take, set by number entities.
enum class Param : uint8_t { CURSOR_X, CURSOR_Y, TPD_REFERENCE, VL_REGISTER, VL_VALUE, CRG_VALUE, COUNT };
// Maintenance commands with values, run by buttons.
enum class Custom : uint8_t {
  CURSOR_MOVE,
  DPC_MARK,
  DPC_UNMARK,
  TPD1_RECALIBRATE,
  TPD2_POINT1,
  TPD2_POINT2,
  VL_READ,
  VL_WRITE,
  CRG_WRITE
};

class ThermalMasterP1 : public usb_host::USBClient, public video::RadiometricSource, public video::ImageSource {
 public:
  ThermalMasterP1(uint16_t vid, uint16_t pid);
  void setup() override;
  void loop() override;
  void dump_config() override;

  void add_radiometric_sink(video::RadiometricSink *sink) override { this->sinks_.add(sink); }
  void add_image_sink(video::ImageSink *sink) override { this->img_sinks_.add(sink); }

  // Thread-safe. Temperatures are t * gain + offset (offset in degrees C).
  void set_gain(float v) { this->gain_ = v; }
  void set_offset(float v) { this->offset_ = v; }
  void set_transfers(uint8_t n) { this->num_transfers_ = n; }
  void set_log_timing(bool v) { this->log_timing_ = v; }
  void set_shutter_after_start(bool v) { this->shutter_after_start_ = v; }
  void set_update_interval(uint32_t ms) { this->poll_ms_ = ms; }
  void set_heartbeat_interval(uint32_t ms) { this->heartbeat_ms_ = ms; }

  // Main loop. Settings: written at every start (before setup: the
  // configuration's values), or now, then read back.
  void set_initial_setting(Setting s, int32_t v);
  // A pair setting has two (one a value).
  void add_setting_entity(Setting s, CameraSetting *e) {
    CameraSetting **slot = this->entities_[static_cast<size_t>(s)];
    slot[slot[0] == nullptr ? 0 : 1] = e;
  }
  void write_setting(Setting s, int32_t v);
  // One value of a pair setting; the other stays as the camera last reported it.
  void write_setting_part(Setting s, int part, int32_t v);
  void press(Action a);
  void trigger_shutter() { this->press(Action::SHUTTER); }
  void set_param(Param p, float v) { this->params_[static_cast<size_t>(p)] = v; }
  void press_custom(Custom c);
  // The point camera_point_temperature reads (the camera's own measurement).
  void set_point(uint16_t x, uint16_t y) {
    this->point_x_ = x;
    this->point_y_ = y;
  }

  void set_reading_sensor(Reading r, sensor::Sensor *s) { this->reading_sensors_[static_cast<size_t>(r)] = s; }
  void set_reading_text(Reading r, text_sensor::TextSensor *s) { this->reading_texts_[static_cast<size_t>(r)] = s; }
  void set_info_text(Info i, text_sensor::TextSensor *s) { this->info_texts_[static_cast<size_t>(i)] = s; }
  void set_camera_max_sensor(sensor::Sensor *s) { this->camera_max_ = s; }
  void set_camera_min_sensor(sensor::Sensor *s) { this->camera_min_ = s; }
  void set_palettes_text(text_sensor::TextSensor *s) { this->palettes_text_ = s; }
  void set_range_text(text_sensor::TextSensor *s) { this->range_text_ = s; }
  void set_shutter_countdown_sensor(sensor::Sensor *s) { this->countdown_sensor_ = s; }
  void set_vl_result_sensor(sensor::Sensor *s) { this->vl_result_ = s; }
#ifdef USE_BINARY_SENSOR
  void set_streaming_sensor(binary_sensor::BinarySensor *s) { this->streaming_sensor_ = s; }
  void set_sun_sensor(binary_sensor::BinarySensor *s) { this->sun_sensor_ = s; }
#endif
  void set_status_sensor(text_sensor::TextSensor *s) { this->status_sensor_ = s; }

 protected:
  enum class Stage : uint8_t { IDLE, INIT, STREAM, BACKOFF };
  enum : uint8_t { CTRL_IDLE, CTRL_PENDING, CTRL_OK, CTRL_FAILED };
  // One step of a request sequence.
  enum class Op : uint8_t {
    COMMAND,          // `cmd` (18 or 24 bytes)
    STATUS,           // the 1-byte status, read again while 1 (busy) for up to `ms`; it must be `expect`
    RESPONSE,         // `bytes` of response, kept for the job
    IDLE_ALT,         // interface 1 to alternate setting 0 (no endpoints)
    CLAIM_STREAM,     // interface 1, alternate setting 1 (bulk IN 0x81), selected
    STREAM_ON,        // the vendor request 0xEE to the device
    WAIT,             // `ms` before the next step
    START_TRANSFERS,  // bulk transfers queued
  };
  struct Step {
    Op op;
    Command cmd{};
    uint16_t bytes{0};
    uint32_t ms{0};  // WAIT: the delay; STATUS: how long the camera may stay busy
    uint8_t expect{0};
  };
  // A sequence of steps and what to do with its response. A critical job's
  // failure restarts the camera; another's is logged and passed to `done`.
  struct Job {
    std::string what;
    std::vector<Step> steps;
    std::function<void(bool ok, const std::vector<uint8_t> &resp)> done;
    bool critical{false};
    std::vector<uint8_t> resp{};
  };
  using Done = std::function<void(bool, const std::vector<uint8_t> &)>;

  void on_connected() override;
  void on_disconnected() override;

  static void add_command_(std::vector<Step> &v, const Command &c);
  Job make_job_(std::string what, const Command &c, Done done = nullptr);
  bool queue_(Job j);
  void queue_init_();
  void queue_after_start_();
  void queue_polls_();
  void queue_reading_(Reading r);
  void queue_setting_read_(Setting s);
  void queue_range_settle_(int tries);
  void queue_palettes_();
  void show_setting_(Setting s, int32_t v);
  void note_range_(int32_t v);
  void run_job_();
  bool start_step_(const Step &s);
  void finish_job_(bool ok);
  void goto_stage_(Stage s, uint32_t delay_ms);
  void fail_(const char *what);
  bool send_control_(uint8_t type, uint8_t request, uint16_t value, uint16_t index, std::vector<uint8_t> data);
  bool start_transfers_();
  void stop_stream_();
  void release_stream_();
  void publish_reading_(Reading r, const std::vector<uint8_t> &resp);
  void publish_info_rows_();

  // USB task.
  static void transfer_done_(usb_transfer_t *t);
  static void control_done_(usb_transfer_t *t);
  void on_transfer_(usb_transfer_t *t);

  // Processing task.
  static void task_entry_(void *arg);
  void task_loop_();
  void process_(const video::FrameRef &raw, uint32_t ts);

  void set_status_(const std::string &s);
  void publish_streaming_(bool on);
  void publish_text_(text_sensor::TextSensor *s, const std::string &v);
  void report_(uint32_t now);

  video::SinkList<video::RadiometricSink> sinks_;
  video::SinkList<video::ImageSink> img_sinks_;

  // Configuration and options.
  uint8_t num_transfers_{4};
  bool log_timing_{false};
  bool shutter_after_start_{false};
  uint32_t poll_ms_{60000}, heartbeat_ms_{0};
  std::atomic<float> gain_{1.0f}, offset_{0.0f};
  int32_t initial_[static_cast<size_t>(Setting::COUNT)]{};
  bool initial_set_[static_cast<size_t>(Setting::COUNT)]{};
  int32_t known_[static_cast<size_t>(Setting::COUNT)]{};  // what the camera last reported (or took)
  CameraSetting *entities_[static_cast<size_t>(Setting::COUNT)][2]{};
  sensor::Sensor *reading_sensors_[static_cast<size_t>(Reading::COUNT)]{};
  text_sensor::TextSensor *reading_texts_[static_cast<size_t>(Reading::COUNT)]{};
  text_sensor::TextSensor *info_texts_[static_cast<size_t>(Info::LAST) + 1]{};
  sensor::Sensor *camera_max_{nullptr}, *camera_min_{nullptr}, *countdown_sensor_{nullptr}, *vl_result_{nullptr};
  text_sensor::TextSensor *palettes_text_{nullptr}, *range_text_{nullptr};
  float params_[static_cast<size_t>(Param::COUNT)]{};
  uint16_t point_x_{W / 2}, point_y_{H / 2};
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *streaming_sensor_{nullptr}, *sun_sensor_{nullptr};
#endif
  text_sensor::TextSensor *status_sensor_{nullptr};

  // The command queue (main loop). The front job runs; the start-up sequence
  // is a job too.
  Stage stage_{Stage::IDLE};
  uint32_t stage_at_{0};
  std::deque<Job> jobs_;
  size_t step_{0};
  bool step_sent_{false};
  uint32_t wait_until_{0};
  uint32_t status_started_ms_{0};  // the step's first status read
  bool status_polling_{false};     // the status said busy; reading it again
  uint8_t failures_{0};
  uint32_t last_poll_ms_{0}, last_heartbeat_ms_{0};
  std::string model_, firmware_, system_version_;
  uint8_t start_answer_{0}, last_status_{0};
  bool iface1_claimed_{false};
  std::string palettes_;

  // The range: the camera's (0 low, 1 high, -1 not known), and the automatic
  // switching (votes in the processing task, changes in the main loop).
  std::atomic<int> range_now_{-1};
  std::atomic<bool> auto_range_{false};
  std::atomic<bool> auto_reset_{false};   // main loop -> processing: start the votes again
  std::atomic<int> range_request_{-1};    // processing -> main loop: the range the votes want
  std::atomic<bool> range_changing_{false};

  // Control transfer in flight (one at a time); completions of abandoned ones are ignored.
  std::atomic<uint8_t> ctrl_state_{CTRL_IDLE};
  std::atomic<uint32_t> ctrl_gen_{0};
  // Control requests go on this component's own transfer: the USB host
  // component's shared control buffers hold only 56 data bytes, and the
  // camera's device information answers are up to 64.
  static constexpr size_t CTRL_MAX_DATA = 256;
  usb_transfer_t *ctrl_xfer_{nullptr};
  std::atomic<bool> ctrl_inflight_{false};
  uint32_t ctrl_xfer_gen_{0};  // the request the transfer in flight belongs to
  uint8_t ctrl_resp_[CTRL_MAX_DATA]{};
  size_t ctrl_resp_len_{0};
  uint32_t ctrl_sent_ms_{0};

  // Streaming.
  std::vector<usb_transfer_t *> transfers_;
  std::atomic<bool> streaming_{false};
  std::atomic<int> inflight_{0};
  std::atomic<uint32_t> last_frame_ms_{0};
  uint32_t stream_started_ms_{0};
  std::atomic<bool> had_frame_{false};       // USB task: a frame came since the start
  std::atomic<bool> unreadable_now_{false};  // processing: the last frame had no temperatures

  // USB task only: the frame being assembled.
  Assembler assembler_;
  video::FrameRef assembling_;
  std::unique_ptr<video::FramePool> raw_pool_, rad_pool_, img_pool_;

  // USB task -> processing task: the newest frame (an older one not yet
  // taken is dropped).
  std::mutex mailbox_mutex_;
  video::FrameRef pending_;
  uint32_t pending_ms_{0};
  TaskHandle_t task_{nullptr};

  // Processing task only.
  uint32_t sequence_{0}, img_sequence_{0};
  AutoRange auto_;

  // Processing -> main loop: the metadata rows of the last frame.
  std::atomic<uint8_t> sun_flag_{0}, ffc_countdown_{0};
  std::atomic<bool> info_seen_{false};
  bool sun_published_{false}, sun_known_{false};

  // Statistics (any thread writes, main loop reads and resets).
  std::atomic<uint32_t> frames_{0}, images_{0}, pictures_{0}, dropped_{0}, errors_{0}, short_{0}, long_{0},
      bad_marker_{0}, no_buffer_{0}, process_us_{0}, unreadable_{0};
  std::atomic<uint32_t> bad_pixels_{0};     // in the last frame processed
  std::atomic<uint32_t> skipped_total_{0};  // camera frames that never came, since start
  std::atomic<float> centre_{0};            // the last frame's centre pixel, degrees C
  uint32_t last_report_ms_{0}, last_skipped_{0};
  std::string status_;
  bool streaming_published_{false}, streaming_known_{false};
};

}  // namespace thermal_master_p1
}  // namespace esphome
