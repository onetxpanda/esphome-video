#pragma once

// Passes on the radiometric frames of whichever camera is connected, so one
// processing chain (radiometric -> isp -> overlay) and one set of its
// controls serve every camera. The first camera in the list that has sent a
// frame within `timeout` is used; with none sending, the last one stays
// chosen. Frames pass on the producing camera's task.

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include "../video_core/frame.h"
#include "camera_choice.h"

namespace esphome {
namespace camera_switch {

class CameraSwitch;

// The switch's input from one camera.
class CameraInput : public video::RadiometricSink {
 public:
  CameraInput(CameraSwitch *sw, uint8_t index) : switch_(sw), index_(index) {}
  void on_radiometric_frame(const video::RadiometricFrame &frame) override;
  bool wants_frames() const override;

 protected:
  CameraSwitch *switch_;
  uint8_t index_;
};

class CameraSwitch : public Component, public video::RadiometricSource {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void loop() override;
  void dump_config() override;

  void add_radiometric_sink(video::RadiometricSink *sink) override { this->sinks_.add(sink); }

  // Code generation: the cameras in order of preference.
  void add_camera(const char *name, video::RadiometricSource *source);
  void set_timeout(uint32_t ms) { this->timeout_ms_ = ms; }
  void set_wait_for_api(bool v) { this->wait_for_api_ = v; }
#ifdef USE_TEXT_SENSOR
  void set_camera_text_sensor(text_sensor::TextSensor *s) { this->camera_sensor_ = s; }
#endif

  // From the inputs (the cameras' tasks).
  void handle_frame(uint8_t index, const video::RadiometricFrame &frame);
  bool wanted() const { return this->sinks_.any_wants(); }

 protected:
  static constexpr size_t MAX_CAMERAS = 8;
  std::vector<const char *> names_;
  std::vector<std::unique_ptr<CameraInput>> inputs_;
  video::SinkList<video::RadiometricSink> sinks_;
  std::atomic<uint32_t> last_ms_[MAX_CAMERAS]{};
  std::atomic<bool> seen_[MAX_CAMERAS]{};
  std::atomic<int> active_{NO_CAMERA};
  uint32_t timeout_ms_{10000};
  uint32_t last_check_ms_{0};
  bool published_{false};
  bool wait_for_api_{false};
  uint32_t api_since_ms_{0};
  std::atomic<bool> passing_{false};  // frames go on to the chain
  std::atomic<uint32_t> passed_{0};   // frames passed on, for the diagnostic log
  uint32_t passing_since_ms_{0};
  uint32_t progress_logged_ms_{0};
  // Frames of two cameras (both plugged in) reach the chain one at a time; a
  // frame arriving while another is delivered is dropped, not waited for.
  std::mutex deliver_mutex_;
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *camera_sensor_{nullptr};
#endif
};

}  // namespace camera_switch
}  // namespace esphome
