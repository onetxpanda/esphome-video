#include "camera_switch.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

namespace esphome {
namespace camera_switch {

static const char *const TAG = "camera_switch";
static constexpr uint32_t CHECK_MS = 500;

void CameraInput::on_radiometric_frame(const video::RadiometricFrame &frame) {
  this->switch_->handle_frame(this->index_, frame);
}
// Every camera is asked for frames: a camera's frames are how the switch
// knows it is connected.
bool CameraInput::wants_frames() const { return this->switch_->wanted(); }

void CameraSwitch::add_camera(const char *name, video::RadiometricSource *source) {
  if (this->names_.size() >= MAX_CAMERAS)
    return;
  auto input = std::make_unique<CameraInput>(this, static_cast<uint8_t>(this->names_.size()));
  source->add_radiometric_sink(input.get());
  this->inputs_.push_back(std::move(input));
  this->names_.push_back(name);
}

void CameraSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "Camera switch (the first camera sending frames within %u ms):",
                static_cast<unsigned>(this->timeout_ms_));
  const int active = this->active_.load();
  for (size_t i = 0; i < this->names_.size(); i++)
    ESP_LOGCONFIG(TAG, "  %s%s", this->names_[i], static_cast<int>(i) == active ? " (in use)" : "");
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Camera", this->camera_sensor_);
#endif
}

void CameraSwitch::handle_frame(uint8_t index, const video::RadiometricFrame &frame) {
  if (index >= this->names_.size())
    return;
  this->last_ms_[index].store(millis());
  this->seen_[index].store(true);
  if (static_cast<int>(index) != this->active_.load() || !this->passing_.load())
    return;
  std::unique_lock<std::mutex> lock(this->deliver_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
    return;
  this->sinks_.deliver(frame,
                       [](video::RadiometricSink *s, const video::RadiometricFrame &f) { s->on_radiometric_frame(f); });
  this->passed_++;
}

void CameraSwitch::loop() {
  const uint32_t now = millis();
  if (now - this->last_check_ms_ < CHECK_MS && this->published_)
    return;
  this->last_check_ms_ = now;
  if (!this->passing_.load()) {
    bool go = !this->wait_for_api_;
#ifdef USE_API
    if (!go) {
      if (api::global_api_server == nullptr || !api::global_api_server->is_connected()) {
        this->api_since_ms_ = 0;
      } else if (this->api_since_ms_ == 0) {
        this->api_since_ms_ = now;
      } else {
        go = now - this->api_since_ms_ >= 10000;
      }
    }
#else
    go = true;
#endif
    if (go) {
      if (this->wait_for_api_)
        ESP_LOGW(TAG, "Home Assistant connected for 10 s: passing camera frames on to the chain now");
      this->passing_since_ms_ = now;
      this->passing_.store(true);
    }
  } else if (this->wait_for_api_ && now - this->passing_since_ms_ <= 120000) {
    // Diagnostics: the chain's progress twice a second for its first 2 minutes.
    ESP_LOGW(TAG, "Chain running %.1f s: %u frames passed on", (now - this->passing_since_ms_) / 1000.0f,
             static_cast<unsigned>(this->passed_.load()));
  }
  uint32_t last[MAX_CAMERAS];
  bool seen[MAX_CAMERAS];
  const size_t n = this->names_.size();
  for (size_t i = 0; i < n; i++) {
    last[i] = this->last_ms_[i].load();
    seen[i] = this->seen_[i].load();
  }
  const int before = this->active_.load();
  const int now_active = choose_camera(last, seen, n, now, this->timeout_ms_, before);
  if (now_active != before) {
    ESP_LOGI(TAG, "Camera: %s -> %s", before >= 0 ? this->names_[before] : "none",
             now_active >= 0 ? this->names_[now_active] : "none");
    this->active_.store(now_active);
  }
  // Whether any camera is sending right now, for the text sensor.
  bool sending = false;
  for (size_t i = 0; i < n; i++)
    sending |= seen[i] && now - last[i] <= this->timeout_ms_;
#ifdef USE_TEXT_SENSOR
  if (this->camera_sensor_ != nullptr) {
    const std::string text = !sending ? std::string("None") : std::string(this->names_[now_active]);
    if (!this->published_ || this->camera_sensor_->get_state() != text)
      this->camera_sensor_->publish_state(text);
  }
#endif
  this->published_ = true;
}

}  // namespace camera_switch
}  // namespace esphome
