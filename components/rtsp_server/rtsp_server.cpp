#include "rtsp_server.h"

#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <esp_heap_caps.h>
#include <freertos/idf_additions.h>

// The task's stack is in PSRAM: internal RAM is needed by the network stack,
// USB and (with Matter) the WebRTC stack.
static constexpr UBaseType_t TASK_STACK_CAPS = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

namespace esphome {
namespace rtsp_server {

static const char *const TAG = "rtsp_server";

void RtspServer::setup() {
  this->config_.now_ms = []() { return millis(); };
  this->config_.random = []() { return random_uint32(); };
  auto *encoder = this->encoder_;
  this->config_.request_keyframe = [encoder]() { encoder->request_keyframe(); };
  this->config_.log = [](LogLevel level, const std::string &msg) {
    switch (level) {
      case LogLevel::ERROR:
        ESP_LOGE(TAG, "%s", msg.c_str());
        break;
      case LogLevel::WARN:
        ESP_LOGW(TAG, "%s", msg.c_str());
        break;
      case LogLevel::INFO:
        ESP_LOGI(TAG, "%s", msg.c_str());
        break;
      case LogLevel::DEBUG:
        ESP_LOGD(TAG, "%s", msg.c_str());
        break;
    }
  };
  this->core_ = std::make_unique<RtspServerCore>(this->config_);
  this->encoder_->add_encoded_sink(this);
  if (xTaskCreatePinnedToCoreWithCaps(RtspServer::task_entry_, "rtsp_srv", 6144, this, 5, &this->task_,
                                      tskNO_AFFINITY, TASK_STACK_CAPS) != pdPASS)
    this->mark_failed(LOG_STR("Could not start the server task"));
}

void RtspServer::task_entry_(void *arg) {
  auto *self = static_cast<RtspServer *>(arg);
  for (;;) {
    if (!self->core_->is_running()) {
      // The port can be busy briefly after a restart; keep trying.
      if (!self->core_->start()) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }
    self->core_->poll(100);
  }
}

void RtspServer::dump_config() {
  ESP_LOGCONFIG(TAG,
                "RTSP server:\n"
                "  URL: rtsp://<device>:%u/%s (RTP over TCP)\n"
                "  Max clients: %u\n"
                "  Per-client buffer: %u bytes",
                this->config_.port, this->config_.path.c_str(), static_cast<unsigned>(this->config_.max_clients),
                static_cast<unsigned>(this->config_.max_client_buffer));
}

void RtspServer::on_encoded_frame(const uint8_t *data, size_t length, uint32_t timestamp_ms, bool keyframe) {
  if (this->core_)
    this->core_->on_access_unit(data, length, timestamp_ms, keyframe);
}

bool RtspServer::wants_frames() const { return this->core_ && this->core_->playing_clients() > 0; }

}  // namespace rtsp_server
}  // namespace esphome
