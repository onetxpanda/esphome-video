#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/component.h"

#include <memory>
#include <string>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/h264_encoder/encoded_sink.h"
#include "esphome/components/h264_encoder/h264_encoder.h"
#include "rtsp_server_core.h"

namespace esphome {
namespace rtsp_server {

// Serves the encoder's output as rtsp://<device>:<port>/<path> (RTP over TCP).
// The protocol work happens in RtspServerCore on a dedicated task; encoded
// frames arrive on the encoder's task. Neither touches the main loop.
class RtspServer : public Component, public h264_encoder::EncodedFrameSink {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void set_encoder(h264_encoder::H264Encoder *encoder) { this->encoder_ = encoder; }
  void set_port(uint16_t port) { this->config_.port = port; }
  void set_path(const std::string &path) { this->config_.path = path; }
  void set_session_name(const std::string &name) { this->config_.session_name = name; }
  void set_max_clients(uint8_t n) { this->config_.max_clients = n; }
  void set_buffer_size(uint32_t bytes) { this->config_.max_client_buffer = bytes; }

  // EncodedFrameSink (called on the encoder task)
  void on_encoded_frame(const uint8_t *data, size_t length, uint32_t timestamp_ms, bool keyframe) override;
  bool wants_frames() const override;

 protected:
  static void task_entry_(void *arg);

  h264_encoder::H264Encoder *encoder_{nullptr};
  ServerConfig config_;
  std::unique_ptr<RtspServerCore> core_;
  TaskHandle_t task_{nullptr};
};

}  // namespace rtsp_server
}  // namespace esphome
