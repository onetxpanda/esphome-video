#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/components/h264_encoder/encoded_sink.h"
#include "esphome/components/h264_encoder/h264_encoder.h"
#include "esphome/components/matter/matter_component.h"

#include <atomic>

namespace esphome {
namespace matter_camera {

// A Matter camera (device type 0x0142) on one endpoint. Controllers stream it
// live over WebRTC: Matter carries the signaling (SDP offer/answer and ICE
// candidates), and the H.264 encoder's output goes to the viewer as RTP over
// DTLS-SRTP. The encoder can feed other consumers (RTSP) at the same time.
class MatterCamera : public Component, public h264_encoder::EncodedFrameSink {
 public:
  MatterCamera();

  void set_encoder(h264_encoder::H264Encoder *encoder) { this->encoder_ = encoder; }
  void set_endpoint_id(uint16_t endpoint_id) { this->endpoint_id_ = endpoint_id; }
  void set_matter(matter::MatterComponent *matter) { this->matter_ = matter; }

  void setup() override;
  void loop() override;
  void dump_config() override;
  // Before the matter component, which creates the endpoints in its setup.
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  // EncodedFrameSink (called on the encoder's task)
  void on_encoded_frame(const uint8_t *data, size_t length, uint32_t timestamp_ms, bool keyframe) override;
  bool wants_frames() const override;

  // Adds the camera clusters to the endpoint; runs before Matter starts.
  static bool build_endpoint(esp_matter::endpoint_t *endpoint);
  // Finishes the cluster setup; runs after Matter has started.
  void on_matter_started();

  static MatterCamera *instance() { return instance_; }
  h264_encoder::H264Encoder *encoder() const { return this->encoder_; }

 protected:
  static MatterCamera *instance_;

  h264_encoder::H264Encoder *encoder_{nullptr};
  matter::MatterComponent *matter_{nullptr};
  uint16_t endpoint_id_{1};
  bool matter_started_{false};
  bool webrtc_started_{false};
  std::atomic<uint32_t> frames_sent_{0};
};

}  // namespace matter_camera
}  // namespace esphome
