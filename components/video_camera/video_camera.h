#pragma once

// An ESPHome camera (Home Assistant camera entity, web server snapshots) fed
// by an image source. Frames are JPEG-encoded by the ESP32-P4's hardware
// encoder, only while a client has asked for an image.

#include <atomic>
#include <memory>
#include <vector>

#include "esphome/components/camera/camera.h"
#include "esphome/core/component.h"

#include "../video_core/frame.h"

#include "driver/jpeg_encode.h"

namespace esphome {
namespace video_camera {

class JpegImage : public camera::CameraImage {
 public:
  JpegImage(uint8_t *data, size_t length, uint8_t requesters)
      : data_(data), length_(length), requesters_(requesters) {}
  uint8_t *get_data_buffer() override { return this->data_; }
  size_t get_data_length() override { return this->length_; }
  bool was_requested_by(camera::CameraRequester r) const override { return (this->requesters_ & (1u << r)) != 0; }

 protected:
  uint8_t *data_;
  size_t length_;
  uint8_t requesters_;
};

class JpegImageReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override {
    this->image_ = std::move(image);
    this->offset_ = 0;
  }
  size_t available() const override { return this->image_ ? this->image_->get_data_length() - this->offset_ : 0; }
  uint8_t *peek_data_buffer() override { return this->image_ ? this->image_->get_data_buffer() + this->offset_ : nullptr; }
  void consume_data(size_t consumed) override { this->offset_ += consumed; }
  void return_image() override { this->image_.reset(); }

 protected:
  std::shared_ptr<camera::CameraImage> image_;
  size_t offset_{0};
};

class VideoCamera : public camera::Camera, public video::ImageSink {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_quality(uint8_t q) { this->quality_ = q; }
  void set_subsampling(jpeg_down_sampling_type_t s) { this->subsampling_ = s; }

  // ImageSink (source's task): encodes when an image was requested.
  void on_image_frame(const video::ImageFrame &frame) override;
  bool wants_frames() const override;

  // camera::Camera (main loop)
  void add_listener(camera::CameraListener *l) override { this->listeners_.push_back(l); }
  camera::CameraImageReader *create_image_reader() override { return new JpegImageReader; }  // NOLINT
  void request_image(camera::CameraRequester r) override { this->single_ |= 1u << r; }
  void start_stream(camera::CameraRequester r) override;
  void stop_stream(camera::CameraRequester r) override;

 protected:
  bool ensure_output_(size_t raw_size);

  uint8_t quality_{80};
  jpeg_down_sampling_type_t subsampling_{JPEG_DOWN_SAMPLING_YUV444};
  jpeg_encoder_handle_t encoder_{nullptr};

  std::vector<camera::CameraListener *> listeners_;
  std::shared_ptr<JpegImage> current_;

  // Requests (main loop writes, source task reads).
  std::atomic<uint8_t> single_{0};
  std::atomic<uint8_t> stream_{0};

  // Hand-over of the one output buffer: IDLE -> (source task encodes) ->
  // ENCODED -> (main loop publishes) -> PUBLISHED -> (listeners release the
  // image) -> IDLE.
  enum State : uint8_t { IDLE, ENCODING, ENCODED, PUBLISHED };
  std::atomic<uint8_t> state_{IDLE};
  uint8_t *out_{nullptr};
  size_t out_cap_{0};
  size_t out_len_{0};
  uint8_t out_requesters_{0};
  std::atomic<uint32_t> errors_{0};
  uint32_t last_errors_{0};
  std::atomic<bool> format_logged_{false};
};

}  // namespace video_camera
}  // namespace esphome
