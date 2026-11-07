#pragma once

// An image source that needs no camera: moving colour bars, a sweeping box
// and a binary frame counter, in RGB565 little-endian. For checking encoders
// and streams. Frames are only generated while a sink wants them.

#include <memory>

#include "esphome/core/component.h"

#include "../video_core/frame.h"

namespace esphome {
namespace video_test_pattern {

class TestPattern : public Component, public video::ImageSource {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_size(uint16_t width, uint16_t height) {
    this->width_ = width;
    this->height_ = height;
  }
  void set_fps(uint8_t fps) { this->fps_ = fps; }
  void add_image_sink(video::ImageSink *sink) override { this->sinks_.add(sink); }

 protected:
  void render_(uint8_t *dst, uint32_t frame);

  uint16_t width_{320};
  uint16_t height_{240};
  uint8_t fps_{10};
  video::SinkList<video::ImageSink> sinks_;
  std::unique_ptr<video::FramePool> pool_;
  uint32_t frame_{0};
  uint32_t next_ms_{0};
};

}  // namespace video_test_pattern
}  // namespace esphome
