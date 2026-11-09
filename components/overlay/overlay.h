#pragma once

// Draws a frame's annotations (reticles with value labels) into the image
// and passes it on. Draws in place, so it must be its source's only sink.
// A mark moves as soon as its annotation does, with its new value; while it
// stays put, its value is refreshed once per hold time, so the label doesn't
// change with every frame's noise.

#include <atomic>
#include <mutex>
#include <vector>

#include "esphome/core/component.h"

#include "../video_core/frame.h"
#include "overlay_core.h"

namespace esphome {
namespace overlay {

enum class Unit : uint8_t { CELSIUS = 0, FAHRENHEIT = 1 };

class Overlay : public Component, public video::ImageSink, public video::ImageSource {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;

  // ImageSink (source's task)
  void on_image_frame(const video::ImageFrame &frame) override;
  bool wants_frames() const override { return this->sinks_.any_wants(); }
  bool modifies_frames() const override { return true; }
  // ImageSource
  void add_image_sink(video::ImageSink *sink) override { this->sinks_.add(sink); }

  void add_kind(video::AnnotationKind k) { this->kinds_ |= 1u << static_cast<uint8_t>(k); }
  void set_hold_ms(uint32_t ms) { this->hold_ms_ = ms; }
  // Thread-safe.
  void set_enabled(bool v) { this->enabled_ = v; }
  void set_unit(Unit u) { this->unit_ = static_cast<uint8_t>(u); }

 protected:
  video::SinkList<video::ImageSink> sinks_;
  uint32_t kinds_{0};
  uint32_t hold_ms_{1000};
  std::atomic<bool> enabled_{true};
  std::atomic<uint8_t> unit_{0};

  // Source's task only.
  MarkHold hold_;
  uint16_t last_w_{0}, last_h_{0};
  Style style_;
  std::vector<Mark> marks_;
};

}  // namespace overlay
}  // namespace esphome
