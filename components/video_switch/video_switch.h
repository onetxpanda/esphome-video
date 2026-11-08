#pragma once

// Passes on the frames of one chosen entry among several image sources, so
// every consumer after it (encoder, camera) shows the same choice. An entry
// may hold several sources, e.g. a camera's two outputs of which only one
// produces frames at a time. Frames pass on the producing source's task.

#include <atomic>
#include <memory>
#include <mutex>
#include <vector>

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "../video_core/frame.h"

namespace esphome {
namespace video_switch {

class VideoSwitch;

// The switch's input from one source; knows which entry the source is in.
class SwitchInput : public video::ImageSink {
 public:
  SwitchInput(VideoSwitch *sw, uint8_t entry) : switch_(sw), entry_(entry) {}
  void on_image_frame(const video::ImageFrame &frame) override;
  bool wants_frames() const override;

 protected:
  VideoSwitch *switch_;
  uint8_t entry_;
};

class VideoSwitch : public Component, public video::ImageSource {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;

  void add_image_sink(video::ImageSink *sink) override { this->sinks_.add(sink); }

  // Code generation: the entries in order, then each source with its entry.
  void add_entry(const char *name) { this->names_.push_back(name); }
  void add_source(video::ImageSource *source, uint8_t entry);

  size_t entry_count() const { return this->names_.size(); }
  uint8_t get_active() const { return this->active_; }
  // Main loop. Consumers are told the content changed (an encoder then starts
  // with a keyframe).
  void set_active(uint8_t entry);

  // From the inputs (the sources' tasks).
  void handle_frame(uint8_t entry, const video::ImageFrame &frame);
  bool entry_wanted(uint8_t entry) const { return entry == this->active_ && this->sinks_.any_wants(); }

 protected:
  std::vector<const char *> names_;
  std::vector<std::unique_ptr<SwitchInput>> inputs_;
  video::SinkList<video::ImageSink> sinks_;
  std::atomic<uint8_t> active_{0};
  // Frames from two sources of the active entry (during a camera's mode
  // change) reach the consumers one at a time; a frame arriving while another
  // is being delivered is dropped, not waited for.
  std::mutex deliver_mutex_;
};

// Select entity choosing the switch's entry; the choice survives reboots
// unless restore_value is false.
class SwitchSelect : public select::Select, public Component {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void dump_config() override;
  void set_switch(VideoSwitch *s) { this->switch_ = s; }
  void set_restore(bool r) { this->restore_ = r; }

 protected:
  void control(size_t index) override;

  VideoSwitch *switch_{nullptr};
  bool restore_{true};
  ESPPreferenceObject pref_;
};

}  // namespace video_switch
}  // namespace esphome
