#include "overlay.h"

#include <cstdio>

#include "esphome/core/log.h"

namespace esphome {
namespace overlay {

static const char *const TAG = "overlay";

void Overlay::setup() {
  if (!this->sinks_.modifying_sink_is_exclusive())
    this->mark_failed(LOG_STR("An overlay after this one must be its only consumer"));
}

void Overlay::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Overlay:\n"
                "  Marks:%s%s%s\n"
                "  Value hold: %u ms, unit: %s, enabled: %s",
                this->kinds_ & (1u << 0) ? " min" : "", this->kinds_ & (1u << 1) ? " max" : "",
                this->kinds_ & (1u << 2) ? " center" : "", static_cast<unsigned>(this->hold_ms_),
                this->unit_ ? "F" : "C", YESNO(this->enabled_.load()));
}

void Overlay::on_image_frame(const video::ImageFrame &frame) {
  if (this->enabled_ && frame.annotations.count > 0) {
    if (frame.width != this->last_w_ || frame.height != this->last_h_) {
      this->style_ = style_for_height(frame.height);
      this->last_w_ = frame.width;
      this->last_h_ = frame.height;
      this->hold_.reset();
    }
    const bool f = this->unit_ == static_cast<uint8_t>(Unit::FAHRENHEIT);
    this->marks_.clear();
    for (const auto &s : this->hold_.update(frame.annotations, this->kinds_, frame.timestamp_ms, this->hold_ms_)) {
      const float v = f ? s.value * 9.0f / 5.0f + 32.0f : s.value;
      char text[16];
      std::snprintf(text, sizeof(text), "%.1f\xB0%c", v, f ? 'F' : 'C');
      this->marks_.push_back(Mark{s.x, s.y, text});
    }
    // Draws into the pixels: the frame is ours alone until delivered (see
    // modifies_frames()).
    draw(frame, this->marks_, this->style_);
  }
  this->sinks_.deliver(frame, [](video::ImageSink *s, const video::ImageFrame &fr) { s->on_image_frame(fr); });
}

}  // namespace overlay
}  // namespace esphome
