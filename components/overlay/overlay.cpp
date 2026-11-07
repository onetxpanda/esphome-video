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
                "  Position hold: %u ms, unit: %s, enabled: %s",
                this->kinds_ & (1u << 0) ? " min" : "", this->kinds_ & (1u << 1) ? " max" : "",
                this->kinds_ & (1u << 2) ? " center" : "", static_cast<unsigned>(this->hold_ms_),
                this->unit_ ? "F" : "C", YESNO(this->enabled_.load()));
}

void Overlay::on_image_frame(const video::ImageFrame &frame) {
  if (this->enabled_ && frame.annotations.count > 0) {
    // Positions: the newest annotations, at most once per hold time (and
    // immediately when the image size changes). Values: always the newest.
    const bool resized = frame.width != this->last_w_ || frame.height != this->last_h_;
    if (!this->have_held_ || resized || frame.timestamp_ms - this->held_ms_ >= this->hold_ms_) {
      this->held_ = frame.annotations;
      this->held_ms_ = frame.timestamp_ms;
      this->have_held_ = true;
    }
    if (resized) {
      this->style_ = style_for_height(frame.height);
      this->last_w_ = frame.width;
      this->last_h_ = frame.height;
    }
    const bool f = this->unit_ == static_cast<uint8_t>(Unit::FAHRENHEIT);
    this->marks_.clear();
    for (uint8_t i = 0; i < frame.annotations.count; i++) {
      const video::Annotation &a = frame.annotations.items[i];
      if (!(this->kinds_ & (1u << static_cast<uint8_t>(a.kind))))
        continue;
      // Held position for this kind, if it had one.
      float x = a.x, y = a.y;
      for (uint8_t j = 0; j < this->held_.count; j++) {
        if (this->held_.items[j].kind == a.kind) {
          x = this->held_.items[j].x;
          y = this->held_.items[j].y;
          break;
        }
      }
      const float v = f ? a.value * 9.0f / 5.0f + 32.0f : a.value;
      char text[16];
      std::snprintf(text, sizeof(text), "%.1f\xB0%c", v, f ? 'F' : 'C');
      this->marks_.push_back(Mark{static_cast<int>(x), static_cast<int>(y), text});
    }
    // Draws into the pixels: the frame is ours alone until delivered (see
    // modifies_frames()).
    draw(frame, this->marks_, this->style_);
  }
  this->sinks_.deliver(frame, [](video::ImageSink *s, const video::ImageFrame &fr) { s->on_image_frame(fr); });
}

}  // namespace overlay
}  // namespace esphome
