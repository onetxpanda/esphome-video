#pragma once

// Draws reticles with text labels directly into an image (only the overlay's
// own pixels are touched, so the cost is independent of the image size).
// Labels avoid the image edges and each other: each one takes the first of
// below / above / right / left of its reticle that fits.
//
// No ESPHome or ESP-IDF dependency; unit-tested on a host.

#include <cstdint>
#include <string>
#include <vector>

#include "../video_core/frame.h"

namespace esphome {
namespace overlay {

struct Mark {
  int x, y;           // reticle centre, image pixels
  std::string label;  // characters available in overlay_font.h
};

struct Style {
  int gap{8};          // empty radius around the reticle centre
  int tick{20};        // tick length
  int line_width{2};
  int glyph_scale{3};  // 5x7 font scaled by this
  int margin{4};       // reticle to label distance
  bool outline{true};  // one-pixel black outline under all marks
};

// Proportions that match a 640x480 image, scaled to `height`.
Style style_for_height(int height);

struct Rect {
  int x0, y0, x1, y1;  // inclusive-exclusive
  bool intersects(const Rect &o) const { return x0 < o.x1 && o.x0 < x1 && y0 < o.y1 && o.y0 < y1; }
};

// Draws `marks` onto `img` (RGB565_LE/BE, RGB888 or GRAY8). Returns the label
// rectangles used, for tests.
std::vector<Rect> draw(const video::ImageFrame &img, const std::vector<Mark> &marks, const Style &style);

// What to show for each annotation, frame by frame. A mark moves as soon as
// its position changes, and its value changes with it. While it stays put,
// its value is refreshed at most once per hold time, so the label doesn't
// change with every frame's noise. Kept per annotation kind (the first of
// each kind; any further one of the same kind shows as it comes).
class MarkHold {
 public:
  struct Shown {
    int x, y;  // image pixels
    float value;
    video::AnnotationKind kind;
  };
  // Takes a frame's annotations of the kinds in `kinds` (a bit per
  // AnnotationKind) at `now_ms`; returns what to draw.
  const std::vector<Shown> &update(const video::Annotations &a, uint32_t kinds, uint32_t now_ms, uint32_t hold_ms);
  // Forgets everything shown (the image size changed): the next frame shows as it comes.
  void reset();

 protected:
  struct Slot {
    bool valid{false};
    int x{0}, y{0};
    float value{0};
    uint32_t since_ms{0};  // when the value shown was taken
  };
  static constexpr size_t SLOTS = 8;
  Slot slots_[SLOTS];
  std::vector<Shown> shown_;
};

}  // namespace overlay
}  // namespace esphome
