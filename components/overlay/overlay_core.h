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

}  // namespace overlay
}  // namespace esphome
