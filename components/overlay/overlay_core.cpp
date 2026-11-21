#include "overlay_core.h"

#include <algorithm>
#include <cstring>

#include "overlay_font.h"

namespace esphome {
namespace overlay {

// Distinct from each other and from the white marks; each is drawn over a
// black outline, so it shows on any palette.
const Color SPOT_COLORS[SPOT_COLOR_COUNT] = {
    {0, 255, 255},    // cyan
    {0, 255, 0},      // green
    {255, 0, 255},    // magenta
    {60, 140, 255},   // blue
    {255, 255, 0},    // yellow
    {255, 140, 0},    // orange
    {255, 105, 180},  // pink
    {160, 100, 255},  // violet
};

Style style_for_height(int height) {
  // 640x480 reference: 8 px gap, 20 px ticks, 2 px lines, 3x glyphs, 4 px margin.
  Style s;
  const float k = height / 480.0f;
  s.gap = std::max(1, static_cast<int>(8 * k + 0.5f));
  s.tick = std::max(2, static_cast<int>(20 * k + 0.5f));
  s.line_width = std::max(1, static_cast<int>(2 * k + 0.5f));
  s.glyph_scale = std::max(1, static_cast<int>(3 * k + 0.5f));
  s.margin = std::max(1, static_cast<int>(4 * k + 0.5f));
  return s;
}

namespace {

struct Canvas {
  const video::ImageFrame &img;
  void set(int x, int y, const Color &c) const {
    if (x < 0 || y < 0 || x >= img.width || y >= img.height)
      return;
    uint8_t *p = img.buffer.data() + static_cast<size_t>(y) * img.stride + static_cast<size_t>(x) * video::bytes_per_pixel(img.format);
    const uint16_t v565 = static_cast<uint16_t>(((c.r & 0xF8) << 8) | ((c.g & 0xFC) << 3) | (c.b >> 3));
    switch (img.format) {
      case video::ImageFormat::RGB565_LE:
        p[0] = static_cast<uint8_t>(v565);
        p[1] = static_cast<uint8_t>(v565 >> 8);
        break;
      case video::ImageFormat::RGB565_BE:
        p[0] = static_cast<uint8_t>(v565 >> 8);
        p[1] = static_cast<uint8_t>(v565);
        break;
      case video::ImageFormat::RGB888:
        p[0] = c.r;
        p[1] = c.g;
        p[2] = c.b;
        break;
      case video::ImageFormat::GRAY8:
        p[0] = static_cast<uint8_t>((77 * c.r + 150 * c.g + 29 * c.b + 128) >> 8);
        break;
    }
  }
};

int text_width(const std::string &t, int gs) { return t.empty() ? 0 : static_cast<int>(t.size()) * GLYPH_ADVANCE * gs - gs; }

template<typename Plot> void reticle_shape(int x, int y, const Style &s, Plot plot) {
  for (int t = 0; t < s.line_width; t++) {
    const int o = t - s.line_width / 2;
    for (int i = s.gap; i < s.gap + s.tick; i++) {
      plot(x - i, y + o);
      plot(x + i, y + o);
      plot(x + o, y - i);
      plot(x + o, y + i);
    }
  }
}

template<typename Plot> void text_shape(int tx, int ty, const std::string &t, int gs, Plot plot) {
  for (size_t i = 0; i < t.size(); i++) {
    const uint8_t *g = glyph_rows(t[i]);
    if (g == nullptr)
      continue;
    const int gx = tx + static_cast<int>(i) * GLYPH_ADVANCE * gs;
    for (int row = 0; row < GLYPH_H; row++)
      for (int col = 0; col < GLYPH_W; col++)
        if (g[row] & (0x10 >> col))
          for (int dy = 0; dy < gs; dy++)
            for (int dx = 0; dx < gs; dx++)
              plot(gx + col * gs + dx, ty + row * gs + dy);
  }
}

}  // namespace

std::vector<Rect> draw(const video::ImageFrame &img, const std::vector<Mark> &marks, const Style &s) {
  std::vector<Rect> placed;
  if (!img.buffer || img.width == 0 || img.height == 0)
    return placed;
  Canvas cv{img};
  const int W = img.width, H = img.height;
  const int reach = s.gap + s.tick + s.margin;

  // Reticles occupy space too: labels must not cover any of them.
  std::vector<Rect> reticles;
  for (const auto &m : marks)
    reticles.push_back(Rect{m.x - s.gap - s.tick, m.y - s.gap - s.tick, m.x + s.gap + s.tick + 1, m.y + s.gap + s.tick + 1});

  std::vector<std::pair<int, int>> label_pos(marks.size(), {0, 0});
  for (size_t i = 0; i < marks.size(); i++) {
    const Mark &m = marks[i];
    const int tw = text_width(m.label, s.glyph_scale), th = GLYPH_H * s.glyph_scale;
    // Below, above, right, left; then (crowded or at an edge) the same four
    // pushed further out one line at a time.
    std::vector<std::pair<int, int>> candidates;
    const int step = th + s.glyph_scale;
    const int rings = std::max(W, H) / std::max(step, 1) + 1;
    for (int k = 0; k <= rings; k++) {
      const int d = k * step;
      candidates.push_back({m.x - tw / 2, m.y + reach + d});
      candidates.push_back({m.x - tw / 2, m.y - reach - th - d});
      candidates.push_back({m.x + reach + d, m.y - th / 2});
      candidates.push_back({m.x - reach - tw - d, m.y - th / 2});
    }
    std::pair<int, int> best = candidates[0];
    int best_score = -1;
    for (const auto &c : candidates) {
      // Keep inside the image, then prefer positions that overlap nothing.
      const int cx = std::min(std::max(c.first, 0), std::max(0, W - tw));
      const int cy = std::min(std::max(c.second, 0), std::max(0, H - th));
      const Rect r{cx, cy, cx + tw, cy + th};
      int score = 2;
      for (const auto &p : placed)
        if (r.intersects(p))
          score = 0;
      // Every reticle, its own included: a position pushed back inside the
      // image (a mark at an edge) can land on its own reticle.
      for (size_t j = 0; j < reticles.size() && score; j++)
        if (r.intersects(reticles[j]))
          score = std::min(score, 1);
      if (score > best_score) {
        best_score = score;
        best = {cx, cy};
        if (score == 2)
          break;
      }
    }
    label_pos[i] = best;
    placed.push_back(Rect{best.first, best.second, best.first + tw, best.second + th});
  }

  // Outline pass, then the marks in their colours.
  static constexpr Color BLACK{0, 0, 0};
  for (int pass = s.outline ? 0 : 1; pass < 2; pass++) {
    const bool outline = pass == 0;
    const Color *color = &BLACK;
    auto plot = [&](int x, int y) {
      if (!outline) {
        cv.set(x, y, *color);
      } else {
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++)
            cv.set(x + dx, y + dy, BLACK);
      }
    };
    for (size_t i = 0; i < marks.size(); i++) {
      color = &marks[i].color;
      reticle_shape(marks[i].x, marks[i].y, s, plot);
      text_shape(label_pos[i].first, label_pos[i].second, marks[i].label, s.glyph_scale, plot);
    }
  }
  return placed;
}

const std::vector<MarkHold::Shown> &MarkHold::update(const video::Annotations &a, uint32_t kinds, uint32_t now_ms,
                                                     uint32_t hold_ms) {
  bool seen[SLOTS] = {};
  this->shown_.clear();
  for (uint8_t i = 0; i < a.count; i++) {
    const video::Annotation &an = a.items[i];
    const auto k = static_cast<uint8_t>(an.kind);
    if (k >= 32 || !(kinds & (1u << k)))
      continue;
    const int x = static_cast<int>(an.x), y = static_cast<int>(an.y);
    const bool spot = an.kind == video::AnnotationKind::SPOT;
    const size_t slot = spot ? k + an.index : k;
    if ((spot && an.index >= SPOT_SLOTS) || slot >= SLOTS || seen[slot]) {
      this->shown_.push_back(Shown{x, y, an.value, an.kind, an.index});  // no slot: as it comes
      continue;
    }
    seen[slot] = true;
    Slot &s = this->slots_[slot];
    // Position and value together, once per hold time (new: at once). A spot
    // that moved was moved by the user: at once too.
    if (!s.valid || now_ms - s.since_ms >= hold_ms || (spot && (x != s.x || y != s.y))) {
      s.valid = true;
      s.x = x;
      s.y = y;
      s.value = an.value;
      s.since_ms = now_ms;
    }
    this->shown_.push_back(Shown{s.x, s.y, s.value, an.kind, an.index});
  }
  // A kind (or spot) missing from this frame starts afresh when it returns.
  for (size_t k = 0; k < SLOTS; k++) {
    if (!seen[k])
      this->slots_[k].valid = false;
  }
  return this->shown_;
}

void MarkHold::reset() {
  for (auto &s : this->slots_)
    s.valid = false;
}

}  // namespace overlay
}  // namespace esphome
