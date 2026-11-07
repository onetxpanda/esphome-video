#include "overlay_core.h"

#include <algorithm>
#include <cstring>

#include "overlay_font.h"

namespace esphome {
namespace overlay {

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
  void set(int x, int y, bool white) const {
    if (x < 0 || y < 0 || x >= img.width || y >= img.height)
      return;
    uint8_t *p = img.buffer.data() + static_cast<size_t>(y) * img.stride + static_cast<size_t>(x) * video::bytes_per_pixel(img.format);
    const uint8_t v = white ? 0xFF : 0x00;
    switch (img.format) {
      case video::ImageFormat::RGB565_LE:
      case video::ImageFormat::RGB565_BE:
        p[0] = v;
        p[1] = v;
        break;
      case video::ImageFormat::RGB888:
        p[0] = p[1] = p[2] = v;
        break;
      case video::ImageFormat::GRAY8:
        p[0] = v;
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
      for (size_t j = 0; j < reticles.size() && score; j++)
        if (j != i && r.intersects(reticles[j]))
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

  // Outline pass, then the marks in white.
  for (int pass = s.outline ? 0 : 1; pass < 2; pass++) {
    const bool white = pass == 1;
    auto plot = [&](int x, int y) {
      if (white) {
        cv.set(x, y, true);
      } else {
        for (int dy = -1; dy <= 1; dy++)
          for (int dx = -1; dx <= 1; dx++)
            cv.set(x + dx, y + dy, false);
      }
    };
    for (size_t i = 0; i < marks.size(); i++) {
      reticle_shape(marks[i].x, marks[i].y, s, plot);
      text_shape(label_pos[i].first, label_pos[i].second, marks[i].label, s.glyph_scale, plot);
    }
  }
  return placed;
}

}  // namespace overlay
}  // namespace esphome
