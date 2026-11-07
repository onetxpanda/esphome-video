#pragma once

// 256-entry colour palettes for tone mapping.

#include <array>

#include "isp_core.h"

namespace esphome {
namespace isp {

enum class Palette : uint8_t { IRON = 0, GRAY = 1 };

inline std::array<Rgb, 256> make_palette(Palette p) {
  std::array<Rgb, 256> out{};
  if (p == Palette::GRAY) {
    for (int i = 0; i < 256; i++)
      out[i] = Rgb{static_cast<uint8_t>(i), static_cast<uint8_t>(i), static_cast<uint8_t>(i)};
    return out;
  }
  // Iron: black, purple, red, orange, yellow, white.
  struct Stop {
    int pos, r, g, b;
  };
  static constexpr Stop STOPS[] = {{0, 0, 0, 0},       {32, 74, 0, 85},     {64, 148, 0, 170},
                                   {96, 196, 0, 85},   {128, 220, 0, 0},    {160, 255, 110, 0},
                                   {192, 255, 210, 0}, {224, 253, 252, 124}, {255, 255, 255, 255}};
  for (int v = 0; v < 256; v++) {
    for (size_t i = 0; i + 1 < sizeof(STOPS) / sizeof(STOPS[0]); i++) {
      const Stop &a = STOPS[i], &b = STOPS[i + 1];
      if (v <= b.pos) {
        const int f = v - a.pos, span = b.pos - a.pos;
        out[v] = Rgb{static_cast<uint8_t>(a.r + (b.r - a.r) * f / span),
                     static_cast<uint8_t>(a.g + (b.g - a.g) * f / span),
                     static_cast<uint8_t>(a.b + (b.b - a.b) * f / span)};
        break;
      }
    }
  }
  return out;
}

}  // namespace isp
}  // namespace esphome
