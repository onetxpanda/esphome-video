#pragma once

// 256-entry colour palettes for tone mapping, in the order of
// video::PALETTE_NAMES (shared with cameras that colour their own image).
// Ironbow is the irradiance app's (11 evenly spaced stops); lava, rainbow,
// iron gray, red hot and rainbow 2 approximate the usual thermal palettes of
// those names and are not copies of any camera's tables.

#include <array>

#include "isp_core.h"

namespace esphome {
namespace isp {

enum class Palette : uint8_t {
  WHITE_HOT = 0,
  BLACK_HOT = 1,
  IRONBOW = 2,
  LAVA = 3,
  RAINBOW = 4,
  IRON_GRAY = 5,
  RED_HOT = 6,
  RAINBOW_2 = 7,
};
static constexpr uint8_t PALETTE_COUNT = 8;

struct PaletteStop {
  int pos, r, g, b;
};

// Linear interpolation between stops; the first stop must be at 0 and the
// last at 255.
template<size_t N> inline std::array<Rgb, 256> palette_from_stops(const PaletteStop (&stops)[N]) {
  std::array<Rgb, 256> out{};
  for (int v = 0; v < 256; v++) {
    for (size_t i = 0; i + 1 < N; i++) {
      const PaletteStop &a = stops[i], &b = stops[i + 1];
      if (v <= b.pos) {
        const int f = v - a.pos, span = b.pos - a.pos > 0 ? b.pos - a.pos : 1;
        out[v] = Rgb{static_cast<uint8_t>(a.r + (b.r - a.r) * f / span),
                     static_cast<uint8_t>(a.g + (b.g - a.g) * f / span),
                     static_cast<uint8_t>(a.b + (b.b - a.b) * f / span)};
        break;
      }
    }
  }
  return out;
}

// Evenly spaced stops, interpolated in float as the irradiance app does
// (except that the last entry is the last stop, not the one before it).
template<size_t N> inline std::array<Rgb, 256> palette_from_even_stops(const uint8_t (&stops)[N][3]) {
  static_assert(N >= 2, "two stops at least");
  std::array<Rgb, 256> out{};
  for (int v = 0; v < 256; v++) {
    const float scaled = v / 255.0f * static_cast<float>(N - 1);
    int idx = static_cast<int>(scaled);
    if (idx > static_cast<int>(N) - 2)
      idx = static_cast<int>(N) - 2;
    const float frac = scaled - static_cast<float>(idx);
    const uint8_t *a = stops[idx], *b = stops[idx + 1];
    out[v] = Rgb{static_cast<uint8_t>(a[0] + frac * (b[0] - a[0])), static_cast<uint8_t>(a[1] + frac * (b[1] - a[1])),
                 static_cast<uint8_t>(a[2] + frac * (b[2] - a[2]))};
  }
  return out;
}

inline std::array<Rgb, 256> make_palette(Palette p) {
  switch (p) {
    case Palette::WHITE_HOT: {
      static constexpr PaletteStop S[] = {{0, 0, 0, 0}, {255, 255, 255, 255}};
      return palette_from_stops(S);
    }
    case Palette::BLACK_HOT: {
      static constexpr PaletteStop S[] = {{0, 255, 255, 255}, {255, 0, 0, 0}};
      return palette_from_stops(S);
    }
    case Palette::LAVA: {  // black, deep blue, purple, red, orange, yellow, white
      static constexpr PaletteStop S[] = {{0, 0, 0, 0},       {40, 20, 10, 90},   {90, 120, 20, 140},
                                          {140, 220, 30, 40}, {190, 250, 130, 0}, {230, 255, 230, 60},
                                          {255, 255, 255, 255}};
      return palette_from_stops(S);
    }
    case Palette::RAINBOW: {  // navy, blue, cyan, green, yellow, orange, red
      static constexpr PaletteStop S[] = {{0, 0, 0, 128},    {40, 0, 0, 255},     {90, 0, 200, 255},
                                          {130, 0, 255, 80}, {170, 255, 255, 0},  {210, 255, 128, 0},
                                          {255, 255, 0, 0}};
      return palette_from_stops(S);
    }
    case Palette::IRON_GRAY: {  // grey, with the hottest part in iron colours
      static constexpr PaletteStop S[] = {{0, 0, 0, 0},         {170, 170, 170, 170}, {200, 200, 40, 40},
                                          {230, 255, 160, 0},   {255, 255, 255, 200}};
      return palette_from_stops(S);
    }
    case Palette::RED_HOT: {  // grey, with the hottest part in red
      static constexpr PaletteStop S[] = {{0, 0, 0, 0}, {170, 170, 170, 170}, {220, 255, 60, 30}, {255, 255, 0, 0}};
      return palette_from_stops(S);
    }
    case Palette::RAINBOW_2: {  // high contrast: black through the spectrum to white
      static constexpr PaletteStop S[] = {{0, 0, 0, 0},      {32, 90, 0, 140},    {64, 0, 0, 255},
                                          {96, 0, 180, 255}, {128, 0, 255, 0},    {160, 200, 255, 0},
                                          {192, 255, 170, 0}, {224, 255, 0, 0},   {255, 255, 255, 255}};
      return palette_from_stops(S);
    }
    case Palette::IRONBOW:
    default: {
      static constexpr uint8_t S[][3] = {
          {0, 0, 10},      // very dark blue-black
          {16, 0, 85},     // dark purple
          {60, 0, 153},    // medium purple
          {100, 0, 157},   // light purple
          {177, 1, 152},   // purple-magenta
          {206, 20, 144},  // magenta-red
          {224, 52, 93},   // red
          {244, 116, 0},   // orange
          {254, 180, 0},   // yellow-orange
          {255, 218, 14},  // yellow
          {255, 255, 246}, // bright yellow-white
      };
      return palette_from_even_stops(S);
    }
  }
}

}  // namespace isp
}  // namespace esphome
