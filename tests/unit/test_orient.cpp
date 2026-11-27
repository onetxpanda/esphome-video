// Orientation (mirroring + rotation), fitting, and spots on a turned picture:
// a spot placed on the picture as shown reads the source pixel shown there,
// for every mirror and rotation, and the same with or without rotation.
#include <cmath>
#include <cstdio>
#include <vector>

#include "isp_core.h"

using namespace esphome::isp;

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

static const Rotation ROTS[] = {Rotation::R0, Rotation::R90, Rotation::R180, Rotation::R270};

int main() {
  // A 5 x 3 frame (like the sensors, wider than tall) with a distinct value per pixel.
  const uint16_t w = 5, h = 3;
  std::vector<float> in(w * h);
  for (int i = 0; i < w * h; i++)
    in[i] = 100.0f + i;

  int combos = 0;
  for (Rotation r : ROTS)
    for (int m = 0; m < 4; m++) {
      const bool mx = m & 1, my = m & 2;
      const uint16_t ow = rotation_swaps(r) ? h : w, oh = rotation_swaps(r) ? w : h;
      std::vector<float> out(w * h, -1.0f);
      orient(in.data(), w, h, mx, my, r, out.data());
      // Every source pixel lands exactly where orient_point says.
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++) {
          float ox, oy;
          orient_point(static_cast<float>(x), static_cast<float>(y), w, h, mx, my, r, &ox, &oy);
          CHECK(ox >= 0 && ox < ow && oy >= 0 && oy < oh);
          CHECK(out[static_cast<int>(oy) * ow + static_cast<int>(ox)] == in[y * w + x]);
        }
      // A spot at each pixel centre of the picture as shown reads that pixel.
      Options plain;
      for (int y = 0; y < oh; y++)
        for (int x = 0; x < ow; x++) {
          const float fx = (x + 0.5f) / ow, fy = (y + 0.5f) / oh;
          CHECK(std::fabs(spot_value(out.data(), ow, oh, plain, fx, fy) - out[y * ow + x]) < 1e-4f);
        }
      // Without rotation, the mirrored path (mirroring done by spot_value)
      // reads what the turned frame shows at the same place.
      if (r == Rotation::R0) {
        Options mo;
        mo.mirror_x = mx;
        mo.mirror_y = my;
        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++) {
            const float fx = (x + 0.5f) / w, fy = (y + 0.5f) / h;
            CHECK(std::fabs(spot_value(in.data(), w, h, mo, fx, fy) - out[y * w + x]) < 1e-4f);
          }
      }
      combos++;
    }
  CHECK(combos == 16);

  // Clockwise: the top-left pixel goes to the top-right at 90 degrees, and to
  // the bottom-left at 270.
  {
    std::vector<float> out(w * h);
    orient(in.data(), w, h, false, false, Rotation::R90, out.data());
    CHECK(out[0 * h + (h - 1)] == in[0]);  // 3 wide now
    orient(in.data(), w, h, false, false, Rotation::R270, out.data());
    CHECK(out[(w - 1) * h + 0] == in[0]);
    orient(in.data(), w, h, false, false, Rotation::R180, out.data());
    CHECK(out[(h - 1) * w + (w - 1)] == in[0]);
  }

  // Fitting into 640 x 480.
  CHECK(fit_scale(32, 24, 640, 480) == 20);   // MLX90640
  CHECK(fit_scale(160, 120, 640, 480) == 4);  // Thermal Master P1
  CHECK(fit_scale(120, 160, 640, 480) == 3);  // P1 turned 90 degrees: 360 x 480
  CHECK(fit_scale(24, 32, 640, 480) == 15);   // MLX turned: 360 x 480
  CHECK(fit_scale(384, 288, 640, 480) == 1);  // HTI-301: already larger than half
  CHECK(fit_scale(1000, 800, 640, 480) == 1);
  CHECK(fit_scale(0, 0, 640, 480) == 1);

  std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "orientation OK", g_failures,
              g_failures == 1 ? "" : "s");
  return g_failures ? 1 : 0;
}
