// camera_switch's choice: the first camera listed that has sent a frame
// within the timeout; with none sending, the last choice stays.
#include <cstdio>

#include "camera_choice.h"

using namespace esphome::camera_switch;

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

int main() {
  const uint32_t T = 10000;
  // Listed: 0 = P1, 1 = MLX90640.
  uint32_t last[2] = {0, 0};
  bool seen[2] = {false, false};
  CHECK(choose_camera(last, seen, 2, 1000, T, NO_CAMERA) == NO_CAMERA);  // nothing yet

  // Only the MLX is plugged in: it is used, at one frame every 4 s.
  seen[1] = true;
  last[1] = 1000;
  CHECK(choose_camera(last, seen, 2, 1500, T, NO_CAMERA) == 1);
  CHECK(choose_camera(last, seen, 2, 1000 + T, T, 1) == 1);  // 4 s apart is well inside 10 s
  // The MLX is unplugged: after the timeout nothing sends, and its picture stays chosen.
  CHECK(choose_camera(last, seen, 2, 1001 + T, T, 1) == 1);

  // The P1 is plugged in instead: it takes over.
  seen[0] = true;
  last[0] = 20000;
  CHECK(choose_camera(last, seen, 2, 20100, T, 1) == 0);
  // Both sending: the one listed first (the P1) wins.
  last[1] = 20050;
  CHECK(choose_camera(last, seen, 2, 20100, T, 1) == 0);
  // The P1 stops (unplugged); the MLX still sends: back to the MLX.
  last[1] = 32000;
  CHECK(choose_camera(last, seen, 2, 32000, T, 0) == 1);
  // Across the millis() wrap.
  last[0] = 0xFFFFFF00u;
  CHECK(choose_camera(last, seen, 2, 0x00000100u, T, 1) == 0);

  std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "camera choice OK", g_failures,
              g_failures == 1 ? "" : "s");
  return g_failures ? 1 : 0;
}
