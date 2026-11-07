// Unit tests for MLX90640 sub-page assembly.
#include <cmath>
#include <cstdio>
#include <vector>

#include "subpage_assembler.h"

using namespace esphome::mlx90640;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    g_checks++;                                                   \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

// Sub-page `sp` of a scene: pixels of that sub-page hold `fresh`, the rest
// hold `stale` (what CalculateTo leaves from the previous sub-page).
static std::vector<float> subpage(int sp, bool chess, float fresh, float stale) {
  std::vector<float> t(SENSOR_PIXELS);
  for (int r = 0; r < SENSOR_ROWS; r++)
    for (int c = 0; c < SENSOR_COLS; c++)
      t[r * SENSOR_COLS + c] = ((chess ? (r + c) & 1 : r & 1) == sp) ? fresh : stale;
  return t;
}

int main() {
  for (bool chess : {true, false}) {
    SubpageAssembler a;
    a.set_mode(FrameMode::MERGE);
    CHECK(!a.add(subpage(0, chess, 20, -99).data(), 0, chess));  // half a frame
    CHECK(a.add(subpage(1, chess, 30, -99).data(), 1, chess));   // pair complete
    bool ok = true;
    for (int r = 0; r < SENSOR_ROWS; r++)
      for (int c = 0; c < SENSOR_COLS; c++) {
        const float want = ((chess ? (r + c) & 1 : r & 1) == 0) ? 20 : 30;
        ok = ok && a.frame()[r * SENSOR_COLS + c] == want;  // measured pixels only, never stale ones
      }
    CHECK(ok);
    CHECK(!a.add(subpage(1, chess, 30, -99).data(), 1, chess));  // next pair starts afresh
    a.set_mode(FrameMode::MERGE);
    CHECK(!a.add(subpage(0, chess, 20, -99).data(), 0, chess));  // mode change restarts the pair
    CHECK(!a.add(nullptr, 0, chess) && !a.add(subpage(0, chess, 1, 1).data(), 2, chess));

    a.set_mode(FrameMode::INTERPOLATE);
    CHECK(a.add(subpage(1, chess, 25, -99).data(), 1, chess));  // every sub-page is a frame
    bool interp = true;
    for (int i = 0; i < SENSOR_PIXELS; i++)
      interp = interp && std::fabs(a.frame()[i] - 25) < 1e-4f;  // stale half replaced from fresh neighbours
    CHECK(interp);
  }
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
