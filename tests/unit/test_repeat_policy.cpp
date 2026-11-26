// The encoder's frame repeats: when a slow source's frames are copied, when
// a repeat is due, when repeats stop, the declared rate, and timestamps that
// never go backwards.
#include <cstdio>

#include "repeat_policy.h"

using namespace esphome::h264_encoder;

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

// Mean frame interval in 1/16 ms, as the encoder measures it.
static uint32_t interval_x16(float fps) { return static_cast<uint32_t>(16000.0f / fps + 0.5f); }

int main() {
  RepeatPolicy off;
  CHECK(!off.enabled());
  CHECK(!off.source_slow(0));
  CHECK(!off.source_slow(interval_x16(0.25f)));
  CHECK(!off.may_repeat(1000, 1000));
  CHECK(off.declared_fps(1) == 1);

  RepeatPolicy p;
  p.min_fps = 5;
  CHECK(p.enabled());
  CHECK(p.period_ms() == 200);

  // Unmeasured and slow sources are copied; fast ones are not.
  CHECK(p.source_slow(0));
  CHECK(p.source_slow(interval_x16(0.25f)));  // MLX90640 at 0.5 Hz, merged
  CHECK(p.source_slow(interval_x16(4.0f)));
  CHECK(!p.source_slow(interval_x16(5.0f)));
  CHECK(!p.source_slow(interval_x16(25.0f)));  // HTI-301

  // A repeat is due one period after the last frame went out.
  CHECK(p.wait_ms(1000, 1000) == 200);
  CHECK(p.wait_ms(1150, 1000) == 50);
  CHECK(p.wait_ms(1200, 1000) == 0);
  CHECK(p.wait_ms(5000, 1000) == 0);
  CHECK(p.wait_ms(100, 0xFFFFFFF0u) == 84);  // across the millis() wrap

  // Repeats stop 10 s after the last source frame.
  CHECK(p.may_repeat(1000, 1000));
  CHECK(p.may_repeat(11000, 1000));
  CHECK(!p.may_repeat(11001, 1000));
  CHECK(p.may_repeat(5000, 0xFFFFF000u));  // across the wrap

  // Declared rate: the source's, not below min_fps.
  CHECK(p.declared_fps(1) == 5);
  CHECK(p.declared_fps(5) == 5);
  CHECK(p.declared_fps(25) == 25);

  // Timestamps only move forward.
  CHECK(next_timestamp(500, 0, false) == 500);
  CHECK(next_timestamp(1200, 1000, true) == 1200);
  CHECK(next_timestamp(1000, 1000, true) == 1001);
  CHECK(next_timestamp(900, 1000, true) == 1001);  // captured before a repeat went out
  CHECK(next_timestamp(5, 0xFFFFFFF0u, true) == 5);  // across the wrap

  // A slow source at 0.25 fps: between two source frames 4 s apart the
  // stream carries a frame every 200 ms, with rising timestamps.
  {
    uint32_t now = 10000, last_out_ms = now, last_ts = now, frames = 1;
    const uint32_t next_source = now + 4000;
    while (true) {
      now += p.wait_ms(now, last_out_ms);
      if (now >= next_source)
        break;
      CHECK(p.may_repeat(now, 10000));
      const uint32_t ts = next_timestamp(last_ts + (now - last_out_ms), last_ts, true);
      CHECK(ts > last_ts);
      last_ts = ts;
      last_out_ms = now;
      frames++;
    }
    CHECK(frames == 20);
  }

  std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "repeat policy OK", g_failures,
              g_failures == 1 ? "" : "s");
  return g_failures ? 1 : 0;
}
