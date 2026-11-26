// The encoder's three-slot hand-over: the source never fills the slot the
// encoder reads (encodes or repeats), never waits for the encoder, and the
// encoder always gets the newest finished frame. Run under ThreadSanitizer,
// which reports any read of a slot while it is being filled.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

#include "slot_exchange.h"

using namespace esphome::h264_encoder;

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

static void rules() {
  SlotExchange x;
  bool fresh = true;
  CHECK(x.take(&fresh) == SlotExchange::NONE && !fresh);

  const int a = x.begin_write();
  CHECK(a != SlotExchange::NONE);
  CHECK(x.begin_write() == SlotExchange::NONE);  // one fill at a time
  CHECK(x.finish_write(a) == SlotExchange::NONE);
  CHECK(x.take(&fresh) == a && fresh);  // the encoder takes it
  CHECK(x.take(&fresh) == a && !fresh);  // and keeps it (repeats)

  const int b = x.begin_write();
  CHECK(b != SlotExchange::NONE && b != a);  // never the encoder's slot
  CHECK(x.finish_write(b) == SlotExchange::NONE);
  const int c = x.begin_write();
  CHECK(c != SlotExchange::NONE && c != a && c != b);  // nor the ready one
  CHECK(x.finish_write(c) == b);  // b was never taken: dropped, and reserved
  CHECK(x.begin_write() == SlotExchange::NONE);  // until released
  x.release_done();
  CHECK(x.take(&fresh) == c && fresh);  // the newest frame, not b

  const int d = x.begin_write();
  CHECK(d != SlotExchange::NONE && d != c);
  x.abandon_write();
  CHECK(x.take(&fresh) == c && !fresh);  // an abandoned fill changes nothing
  CHECK(x.begin_write() != SlotExchange::NONE);
}

// A fast source and a slow encoder on two threads.
static void threads() {
  constexpr int SLOTS = SlotExchange::SLOTS, WORDS = 4096, FRAMES = 3000;
  SlotExchange x;
  std::vector<int> data[SLOTS];
  int seq[SLOTS] = {};
  for (auto &d : data)
    d.assign(WORDS, 0);
  std::atomic<bool> filling[SLOTS] = {};
  std::atomic<bool> done{false};
  std::atomic<int> torn{0}, backwards{0}, read_while_filling{0}, taken{0}, dropped{0}, skipped{0};

  std::thread source([&] {
    for (int n = 1; n <= FRAMES; n++) {
      const int i = x.begin_write();
      if (i == SlotExchange::NONE) {
        skipped++;
        continue;
      }
      filling[i] = true;
      for (int w = 0; w < WORDS; w++)
        data[i][w] = n;
      seq[i] = n;
      filling[i] = false;
      const int r = x.finish_write(i);
      if (r != SlotExchange::NONE) {
        data[r][0] = -1;  // "release" what the dropped frame held
        x.release_done();
        dropped++;
      }
      if (n % 7 == 0)
        std::this_thread::yield();
    }
    done = true;
  });

  std::thread encoder([&] {
    int last = 0;
    while (!done || last < FRAMES) {
      bool fresh = false;
      const int i = x.take(&fresh);
      if (i == SlotExchange::NONE) {
        std::this_thread::yield();
        continue;
      }
      if (filling[i])
        read_while_filling++;
      const int n = seq[i];
      for (int w = 0; w < WORDS; w++) {
        if (data[i][w] != n) {
          torn++;
          break;
        }
      }
      if (fresh) {
        taken++;
        if (n <= last)
          backwards++;
        last = n;
      }
      // Encoding (and repeating) takes longer than a fill.
      std::this_thread::sleep_for(std::chrono::microseconds(50));
      if (done && !fresh)
        break;
    }
  });
  source.join();
  encoder.join();

  std::printf("  %d frames: %d encoded, %d dropped for newer ones, %d skipped\n", FRAMES, taken.load(),
              dropped.load(), skipped.load());
  CHECK(read_while_filling == 0);
  CHECK(torn == 0);
  CHECK(backwards == 0);
  CHECK(skipped == 0);  // a single source never waits or skips
  CHECK(taken > 0 && dropped > 0);
}

int main() {
  rules();
  threads();
  std::printf("%s (%d failure%s)\n", g_failures ? "FAILED" : "slot exchange OK", g_failures,
              g_failures == 1 ? "" : "s");
  return g_failures ? 1 : 0;
}
