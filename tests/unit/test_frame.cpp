// Unit tests for video_core frames and the buffer pool.
#include <atomic>
#include <cstdio>
#include <cstdint>
#include <thread>
#include <vector>

#include "frame.h"

using namespace esphome::video;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    g_checks++;                                                         \
    if (!(cond)) {                                                      \
      g_failures++;                                                     \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
    }                                                                   \
  } while (0)

static void test_pool_basics() {
  FramePool pool(2, 1000, 64);
  CHECK(pool.ok() && pool.free_count() == 2);
  FrameRef a = pool.acquire();
  FrameRef b = pool.acquire();
  CHECK(a && b && a.data() != b.data());
  CHECK(reinterpret_cast<uintptr_t>(a.data()) % 64 == 0);
  CHECK(a.capacity() == 1024 && pool.buffer_size() == 1024);  // rounded up to the alignment (DMA engines need it)
  CHECK(!pool.acquire());  // exhausted: producer drops the frame
  {
    FrameRef c = a;  // a consumer keeps the frame
    a.reset();
    CHECK(pool.free_count() == 0);  // still referenced
  }
  CHECK(pool.free_count() == 1);  // last reference returned it
  FrameRef d = std::move(b);
  CHECK(!b && d);
  d = FrameRef();
  CHECK(pool.free_count() == 2);
  FrameRef e = pool.acquire();
  e = e;  // self-assignment keeps the reference
  CHECK(e && pool.free_count() == 1);
}

static void test_frames_hold_buffers() {
  FramePool pool(1, 32 * 24 * sizeof(float));
  RadiometricFrame f;
  f.buffer = pool.acquire();
  f.width = 32;
  f.height = 24;
  f.temps()[0] = 21.5f;
  RadiometricFrame kept = f;  // copying a frame keeps its buffer
  f = RadiometricFrame();
  CHECK(pool.free_count() == 0 && kept.temps()[0] == 21.5f);
  kept = RadiometricFrame();
  CHECK(pool.free_count() == 1);
}

static void test_failed_allocation() {
  BufferAllocator none;
  none.alloc = [](size_t, size_t) -> void * { return nullptr; };
  none.free = [](void *) {};
  FramePool pool(3, 100, 64, none);
  CHECK(!pool.ok() && !pool.acquire());
}

// Producer on one thread, consumer on another holding frames for a while:
// every buffer must come back, and a held buffer must never be handed out.
static void test_threads() {
  FramePool pool(3, 4096);
  std::atomic<bool> done{false};
  std::atomic<int> handed{0}, consumed{0}, dropped{0}, reused_while_held{0};
  std::vector<FrameRef> queue;
  std::mutex qm;
  std::thread consumer([&] {
    while (!done || !queue.empty()) {
      FrameRef r;
      {
        std::lock_guard<std::mutex> l(qm);
        if (!queue.empty()) {
          r = std::move(queue.front());
          queue.erase(queue.begin());
        }
      }
      if (r) {
        const uint8_t tag = r.data()[0];
        std::this_thread::yield();
        if (r.data()[0] != tag)
          reused_while_held++;
        consumed++;
      }
    }
  });
  for (int i = 0; i < 20000; i++) {
    FrameRef r = pool.acquire();
    if (!r) {
      dropped++;
      continue;
    }
    r.data()[0] = static_cast<uint8_t>(i);
    handed++;
    std::lock_guard<std::mutex> l(qm);
    queue.push_back(std::move(r));
  }
  done = true;
  consumer.join();
  CHECK(handed == consumed);
  CHECK(reused_while_held == 0);
  CHECK(pool.free_count() == 3);
  std::printf("  threads: %d frames handed, %d dropped (pool full)\n", handed.load(), dropped.load());
}

static void test_large_buffer() {
  LargeBuffer<float> b;
  CHECK(b.size() == 0 && b.data() == nullptr);
  CHECK(b.ensure(1000));
  CHECK(b.size() == 1000 && b.data() != nullptr);
  bool zero = true;
  for (size_t i = 0; i < b.size(); i++)
    zero = zero && b[i] == 0.0f;
  CHECK(zero);
  b[999] = 3.5f;
  float *p = b.data();
  CHECK(b.ensure(1000));  // same size: kept, contents unchanged
  CHECK(b.data() == p && b[999] == 3.5f);
  CHECK(b.ensure(10));  // new size: reallocated and zeroed
  CHECK(b.size() == 10 && b[9] == 0.0f);
  CHECK(b.ensure(0) && b.size() == 0);
  b.reset();
  CHECK(b.data() == nullptr);
}

int main() {
  test_large_buffer();
  test_pool_basics();
  test_frames_hold_buffers();
  test_failed_allocation();
  test_threads();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
