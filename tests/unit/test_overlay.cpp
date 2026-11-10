// Unit tests for overlay drawing: bounds, colours per format, label placement.
#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#include "overlay_core.h"

using namespace esphome;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    g_checks++;                                                   \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

static video::ImageFrame make(video::FramePool &pool, int w, int h, video::ImageFormat f, uint8_t fill) {
  video::ImageFrame img;
  img.buffer = pool.acquire();
  img.width = w;
  img.height = h;
  img.format = f;
  img.stride = w * video::bytes_per_pixel(f);
  std::memset(img.buffer.data(), fill, pool.buffer_size());
  return img;
}

static bool overlaps(const std::vector<overlay::Rect> &r) {
  for (size_t i = 0; i < r.size(); i++)
    for (size_t j = i + 1; j < r.size(); j++)
      if (r[i].intersects(r[j]))
        return true;
  return false;
}

static void test_formats() {
  video::FramePool pool(1, 64 * 64 * 3, 16);
  overlay::Style s;  // 640x480 proportions: the outline stays clear of the centre
  s.tick = 8;
  const video::ImageFormat fmts[] = {video::ImageFormat::RGB565_LE, video::ImageFormat::RGB565_BE,
                                     video::ImageFormat::RGB888, video::ImageFormat::GRAY8};
  for (auto f : fmts) {
    auto img = make(pool, 64, 64, f, 0x55);
    overlay::draw(img, {{32, 32, "25.0C"}}, s);
    const size_t bpp = video::bytes_per_pixel(f);
    const uint8_t *c = img.buffer.data() + 32 * img.stride + 32 * bpp;
    CHECK(c[0] == 0x55);  // reticle centre left untouched
    // A tick pixel is white in every byte, and its outline neighbour black.
    const uint8_t *t = img.buffer.data() + 32 * img.stride + (32 + s.gap + 1) * bpp;
    bool white = true;
    for (size_t b = 0; b < bpp; b++)
      white = white && t[b] == 0xFF;
    CHECK(white);
    // Just above the horizontal tick: outline (black).
    const uint8_t *o = img.buffer.data() + (32 - s.line_width / 2 - 1) * img.stride + (32 + s.gap + 1) * bpp;
    CHECK(o[0] == 0x00);
    // Nothing outside the image is written: pool buffer tail beyond length() is intact.
    const uint8_t *end = img.buffer.data() + img.length();
    if (img.length() < pool.buffer_size())
      CHECK(end[0] == 0x55);
  }
}

// Marks anywhere, including off the image and at its corners, never write out
// of bounds (ASan checks this) and labels stay inside the image.
static void test_bounds_sweep() {
  const int W = 96, H = 72;
  video::FramePool pool(1, W * H * 2, 16);
  overlay::Style s = overlay::style_for_height(H);
  bool inside = true;
  for (int y = -40; y < H + 40; y += 7)
    for (int x = -40; x < W + 40; x += 7) {
      auto img = make(pool, W, H, video::ImageFormat::RGB565_LE, 0);
      auto r = overlay::draw(img, {{x, y, "-12.3C"}, {W - x, H - y, "99.9C"}}, s);
      for (const auto &q : r)
        if (q.x0 < 0 || q.y0 < 0 || q.x1 > W || q.y1 > H)
          inside = false;
    }
  CHECK(inside);
}

// Three marks bunched at the centre (hot spot = centre spot) at 640x480: no
// two labels overlap.
static void test_collisions() {
  const int W = 640, H = 480;
  video::FramePool pool(1, W * H * 2, 16);
  overlay::Style s = overlay::style_for_height(H);
  auto img = make(pool, W, H, video::ImageFormat::RGB565_LE, 0);
  auto r = overlay::draw(img, {{320, 240, "31.2C"}, {330, 250, "33.0C"}, {320, 240, "30.1C"}}, s);
  CHECK(r.size() == 3 && !overlaps(r));
  // Same, pushed into a corner where only some positions fit.
  auto r2 = overlay::draw(img, {{5, 5, "31.2C"}, {12, 8, "33.0C"}, {5, 5, "30.1C"}}, s);
  CHECK(r2.size() == 3 && !overlaps(r2));
  // A mark in a corner: its label is moved inside the image, and must not
  // land on its own reticle (seen on hardware with the hot spot bottom-left).
  for (auto c : {std::make_pair(0, H - 1), std::make_pair(W - 1, H - 1), std::make_pair(0, 0),
                 std::make_pair(W - 1, 0)}) {
    auto rc = overlay::draw(img, {{c.first, c.second, "93.4C"}}, s);
    const int reach = s.gap + s.tick;
    const overlay::Rect own{c.first - reach, c.second - reach, c.first + reach + 1, c.second + reach + 1};
    CHECK(rc.size() == 1 && !rc[0].intersects(own));
  }
  // Spread out marks get the preferred position (below).
  auto r3 = overlay::draw(img, {{100, 100, "20.0C"}}, s);
  CHECK(r3.size() == 1 && r3[0].y0 > 100);
}

static void bench() {
  const int W = 640, H = 480;
  video::FramePool pool(1, W * H * 2, 16);
  overlay::Style s = overlay::style_for_height(H);
  auto img = make(pool, W, H, video::ImageFormat::RGB565_LE, 0);
  const std::vector<overlay::Mark> marks = {{320, 240, "31.2C"}, {100, 80, "36.4C"}, {500, 400, "18.9C"}};
  auto a = std::chrono::steady_clock::now();
  for (int i = 0; i < 200; i++)
    overlay::draw(img, marks, s);
  const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count() / 200;
  std::printf("  host: overlay 3 marks on 640x480 RGB565 = %.3f ms\n", ms);
}

// A mark's position and value are refreshed together once per hold time.
static void test_mark_hold() {
  using K = video::AnnotationKind;
  const uint32_t MIN_MAX = (1u << 0) | (1u << 1);
  auto frame = [](std::initializer_list<video::Annotation> l) {
    video::Annotations a;
    for (const auto &x : l)
      a.add(x);
    return a;
  };
  overlay::MarkHold h;
  auto one = [&](const video::Annotations &a, uint32_t now, uint32_t hold = 1000) {
    const auto &s = h.update(a, MIN_MAX, now, hold);
    return s.empty() ? overlay::MarkHold::Shown{-1, -1, -1.0f, K::OTHER} : s[0];
  };
  auto s = one(frame({{10.4f, 10.0f, 30.0f, K::MAX}}), 0);
  CHECK(s.x == 10 && s.y == 10 && s.value == 30.0f);  // first: as it comes
  s = one(frame({{10.0f, 10.9f, 31.0f, K::MAX}}), 40);
  CHECK(s.x == 10 && s.y == 10 && s.value == 30.0f);  // same pixel: value held
  s = one(frame({{14.0f, 12.0f, 31.5f, K::MAX}}), 500);
  CHECK(s.x == 10 && s.y == 10 && s.value == 30.0f);  // moved: position held too
  s = one(frame({{10.0f, 10.0f, 31.5f, K::MAX}}), 999);
  CHECK(s.value == 30.0f);
  s = one(frame({{11.0f, 10.0f, 33.0f, K::MAX}}), 1000);
  CHECK(s.x == 11 && s.y == 10 && s.value == 33.0f);  // a hold time later: position and value
  s = one(frame({{12.0f, 10.0f, 34.0f, K::MAX}}), 1040);
  CHECK(s.x == 11 && s.value == 33.0f);  // held again

  // Kinds are held separately; a kind outside the mask is not shown.
  const auto &both = h.update(frame({{11.0f, 10.0f, 35.0f, K::MAX}, {2.0f, 3.0f, 5.0f, K::MIN},
                                     {20.0f, 20.0f, 25.0f, K::CENTER}}),
                              MIN_MAX, 1120, 1000);
  CHECK(both.size() == 2 && both[0].kind == K::MAX && both[0].value == 33.0f && both[1].kind == K::MIN &&
        both[1].value == 5.0f);

  // A kind missing from a frame starts afresh when it returns.
  one(frame({{2.0f, 3.0f, 6.0f, K::MIN}}), 1160);
  s = one(frame({{11.0f, 10.0f, 36.0f, K::MAX}}), 1200);
  CHECK(s.value == 36.0f);

  // reset() (a new image size): the next frame shows as it comes.
  h.reset();
  s = one(frame({{11.0f, 10.0f, 37.0f, K::MAX}}), 1240);
  CHECK(s.value == 37.0f);

  // Hold 0: every frame's value.
  s = one(frame({{11.0f, 10.0f, 38.0f, K::MAX}}), 1280, 0);
  CHECK(s.value == 38.0f);

  // The clock wrapping does not freeze a value.
  overlay::MarkHold w;
  w.update(frame({{1.0f, 1.0f, 1.0f, K::MAX}}), MIN_MAX, 0xFFFFFF00u, 1000);
  CHECK(w.update(frame({{1.0f, 1.0f, 2.0f, K::MAX}}), MIN_MAX, 0x00000100u, 1000)[0].value == 1.0f);
  CHECK(w.update(frame({{1.0f, 1.0f, 3.0f, K::MAX}}), MIN_MAX, 0x00000400u, 1000)[0].value == 3.0f);

  // A second annotation of the same kind shows as it comes.
  const auto &dup = w.update(frame({{1.0f, 1.0f, 4.0f, K::MAX}, {5.0f, 5.0f, 9.0f, K::MAX}}), MIN_MAX,
                             0x00000440u, 1000);
  CHECK(dup.size() == 2 && dup[0].value == 3.0f && dup[1].value == 9.0f && dup[1].x == 5);
}

int main() {
  test_mark_hold();
  test_formats();
  test_bounds_sweep();
  test_collisions();
  bench();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
