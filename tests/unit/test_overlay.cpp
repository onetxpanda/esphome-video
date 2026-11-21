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
    return s.empty() ? overlay::MarkHold::Shown{-1, -1, -1.0f, K::OTHER, 0} : s[0];
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

// Marks in colour: the outline black, the reticle and label in the mark's
// colour, in every format.
static void test_colors() {
  video::FramePool pool(1, 200 * 150 * 3);
  const overlay::Color c{0, 255, 255};
  overlay::Style s = overlay::style_for_height(150);
  struct F {
    video::ImageFormat f;
    std::vector<uint8_t> want;
  } formats[] = {
      {video::ImageFormat::RGB565_LE, {0xFF, 0x07}},  // 0x07FF
      {video::ImageFormat::RGB565_BE, {0x07, 0xFF}},
      {video::ImageFormat::RGB888, {0, 255, 255}},
      {video::ImageFormat::GRAY8, {static_cast<uint8_t>((150 * 255 + 29 * 255 + 128) >> 8)}},
  };
  for (const auto &fm : formats) {
    auto img = make(pool, 200, 150, fm.f, 0x80);
    overlay::Mark m{100, 75, "1"};
    m.color = c;
    overlay::draw(img, {m}, s);
    const int bpp = video::bytes_per_pixel(fm.f);
    // A point on the right tick, on its centre line.
    const uint8_t *p = img.buffer.data() + 75 * img.stride + (100 + s.gap + s.tick / 2) * bpp;
    CHECK(std::memcmp(p, fm.want.data(), bpp) == 0);
    // Just outside the tick's line: the black outline.
    const uint8_t *o = img.buffer.data() + (75 + s.line_width) * img.stride + (100 + s.gap + s.tick / 2) * bpp;
    bool black = true;
    for (int i = 0; i < bpp; i++)
      black = black && o[i] == 0;
    CHECK(black);
  }
  // A mark without a colour is white, as before.
  auto img = make(pool, 200, 150, video::ImageFormat::RGB888, 0x80);
  overlay::draw(img, {overlay::Mark{100, 75, "1"}}, s);
  const uint8_t *p = img.buffer.data() + 75 * img.stride + (100 + s.gap + s.tick / 2) * 3;
  CHECK(p[0] == 255 && p[1] == 255 && p[2] == 255);
  // The spot colours differ from each other and from white.
  for (int i = 0; i < overlay::SPOT_COLOR_COUNT; i++) {
    const overlay::Color &a = overlay::SPOT_COLORS[i];
    CHECK(!(a.r == 255 && a.g == 255 && a.b == 255));
    for (int j = i + 1; j < overlay::SPOT_COLOR_COUNT; j++) {
      const overlay::Color &b = overlay::SPOT_COLORS[j];
      CHECK(a.r != b.r || a.g != b.g || a.b != b.b);
    }
  }
}

// Spots are held each on its own; a spot the user moves moves at once.
static void test_spot_hold() {
  using K = video::AnnotationKind;
  const uint32_t SPOTS = 1u << static_cast<uint8_t>(K::SPOT);
  auto frame = [](std::initializer_list<video::Annotation> l) {
    video::Annotations a;
    for (const auto &x : l)
      a.add(x);
    return a;
  };
  overlay::MarkHold h;
  auto r = h.update(frame({{100, 100, 20.0f, K::SPOT, 0}, {300, 200, 30.0f, K::SPOT, 1}}), SPOTS, 0, 1000);
  CHECK(r.size() == 2 && r[0].value == 20.0f && r[1].value == 30.0f);
  r = h.update(frame({{100, 100, 21.0f, K::SPOT, 0}, {300, 200, 31.0f, K::SPOT, 1}}), SPOTS, 400, 1000);
  CHECK(r.size() == 2 && r[0].value == 20.0f && r[1].value == 30.0f);  // both held
  r = h.update(frame({{100, 100, 22.0f, K::SPOT, 0}, {320, 200, 32.0f, K::SPOT, 1}}), SPOTS, 500, 1000);
  CHECK(r[0].value == 20.0f && r[1].x == 320 && r[1].value == 32.0f);  // spot 2 moved: at once
  r = h.update(frame({{100, 100, 23.0f, K::SPOT, 0}, {320, 200, 33.0f, K::SPOT, 1}}), SPOTS, 1000, 1000);
  CHECK(r[0].value == 23.0f && r[1].value == 32.0f);  // spot 1's hold time is up, spot 2's is not
  // Spots are drawn only when the mask asks for them; other kinds alongside.
  r = h.update(frame({{5, 5, 1.0f, K::MAX}, {100, 100, 24.0f, K::SPOT, 0}}), 1u << 1, 1100, 1000);
  CHECK(r.size() == 1 && r[0].kind == K::MAX);
  // Annotations hold up to 12: min, max, centre and 8 spots.
  video::Annotations a;
  for (int i = 0; i < 3; i++)
    CHECK(a.add({0, 0, 0, K::MIN}));
  for (uint8_t i = 0; i < 8; i++)
    CHECK(a.add({0, 0, 0, K::SPOT, i}));
  CHECK(a.count == 11);
  // A spot index beyond the slots shows as it comes.
  r = h.update(frame({{7, 7, 9.0f, K::SPOT, 8}}), SPOTS, 1200, 1000);
  CHECK(r.size() == 1 && r[0].value == 9.0f && r[0].x == 7);
}

int main() {
  test_mark_hold();
  test_spot_hold();
  test_colors();
  test_formats();
  test_bounds_sweep();
  test_collisions();
  bench();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
