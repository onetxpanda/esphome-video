// Unit tests for the radiometric processor and the CPU ISP.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "isp_core.h"
#include "palette.h"
#include "radiometric_core.h"

using namespace esphome;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    g_checks++;                                                         \
    if (!(cond)) {                                                      \
      g_failures++;                                                     \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
    }                                                                   \
  } while (0)
#define NEAR(a, b, e) (std::fabs((a) - (b)) <= (e))

static const int W = 32, H = 24, N = W * H;

static void test_radiometric() {
  radiometric::Processor p;
  radiometric::Options o;
  o.outlier_level = 0;
  p.set_options(o);
  std::vector<float> t(N, 25.0f), out(N);
  t[5 * W + 5] = NAN;
  t[3 * W + 2] = 50.0f;
  t[20 * W + 30] = 5.0f;
  p.process(t.data(), out.data(), W, H);
  const auto &s = p.stats();
  CHECK(NEAR(out[5 * W + 5], 25.0f, 1e-4f));  // repaired
  CHECK(NEAR(s.max, 50.0f, 1e-4f) && s.max_x == 2 && s.max_y == 3);
  CHECK(NEAR(s.min, 5.0f, 1e-4f) && s.min_x == 30 && s.min_y == 20);  // not clamped to any colour range
  CHECK(NEAR(s.median, 25.0f, 1e-4f) && NEAR(s.center, 25.0f, 1e-4f));

  o.outlier_level = 10.0f;
  p.set_options(o);
  std::vector<float> spike(N, 25.0f);
  spike[10 * W + 10] = 60.0f;
  p.process(spike.data(), out.data(), W, H);
  CHECK(NEAR(out[10 * W + 10], 25.0f, 1e-4f));

  o.outlier_level = 0;
  o.temporal_frames = 2;
  p.set_options(o);
  std::vector<float> a(N, 20.0f), b(N, 30.0f);
  p.process(a.data(), out.data(), W, H);
  CHECK(NEAR(out[100], 20.0f, 1e-4f));
  p.process(b.data(), out.data(), W, H);
  CHECK(NEAR(out[100], 25.0f, 1e-4f));
  p.process(b.data(), b.data(), W, H);  // in place
  CHECK(NEAR(b[100], 30.0f, 1e-4f));

  o.temporal_frames = 1;
  o.median_filter = true;
  p.set_options(o);
  std::vector<float> m(N, 22.0f);
  m[12 * W + 16] = 80.0f;
  m[0] = 80.0f;
  p.process(m.data(), out.data(), W, H);
  CHECK(NEAR(out[12 * W + 16], 22.0f, 1e-4f) && NEAR(out[0], 22.0f, 1e-4f));
}

static std::array<isp::Rgb, 256> gray_palette() {
  std::array<isp::Rgb, 256> p{};
  for (int i = 0; i < 256; i++)
    p[i] = isp::Rgb{static_cast<uint8_t>(i), static_cast<uint8_t>(i), static_cast<uint8_t>(i)};
  return p;
}

static void test_isp() {
  isp::CpuIsp c;
  c.set_palette(gray_palette());
  isp::Options o;
  o.range_min = 0;
  o.range_max = 255;
  o.scale = 20;
  o.format = video::ImageFormat::GRAY8;
  std::vector<float> t(N, 0.0f);
  t[0] = 255.0f;
  c.tone_map(t.data(), W, H, o);
  const int ow = W * 20, oh = H * 20;
  std::vector<uint8_t> g(ow * oh, 7);
  c.render(o, g.data(), ow);
  CHECK(g[0] == 255 && g[19] == 255 && g[20] == 0 && g[19 * ow + 19] == 255 && g[20 * ow] == 0);

  o.upscale = isp::Upscale::BILINEAR;
  c.render(o, g.data(), ow);
  CHECK(g[9] >= 250);  // pixel centre keeps its value
  bool mono = true;
  for (int x = 10; x < 30; x++)
    if (g[x + 1] > g[x])
      mono = false;
  CHECK(mono && g[29] < 15);

  // RGB565 LE / BE byte order for white
  o.upscale = isp::Upscale::NEAREST;
  o.format = video::ImageFormat::RGB565_LE;
  std::vector<uint8_t> rgb(ow * oh * 2);
  c.render(o, rgb.data(), ow * 2);
  CHECK(rgb[0] == 0xFF && rgb[1] == 0xFF && rgb[40] == 0x00);
  std::array<isp::Rgb, 256> red{};
  red[255] = isp::Rgb{255, 0, 0};
  c.set_palette(red);
  c.render(o, rgb.data(), ow * 2);
  CHECK(rgb[0] == 0x00 && rgb[1] == 0xF8);  // 0xF800 little-endian
  o.format = video::ImageFormat::RGB565_BE;
  c.render(o, rgb.data(), ow * 2);
  CHECK(rgb[0] == 0xF8 && rgb[1] == 0x00);

  // Mirroring and map_point agree: the hot pixel (0,0) lands where map_point says.
  c.set_palette(gray_palette());
  o.format = video::ImageFormat::GRAY8;
  o.mirror_x = true;
  c.tone_map(t.data(), W, H, o);
  c.render(o, g.data(), ow);
  const isp::Point pt = isp::map_point(o, W, H, 0, 0);
  CHECK(pt.x == (W - 1) * 20 + 10 && pt.y == 10);
  CHECK(g[pt.y * ow + pt.x] == 255 && g[10 * ow + 10] == 0);

  // Out-of-range and NaN clamp to the palette ends.
  std::vector<float> e(N, 1000.0f);
  e[1] = -1000.0f;
  e[2] = NAN;
  o.mirror_x = false;
  c.tone_map(e.data(), W, H, o);
  CHECK(c.indices()[0] == 255 && c.indices()[1] == 0 && c.indices()[2] == 0);
}

// Relative speed of the new CPU renderer against the float renderer it replaces.
static void bench() {
  isp::CpuIsp c;
  c.set_palette(gray_palette());
  isp::Options o;
  o.scale = 20;
  std::vector<float> t(N);
  for (int i = 0; i < N; i++)
    t[i] = 15.0f + (i % 97) * 0.25f;
  std::vector<uint8_t> out(640 * 480 * 2);
  auto time = [&](auto fn) {
    auto a = std::chrono::steady_clock::now();
    for (int i = 0; i < 20; i++)
      fn();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count() / 20;
  };
  for (auto up : {isp::Upscale::NEAREST, isp::Upscale::BILINEAR}) {
    o.upscale = up;
    const double ms = time([&] {
      c.tone_map(t.data(), W, H, o);
      c.render(o, out.data(), 640 * 2);
    });
    std::printf("  host: %s 640x480 RGB565 = %.3f ms\n", up == isp::Upscale::NEAREST ? "nearest " : "bilinear", ms);
  }
}

// The HTI-301's 384x288 frames: every stage's working memory at that size,
// temporal history and median included, with the results still correct.
static void test_large_frames() {
  const int w = 384, h = 288, n = w * h;
  radiometric::Processor p;
  radiometric::Options o;
  o.temporal_frames = 4;
  o.median_filter = true;
  p.set_options(o);
  std::vector<float> t(n, 22.0f), out(n);
  t[100 * w + 200] = 60.0f;
  t[10 * w + 10] = NAN;
  bool ok = true;
  for (int i = 0; i < 6; i++)
    ok = p.process(t.data(), out.data(), w, h) && ok;
  CHECK(ok);
  CHECK(p.stats().valid && NEAR(p.stats().median, 22.0f, 1e-4f) && NEAR(out[10 * w + 10], 22.0f, 1e-4f));
  CHECK(NEAR(out[100 * w + 200], 22.0f, 1e-4f));  // a lone hot pixel is removed by the median
  isp::CpuIsp c;
  c.set_palette(gray_palette());
  isp::Options io;
  io.range_min = 0;
  io.range_max = 255;
  io.scale = 1;
  io.format = video::ImageFormat::RGB565_LE;
  CHECK(c.tone_map(out.data(), w, h, io));
  std::vector<uint8_t> img(static_cast<size_t>(n) * 2, 0);
  c.render(io, img.data(), w * 2);
  CHECK(c.width() == w && c.height() == h);
}

// Every palette is defined over the whole range, in the shared order.
static void test_palettes() {
  CHECK(isp::PALETTE_COUNT == video::PALETTE_COUNT);
  auto lum = [](const isp::Rgb &c) { return c.r * 299 + c.g * 587 + c.b * 114; };
  for (uint8_t i = 0; i < isp::PALETTE_COUNT; i++) {
    const auto p = isp::make_palette(static_cast<isp::Palette>(i));
    // The hot end is brighter than the cold end, except for black hot.
    if (i == static_cast<uint8_t>(isp::Palette::BLACK_HOT)) {
      CHECK(lum(p[0]) > lum(p[255]));
    } else {
      CHECK(lum(p[255]) > lum(p[0]));
    }
  }
  const auto w = isp::make_palette(isp::Palette::WHITE_HOT);
  CHECK(w[0].r == 0 && w[255].r == 255 && w[128].g == 128 && w[128].b == 128);
  const auto b = isp::make_palette(isp::Palette::BLACK_HOT);
  CHECK(b[0].r == 255 && b[255].r == 0);
  // The irradiance app's ironbow: its stops at 0, 51 (stop 2) and 255.
  const auto iron = isp::make_palette(isp::Palette::IRONBOW);
  CHECK(iron[0].r == 0 && iron[0].g == 0 && iron[0].b == 10);
  CHECK(iron[51].r == 60 && iron[51].g == 0 && iron[51].b == 153);
  CHECK(iron[255].r == 255 && iron[255].g == 255 && iron[255].b == 246);
  const auto red = isp::make_palette(isp::Palette::RED_HOT);
  CHECK(red[100].r == red[100].g && red[255].r == 255 && red[255].g == 0);  // grey, then red at the top
}

// The validity range, the optional median statistic, and the median filter
// working on the repaired frame.
static void test_valid_range_and_median() {
  std::vector<float> t(N, 25.0f), out(N);
  t[6 * W + 6] = 400.0f;  // a real reading for the HTI-301's high range; nonsense for the MLX90640
  radiometric::Processor p;
  radiometric::Options o;
  o.outlier_level = 0;
  p.set_options(o);
  CHECK(p.process(t.data(), out.data(), W, H));
  CHECK(NEAR(out[6 * W + 6], 25.0f, 1e-4f));  // default range: repaired
  CHECK(!std::isnan(p.stats().median));
  o.valid_max = 600.0f;
  o.median_stat = false;
  p.set_options(o);
  CHECK(p.process(t.data(), out.data(), W, H));
  CHECK(NEAR(out[6 * W + 6], 400.0f, 1e-4f) && NEAR(p.stats().max, 400.0f, 1e-4f));  // kept
  CHECK(std::isnan(p.stats().median));  // not computed
  // Median filter after repair: an invalid pixel next to a spike is repaired
  // first, then the 3x3 median removes the spike.
  std::vector<float> m(N, 20.0f);
  m[10 * W + 10] = NAN;
  m[10 * W + 11] = 80.0f;
  radiometric::Options mo;
  mo.outlier_level = 0;
  mo.median_filter = true;
  p.set_options(mo);
  CHECK(p.process(m.data(), out.data(), W, H));
  CHECK(NEAR(out[10 * W + 10], 20.0f, 1e-4f) && NEAR(out[10 * W + 11], 20.0f, 1e-4f));
  CHECK(!std::isnan(out[10 * W + 10]));
}

// With no filter active, analyze() gives exactly process()'s statistics in
// one read-only pass, and refuses a frame that needs repair.
static void test_analyze_matches_process() {
  const int w = 384, h = 288, n = w * h;
  std::vector<float> t(n), out(n);
  uint32_t seed = 12345;
  for (int i = 0; i < n; i++) {
    seed = seed * 1103515245u + 12345u;
    t[i] = 15.0f + static_cast<float>((seed >> 8) % 3000) / 100.0f;  // 15..45 C
  }
  radiometric::Options o;
  o.outlier_level = 0;
  for (bool median : {false, true}) {
    o.median_stat = median;
    radiometric::Processor a, b;
    a.set_options(o);
    b.set_options(o);
    CHECK(!a.filters_active());
    CHECK(a.analyze(t.data(), w, h));
    CHECK(b.process(t.data(), out.data(), w, h));
    const auto &sa = a.stats(), &sb = b.stats();
    CHECK(sa.valid && sa.min == sb.min && sa.max == sb.max && sa.mean == sb.mean && sa.center == sb.center);
    CHECK(sa.min_x == sb.min_x && sa.min_y == sb.min_y && sa.max_x == sb.max_x && sa.max_y == sb.max_y);
    CHECK(median ? sa.median == sb.median : (std::isnan(sa.median) && std::isnan(sb.median)));
    CHECK(out == t);  // no filter: process() changed nothing either
  }
  t[1000] = NAN;
  radiometric::Processor p;
  p.set_options(o);
  CHECK(!p.analyze(t.data(), w, h));
  o.outlier_level = 5.0f;
  p.set_options(o);
  CHECK(p.filters_active());
}

// The one-pass unscaled path writes exactly what tone_map() + render() write
// at scale 1: every format, every mirroring, clamping and NaN, odd sizes and
// a padded stride (padding left alone).
static void test_map_direct() {
  const int w = 37, h = 23, n = w * h;
  std::vector<float> t(n);
  uint32_t seed = 777;
  for (int i = 0; i < n; i++) {
    seed = seed * 1103515245u + 12345u;
    t[i] = -10.0f + static_cast<float>((seed >> 8) % 6000) / 100.0f;  // -10..50 C
  }
  t[5] = NAN;
  t[6] = 1e6f;
  t[7] = -1e6f;
  isp::CpuIsp c;
  c.set_palette(isp::make_palette(isp::Palette::IRONBOW));
  isp::Options o;
  o.range_min = 5.0f;
  o.range_max = 35.0f;
  o.scale = 1;
  const video::ImageFormat formats[] = {video::ImageFormat::RGB565_LE, video::ImageFormat::RGB565_BE,
                                        video::ImageFormat::RGB888, video::ImageFormat::GRAY8};
  for (const auto f : formats) {
    for (int m = 0; m < 4; m++) {
      o.format = f;
      o.mirror_x = (m & 1) != 0;
      o.mirror_y = (m & 2) != 0;
      const uint32_t stride = w * video::bytes_per_pixel(f) + 5;
      std::vector<uint8_t> a(stride * h, 0xA5), b(stride * h, 0xA5);
      c.tone_map(t.data(), w, h, o);
      c.render(o, a.data(), stride);
      c.map_direct(t.data(), w, h, o, b.data(), stride);
      CHECK(a == b);
    }
  }
}

// The palette index for every value, against its definition: (t - lo) * k
// rounded half up, 0 at or below the range and for NaN, 255 at or above it.
static void test_index_rule() {
  std::vector<float> e;
  for (int i = -4000; i <= 60000; i++)
    e.push_back(static_cast<float>(i) / 100.0f);
  const float edges[] = {0.0f, -0.0f, 0.49999f, 0.5f, 0.50001f, 254.49f, 254.5f, 254.51f, 255.0f, 255.01f,
                         INFINITY, -INFINITY, NAN, 1e30f, -1e30f, 15.0f, 40.0f};
  for (float v : edges)
    e.push_back(v);
  const float ranges[][2] = {{0.0f, 255.0f}, {15.0f, 40.0f}, {-20.0f, 450.0f}};
  for (const auto &r : ranges) {
    isp::Options o;
    o.range_min = r[0];
    o.range_max = r[1];
    o.format = video::ImageFormat::GRAY8;
    const float k = 255.0f / std::max(o.range_max - o.range_min, 1e-3f);
    isp::CpuIsp c;
    c.set_palette(isp::make_palette(isp::Palette::WHITE_HOT));
    CHECK(c.tone_map(e.data(), static_cast<uint16_t>(e.size() / 64), 64, o));
    bool same = true;
    const size_t n = (e.size() / 64) * 64;
    for (size_t i = 0; i < n; i++) {
      const float v = (e[i] - o.range_min) * k;
      const uint8_t want = !(v > 0.0f) ? 0 : v >= 255.0f ? 255 : static_cast<uint8_t>(v + 0.5f);
      same = same && c.indices()[i] == want;
    }
    CHECK(same);
  }
}

// adopt() takes a source's own statistics in place of analyze(): the same
// results, and it declines whenever analyze() would have to look at pixels.
static void test_adopt() {
  const int w = 384, h = 288, n = w * h;
  std::vector<float> t(n);
  uint32_t seed = 4242;
  for (int i = 0; i < n; i++) {
    seed = seed * 1103515245u + 12345u;
    t[i] = 15.0f + static_cast<float>((seed >> 8) % 3000) / 100.0f;
  }
  // The statistics as a source works them out (the HTI-301 conversion).
  video::FrameStats fs;
  float mn = INFINITY, mx = -INFINITY;
  int mi = 0, ma = 0;
  double sum = 0;
  for (int y = 0; y < h; y++) {
    float rs = 0.0f;
    for (int x = 0; x < w; x++) {
      const float v = t[y * w + x];
      rs += v;
      if (v < mn) {
        mn = v;
        mi = y * w + x;
      }
      if (v > mx) {
        mx = v;
        ma = y * w + x;
      }
    }
    sum += rs;
  }
  fs.valid = true;
  fs.min = mn;
  fs.max = mx;
  fs.mean = static_cast<float>(sum / n);
  fs.min_x = mi % w;
  fs.min_y = mi / w;
  fs.max_x = ma % w;
  fs.max_y = ma / w;

  radiometric::Options o;
  o.outlier_level = 0;
  o.median_stat = false;  // as with no median sensor
  radiometric::Processor a, b;
  a.set_options(o);
  b.set_options(o);
  CHECK(a.adopt(fs, t.data(), w, h));
  CHECK(b.analyze(t.data(), w, h));
  const auto &sa = a.stats(), &sb = b.stats();
  CHECK(sa.valid && sa.min == sb.min && sa.max == sb.max && sa.mean == sb.mean && sa.center == sb.center);
  CHECK(sa.min_x == sb.min_x && sa.min_y == sb.min_y && sa.max_x == sb.max_x && sa.max_y == sb.max_y);
  CHECK(std::isnan(sa.median));

  // Declined: no statistics, a median wanted, a pixel outside the valid range.
  radiometric::Processor p;
  p.set_options(o);
  CHECK(!p.adopt(video::FrameStats(), t.data(), w, h));
  CHECK(!p.stats().valid);
  o.median_stat = true;
  p.set_options(o);
  CHECK(!p.adopt(fs, t.data(), w, h));
  o.median_stat = false;
  o.valid_max = 40.0f;  // the frame reaches 45 C
  p.set_options(o);
  CHECK(!p.adopt(fs, t.data(), w, h));
  CHECK(!p.analyze(t.data(), w, h));  // which analyze() agrees needs repair
}

int main() {
  test_palettes();
  test_map_direct();
  test_index_rule();
  test_adopt();
  test_analyze_matches_process();
  test_valid_range_and_median();
  test_radiometric();
  test_isp();
  test_large_frames();
  bench();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
