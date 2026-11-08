// Unit tests for the radiometric processor and the CPU ISP.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "isp_core.h"
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

int main() {
  test_radiometric();
  test_isp();
  test_large_frames();
  bench();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
