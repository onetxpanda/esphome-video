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
  // A row's last pixel is not compared with the next row's first: a cold left
  // edge and a hot right edge stay as they are.
  std::vector<float> edges(N, 25.0f);
  for (int r = 0; r < H; r++) {
    edges[r * W] = 5.0f;
    edges[r * W + W - 1] = 45.0f;
  }
  p.set_options(o);
  p.process(edges.data(), out.data(), W, H);
  CHECK(NEAR(out[5 * W + W - 1], 45.0f, 1e-3f) && NEAR(out[6 * W], 5.0f, 1e-3f));

  // A change far above the motion threshold everywhere is movement: the
  // temporal filter passes it through at once instead of averaging.
  o.outlier_level = 0;
  o.temporal_frames = 4;
  p.set_options(o);
  std::vector<float> a(N, 20.0f), b(N, 30.0f);
  p.process(a.data(), out.data(), W, H);
  CHECK(NEAR(out[100], 20.0f, 1e-4f));
  p.process(b.data(), out.data(), W, H);
  CHECK(NEAR(out[100], 30.0f, 1e-4f));
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

static void test_sensor_frame_due() {
  using radiometric::sensor_frame_due;
  CHECK(sensor_frame_due(5, 0, 1000));  // no frame yet
  CHECK(!sensor_frame_due(1999, 1000, 1000));
  CHECK(sensor_frame_due(2000, 1000, 1000));
  // Across the millis() wrap: 1000 ms after 0xFFFFFF00 is 0x2E8.
  CHECK(!sensor_frame_due(0x2E7u, 0xFFFFFF00u, 1000));
  CHECK(sensor_frame_due(0x2E8u, 0xFFFFFF00u, 1000));
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

// Deterministic noise in [-amp, amp].
static float noise(uint32_t &seed, float amp) {
  seed = seed * 1664525u + 1013904223u;
  return amp * (static_cast<float>(seed >> 8) / 8388608.0f - 1.0f);
}

static float stddev(const std::vector<float> &v, size_t from, size_t to) {
  double m = 0, q = 0;
  for (size_t i = from; i < to; i++)
    m += v[i];
  m /= (to - from);
  for (size_t i = from; i < to; i++)
    q += (v[i] - m) * (v[i] - m);
  return static_cast<float>(std::sqrt(q / (to - from)));
}

// The motion-adaptive temporal filter: still noise is averaged away, a
// moving hot spot is followed at once, and frames = 1 passes frames through.
static void test_temporal_filter() {
  radiometric::Processor p;
  radiometric::Options o;
  o.outlier_level = 0;
  o.temporal_frames = 8;
  o.motion_threshold = 0.5f;
  p.set_options(o);
  uint32_t seed = 1;
  std::vector<float> f(N), out(N);
  // Still scene with +-0.3 noise: the output's spread over time at one
  // pixel is far below the input's.
  std::vector<float> in_hist, out_hist;
  for (int k = 0; k < 60; k++) {
    for (int i = 0; i < N; i++)
      f[i] = 25.0f + noise(seed, 0.3f);
    p.process(f.data(), out.data(), W, H);
    if (k >= 20) {
      in_hist.push_back(f[12 * W + 16]);
      out_hist.push_back(out[12 * W + 16]);
    }
  }
  const float sin = stddev(in_hist, 0, in_hist.size()), sout = stddev(out_hist, 0, out_hist.size());
  CHECK(sout < 0.5f * sin);

  // A 3x3 hot spot (40 C) appears: it shows at full strength in the same
  // frame; then it moves, and the old place is cool again at once.
  for (int i = 0; i < N; i++)
    f[i] = 25.0f + noise(seed, 0.3f);
  for (int r = 10; r < 13; r++)
    for (int c = 10; c < 13; c++)
      f[r * W + c] = 40.0f;
  p.process(f.data(), out.data(), W, H);
  CHECK(out[11 * W + 11] > 39.0f);
  for (int r = 10; r < 13; r++)
    for (int c = 10; c < 13; c++)
      f[r * W + c] = 25.0f;
  for (int r = 10; r < 13; r++)
    for (int c = 20; c < 23; c++)
      f[r * W + c] = 40.0f;
  p.process(f.data(), out.data(), W, H);
  CHECK(out[11 * W + 21] > 39.0f);
  CHECK(out[11 * W + 11] < 26.0f);

  o.temporal_frames = 1;
  p.set_options(o);
  for (int i = 0; i < N; i++)
    f[i] = 25.0f + noise(seed, 0.3f);
  p.process(f.data(), out.data(), W, H);
  bool same = true;
  for (int i = 0; i < N; i++)
    if (out[i] != f[i])
      same = false;
  CHECK(same);
}

// Edge-preserving smoothing: noise on both sides of a 10 C edge is reduced,
// and the edge stays where it was, as sharp as it was.
static void test_edge_smoothing() {
  radiometric::Processor p;
  radiometric::Options o;
  o.outlier_level = 0;
  o.edge_smoothing = 1.0f;
  p.set_options(o);
  CHECK(p.filters_active());
  uint32_t seed = 7;
  std::vector<float> f(N), out(N);
  for (int r = 0; r < H; r++)
    for (int c = 0; c < W; c++)
      f[r * W + c] = (c < 16 ? 20.0f : 30.0f) + noise(seed, 0.3f);
  p.process(f.data(), out.data(), W, H);
  std::vector<float> fin, fout;
  for (int r = 2; r < H - 2; r++)
    for (int c = 2; c < 13; c++) {
      fin.push_back(f[r * W + c]);
      fout.push_back(out[r * W + c]);
    }
  CHECK(stddev(fout, 0, fout.size()) < 0.6f * stddev(fin, 0, fin.size()));
  bool edge = true;
  for (int r = 0; r < H; r++)
    if (!(out[r * W + 15] < 20.5f && out[r * W + 16] > 29.5f))
      edge = false;
  CHECK(edge);
}

// Equalization and detail: off gives exactly the linear map; equalization
// is monotonic and spreads a crowded temperature band over more colours;
// detail leaves flat areas alone and overshoots at an edge.
// Equalization follows the scene over time, whatever the frame rate; a curve
// taken at once (a frame drawn again after a change) needs no settling.
static void test_curve_blend() {
  CHECK(NEAR(isp::curve_blend_for(40), 0.234f, 0.01f));  // 25 frames/s: about a quarter per frame
  CHECK(isp::curve_blend_for(4000) > 0.999f);           // a frame every 4 s: the new curve
  CHECK(isp::curve_blend_for(0) == 0.0f);
  CHECK(isp::curve_blend_for(62) > isp::curve_blend_for(40));

  isp::Options o;
  o.range_min = 15;
  o.range_max = 40;
  o.scale = 1;
  o.format = video::ImageFormat::GRAY8;
  o.equalize = 1.0f;
  uint32_t seed = 5;
  std::vector<float> a(N), b(N);
  for (int i = 0; i < N; i++) {
    a[i] = 20.0f + noise(seed, 0.5f);
    b[i] = i % 2 ? 35.0f + noise(seed, 0.5f) : 20.0f + noise(seed, 0.5f);
  }
  isp::CpuIsp slow, fast, settled;
  for (auto *c : {&slow, &fast, &settled})
    c->set_palette(gray_palette());
  // Settled on b: the reference.
  o.curve_blend = 1.0f;
  settled.tone_map(b.data(), W, H, o);
  // Both start on a; then one frame of b, blended a quarter or all of it.
  slow.tone_map(a.data(), W, H, o);
  fast.tone_map(a.data(), W, H, o);
  o.curve_blend = 0.25f;
  slow.tone_map(b.data(), W, H, o);
  o.curve_blend = 1.0f;
  fast.tone_map(b.data(), W, H, o);
  CHECK(std::equal(fast.indices(), fast.indices() + N, settled.indices()));
  CHECK(!std::equal(slow.indices(), slow.indices() + N, settled.indices()));
}

static void test_tone_curve() {
  isp::Options o;
  o.range_min = 15;
  o.range_max = 40;
  o.scale = 1;
  o.format = video::ImageFormat::GRAY8;
  uint32_t seed = 3;
  std::vector<float> t(N);
  for (int i = 0; i < N; i++)
    t[i] = i % 10 == 0 ? 35.0f + noise(seed, 0.5f) : 20.0f + noise(seed, 0.5f);  // 90% near 20 C

  isp::CpuIsp lin, eq;
  lin.set_palette(gray_palette());
  eq.set_palette(gray_palette());
  lin.tone_map(t.data(), W, H, o);
  std::vector<uint8_t> lin_idx(lin.indices(), lin.indices() + N);
  isp::Options e = o;
  e.equalize = 0.0f;
  e.detail = 0.0f;
  CHECK(isp::linear_tone(e));
  eq.tone_map(t.data(), W, H, e);
  CHECK(std::equal(lin_idx.begin(), lin_idx.end(), eq.indices()));

  e.equalize = 1.0f;
  CHECK(!isp::linear_tone(e));
  for (int k = 0; k < 20; k++)  // let the smoothed curve settle
    eq.tone_map(t.data(), W, H, e);
  const uint8_t *q = eq.indices();
  // Monotonic: a warmer pixel never gets a lower index.
  std::vector<int> order(N);
  for (int i = 0; i < N; i++)
    order[i] = i;
  std::sort(order.begin(), order.end(), [&](int a, int b) { return t[a] < t[b]; });
  bool mono = true;
  for (int i = 1; i < N; i++)
    if (q[order[i]] < q[order[i - 1]])
      mono = false;
  CHECK(mono);
  // The 20 C band (index spread over its pixels) gets more colours than linear.
  int lmin = 255, lmax = 0, qmin = 255, qmax = 0;
  for (int i = 0; i < N; i++)
    if (i % 10 != 0) {
      lmin = std::min<int>(lmin, lin_idx[i]);
      lmax = std::max<int>(lmax, lin_idx[i]);
      qmin = std::min<int>(qmin, q[i]);
      qmax = std::max<int>(qmax, q[i]);
    }
  CHECK(qmax - qmin > 2 * (lmax - lmin));

  // Detail: flat stays flat (and linear); a step overshoots on both sides.
  std::vector<float> st(N);
  for (int r = 0; r < H; r++)
    for (int c = 0; c < W; c++)
      st[r * W + c] = c < 16 ? 20.0f : 30.0f;
  isp::CpuIsp d;
  d.set_palette(gray_palette());
  isp::Options dopt = o;
  dopt.detail = 1.0f;
  d.tone_map(st.data(), W, H, dopt);
  lin.tone_map(st.data(), W, H, o);
  const int row = 12 * W;
  CHECK(d.indices()[row + 4] == lin.indices()[row + 4] && d.indices()[row + 28] == lin.indices()[row + 28]);
  CHECK(d.indices()[row + 16] > lin.indices()[row + 16] && d.indices()[row + 15] < lin.indices()[row + 15]);

  // Mirroring applies on the equalized path too.
  std::vector<float> hot(N, 20.0f);
  hot[0] = 39.0f;
  isp::Options m = o;
  m.equalize = 0.5f;
  m.mirror_x = true;
  isp::CpuIsp mi;
  mi.set_palette(gray_palette());
  mi.tone_map(hot.data(), W, H, m);
  CHECK(*std::max_element(mi.indices(), mi.indices() + N) == mi.indices()[W - 1]);
  CHECK(mi.indices()[W - 1] > mi.indices()[0] + 50);
}

// Bicubic: a flat image stays flat; a linear ramp stays linear (matching
// bilinear inside the image); a single hot pixel peaks at its own place.
static void test_bicubic() {
  isp::CpuIsp c;
  c.set_palette(gray_palette());
  isp::Options o;
  o.range_min = 0;
  o.range_max = 255;
  o.scale = 20;
  o.format = video::ImageFormat::GRAY8;
  const int ow = W * 20, oh = H * 20;
  std::vector<float> t(N, 100.0f);
  c.tone_map(t.data(), W, H, o);
  std::vector<uint8_t> a(ow * oh), b(ow * oh);
  o.upscale = isp::Upscale::BICUBIC;
  c.render(o, a.data(), ow);
  CHECK(std::all_of(a.begin(), a.end(), [](uint8_t v) { return v == 100; }));

  for (int r = 0; r < H; r++)
    for (int x = 0; x < W; x++)
      t[r * W + x] = 8.0f * x;
  c.tone_map(t.data(), W, H, o);
  c.render(o, a.data(), ow);
  o.upscale = isp::Upscale::BILINEAR;
  c.render(o, b.data(), ow);
  bool close = true;
  for (int y = 0; y < oh; y++)
    for (int x = 40; x < ow - 40; x++)
      if (std::abs(a[y * ow + x] - b[y * ow + x]) > 1)
        close = false;
  CHECK(close);

  std::fill(t.begin(), t.end(), 0.0f);
  t[12 * W + 16] = 255.0f;
  c.tone_map(t.data(), W, H, o);
  o.upscale = isp::Upscale::BICUBIC;
  c.render(o, a.data(), ow);
  const auto peak = std::max_element(a.begin(), a.end()) - a.begin();
  CHECK(a[peak] > 220 && peak / ow / 20 == 12 && peak % ow / 20 == 16);
  CHECK(a[0] == 0 && a[ow * oh - 1] == 0);
}

// A scene much narrower than the range: neighbouring pixels are a fraction of
// a palette entry apart. The interpolating scalers round after interpolating,
// so the steps between entries lie where the scene's own (here straight,
// slanted) contours are, not on the input's pixel grid.
static void test_scaling_precision() {
  const int s = 8, ow = W * s, oh = H * s;
  isp::Options o;
  o.range_min = 0;
  o.range_max = 255;
  o.scale = s;
  o.format = video::ImageFormat::GRAY8;
  std::vector<float> t(N);
  auto plane = [](float x, float y) { return 100.0f + 0.23f * x + 0.11f * y; };
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      t[y * W + x] = plane(x, y);
  // How many inner output pixels are not the plane's own palette entry there
  // (pixels the plane puts within 0.05 of a rounding boundary do not count).
  auto wrong = [&](const std::vector<uint8_t> &g) {
    int bad = 0, n = 0;
    for (int oy = 2 * s; oy < oh - 2 * s; oy++)
      for (int ox = 2 * s; ox < ow - 2 * s; ox++) {
        const float v = plane((ox + 0.5f) / s - 0.5f, (oy + 0.5f) / s - 0.5f);
        if (std::fabs(v - std::floor(v) - 0.5f) < 0.05f)
          continue;
        n++;
        bad += g[oy * ow + ox] != static_cast<uint8_t>(v + 0.5f);
      }
    return 100.0f * bad / n;
  };
  for (auto up : {isp::Upscale::BILINEAR, isp::Upscale::BICUBIC}) {
    isp::CpuIsp c;
    c.set_palette(gray_palette());
    std::vector<uint8_t> g(ow * oh);
    o.upscale = up;
    CHECK(c.tone_map(t.data(), W, H, o));
    c.render(o, g.data(), ow);
    CHECK(wrong(g) < 0.1f);
    // The palette indices are what they always were.
    bool same = true;
    for (int i = 0; i < N; i++)
      same = same && c.indices()[i] == static_cast<uint8_t>(t[i] + 0.5f);
    CHECK(same);
    // Equalization and detail go through the same precision: mixed in fully
    // linear (equalize just above 0 keeps the curve all but linear).
    o.equalize = 1e-4f;
    CHECK(c.tone_map(t.data(), W, H, o));
    c.render(o, g.data(), ow);
    CHECK(wrong(g) < 0.1f);
    o.equalize = 0.0f;
    // Mapped for another scaler, then rendered with this one: still an
    // image, only as coarse as the indices (steps on the pixel grid).
    o.upscale = isp::Upscale::NEAREST;
    CHECK(c.tone_map(t.data(), W, H, o));
    o.upscale = up;
    c.render(o, g.data(), ow);
    const float coarse = wrong(g);
    CHECK(coarse > 0.5f && coarse < 5.0f);
  }
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
  for (auto up : {isp::Upscale::NEAREST, isp::Upscale::BILINEAR, isp::Upscale::BICUBIC}) {
    o.upscale = up;
    const double ms = time([&] {
      c.tone_map(t.data(), W, H, o);
      c.render(o, out.data(), 640 * 2);
    });
    std::printf("  host: %s 640x480 RGB565 = %.3f ms\n",
                up == isp::Upscale::NEAREST ? "nearest " : (up == isp::Upscale::BILINEAR ? "bilinear" : "bicubic "), ms);
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

// Looking the palette up in a scaled index image (what the PPA path does)
// gives exactly the CPU's bilinear output in that format: every format,
// mirroring, a padded stride (padding left alone).
static void test_lookup() {
  const int w = 32, h = 24, n = w * h, s = 5, ow = w * s, oh = h * s;
  std::vector<float> t(n);
  uint32_t seed = 4242;
  for (int i = 0; i < n; i++) {
    seed = seed * 1103515245u + 12345u;
    t[i] = 10.0f + static_cast<float>((seed >> 8) % 3000) / 100.0f;  // 10..40 C
  }
  isp::CpuIsp c;
  c.set_palette(isp::make_palette(isp::Palette::IRONBOW));
  isp::Options o;
  o.range_min = 15.0f;
  o.range_max = 40.0f;
  o.scale = s;
  o.upscale = isp::Upscale::BILINEAR;
  const video::ImageFormat formats[] = {video::ImageFormat::RGB565_LE, video::ImageFormat::RGB565_BE,
                                        video::ImageFormat::RGB888, video::ImageFormat::GRAY8};
  for (int m = 0; m < 2; m++) {
    o.mirror_x = m == 1;
    o.format = video::ImageFormat::GRAY8;
    CHECK(c.tone_map(t.data(), w, h, o));
    std::vector<uint8_t> scaled(ow * oh);
    c.render(o, scaled.data(), ow);
    for (const auto f : formats) {
      o.format = f;
      const uint32_t stride = ow * video::bytes_per_pixel(f) + 3;
      std::vector<uint8_t> a(stride * oh, 0xA5), b(stride * oh, 0xA5);
      c.render(o, a.data(), stride);
      c.lookup(scaled.data(), ow, oh, f, b.data(), stride);
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

// Spots: placed on the image as shown, measured on the input under them.
static void test_spots() {
  isp::Options o;
  o.scale = 20;
  std::vector<float> t(N);
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      t[y * W + x] = 10.0f * x + 1000.0f * y;  // a ramp: bilinear is exact on it
  // The centre of the image lies between pixels 15 and 16, rows 11 and 12.
  CHECK(NEAR(isp::spot_value(t.data(), W, H, o, 0.5f, 0.5f), 155.0f + 11500.0f, 1e-2f));
  // A pixel's centre gives that pixel's value.
  CHECK(NEAR(isp::spot_value(t.data(), W, H, o, 3.5f / W, 2.5f / H), 30.0f + 2000.0f, 1e-2f));
  // Edges and beyond clamp to the outermost pixels.
  CHECK(NEAR(isp::spot_value(t.data(), W, H, o, 0.0f, 0.0f), 0.0f, 1e-3f));
  CHECK(NEAR(isp::spot_value(t.data(), W, H, o, 1.0f, 1.0f), 310.0f + 23000.0f, 1e-2f));
  CHECK(NEAR(isp::spot_value(t.data(), W, H, o, -1.0f, NAN), 0.0f, 1e-3f));
  // Mirrored: the spot stays where it is on the image, so it measures the
  // pixel that is now shown there.
  o.mirror_x = true;
  CHECK(NEAR(isp::spot_value(t.data(), W, H, o, 3.5f / W, 2.5f / H), 280.0f + 2000.0f, 1e-2f));
  o.mirror_y = true;
  CHECK(NEAR(isp::spot_value(t.data(), W, H, o, 3.5f / W, 2.5f / H), 280.0f + 21000.0f, 1e-2f));
  // ... and it agrees with map_point: the input pixel measured is drawn under the spot.
  const isp::Point drawn = isp::map_point(o, W, H, 28, 21);
  const isp::Point spot = isp::spot_point(W * 20, H * 20, 3.5f / W, 2.5f / H);
  CHECK(drawn.x == spot.x && drawn.y == spot.y);
  // Output pixels: inside the image whatever the fraction.
  isp::Point p = isp::spot_point(640, 480, 0.5f, 0.25f);
  CHECK(p.x == 320 && p.y == 120);
  p = isp::spot_point(640, 480, 1.0f, 2.0f);
  CHECK(p.x == 639 && p.y == 479);
  p = isp::spot_point(640, 480, -0.5f, 0.0f);
  CHECK(p.x == 0 && p.y == 0);
  CHECK(isp::spot_value(nullptr, W, H, o, 0.5f, 0.5f) == 0.0f);

  // Clicks: on empty image add a spot, on a spot remove it.
  isp::Spot sp[isp::MAX_SPOTS];
  uint8_t n = 0;
  const float aspect = 4.0f / 3.0f;
  CHECK(isp::toggle_spot(sp, n, 0.25f, 0.5f, aspect, 1) && n == 1 && sp[0].material == 1);
  CHECK(isp::toggle_spot(sp, n, 0.75f, 0.5f, aspect, 0) && n == 2);
  CHECK(isp::toggle_spot(sp, n, 0.27f, 0.52f, aspect, 3) && n == 1 && NEAR(sp[0].x, 0.75f, 1e-6f));  // near: removed
  // The radius is round on screen: 0.06 of the height is 0.045 of a 4:3 width.
  CHECK(isp::toggle_spot(sp, n, 0.75f + 0.05f, 0.5f, aspect, 0) && n == 2);  // just outside: added
  CHECK(isp::toggle_spot(sp, n, 0.75f, 0.5f + 0.055f, aspect, 0) && n == 1);  // inside: removed (the nearer)
  n = 0;
  for (int i = 0; i < isp::MAX_SPOTS; i++)
    CHECK(isp::toggle_spot(sp, n, 0.1f * i + 0.05f, 0.1f, aspect, 0));
  CHECK(!isp::toggle_spot(sp, n, 0.5f, 0.9f, aspect, 0) && n == isp::MAX_SPOTS);  // full: nothing added
  CHECK(isp::toggle_spot(sp, n, 0.05f, 0.1f, aspect, 0) && n == isp::MAX_SPOTS - 1);  // removing still works
  CHECK(isp::toggle_spot(sp, n, 2.0f, -1.0f, aspect, 200) && sp[n - 1].x == 1.0f && sp[n - 1].y == 0.0f &&
        sp[n - 1].material == 0);  // clamped; unknown material: default

  // Colours: each new spot the lowest one free; a spot keeps its colour when
  // others go.
  n = 0;
  isp::toggle_spot(sp, n, 0.1f, 0.5f, aspect, 0);
  isp::toggle_spot(sp, n, 0.3f, 0.5f, aspect, 0);
  isp::toggle_spot(sp, n, 0.5f, 0.5f, aspect, 0);
  CHECK(n == 3 && sp[0].color == 0 && sp[1].color == 1 && sp[2].color == 2);
  isp::toggle_spot(sp, n, 0.1f, 0.5f, aspect, 0);  // the first goes
  CHECK(n == 2 && sp[0].color == 1 && sp[1].color == 2);  // the others keep theirs
  isp::toggle_spot(sp, n, 0.7f, 0.5f, aspect, 0);
  CHECK(n == 3 && sp[2].color == 0);  // the freed colour is reused
  n = 0;
  for (int i = 0; i < isp::MAX_SPOTS; i++)
    isp::toggle_spot(sp, n, 0.1f * i + 0.05f, 0.3f, aspect, 0);
  bool distinct = true;
  for (int i = 0; i < n; i++)
    for (int j = i + 1; j < n; j++)
      distinct = distinct && sp[i].color != sp[j].color;
  CHECK(n == isp::MAX_SPOTS && distinct && isp::free_spot_color(sp, n) == isp::MAX_SPOTS);

  // Materials by key or label, any case; unknown: default.
  CHECK(isp::material_index("tortoise_shell") == isp::material_index("Tortoise Shell"));
  CHECK(isp::MATERIALS[isp::material_index("Tortoise shell")].emissivity == 0.96f);
  CHECK(isp::material_index("aluminum") != 0 && isp::material_index("unobtainium") == 0 &&
        isp::material_index("") == 0 && isp::material_index("tortoise") == 0);
  for (uint8_t i = 1; i < isp::MATERIAL_COUNT; i++)
    CHECK(isp::MATERIALS[i].emissivity > 0.0f && isp::MATERIALS[i].emissivity <= 1.0f);

  // Emissivity: the same emissivity, or one unknown, changes nothing.
  CHECK(isp::correct_emissivity(30.0f, 0.95f, 0.95f, 15.0f) == 30.0f);
  CHECK(isp::correct_emissivity(30.0f, NAN, 0.9f, 15.0f) == 30.0f);
  CHECK(isp::correct_emissivity(30.0f, 0.95f, 0.0f, 15.0f) == 30.0f);  // default material
  CHECK(isp::correct_emissivity(30.0f, 0.95f, 0.9f, NAN) == 30.0f);
  // At the reflected temperature every emissivity reads the same.
  CHECK(NEAR(isp::correct_emissivity(20.0f, 0.95f, 0.3f, 20.0f), 20.0f, 1e-3f));
  // Warmer than the background: a lower emissivity means the surface is warmer than it read.
  const float wood = isp::correct_emissivity(30.0f, 0.95f, 0.90f, 15.0f);
  const float bare = isp::correct_emissivity(30.0f, 0.95f, 0.10f, 15.0f);
  CHECK(wood > 30.5f && wood < 31.5f && bare > wood);
  // Round trip: what a source at 0.95 reports for a 0.9 surface at 40 C, corrected, is 40 C.
  {
    const double k = 273.15, to = 40 + k, r = 15 + k;
    const double w = 0.9 * to * to * to * to + 0.1 * r * r * r * r;  // radiance seen
    const double ts = std::pow((w - 0.05 * r * r * r * r) / 0.95, 0.25) - k;  // as the source computes it
    CHECK(NEAR(isp::correct_emissivity(static_cast<float>(ts), 0.95f, 0.9f, 15.0f), 40.0f, 1e-3f));
  }
  // Colder than the background: lower emissivity, colder surface.
  CHECK(isp::correct_emissivity(5.0f, 0.95f, 0.9f, 15.0f) < 5.0f);

  // The list as text for the card.
  isp::Spot two[2] = {{0.25f, 0.5f, isp::material_index("wood"), 3}, {1.0f, 0.0f, 0, 0}};
  CHECK(isp::format_spots(two, 2) == "25.0,50.0,wood,3;100.0,0.0,default,0");
  CHECK(isp::format_spots(two, 0).empty());
  // The longest list fits a Home Assistant state (255 characters).
  isp::Spot most[isp::MAX_SPOTS];
  uint8_t longest = 0;
  for (uint8_t i = 0; i < isp::MATERIAL_COUNT; i++)
    if (std::strlen(isp::MATERIALS[i].key) > std::strlen(isp::MATERIALS[longest].key))
      longest = i;
  for (uint8_t i = 0; i < isp::MAX_SPOTS; i++)
    most[i] = isp::Spot{1.0f, 1.0f, longest, i};
  CHECK(isp::format_spots(most, isp::MAX_SPOTS).size() <= 255);
}

int main() {
  test_spots();
  test_palettes();
  test_map_direct();
  test_lookup();
  test_index_rule();
  test_adopt();
  test_analyze_matches_process();
  test_valid_range_and_median();
  test_radiometric();
  test_sensor_frame_due();
  test_isp();
  test_temporal_filter();
  test_edge_smoothing();
  test_curve_blend();
  test_tone_curve();
  test_bicubic();
  test_scaling_precision();
  test_large_frames();
  bench();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
