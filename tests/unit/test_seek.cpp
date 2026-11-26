// Unit tests for the Seek Compact Pro core: frame header and geometry, the
// factory block, the tables the camera sends after a start (pixel gains and
// bad pixels, the raw-value table, the temperature correction table, the
// second range), shutter offsets, the sensor value, the temperature
// polynomial, warm-up, and the order the frames must come in.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "seek_core.h"

using namespace esphome::seek_compact_pro;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    g_checks++;                                                   \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)
#define NEAR(a, b, e) (std::fabs((a) - (b)) <= (e))

static uint16_t g_counter = 0;

// A raw frame: the header (id, counter, uptime in ms), row 2 at `row2`, the
// picture from px(x, y), and other sensor rows/columns at 7000.
template<typename F>
static std::vector<uint16_t> frame(uint16_t id, F px, uint16_t row2 = 6500, uint64_t uptime_ms = 3600000) {
  std::vector<uint16_t> raw(RAW_WORDS, 7000);
  for (int k = 0; k < 16; k++)
    raw[k] = 0;  // the header words not set below are 0, as the camera sends them
  raw[0] = FRAME_MAGIC;
  raw[1] = g_counter++;
  raw[2] = id;
  raw[8] = static_cast<uint16_t>(uptime_ms >> 48);
  raw[9] = static_cast<uint16_t>(uptime_ms >> 32);
  raw[10] = static_cast<uint16_t>(uptime_ms >> 16);
  raw[11] = static_cast<uint16_t>(uptime_ms);
  for (int x = 0; x < RAW_W; x++)
    raw[2 * RAW_W + x] = row2;
  for (int y = 0; y < IMG_H; y++)
    for (int x = 0; x < IMG_W; x++)
      raw[(IMG_ROW0 + y) * RAW_W + IMG_COL0 + x] = px(x, y);
  return raw;
}

// ---------------------------------------------------------------- the factory block
static void put_f(std::vector<uint8_t> &b, size_t off, float v) { std::memcpy(&b[off], &v, 4); }
static void put_u16(std::vector<uint8_t> &b, size_t off, uint16_t v) { std::memcpy(&b[off], &v, 2); }
static void put_i32(std::vector<uint8_t> &b, size_t off, int32_t v) { std::memcpy(&b[off], &v, 4); }
static constexpr float SENSOR_REF = 6507.0f;
enum Coef { CB, CA, CC, CD, CE, CF, CG };
// A block whose temperature is simply T = x = pixel - 3000 (C = 1, the rest
// 0), with C depending on s by `s_slope`.
static std::vector<uint8_t> factory_block(float s_slope = 0.0f) {
  std::vector<uint8_t> b(FACTORY_BYTES, 0);
  put_u16(b, 0x02, 0xF731);
  put_u16(b, 0x4E, 1401);
  put_u16(b, 0x48, 324);
  put_u16(b, 0x52, static_cast<uint16_t>(-1300));
  put_i32(b, 0x60, 3);
  put_f(b, 0x64, SENSOR_REF);
  put_f(b, 0x98, 0.0f);
  put_f(b, 0x9C, 1.0f);
  put_f(b, 0xF4, 1e6f);  // threshold: the cubic everywhere
  put_f(b, 0xA0 + 12 * CC + 4, s_slope);
  put_f(b, 0xA0 + 12 * CC + 8, 1.0f);
  return b;
}

static void test_header() {
  auto raw = frame(FRAME_SHUTTER, [](int x, int y) { return uint16_t(2000 + x + y); }, 6500, 0x0102030405ULL);
  CHECK(frame_magic(raw.data()) == 1401 && frame_id(raw.data()) == FRAME_SHUTTER);
  CHECK(image_px(raw.data(), 0) == raw[12 * RAW_W + 2] && image_px(raw.data(), 0) == 2000);
  CHECK(image_px(raw.data(), IMG_PIXELS - 1) == raw[251 * RAW_W + 321] &&
        image_px(raw.data(), IMG_PIXELS - 1) == 2000 + 319 + 239);
  CHECK(RAW_BYTES == 177840 && RAW_BYTES % CHUNK_BYTES == 0 && RAW_BYTES / CHUNK_BYTES == 13);
  CHECK(frame_uptime_us(raw.data()) == 0x0102030405ULL * 1000);
  raw[15] = 7;  // microseconds win when present
  CHECK(frame_uptime_us(raw.data()) == 7);
  CHECK(frame_used(FRAME_GAIN) && frame_used(FRAME_LUT) && frame_used(FRAME_ADJUST) && frame_used(FRAME_RANGE2) &&
        frame_used(FRAME_PRE_SHUTTER) && !frame_used(5) && !frame_used(15));
}

static void test_factory() {
  auto b = factory_block(0.25f);
  Factory f;
  CHECK(parse_factory(b.data(), b.size(), &f) && f.valid);
  CHECK(f.version == 3 && f.sensor_ref == SENSOR_REF && f.ref_pixels == 324 && f.range2_offset == -1300 && f.lut_shift);
  CHECK(f.coef[CC][1] == 0.25f && f.coef[CC][2] == 1.0f && f.k1 == 1.0f);
  CHECK(f.warm_a == 6.7f && f.warm_tau == 95.0f);  // the Compact Pro's default warm-up
  auto m = b;
  for (size_t k = 0; k < 5; k++)
    m[0xC04 + k] = static_cast<uint8_t>("2003"[k]);
  CHECK(parse_factory(m.data(), m.size(), &f) && f.warm_a == 1.5f && f.warm_tau == 210.0f);
  m = b;
  put_f(m, 0x304, 3.0f);
  put_f(m, 0x308, 60.0f);
  CHECK(parse_factory(m.data(), m.size(), &f) && f.warm_a == 3.0f && f.warm_tau == 60.0f);
  m = b;
  put_i32(m, 0x43C, 1);
  CHECK(parse_factory(m.data(), m.size(), &f) && !f.lut_shift);
  // Not supported: another version, no valid mark, another camera, a bad number, a short block.
  m = b;
  put_i32(m, 0x60, 4);
  CHECK(!parse_factory(m.data(), m.size(), &f) && !f.valid);
  m = b;
  put_u16(m, 0x02, 0);
  CHECK(!parse_factory(m.data(), m.size(), &f));
  m = b;
  put_u16(m, 0x4E, 1400);
  CHECK(!parse_factory(m.data(), m.size(), &f));
  m = b;
  put_f(m, 0xA0 + 12 * CD + 8, NAN);
  CHECK(!parse_factory(m.data(), m.size(), &f));
  CHECK(!parse_factory(b.data(), 0x400, &f) && !parse_factory(nullptr, 0, &f));
}

// ---------------------------------------------------------------- the start-up tables
// Frame 4: 5000 everywhere, 10000 at (100, 50) (gain 1/2), 0 at the bad pixels.
static bool is_bad(int x, int y) {
  return (x == 10 && y == 20) || (x >= 50 && x <= 52 && y >= 60 && y <= 62) || (x == 0 && y == 0);
}
static constexpr int NUM_BAD = 11;
static std::vector<uint16_t> gain_frame() {
  return frame(FRAME_GAIN, [](int x, int y) { return uint16_t(is_bad(x, y) ? 0 : (x == 100 && y == 50 ? 10000 : 5000)); });
}
// Frame 9: entry i = 2 i + 100 below 15000 (halved: i + 50), 40000 from it (doubled, capped at 65535).
static std::vector<uint16_t> lut_frame() {
  auto f = frame(FRAME_LUT, [](int, int) { return uint16_t(0); });
  for (size_t i = 0; i < 16384; i++)
    f[(4 + i / 324) * RAW_W + i % 324] = i < SECOND_RANGE ? static_cast<uint16_t>(2 * i + 100) : 40000;
  return f;
}
// Frame 14: one format-3 table. Sensor axis at s = -100 and +100 counts, two
// temperature nodes, 0 and 100 C; the correction at them is 1 and 3 C at
// s = -100, 3 and 5 C at s = +100.
static void put_payload(std::vector<uint16_t> &f, const std::vector<uint8_t> &bytes) {
  for (size_t n = 0; n + 1 < bytes.size(); n += 2) {
    const size_t w = n / 2;
    f[(4 + w / 324) * RAW_W + w % 324] = static_cast<uint16_t>(bytes[n] | (bytes[n + 1] << 8));
  }
}
static std::vector<uint16_t> adjust_frame(bool with_table) {
  auto f = frame(FRAME_ADJUST, [](int, int) { return uint16_t(0xFFFF); });
  std::vector<uint8_t> p(0x6600, 0xFF);
  if (with_table) {
    std::fill(p.begin(), p.begin() + 0x2200, 0);
    const uint32_t magic = 0x86AD416D;
    std::memcpy(&p[0x80], &magic, 4);
    const int32_t ns = 2, nt = 2;
    std::memcpy(&p[0xE8], &ns, 4);
    std::memcpy(&p[0xEC], &nt, 4);
    const int16_t axis[2] = {-100 * 32, 100 * 32};
    std::memcpy(&p[0x1C0], axis, 4);
    auto grid = [&](size_t base, int j, int i, int16_t v) { std::memcpy(&p[base + 2 * (j * 32 + i)], &v, 2); };
    for (int i = 0; i < 2; i++) {
      grid(0x200, 0, i, 0);         // node 0: 0 C
      grid(0x200, 1, i, 100 * 32);  // node 1: 100 C
      grid(0x1200, 0, i, static_cast<int16_t>((i == 0 ? 1 : 3) * 32));
      grid(0x1200, 1, i, static_cast<int16_t>((i == 0 ? 3 : 5) * 32));
    }
  }
  put_payload(f, p);
  return f;
}

// Shutter frames are a fixed pattern around 6000 (raw); images add to it.
static int pattern(int x, int y) { return 6000 + ((x * 7 + y * 13) % 41) - 20; }

static void start(Processor &p, float s_slope, bool adjust) {
  p.set_scene_filter(false);  // the calibration alone; the filter has its own test
  auto b = factory_block(s_slope);
  CHECK(p.set_factory(b.data(), b.size()));
  std::vector<float> t(IMG_PIXELS);
  CHECK(p.feed(gain_frame().data(), t.data()) == Processor::Result::TABLE);
  CHECK(p.feed(lut_frame().data(), t.data()) == Processor::Result::TABLE);
  CHECK(p.feed(adjust_frame(adjust).data(), t.data()) == Processor::Result::TABLE);
}

static void test_pipeline() {
  std::vector<float> t(IMG_PIXELS);
  esphome::video::FrameStats st;
  Processor p;
  p.set_scene_filter(false);
  auto b = factory_block();
  CHECK(p.set_factory(b.data(), b.size()));
  // Images and shutter frames wait for all three tables.
  auto sh = frame(FRAME_START_SHUTTER, [](int x, int y) { return uint16_t(pattern(x, y)); });
  CHECK(!p.has_table());
  CHECK(p.feed(gain_frame().data(), t.data()) == Processor::Result::TABLE);
  CHECK(p.dead_pixels() == NUM_BAD && !p.has_table());
  CHECK(p.feed(lut_frame().data(), t.data()) == Processor::Result::TABLE && !p.has_table());
  CHECK(p.feed(adjust_frame(false).data(), t.data()) == Processor::Result::TABLE && p.has_table());
  CHECK(p.adjust_tables() == 0 && !p.ready());
  auto img0 = frame(FRAME_IMAGE, [](int x, int y) { return uint16_t(pattern(x, y)); });
  CHECK(p.feed(img0.data(), t.data()) == Processor::Result::NOT_READY);
  CHECK(p.feed(sh.data(), t.data()) == Processor::Result::SHUTTER && p.ready());

  // T = pixel - 3000 = (table 9 of raw - table 9 of shutter) x gain: the
  // pattern goes, +400 counts reads 400 "C", the half-gain pixel 200, bad
  // pixels their neighbours' value. Uptime an hour: no warm-up left.
  auto delta = [](int x, int y) { return x < 160 ? 400 : (y < 120 ? -200 : 0); };
  auto img = frame(FRAME_IMAGE, [&](int x, int y) { return uint16_t(is_bad(x, y) ? 9 : pattern(x, y) + delta(x, y)); });
  CHECK(p.feed(img.data(), t.data(), &st) == Processor::Result::IMAGE);
  CHECK(NEAR(t[5 * IMG_W + 5], 400.0f, 1e-3f) && NEAR(t[10 * IMG_W + 300], -200.0f, 1e-3f) &&
        NEAR(t[200 * IMG_W + 250], 0.0f, 1e-3f));
  CHECK(NEAR(t[50 * IMG_W + 100], 200.0f, 1e-3f));  // gain 1/2
  CHECK(NEAR(t[20 * IMG_W + 10], 400.0f, 1e-3f) && NEAR(t[61 * IMG_W + 51], 400.0f, 1e-3f) && NEAR(t[0], 400.0f, 1e-3f));
  CHECK(st.valid && NEAR(st.max, 400.0f, 1e-3f) && NEAR(st.min, -200.0f, 1e-3f) && st.max_x < 160);
  CHECK(p.warm_up_c() == 0.0f);

  // A new shutter frame replaces the offset: a scene 50 counts warmer, with
  // the shutter 50 counts warmer too, reads the same.
  auto sh2 = frame(FRAME_SHUTTER, [](int x, int y) { return uint16_t(pattern(x, y) + 50); });
  CHECK(p.feed(sh2.data(), t.data()) == Processor::Result::SHUTTER);
  auto img2 = frame(FRAME_IMAGE, [&](int x, int y) { return uint16_t(pattern(x, y) + 50 + delta(x, y)); });
  CHECK(p.feed(img2.data(), t.data()) == Processor::Result::IMAGE && NEAR(t[5 * IMG_W + 5], 400.0f, 1e-3f));

  // Gain and offset apply last.
  p.set_correction(1.1f, -2.0f);
  CHECK(p.feed(img2.data(), t.data()) == Processor::Result::IMAGE && NEAR(t[5 * IMG_W + 5], 400.0f * 1.1f - 2.0f, 1e-2f));
  p.set_correction(1.0f, 0.0f);

  // Warm-up: 6.7 C off at start, 6.7 / e after 95 s.
  auto early = frame(FRAME_IMAGE, [&](int x, int y) { return uint16_t(pattern(x, y) + 50 + delta(x, y)); }, 6500, 0);
  CHECK(p.feed(early.data(), t.data()) == Processor::Result::IMAGE && NEAR(p.warm_up_c(), 6.7f, 1e-4f));
  CHECK(NEAR(t[5 * IMG_W + 5], 400.0f - 6.7f, 1e-3f));
  auto later = frame(FRAME_IMAGE, [&](int x, int y) { return uint16_t(pattern(x, y) + 50 + delta(x, y)); }, 6500, 95000);
  CHECK(p.feed(later.data(), t.data()) == Processor::Result::IMAGE && NEAR(p.warm_up_c(), 6.7f / std::exp(1.0f), 1e-3f));

  // Other frame ids are left alone; reset() needs the tables again.
  auto odd = frame(5, [](int, int) { return uint16_t(0); });
  CHECK(p.feed(odd.data(), t.data()) == Processor::Result::IGNORED);
  p.reset();
  CHECK(!p.has_table() && p.feed(img.data(), t.data()) == Processor::Result::NOT_READY);
}

// The sensor value: s = smoothed row-2 mean - the factory reference, and a
// coefficient that depends on it (C = 1 + 0.01 s).
static void test_sensor() {
  std::vector<float> t(IMG_PIXELS);
  Processor p;
  start(p, 0.01f, false);
  auto sh = frame(FRAME_START_SHUTTER, [](int x, int y) { return uint16_t(pattern(x, y)); }, 6407);  // s = -100
  CHECK(p.feed(sh.data(), t.data()) == Processor::Result::SHUTTER);
  auto img = frame(FRAME_IMAGE, [](int x, int y) { return uint16_t(pattern(x, y) + 400); }, 6407);
  CHECK(p.feed(img.data(), t.data()) == Processor::Result::IMAGE);
  CHECK(NEAR(p.sensor_counts(), -100.0f, 1e-3f) && NEAR(t[1000], 0.0f, 1e-3f));  // C = 0
  // Row 2 steps to s = +60: the smoothing (7:1 per frame) moves s an eighth of the way, to -80.
  auto img2 = frame(FRAME_IMAGE, [](int x, int y) { return uint16_t(pattern(x, y) + 400); }, 6567);
  CHECK(p.feed(img2.data(), t.data()) == Processor::Result::IMAGE);
  CHECK(NEAR(p.sensor_counts(), -80.0f, 1e-2f) && NEAR(t[1000], 80.0f, 0.05f));  // C = 0.2
}

// The temperature correction table, along T and along s.
static void test_adjust() {
  std::vector<float> t(IMG_PIXELS);
  Processor p;
  start(p, 0.0f, true);
  CHECK(p.adjust_tables() == 1);
  auto sh = frame(FRAME_START_SHUTTER, [](int x, int y) { return uint16_t(pattern(x, y)); }, 6507);  // s = 0
  CHECK(p.feed(sh.data(), t.data()) == Processor::Result::SHUTTER);
  // s = 0: the correction is 2 C at 0 C and 4 C at 100 C; T = 50 -> 50 - 3, T = 100 -> 100 - 4.
  auto img = frame(FRAME_IMAGE, [](int x, int y) { return uint16_t(pattern(x, y) + (x < 160 ? 50 : 100)); }, 6507);
  CHECK(p.feed(img.data(), t.data()) == Processor::Result::IMAGE);
  CHECK(NEAR(t[5 * IMG_W + 5], 47.0f, 0.01f) && NEAR(t[5 * IMG_W + 300], 96.0f, 0.01f));
  // Past the last node the last segment goes on: T = 150 -> 150 - 5.
  auto hot = frame(FRAME_IMAGE, [](int x, int y) { return uint16_t(pattern(x, y) + 150); }, 6507);
  CHECK(p.feed(hot.data(), t.data()) == Processor::Result::IMAGE && NEAR(t[1000], 145.0f, 0.01f));
  // s = -100 (the first axis point): 1 C at 0 C, 3 C at 100 C; T = 50 -> 50 - 2.
  Processor q;
  start(q, 0.0f, true);
  auto sh2 = frame(FRAME_START_SHUTTER, [](int x, int y) { return uint16_t(pattern(x, y)); }, 6407);
  CHECK(q.feed(sh2.data(), t.data()) == Processor::Result::SHUTTER);
  auto img2 = frame(FRAME_IMAGE, [](int x, int y) { return uint16_t(pattern(x, y) + 50); }, 6407);
  CHECK(q.feed(img2.data(), t.data()) == Processor::Result::IMAGE && NEAR(t[1000], 48.0f, 0.01f));
}

// The second range: raw values from 15000 up take the frame-7 offsets too.
static void test_second_range() {
  std::vector<float> t(IMG_PIXELS);
  Processor p;
  start(p, 0.0f, false);
  auto sh = frame(FRAME_START_SHUTTER, [](int, int) { return uint16_t(6000); });
  CHECK(p.feed(sh.data(), t.data()) == Processor::Result::SHUTTER);
  // Frame 7 (through table 9) less the start-up shutter frame, plus factory 0x52 (-1300).
  auto r2 = frame(FRAME_RANGE2, [](int, int) { return uint16_t(9000); });
  CHECK(p.feed(r2.data(), t.data()) == Processor::Result::TABLE);
  auto img = frame(FRAME_IMAGE, [](int x, int) { return uint16_t(x < 160 ? 7000 : 15500); });
  CHECK(p.feed(img.data(), t.data()) == Processor::Result::IMAGE);
  CHECK(NEAR(t[5 * IMG_W + 5], 1000.0f, 1e-3f));  // 7050 - 6050
  // 15500 -> min(40000 x 2, 65535); offset 6050 + (9050 - 6050 - 1300) = 7750.
  CHECK(NEAR(t[5 * IMG_W + 300], 65535.0f - 7750.0f, 1e-2f));
}

// Bad columns, and a factory block this calibration does not support.
static void test_columns_and_factory() {
  std::vector<float> t(IMG_PIXELS);
  esphome::video::FrameStats st;
  Processor p;
  p.set_scene_filter(false);
  CHECK(p.add_bad_column(108) && !p.add_bad_column(IMG_W));
  auto b = factory_block();
  CHECK(p.set_factory(b.data(), b.size()));
  CHECK(p.feed(gain_frame().data(), t.data()) == Processor::Result::TABLE && p.dead_pixels() == NUM_BAD + IMG_H);
  CHECK(p.feed(lut_frame().data(), t.data()) == Processor::Result::TABLE);
  CHECK(p.feed(adjust_frame(false).data(), t.data()) == Processor::Result::TABLE);
  auto sh = frame(FRAME_START_SHUTTER, [](int x, int y) { return uint16_t(pattern(x, y)); });
  CHECK(p.feed(sh.data(), t.data()) == Processor::Result::SHUTTER);
  auto img = frame(FRAME_IMAGE, [](int x, int y) {
    return uint16_t(pattern(x, y) + 400 + (x == 108 ? ((y % 2) ? 300 : -300) : 0));
  });
  CHECK(p.feed(img.data(), t.data()) == Processor::Result::IMAGE);
  CHECK(NEAR(t[7 * IMG_W + 108], 400.0f, 1e-3f) && NEAR(t[8 * IMG_W + 108], 400.0f, 1e-3f));
  PixelTable table;
  for (int k = 0; k < static_cast<int>(MAX_BAD_COLUMNS); k++)
    CHECK(table.add_bad_column(static_cast<uint16_t>(k)));
  CHECK(!table.add_bad_column(200));

  // Frames go through, but there are no temperatures.
  Processor q;
  auto bad = factory_block();
  put_i32(bad, 0x60, 5);
  CHECK(!q.set_factory(bad.data(), bad.size()));
  CHECK(q.feed(gain_frame().data(), t.data()) == Processor::Result::TABLE);
  CHECK(q.feed(lut_frame().data(), t.data()) == Processor::Result::TABLE);
  CHECK(q.feed(adjust_frame(false).data(), t.data()) == Processor::Result::TABLE);
  CHECK(q.feed(sh.data(), t.data()) == Processor::Result::SHUTTER);
  CHECK(q.feed(img.data(), t.data(), &st) == Processor::Result::IMAGE && std::isnan(t[0]) && !st.valid);
}

// Seek's picture filter: single pixels far off their 3x3 median take it,
// edges stay, small frame-to-frame changes are smoothed, a shutter frame
// restarts the smoothing, and a still scene teaches it nothing.
static void test_scene_filter() {
  std::vector<uint16_t> raw(RAW_WORDS, 6000);  // no second-range pixels
  std::vector<uint16_t> img(IMG_PIXELS);
  auto picture = [&](int spot, int step) {
    for (int y = 0; y < IMG_H; y++)
      for (int x = 0; x < IMG_W; x++)
        img[y * IMG_W + x] = static_cast<uint16_t>(3000 + (x >= 160 ? step : 0));
    img[100 * IMG_W + 50] = static_cast<uint16_t>(3000 + spot);  // one pixel off its neighbours
  };
  SceneFilter f;
  f.set_sigma(4);  // smoothing below 24 counts
  picture(100, 300);
  CHECK(f.process(img.data(), raw.data()));
  CHECK(img[100 * IMG_W + 50] == 3000);                                  // 100 counts off: takes the median
  CHECK(img[10 * IMG_W + 159] == 3000 && img[10 * IMG_W + 160] == 3300);  // the edge stays
  picture(10, 300);
  CHECK(f.process(img.data(), raw.data()) && img[100 * IMG_W + 50] != 3000 && img[100 * IMG_W + 50] < 3010);
  // 10 counts off (under 15) stays, but the temporal filter cores the jump from 3000: (10 x 10) / 24 = 4.
  CHECK(img[100 * IMG_W + 50] == 3004);
  // A jump of 24 counts or more passes at once; a shutter frame restarts the smoothing.
  picture(0, 300);
  for (auto &v : img)
    v = static_cast<uint16_t>(v + 30);
  CHECK(f.process(img.data(), raw.data()) && img[50 * IMG_W + 10] == 3030);
  picture(0, 300);
  for (auto &v : img)
    v = static_cast<uint16_t>(v + 40);
  f.shutter();
  CHECK(f.process(img.data(), raw.data()) && img[50 * IMG_W + 10] == 3040);
  // Level jumps across the whole picture count as motion (it learns, but a
  // flat picture has nothing to learn); a still scene does not update at all.
  picture(0, 300);
  CHECK(f.process(img.data(), raw.data()));
  const uint32_t u = f.updates();
  for (int k = 0; k < 5; k++) {
    picture(0, 300);
    CHECK(f.process(img.data(), raw.data()));
  }
  CHECK(f.updates() == u && img[50 * IMG_W + 10] == 3000);
  // Through the Processor (on by default): the half-gain pixel of the pipeline
  // test is a single pixel 200 counts off, and takes its neighbours' value.
  Processor p;
  auto b = factory_block();
  CHECK(p.set_factory(b.data(), b.size()) && p.scene_filter());
  std::vector<float> t(IMG_PIXELS);
  CHECK(p.feed(gain_frame().data(), t.data()) == Processor::Result::TABLE);
  CHECK(p.feed(lut_frame().data(), t.data()) == Processor::Result::TABLE);
  CHECK(p.feed(adjust_frame(false).data(), t.data()) == Processor::Result::TABLE);
  auto sh = frame(FRAME_START_SHUTTER, [](int x, int y) { return uint16_t(pattern(x, y)); });
  CHECK(p.feed(sh.data(), t.data()) == Processor::Result::SHUTTER);
  auto im = frame(FRAME_IMAGE, [](int x, int y) { return uint16_t(pattern(x, y) + 400); });
  CHECK(p.feed(im.data(), t.data()) == Processor::Result::IMAGE);
  CHECK(NEAR(t[50 * IMG_W + 100], 400.0f, 1e-3f) && NEAR(t[5 * IMG_W + 5], 400.0f, 1e-3f) && p.scene_replaced() >= 1);
}

int main() {
  test_header();
  test_factory();
  test_pipeline();
  test_sensor();
  test_adjust();
  test_second_range();
  test_columns_and_factory();
  test_scene_filter();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
