// Unit tests for the HTI-301 core: descriptors, probe/commit, packet
// assembly, metadata, temperatures, YUYV conversion.
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "hti301_core.h"

using namespace esphome::hti301;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    g_checks++;                                                   \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

// ---------------------------------------------------------------- descriptors

// The camera's configuration descriptor, rebuilt from its lsusb dump
// (doc/HTI-301.txt): 162 bytes.
static std::vector<uint8_t> hti301_config() {
  std::vector<uint8_t> d = {
      9,  2,    0xA2, 0x00, 2, 1, 4, 0x80, 250,  // configuration
      8,  11,   0,    2,    14, 3, 0, 0,          // interface association
      9,  4,    0,    0,    0, 14, 1, 0, 0,       // VC interface 0
      13, 0x24, 1,    0x00, 0x01, 0x28, 0x00, 0x80, 0x8D, 0x5B, 0x00, 1, 1,  // VC header, UVC 1.00, 6 MHz
      18, 0x24, 2,    1,    0x01, 0x02, 2, 6, 0, 0, 0, 0, 0, 0, 3, 0x20, 0x02, 0x00,  // camera terminal
      9,  0x24, 3,    2,    0x01, 0x01, 1, 1, 0,                                      // output terminal
      9,  4,    1,    0,    0, 14, 2, 0, 9,                                           // VS interface 1 alt 0
      14, 0x24, 1,    1,    0x47, 0x00, 0x81, 0, 2, 0, 0, 0, 1, 0,                    // VS input header
      // Uncompressed format 1, YUY2, 16 bpp, 1 frame
      27, 0x24, 4, 1, 1, 0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B,
      0x71, 16, 1, 0, 0, 0, 0,
      // Frame 1: 384x292, 224256 bytes, 400000 x 100 ns
      30, 0x24, 5, 1, 0, 0x80, 0x01, 0x24, 0x01, 0x00, 0x8C, 0x55, 0x00, 0x00, 0x8C, 0x55, 0x00, 0x00, 0x6C, 0x03,
      0x00, 0x80, 0x1A, 0x06, 0x00, 1, 0x80, 0x1A, 0x06, 0x00,
      9,  4,    1,    1,    1, 14, 2, 0, 0,        // VS interface 1 alt 1
      7,  5,    0x81, 0x05, 0x0C, 0x03, 1,         // isochronous IN, 780 bytes, bInterval 1
  };
  return d;
}

static void test_descriptors() {
  const auto cfg = hti301_config();
  CHECK(cfg.size() == 162);
  const UvcLayout l = parse_descriptors(cfg.data(), cfg.size());
  CHECK(l.ok);
  CHECK(l.uvc_version == 0x0100);
  CHECK(l.probe_size() == 26);
  CHECK(l.vc_interface == 0);
  CHECK(l.vs_interface == 1);
  CHECK(l.format_index == 1);
  CHECK(l.frame_index == 1);
  CHECK(l.width == 384);
  CHECK(l.height == 292);
  CHECK(l.max_frame_size == 224256);
  CHECK(l.frame_interval == 400000);
  CHECK(l.alt_count == 1);
  CHECK(l.alts[0].alt == 1 && l.alts[0].endpoint == 0x81 && l.alts[0].packet_size == 780 &&
        l.alts[0].transactions == 1 && l.alts[0].interval == 1);
  CHECK(l.alt_for(780) == &l.alts[0]);
  CHECK(l.alt_for(0) == &l.alts[0]);
  CHECK(l.alt_for(781) == nullptr);

  // Truncated: no streaming endpoint -> not usable, and no overrun.
  for (size_t n = 0; n < cfg.size(); n++) {
    const UvcLayout t = parse_descriptors(cfg.data(), n);
    CHECK(!t.ok);
  }
  // A zero-length descriptor stops the walk instead of looping.
  auto bad = cfg;
  bad[9] = 0;
  CHECK(!parse_descriptors(bad.data(), bad.size()).ok);
}

// ---------------------------------------------------------------- probe

static void test_probe() {
  Probe p;
  p.hint = 1;
  p.format_index = 1;
  p.frame_index = 1;
  p.frame_interval = 400000;
  uint8_t buf[34];
  std::memset(buf, 0xEE, sizeof(buf));
  encode_probe(p, buf, 26);
  const uint8_t want[26] = {0x01, 0x00, 1, 1, 0x80, 0x1A, 0x06, 0x00, 0, 0, 0, 0, 0,
                            0,    0,    0, 0, 0,    0,    0,    0,    0, 0, 0, 0, 0};
  CHECK(std::memcmp(buf, want, 26) == 0);
  CHECK(buf[26] == 0xEE);  // writes only the length asked for

  // The camera's reply (doc/hti301usbdriverkit.md): 224256-byte frames, 780-byte payloads.
  uint8_t reply[26] = {0x01, 0x00, 1, 1, 0x80, 0x1A, 0x06, 0x00, 0, 0, 0, 0, 0,
                       0,    0,    0, 0, 0,    0x00, 0x6C, 0x03, 0x00, 0x0C, 0x03, 0x00, 0x00};
  Probe got;
  CHECK(decode_probe(reply, 26, &got));
  CHECK(got.frame_interval == 400000 && got.max_frame_size == 224256 && got.max_payload == 780);
  CHECK(!decode_probe(reply, 25, &got));
}

// ---------------------------------------------------------------- assembly

struct Sink {
  std::vector<uint8_t> a, b;  // two buffers, swapped on each complete frame
  bool use_a{true};
  std::vector<std::pair<bool, size_t>> ends;
  std::vector<std::vector<uint8_t>> complete_frames;
  uint8_t *buf() { return this->use_a ? this->a.data() : this->b.data(); }
};

static std::vector<uint8_t> packet(bool fid, bool eof, const std::vector<uint8_t> &payload, uint8_t extra = 0) {
  // 12-byte header (PTS + SCR) as UVC cameras commonly send.
  std::vector<uint8_t> p = {12, static_cast<uint8_t>(0x80 | 0x08 | 0x04 | (fid ? 1 : 0) | (eof ? 2 : 0) | extra)};
  p.resize(12, 0);
  p.insert(p.end(), payload.begin(), payload.end());
  return p;
}

static std::vector<uint8_t> chunk(uint8_t seed, size_t n) {
  std::vector<uint8_t> c(n);
  for (size_t i = 0; i < n; i++)
    c[i] = static_cast<uint8_t>(seed + i * 7);
  return c;
}

static void feed(UvcAssembler &as, Sink &s, uint8_t *&buf, const std::vector<uint8_t> &p, size_t cap = 1000) {
  as.feed(p.data(), p.size(), buf, cap, [&](bool complete, size_t bytes) {
    s.ends.emplace_back(complete, bytes);
    if (complete) {
      s.complete_frames.emplace_back(buf, buf + bytes);
      s.use_a = !s.use_a;
      buf = s.buf();
    }
  });
}

static void test_assembler() {
  Sink s;
  s.a.resize(1000);
  s.b.resize(1000);
  UvcAssembler as;
  as.set_frame_size(1000);
  uint8_t *buf = s.buf();

  // Joined mid-frame: 3 packets of a frame (FID 0), then a whole frame (FID 1, ends at its size).
  for (int i = 0; i < 3; i++)
    feed(as, s, buf, packet(false, false, chunk(1, 100)));
  std::vector<uint8_t> want;
  for (int i = 0; i < 10; i++) {
    auto c = chunk(static_cast<uint8_t>(10 + i), 100);
    want.insert(want.end(), c.begin(), c.end());
    feed(as, s, buf, packet(true, false, c));
  }
  CHECK(s.ends.size() == 2);
  CHECK(s.ends[0].first == false && s.ends[0].second == 300);  // partial first frame, dropped
  CHECK(s.ends[1].first == true && s.ends[1].second == 1000);
  CHECK(s.complete_frames.size() == 1 && s.complete_frames[0] == want);

  // Empty packets change nothing; the next frame (FID 0) ends with EOF at its size.
  feed(as, s, buf, std::vector<uint8_t>{});
  feed(as, s, buf, packet(false, false, {}));  // header only
  for (int i = 0; i < 10; i++)
    feed(as, s, buf, packet(false, i == 9, chunk(static_cast<uint8_t>(50 + i), 100)));
  CHECK(s.ends.size() == 3 && s.ends[2].first);

  // A lost packet (skipped service interval) spoils its frame only.
  for (int i = 0; i < 10; i++) {
    if (i == 4)
      as.lost();
    feed(as, s, buf, packet(true, false, chunk(1, 100)));
  }
  CHECK(s.ends.size() == 4 && !s.ends[3].first);
  for (int i = 0; i < 10; i++)
    feed(as, s, buf, packet(false, false, chunk(2, 100)));
  CHECK(s.ends.size() == 5 && s.ends[4].first);

  // Error bit and malformed headers count as lost packets.
  feed(as, s, buf, packet(true, false, chunk(3, 100), 0x40));
  for (int i = 0; i < 10; i++)
    feed(as, s, buf, packet(true, false, chunk(3, 100)));
  CHECK(s.ends.size() == 6 && !s.ends[5].first);
  const uint8_t too_long[3] = {12, 0x80, 0};
  const uint8_t too_short[3] = {1, 0x80, 0};
  as.feed(too_long, 3, buf, 1000, [&](bool, size_t) { CHECK(false); });
  as.feed(too_short, 3, buf, 1000, [&](bool, size_t) { CHECK(false); });
  for (int i = 0; i < 10; i++)
    feed(as, s, buf, packet(false, false, chunk(4, 100)));
  CHECK(s.ends.size() == 7 && !s.ends[6].first);  // the next frame carries the loss
  for (int i = 0; i < 10; i++)
    feed(as, s, buf, packet(true, false, chunk(4, 100)));
  CHECK(s.ends.size() == 8 && s.ends[7].first);

  // A short frame ended by EOF, and one ended by the frame-ID change, are incomplete.
  for (int i = 0; i < 5; i++)
    feed(as, s, buf, packet(false, i == 4, chunk(5, 100)));
  CHECK(s.ends.size() == 9 && !s.ends[8].first && s.ends[8].second == 500);
  for (int i = 0; i < 5; i++)
    feed(as, s, buf, packet(true, false, chunk(6, 100)));
  feed(as, s, buf, packet(false, false, chunk(7, 100)));
  CHECK(s.ends.size() == 10 && !s.ends[9].first && s.ends[9].second == 500);

  // No buffer: the frame is dropped but its boundaries are still followed.
  uint8_t *none = nullptr;
  for (int i = 1; i < 10; i++)
    feed(as, s, none, packet(false, false, chunk(8, 100)));
  CHECK(s.ends.size() == 11 && !s.ends[10].first && s.ends[10].second == 1000);

  // A payload that would run past the buffer is not written.
  std::vector<uint8_t> small(150, 0);
  uint8_t *sb = small.data();
  UvcAssembler as2;
  as2.set_frame_size(1000);
  size_t ends = 0;
  for (int i = 0; i < 10; i++)
    as2.feed(packet(true, false, chunk(9, 100)).data(), 112, sb, small.size(), [&](bool c, size_t) {
      ends++;
      CHECK(!c);
    });
  CHECK(ends == 1);
}

// ---------------------------------------------------------------- metadata and temperatures

static constexpr uint16_t W = 384, ROWS = 288;

static void put_f32(std::vector<uint16_t> &f, size_t at, float v) {
  uint32_t b;
  std::memcpy(&b, &v, 4);
  f[at] = static_cast<uint16_t>(b & 0xFFFF);
  f[at + 1] = static_cast<uint16_t>(b >> 16);
}

struct Cal {
  uint16_t fpa_raw, cal_00, shutter_raw, core_raw;
  float c1, c2, c3, c4, c5;
  float correction, reflected, ambient, humidity, emissivity, distance;
};

static std::vector<uint16_t> make_frame(const Cal &c, uint16_t width = W, uint16_t rows = ROWS) {
  std::vector<uint16_t> f(static_cast<size_t>(width) * (rows + METADATA_ROWS), 0);
  const size_t md = static_cast<size_t>(width) * rows;
  const size_t cal = (width == 384 || width == 640) ? width * 3u : width;
  f[md + 1] = c.fpa_raw;
  f[md + 2] = 10;  // max x
  f[md + 3] = 20;  // max y
  f[md + 4] = 9000;
  f[md + 5] = 30;
  f[md + 6] = 40;
  f[md + 7] = 4000;
  f[md + 12] = 6000;
  f[md + cal] = c.cal_00;
  f[md + cal + 1] = c.shutter_raw;
  f[md + cal + 2] = c.core_raw;
  put_f32(f, md + cal + 3, c.c1);
  put_f32(f, md + cal + 5, c.c2);
  put_f32(f, md + cal + 7, c.c3);
  put_f32(f, md + cal + 9, c.c4);
  put_f32(f, md + cal + 11, c.c5);
  if (1279 + 12 <= static_cast<size_t>(width) * METADATA_ROWS) {
    put_f32(f, md + 1279, c.correction);
    put_f32(f, md + 1281, c.reflected);
    put_f32(f, md + 1283, c.ambient);
    put_f32(f, md + 1285, c.humidity);
    put_f32(f, md + 1287, c.emissivity);
    put_f32(f, md + 1289, c.distance);
  }
  return f;
}

// XTherm.mm (from ht301_hacklib), transcribed in its own float arithmetic.
static std::vector<float> reference_table(const Metadata &m, float emissivity, float reflected, float ambient,
                                          float humidity, float distance, float correction) {
  const double ZEROC = 273.15;
  float distance_adjusted = distance >= 20.0f ? 20.0f : distance;
  auto water = [&](double h, double t) {
    return h * std::exp(1.5587 + 0.06939 * t + -2.7816e-4 * std::pow(t, 2) + 6.8455e-7 * std::pow(t, 3));
  };
  const double nsqd = -std::sqrt(static_cast<double>(distance_adjusted));
  const double sqw = std::sqrt(water(humidity, ambient));
  float atm = static_cast<float>(1.9 * std::exp(nsqd * (0.006569 + -0.002276 * sqw)) +
                                 (1.0 - 1.9) * std::exp(nsqd * (0.01262 + -0.00667 * sqw)));
  float numerator_sub = static_cast<float>((1.0 - emissivity) * atm * std::pow(reflected + ZEROC, 4) +
                                           (1.0 - atm) * std::pow(ambient + ZEROC, 4));
  float denominator = emissivity * atm;
  float ts = m.shutter_temp, tfpa = m.fpa_temp;
  float cal_01 = m.cal_01, cal_02 = m.cal_02;
  float cal_a = cal_02 / (cal_01 + cal_01);
  float cal_b = cal_02 * cal_02 / (cal_01 * cal_01 * 4.0f);
  float cal_c = static_cast<float>(cal_01 * std::pow(ts, 2) + ts * cal_02);
  float cal_d = static_cast<float>(m.cal_03 * std::pow(tfpa, 2) + m.cal_04 * tfpa + m.cal_05);
  int cal_00_corr = int(390.0f - tfpa * 7.05f);
  int table_offset = cal_00_corr > 0 ? m.cal_00 - cal_00_corr : m.cal_00;
  std::vector<float> t(16384);
  for (int i = 0; i < 16384; i++) {
    float val = ((static_cast<float>(i) - table_offset) * cal_d + cal_c) / cal_01 + cal_b;
    float n = std::sqrt(std::fabs(val));
    if (std::isnan(n))
      n = 0.0f;
    float wtot = std::pow(n - cal_a + static_cast<float>(ZEROC), 4.0f);
    float ttot = std::pow((wtot - numerator_sub) / denominator, 0.25f) - static_cast<float>(ZEROC);
    float temp = ttot + (distance_adjusted * 0.85f - 1.125f) * (ttot - ambient) / 100.0f + correction;
    t[i] = temp;
  }
  return t;
}

static void test_temperatures() {
  // FPA raw 7800 -> 20 C; shutter 2982 -> 25.05 C.
  const Cal c{7800, 5000, 2982, 3000, 0.8f, 3.2f, 0.0001f, -0.002f, 0.45f, 0.0f, 22.0f, 23.0f, 0.4f, 0.97f, 1.5f};
  auto frame = make_frame(c);
  const Metadata m = parse_metadata(frame.data(), W, ROWS);
  CHECK(m.valid);
  CHECK(std::fabs(m.fpa_temp - 20.0f) < 1e-4f);
  CHECK(std::fabs(m.shutter_temp - 25.05f) < 1e-3f);
  CHECK(std::fabs(m.core_temp - 26.85f) < 1e-3f);
  CHECK(m.cal_00 == 5000 && m.cal_01 == 0.8f && m.cal_05 == 0.45f);
  CHECK(m.has_environment && m.emissivity == 0.97f && m.distance == 1.5f && m.humidity == 0.4f);
  CHECK(m.max_x == 10 && m.max_y == 20 && m.max_raw == 9000 && m.min_raw == 4000 && m.center_raw == 6000);

  // The table matches the reference across all raw values (NaN in the same places).
  const Environment none{NAN, NAN, NAN, NAN, NAN, NAN};
  const Environment env = resolve_environment(none, m);
  CHECK(env.emissivity == 0.97f && env.reflected == 22.0f && env.ambient == 23.0f && env.distance == 1.5f);
  TemperatureTable table;
  CHECK(table.prepare(m, env));
  const auto ref = reference_table(m, 0.97f, 22.0f, 23.0f, 0.4f, 1.5f, 0.0f);
  int mismatches = 0, finite = 0;
  float worst = 0;
  for (int r = 0; r < 16384; r++) {
    const float a = table.temperature(static_cast<uint16_t>(r)), b = ref[r];
    if (std::isnan(a) != std::isnan(b)) {
      mismatches++;
      continue;
    }
    if (std::isnan(a))
      continue;
    finite++;
    worst = std::fmax(worst, std::fabs(a - b));
  }
  CHECK(mismatches == 0);
  CHECK(finite > 10000);
  CHECK(worst < 0.01f);
  std::printf("  temperature table: %d finite values, worst difference from reference %.5f C\n", finite, worst);

  // Physics sanity: emissivity 1, no distance, ambient = shutter temperature:
  // the raw value at the table offset reads the shutter temperature.
  Environment e1{1.0f, 25.05f, 25.05f, 0.5f, 0.0f, 0.0f};
  CHECK(table.prepare(m, e1));
  const int offset = 5000 - int(390.0f - 20.0f * 7.05f);  // 4751
  CHECK(std::fabs(table.temperature(static_cast<uint16_t>(offset)) - 25.05f) < 0.01f);
  CHECK(table.temperature(static_cast<uint16_t>(offset + 2000)) > table.temperature(static_cast<uint16_t>(offset + 1000)));
  // Lower emissivity reads a warm object warmer (less of the signal is its own).
  const float t1 = table.temperature(static_cast<uint16_t>(offset + 3000));
  e1.emissivity = 0.8f;
  CHECK(table.prepare(m, e1));
  CHECK(table.temperature(static_cast<uint16_t>(offset + 3000)) > t1);

  // Conversion of an image through the table, masking the top two bits.
  std::vector<uint16_t> raw(64);
  for (size_t i = 0; i < raw.size(); i++)
    raw[i] = static_cast<uint16_t>(5000 + 50 * i) | (i % 3 == 0 ? 0xC000 : 0);
  std::vector<float> out(raw.size());
  TemperatureTable t2;
  CHECK(raw_to_temperatures(raw.data(), raw.size(), m, env, t2, out.data()));
  bool same = true;
  for (size_t i = 0; i < raw.size(); i++)
    same = same && out[i] == t2.temperature(raw[i] & RAW_MASK);
  CHECK(same);

  // The table range starts from the camera's reported min/max and is
  // extended for pixels outside it: exact whether that range is right, too
  // narrow, too wide, or nonsense.
  {
    std::vector<uint16_t> img(2000);
    for (size_t i = 0; i < img.size(); i++)
      img[i] = static_cast<uint16_t>(4200 + (i * 37) % 4000);
    const uint16_t ranges[][2] = {{4200, 8199}, {6000, 6100}, {100, 16000}, {9000, 3000}, {20000, 30000}};
    for (const auto &rg : ranges) {
      Metadata mr = m;
      mr.min_raw = rg[0];
      mr.max_raw = rg[1];
      std::vector<float> res(img.size());
      TemperatureTable tt;
      CHECK(raw_to_temperatures(img.data(), img.size(), mr, env, tt, res.data()));
      bool exact = true;
      for (size_t i = 0; i < img.size(); i++)
        exact = exact && res[i] == tt.temperature(img[i]);
      CHECK(exact);
    }
  }

  // Unusable calibration is refused.
  Cal bad = c;
  bad.c1 = 0.0f;
  auto bf = make_frame(bad);
  const Metadata bm = parse_metadata(bf.data(), W, ROWS);
  CHECK(!bm.valid);
  CHECK(!raw_to_temperatures(raw.data(), raw.size(), bm, env, t2, out.data()));
  Cal nan_cal = c;
  nan_cal.c3 = NAN;
  auto nf = make_frame(nan_cal);
  CHECK(!parse_metadata(nf.data(), W, ROWS).valid);
}

static void test_environment() {
  const Cal c{7800, 5000, 2982, 3000, 0.8f, 3.2f, 0.0001f, -0.002f, 0.45f, 0.5f, 22.0f, 23.0f, 0.4f, 0.97f, 1.5f};
  auto frame = make_frame(c);
  Metadata m = parse_metadata(frame.data(), W, ROWS);
  // Overrides win.
  Environment o{0.9f, 30.0f, 31.0f, 0.6f, 2.0f, 0.0f};
  Environment e = resolve_environment(o, m);
  CHECK(e.emissivity == 0.9f && e.reflected == 30.0f && e.ambient == 31.0f && e.humidity == 0.6f &&
        e.distance == 2.0f && e.correction == 0.0f);
  // Implausible stored values fall back to the defaults.
  m.emissivity = 0.0f;
  m.humidity = 45.0f;  // a percentage, not a fraction
  m.ambient = NAN;
  const Environment none{NAN, NAN, NAN, NAN, NAN, NAN};
  e = resolve_environment(none, m);
  CHECK(e.emissivity == DEFAULT_ENVIRONMENT.emissivity);
  CHECK(e.humidity == DEFAULT_ENVIRONMENT.humidity);
  CHECK(e.ambient == DEFAULT_ENVIRONMENT.ambient);
  CHECK(e.correction == 0.5f && e.distance == 1.5f);
  // Narrow models have no stored environment; their calibration is one row in.
  auto narrow = make_frame(c, 256, 192);
  const Metadata nm = parse_metadata(narrow.data(), 256, 192);
  CHECK(nm.valid && !nm.has_environment && nm.cal_00 == 5000 && nm.cal_05 == 0.45f);
  e = resolve_environment(none, nm);
  CHECK(e.emissivity == DEFAULT_ENVIRONMENT.emissivity && e.distance == DEFAULT_ENVIRONMENT.distance);
  // Too narrow to hold the calibration block at all.
  std::vector<uint16_t> tiny(4 * (2 + METADATA_ROWS), 0);
  CHECK(!parse_metadata(tiny.data(), 4, 2).valid);
}

// ---------------------------------------------------------------- YUYV

static void test_yuyv() {
  // white, black, red, (full-range BT.601) as two pixel pairs each
  const uint8_t src[] = {255, 128, 255, 128, 0, 128, 0, 128, 76, 85, 76, 255, 150, 44, 150, 21};
  uint8_t dst[2 * 8 + 4];
  std::memset(dst, 0xAA, sizeof(dst));
  yuyv_to_rgb565(src, 8, 1, dst, 16);
  auto px = [&](int i) { return static_cast<uint16_t>(dst[2 * i] | (dst[2 * i + 1] << 8)); };
  CHECK(px(0) == 0xFFFF && px(1) == 0xFFFF);
  CHECK(px(2) == 0x0000 && px(3) == 0x0000);
  CHECK((px(4) >> 11) == 31 && ((px(4) >> 5) & 0x3F) <= 1 && (px(4) & 0x1F) == 0);  // red
  CHECK(dst[8] == static_cast<uint8_t>(px(4)));                                    // low byte first
  // Y=150 U=44 V=21: R = 0 (clamped), G = 255 (clamped), B = 1 -> green
  CHECK((px(6) >> 11) == 0 && ((px(6) >> 5) & 0x3F) == 63 && (px(6) & 0x1F) == 0);
  CHECK(dst[16] == 0xAA);  // nothing past the row

  // Stride: rows land at their stride.
  const uint8_t two_rows[] = {255, 128, 255, 128, 0, 128, 0, 128};
  uint8_t out[2 * 12];
  std::memset(out, 0xAA, sizeof(out));
  yuyv_to_rgb565(two_rows, 2, 2, out, 12);
  CHECK(out[0] == 0xFF && out[3] == 0xFF && out[4] == 0xAA && out[12] == 0x00 && out[15] == 0x00 && out[16] == 0xAA);
}

int main() {
  test_descriptors();
  test_probe();
  test_assembler();
  test_temperatures();
  test_environment();
  test_yuyv();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
