// Unit tests for the Thermal Master P1 core: the commands (byte for byte as
// the vendor's tool sends them, from the captures in jvdillon/p3-ir-camera,
// and as the vendor app's command library builds them), the frame markers,
// frame assembly from bulk transfers (start-up part way through a frame, the
// long frame after a shutter cycle, bad markers, no buffer, frames the camera
// skipped), temperatures in both ranges, the metadata rows, and the vendor's
// automatic range switching.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "tm_core.h"

using namespace esphome::thermal_master_p1;

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

static std::string hex(const Command &c) {
  std::string s;
  char b[3];
  for (size_t i = 0; i < c.size; i++) {
    std::snprintf(b, sizeof(b), "%02x", c[i]);
    s += b;
  }
  return s;
}

static void test_commands() {
  // As captured from the vendor's Windows tool (P3_PROTOCOL.md, issue #2).
  CHECK(hex(command(Cmd::READ_MODEL)) == "0101810001000000000000001e0000004f90");
  CHECK(hex(command(Cmd::READ_FIRMWARE)) == "0101810002000000000000000c0000001f63");
  CHECK(hex(command(Cmd::READ_SYSTEM_VERSION)) == "010181000a00000000000000400000001959");
  CHECK(hex(command(Cmd::READ_RANGE)) == "012f81000000000000000000010000004930");
  CHECK(hex(command(Cmd::RANGE_HIGH)) == "012f41000100000000000000000000004939");
  CHECK(hex(command(Cmd::SHUTTER)) == "01364300000000000000000000000000cd0b");
  // Others from the same captures, through the same builder.
  CHECK(hex(make_command(0x012F, 0x41, 0, 0)) == "012f41000000000000000000000000003c3a");  // low range
  CHECK(hex(make_command(0x0101, 0x81, 0x07, 64)) == "01018100070000000000000040000000104c");
  CHECK(hex(make_command(0x1021, 0x81, 0, 2)) == "1021810000000000000000000200000095d1");
  CHECK(command(Cmd::READ_MODEL).response_bytes() == 30);
  CHECK(command(Cmd::READ_FIRMWARE).response_bytes() == 12);
  CHECK(command(Cmd::READ_SYSTEM_VERSION).response_bytes() == 64);
  CHECK(command(Cmd::READ_RANGE).response_bytes() == 1);
  CHECK(command(Cmd::RANGE_HIGH).response_bytes() == 0);
  CHECK(command(Cmd::SHUTTER).response_bytes() == 0);
  // The time the library gives them: 22 s for a range change, 4 s for a shutter cycle.
  CHECK(command(Cmd::RANGE_HIGH).budget_ms == 22000);
  CHECK(command(Cmd::SHUTTER).budget_ms == 4000);
  CHECK(command(Cmd::READ_MODEL).budget_ms == DEFAULT_BUDGET_MS);
}

static bool crc_ok(const Command &c) {
  const size_t n = c.is_long() ? 22 : 16;
  return crc16_xmodem(c.data(), n) == (c[n] | (c[n + 1] << 8));
}

// The vendor library's command set (Thermal Master 2.3.11, libircmd020).
static void test_command_set() {
  // The captures and the library build the same bytes.
  CHECK(setting_write(Setting::RANGE, 1) == command(Cmd::RANGE_HIGH));
  CHECK(setting_write(Setting::RANGE, 1).budget_ms == 22000);
  CHECK(hex(setting_write(Setting::RANGE, 0)) == "012f41000000000000000000000000003c3a");
  CHECK(setting_read(Setting::RANGE) == command(Cmd::READ_RANGE));
  CHECK(action_command(Action::SHUTTER) == command(Cmd::SHUTTER));
  CHECK(action_command(Action::SHUTTER).budget_ms == 4000);
  CHECK(info_command(Info::SERIAL) == make_command(0x0101, 0x81, 0x07, 64));
  CHECK(reading_command(Reading::SYSTEM_DATA_FLAG) == make_command(0x1021, 0x81, 0, 2));

  // An id 0xAABBCC: b0 = CC, b1 = BB, b2 = AA.
  const Command br = setting_write(Setting::BRIGHTNESS, 150);  // clamped to 100
  CHECK(br.size == 18 && br[0] == 0x10 && br[1] == 0x04 && br[2] == 0x47 && br[3] == 0);
  CHECK(br[4] == 100 && br[5] == 0 && br.response_bytes() == 0 && crc_ok(br));
  // The palette, the isothermal mode and switch: byte 5.
  const Command pal = setting_write(Setting::PALETTE, 7);
  CHECK(pal[0] == 0x10 && pal[1] == 0x03 && pal[2] == 0x45 && pal[4] == 0 && pal[5] == 7 && crc_ok(pal));
  CHECK(setting_write(Setting::ISOTHERMAL_MODE, 3)[5] == 3 && setting_write(Setting::ISOTHERMAL_MODE, 3)[4] == 0);
  CHECK(setting_write(Setting::ISOTHERMAL, 1)[5] == 1 && setting_write(Setting::ISOTHERMAL, 1).budget_ms == 7000);
  // 16 bits at b4-5: emissivity (1..16384) and the kelvin temperatures (233..373).
  const Command em = setting_write(Setting::EMISSIVITY, 15565);
  CHECK(em[0] == 0x10 && em[1] == 0x07 && em[2] == 0x44 && em[4] == (15565 & 0xFF) && em[5] == (15565 >> 8));
  CHECK(crc_ok(em));
  CHECK(setting_write(Setting::EMISSIVITY, 0)[4] == 1);  // at least 1
  const Command ta = setting_write(Setting::AMBIENT, 400);  // at most 373 K
  CHECK(ta[2] == 0x43 && ta[4] == (373 & 0xFF) && ta[5] == (373 >> 8));
  CHECK(setting_write(Setting::REFLECTED, 298)[2] == 0x42);
  // The auto shutter parameters: the type at b4, the value at b5-6.
  const Command th = setting_write(Setting::AUTO_SHUTTER_MIN_INTERVAL, 0x1234);
  CHECK(th[0] == 0x10 && th[1] == 0x02 && th[2] == 0x42 && th[4] == 1 && th[5] == 0x34 && th[6] == 0x12);
  CHECK(setting_write(Setting::AUTO_SHUTTER_MAX_INTERVAL, 2)[5] == 5);  // more than 4
  // Pairs: sun detection (min, max at b4, b5), isothermal limits (two int16 at b8 and b10).
  const int32_t ratio = pack_pair(Setting::SUN_DETECT_PIXEL_RATIO, 5, 120);  // max clamped to 100
  const Command sd = setting_write(Setting::SUN_DETECT_PIXEL_RATIO, ratio);
  CHECK(sd[2] == 0x49 && sd[4] == 5 && sd[5] == 100 && crc_ok(sd));
  const int32_t lim = pack_pair(Setting::ISOTHERMAL_LIMIT, -20, 120);
  CHECK(pair_part(Setting::ISOTHERMAL_LIMIT, lim, 0) == -20 && pair_part(Setting::ISOTHERMAL_LIMIT, lim, 1) == 120);
  const Command il = setting_write(Setting::ISOTHERMAL_LIMIT, lim);
  CHECK(il[2] == 0x47 && il[4] == 0 && il[8] == 0xEC && il[9] == 0xFF && il[10] == 120 && il[11] == 0);
  CHECK(pair_part(Setting::ISOTHERMAL_LIMIT, setting_clamp(Setting::ISOTHERMAL_LIMIT,
                                                            pack_pair(Setting::ISOTHERMAL_LIMIT, -100, 900)), 0) == -70);
  // The camera's frame temperature display: the 24-byte header.
  const Command fd = setting_write(Setting::FRAME_TEMP_DISPLAY, 1);
  CHECK(fd.size == 24 && fd[0] == 0x03 && fd[1] == 0x02 && fd[2] == 0x51 && fd[4] == 1 && crc_ok(fd));

  // Reads: b4 = 0, except the picture levels' "current" (1) and the auto
  // shutter parameters' type (with b5 = 1).
  const Command mf = setting_read(Setting::MIRROR_FLIP);
  CHECK(mf[0] == 0x10 && mf[1] == 0x10 && mf[2] == 0x83 && mf[4] == 0 && mf.response_bytes() == 1 && crc_ok(mf));
  CHECK(setting_read(Setting::BRIGHTNESS)[4] == 1 && setting_read(Setting::BRIGHTNESS)[2] == 0x87);
  CHECK(setting_read(Setting::SPATIAL_NOISE_REDUCTION)[2] == 0x8B && setting_read(Setting::SPATIAL_NOISE_REDUCTION)[4] == 1);
  const Command ap = setting_read(Setting::AUTO_SHUTTER_MAX_INTERVAL);
  CHECK(ap[2] == 0x82 && ap[4] == 2 && ap[5] == 1 && ap.response_bytes() == 2);
  CHECK(setting_read(Setting::EMISSIVITY).response_bytes() == 2);
  CHECK(setting_read(Setting::ISOTHERMAL_LIMIT).response_bytes() == 4);
  CHECK(setting_read(Setting::SUN_DETECT_ROUNDNESS).response_bytes() == 2);
  CHECK(setting_read(Setting::SHUTTER_OPEN)[2] == 0x83 && setting_read(Setting::SHUTTER_OPEN)[1] == 0x02);
  CHECK(setting_info(Setting::PROFESSION_MODE).get == 0);
  const uint8_t two[4] = {0xB6, 0x03, 0x78, 0x00};
  CHECK(setting_value(Setting::EMISSIVITY, two, 2) == 950);
  CHECK(setting_value(Setting::EMISSIVITY, two, 1) == UNKNOWN);
  CHECK(setting_value(Setting::MIRROR_FLIP, two, 1) == 0xB6);
  CHECK(setting_value(Setting::MIRROR_FLIP, two, 0) == UNKNOWN);
  CHECK(setting_value(Setting::PROFESSION_MODE, two, 1) == UNKNOWN);  // no read
  const int32_t pr = setting_value(Setting::SUN_DETECT_PIXEL_RATIO, two, 2);
  CHECK(pair_part(Setting::SUN_DETECT_PIXEL_RATIO, pr, 0) == 0xB6 && pair_part(Setting::SUN_DETECT_PIXEL_RATIO, pr, 1) == 3);
  const uint8_t lims[4] = {0xEC, 0xFF, 0x78, 0x00};
  const int32_t lv = setting_value(Setting::ISOTHERMAL_LIMIT, lims, 4);
  CHECK(pair_part(Setting::ISOTHERMAL_LIMIT, lv, 0) == -20 && pair_part(Setting::ISOTHERMAL_LIMIT, lv, 1) == 120);

  // The camera's limits for a setting.
  CHECK(has_attributes(Setting::BRIGHTNESS) && !has_attributes(Setting::PALETTE));
  const Command at = attribute_read(Setting::CONTRAST);
  CHECK(at[2] == 0x88 && at[4] == 2 && at.response_bytes() == 6);
  const Command af = attribute_read(Setting::AUTO_SHUTTER_THRESHOLD);
  CHECK(af[2] == 0x82 && af[4] == 0 && af[5] == 2 && af.response_bytes() == 6);
  const uint8_t ar[6] = {100, 0, 0, 0, 1, 0};
  Attributes a{};
  CHECK(parse_attributes(ar, 6, &a) && a.max == 100 && a.min == 0 && a.step == 1);
  CHECK(!parse_attributes(ar, 5, &a));

  // Actions: resets go without a status read; arguments at b4.
  CHECK(action_command(Action::REBOOT).no_status && action_command(Action::REBOOT)[2] == 0x41 &&
        action_command(Action::REBOOT)[0] == 0x01);
  CHECK(action_command(Action::RESET_TO_ROM).no_status && action_command(Action::RESTORE_SETTINGS).no_status);
  CHECK(!action_command(Action::SAVE_SETTINGS).no_status);
  CHECK(action_command(Action::SHUTTER_BOTH)[4] == 2 && action_command(Action::SHUTTER_OOC)[4] == 1);
  CHECK(action_command(Action::K_CALIBRATE_HIGH)[2] == 0x41 && action_command(Action::K_CALIBRATE_HIGH)[4] == 1);
  CHECK(action_command(Action::DPC_AUTO).budget_ms == 12000 && action_command(Action::RMCOVER_AUTO).budget_ms == 32000);

  // Commands with values.
  const Command cp = cursor_position(0x0102, 0x0304);
  CHECK(cp[2] == 0x58 && cp[4] == 0x02 && cp[5] == 0x01 && cp[6] == 0x04 && cp[7] == 0x03 && crc_ok(cp));
  CHECK(mark_dead_pixel(3, 4)[3] == 0 && unmark_dead_pixel(3, 4)[3] == 1 && unmark_dead_pixel(3, 4)[6] == 4);
  const Command tp = tpd_one_point(-5);
  CHECK(tp[2] == 0x71 && tp[4] == 0xFB && tp[5] == 0xFF && tp.budget_ms == 7000 && crc_ok(tp));
  const Command t2 = tpd_two_point(1, 300);
  CHECK(t2[2] == 0x76 && t2[3] == 3 && t2[4] == (300 & 0xFF) && t2[8] == 1 && crc_ok(t2));
  const Command vw = vl_write(0x12345678, 0x9ABCDEF0);
  CHECK(vw[4] == 0x78 && vw[7] == 0x12 && vw[8] == 0xF0 && vw[11] == 0x9A && crc_ok(vw));
  const Command vr = vl_read(0x1234);
  CHECK(vr[2] == 0x9C && vr[4] == 0x34 && vr[5] == 0x12 && vr.response_bytes() == 1 && crc_ok(vr));

  // Readings. The camera's own measurements use the 24-byte header.
  const Command pt = reading_command(Reading::CAMERA_POINT, 80, 60);
  CHECK(pt.size == 24 && pt[0] == 0x03 && pt[1] == 0x02 && pt[2] == 0x82 && pt[4] == 80 && pt[6] == 60);
  CHECK(pt.response_bytes() == 2 && pt[16] == 2 && pt[12] == 0 && crc_ok(pt));
  CHECK(reading_command(Reading::CAMERA_FRAME).is_long() && reading_command(Reading::CAMERA_FRAME).response_bytes() == 12);
  CHECK(reading_command(Reading::DEVICE_STATUS).response_bytes() == 1);
  CHECK(reading_command(Reading::DEVICE_TEMPERATURE)[2] == 0x91);
  CHECK(std::string(device_status_name(2)) == "Preview" && std::string(device_status_name(5)) == "Start-up");
  CHECK(std::string(device_status_name(9)) == "Error");
  CHECK(std::string(shutter_status_name(0)) == "Closed" && std::string(shutter_status_name(1)) == "Open");
  CHECK(std::string(stream_source_name(0x16)) == "Picture" && std::string(stream_source_name(0x0A)) == "SNR");
  // The frame's minimum and maximum: T(b0-1) at (b4-7), T(b2-3) at (b8-11).
  const uint16_t cold = static_cast<uint16_t>(std::lround((20.0f + 273.15f) * 64));
  const uint16_t warm = static_cast<uint16_t>(std::lround((36.0f + 273.15f) * 64));
  const uint8_t fi[12] = {static_cast<uint8_t>(cold), static_cast<uint8_t>(cold >> 8), static_cast<uint8_t>(warm),
                          static_cast<uint8_t>(warm >> 8), 3, 0, 4, 0, 80, 0, 60, 0};
  FrameInfo info{};
  CHECK(parse_frame_info(fi, 12, &info));
  CHECK(NEAR(info.max_c, 36.0f, 0.01f) && info.max_x == 80 && info.max_y == 60);
  CHECK(NEAR(info.min_c, 20.0f, 0.01f) && info.min_x == 3 && info.min_y == 4);
  CHECK(!parse_frame_info(fi, 11, &info));

  // Device information and palettes.
  CHECK(info_command(Info::NAME) == command(Cmd::READ_MODEL));
  CHECK(info_command(Info::PACKAGE_VERSION).response_bytes() == 50 && info_command(Info::PACKAGE_VERSION)[4] == 13);
  CHECK(info_command(Info::CUSTOMER_ID).response_bytes() == 10);
  CHECK(palette_count_command()[5] == 0xFF && palette_count_command()[2] == 0x83 && palette_count_command()[1] == 0x32);
  CHECK(palette_name_command(4)[5] == 4 && palette_name_command(4).response_bytes() == 10);

  // Every command well formed; names unique; the maintenance ones marked.
  std::vector<std::string> names;
  for (int i = 0; i < static_cast<int>(Setting::COUNT); i++) {
    const Setting s = static_cast<Setting>(i);
    CHECK(crc_ok(setting_write(s, setting_info(s).min)));
    if (setting_info(s).get != 0)
      CHECK(crc_ok(setting_read(s)) && setting_read(s).response_bytes() == setting_read_bytes(s));
    names.push_back(setting_info(s).name);
  }
  for (int i = 0; i < static_cast<int>(Action::COUNT); i++) {
    const Action a = static_cast<Action>(i);
    CHECK(crc_ok(action_command(a)) && action_command(a).response_bytes() == 0);
    names.push_back(action_name(a));
    CHECK(action_is_maintenance(a) == (a >= Action::REBOOT));
  }
  for (int i = 0; i < static_cast<int>(Reading::COUNT); i++)
    CHECK(crc_ok(reading_command(static_cast<Reading>(i))) &&
          reading_command(static_cast<Reading>(i)).response_bytes() > 0);
  std::vector<std::string> sorted = names;
  std::sort(sorted.begin(), sorted.end());
  CHECK(std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end());
}

static void test_picture() {
  std::vector<uint8_t> f(FRAME_BYTES, 0x80);  // grey everywhere
  for (int x = 0; x < W; x += 2) {         // a red pair at the top left: V high
    uint8_t *p = f.data() + MARKER_BYTES + 2 * x;
    if (x == 0) {
      p[0] = 76;
      p[1] = 85;
      p[2] = 76;
      p[3] = 255;
    }
  }
  std::vector<uint8_t> out(static_cast<size_t>(W) * H * 2);
  picture_to_rgb565(f.data(), out.data(), static_cast<size_t>(W) * 2);
  auto px = [&](int x, int y) { return static_cast<uint16_t>(out[(y * W + x) * 2] | (out[(y * W + x) * 2 + 1] << 8)); };
  CHECK(px(5, 5) == 0x8410);         // (128, 128, 128)
  CHECK((px(0, 0) >> 11) >= 28);     // mostly red
  CHECK(((px(0, 0) >> 5) & 0x3F) < 8);
  CHECK((px(0, 0) & 0x1F) < 4);
}

static void test_geometry() {
  CHECK(ROWS_BYTES == 77440);
  CHECK(FRAME_BYTES == 77464);
  CHECK(BODY_BYTES % 512 != 0);  // the rows end with a short packet
}

static void test_markers() {
  // A captured start marker: (12, 141, 601097, 607324, 40).
  const uint8_t b[12] = {0x0c, 0x8d, 0x09, 0x2c, 0x09, 0x00, 0x5c, 0x44, 0x09, 0x00, 0x28, 0x00};
  const Marker m = parse_marker(b);
  CHECK(m.length == 12);
  CHECK(m.sync == 0x8D);
  CHECK(m.frame == 601097);
  CHECK(m.time == 607324);
  CHECK(m.phase == 40);
  CHECK(is_start_marker(m));
  CHECK(!is_end_marker(m));
  Marker e = m;
  e.sync = 0x8F;
  CHECK(is_end_marker(e));
  CHECK(!is_start_marker(e));
}

// ---------------------------------------------------------------- assembly
static void put_marker(uint8_t *p, uint8_t sync, uint32_t frame, uint16_t phase) {
  p[0] = 12;
  p[1] = sync;
  std::memcpy(p + 2, &frame, 4);
  const uint32_t t = frame + 6000;
  std::memcpy(p + 6, &t, 4);
  std::memcpy(p + 10, &phase, 2);
}

// Frame k as the camera sends it: start marker, rows (every temperature
// `celsius` + k / 100), end marker.
static std::vector<uint8_t> camera_frame(uint32_t k, uint16_t phase, float celsius = 25.0f) {
  std::vector<uint8_t> f(FRAME_BYTES, 0);
  const uint8_t odd = k & 1;
  put_marker(f.data(), 0x8C | odd, 600000 * k, phase);
  for (size_t i = MARKER_BYTES; i < BODY_BYTES; i += 2) {
    f[i] = static_cast<uint8_t>(k);  // the picture and metadata rows: anything
    f[i + 1] = 0x80;
  }
  const uint16_t raw = static_cast<uint16_t>(std::lround((celsius + k / 100.0f + 273.15f) * 64));
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++) {
      uint8_t *p = f.data() + MARKER_BYTES + 2 * (static_cast<size_t>(TEMP_ROW + y) * W + x);
      p[0] = static_cast<uint8_t>(raw);
      p[1] = static_cast<uint8_t>(raw >> 8);
    }
  put_marker(f.data() + BODY_BYTES, 0x8E | odd, 600000 * k, phase + 40);
  return f;
}

// The transfers a host with `chunk`-byte bulk transfers gets for `bytes` of
// stream: whole transfers, then the rest, which ends with a short packet.
static void split(const uint8_t *p, size_t bytes, size_t chunk, std::vector<std::vector<uint8_t>> *out) {
  for (size_t off = 0; off < bytes; off += chunk)
    out->emplace_back(p + off, p + std::min(bytes, off + chunk));
}

static std::vector<std::vector<uint8_t>> transfers_of(const std::vector<uint8_t> &f, size_t chunk = 8192) {
  std::vector<std::vector<uint8_t>> t;
  split(f.data(), BODY_BYTES, chunk, &t);
  t.emplace_back(f.data() + BODY_BYTES, f.data() + FRAME_BYTES);  // the end marker, alone
  return t;
}

struct Run {
  std::vector<Assembler::Result> results;
  int frames{0};
  std::vector<uint8_t> last;
};

static Run feed_all(Assembler &a, const std::vector<std::vector<uint8_t>> &transfers, bool buffer = true) {
  Run r;
  std::vector<uint8_t> dst(FRAME_BYTES);
  for (const auto &t : transfers) {
    const Assembler::Result res = a.feed(t.data(), t.size(), buffer ? dst.data() : nullptr);
    if (res != Assembler::Result::PART)
      r.results.push_back(res);
    if (res == Assembler::Result::FRAME) {
      r.frames++;
      r.last = dst;
    }
  }
  return r;
}

static void test_assembly() {
  // Whole frames, two transfer sizes.
  for (size_t chunk : {size_t(8192), size_t(16384)}) {
    Assembler a;
    std::vector<std::vector<uint8_t>> t;
    for (uint32_t k = 0; k < 3; k++)
      for (auto &x : transfers_of(camera_frame(k, 40 * k), chunk))
        t.push_back(x);
    const Run r = feed_all(a, t);
    CHECK(r.frames == 3);
    CHECK(r.results.size() == 3);
    CHECK(r.last == camera_frame(2, 80));
    CHECK(a.camera_skipped() == 0);
  }

  // Started part way through a frame: its end marker comes early, then frames.
  {
    Assembler a;
    std::vector<std::vector<uint8_t>> t;
    auto f0 = transfers_of(camera_frame(0, 0));
    t.insert(t.end(), f0.begin() + 4, f0.end());
    for (auto &x : transfers_of(camera_frame(1, 40)))
      t.push_back(x);
    const Run r = feed_all(a, t);
    CHECK(r.frames == 1);
    CHECK(r.results.size() == 2);
    CHECK(r.results[0] == Assembler::Result::SHORT);
    CHECK(r.last == camera_frame(1, 40));
  }

  // Started with only an end marker to come: nothing to report.
  {
    Assembler a;
    auto f0 = transfers_of(camera_frame(0, 0));
    std::vector<std::vector<uint8_t>> t = {f0.back()};
    for (auto &x : transfers_of(camera_frame(1, 40)))
      t.push_back(x);
    const Run r = feed_all(a, t);
    CHECK(r.frames == 1);
    CHECK(r.results.size() == 1);
  }

  // After a shutter cycle the camera sends a longer frame, with a second start
  // marker in it (as on the P3: a partial frame, then the frame). It is
  // dropped once; the next frame is kept.
  {
    Assembler a;
    std::vector<uint8_t> extra(5760, 0x11);  // 36 rows of a partial frame
    std::vector<uint8_t> f1 = camera_frame(1, 40);
    std::vector<uint8_t> longer(f1.begin(), f1.begin() + BODY_BYTES);
    longer.insert(longer.begin() + MARKER_BYTES, extra.begin(), extra.end());
    std::vector<std::vector<uint8_t>> t;
    for (auto &x : transfers_of(camera_frame(0, 0)))
      t.push_back(x);
    split(longer.data(), longer.size(), 8192, &t);
    t.emplace_back(f1.begin() + BODY_BYTES, f1.end());
    for (auto &x : transfers_of(camera_frame(2, 80)))
      t.push_back(x);
    const Run r = feed_all(a, t);
    CHECK(r.frames == 2);
    CHECK(r.results.size() == 3);
    CHECK(r.results[1] == Assembler::Result::LONG);
    CHECK(r.last == camera_frame(2, 80));
  }

  // The right size but markers from different frames.
  {
    Assembler a;
    std::vector<uint8_t> f = camera_frame(4, 160);
    std::vector<uint8_t> other = camera_frame(5, 200);
    std::memcpy(f.data() + BODY_BYTES, other.data() + BODY_BYTES, MARKER_BYTES);
    const Run r = feed_all(a, transfers_of(f));
    CHECK(r.frames == 0);
    CHECK(r.results.size() == 1 && r.results[0] == Assembler::Result::BAD_MARKER);
  }

  // Not a marker at the start (bytes lost at the start of a frame).
  {
    Assembler a;
    std::vector<uint8_t> f = camera_frame(4, 160);
    f[0] = 0;
    const Run r = feed_all(a, transfers_of(f));
    CHECK(r.results.size() == 1 && r.results[0] == Assembler::Result::BAD_MARKER);
  }

  // No buffer: the stream is followed, the frame is counted as lost.
  {
    Assembler a;
    std::vector<std::vector<uint8_t>> t;
    for (uint32_t k = 0; k < 2; k++)
      for (auto &x : transfers_of(camera_frame(k, 40 * k)))
        t.push_back(x);
    const Run r = feed_all(a, t, false);
    CHECK(r.frames == 0);
    CHECK(r.results.size() == 2);
    CHECK(r.results[0] == Assembler::Result::NO_BUFFER && r.results[1] == Assembler::Result::NO_BUFFER);
  }

  // Frames the camera made that never came: the phase steps by 40 a frame and
  // wraps at 2048.
  {
    Assembler a;
    std::vector<std::vector<uint8_t>> t;
    const uint16_t phases[] = {2000, 2040, 32, 152, 192};  // 2040 -> 32 is one step; 32 -> 152 skips two
    uint32_t k = 0;
    for (uint16_t p : phases)
      for (auto &x : transfers_of(camera_frame(k++, p)))
        t.push_back(x);
    const Run r = feed_all(a, t);
    CHECK(r.frames == 5);
    CHECK(a.camera_skipped() == 2);
  }
}

// ---------------------------------------------------------------- temperatures
static void test_temperatures() {
  std::vector<uint8_t> f = camera_frame(0, 0, 25.0f);
  auto set = [&](int x, int y, uint16_t raw) {
    uint8_t *p = f.data() + MARKER_BYTES + 2 * (static_cast<size_t>(TEMP_ROW + y) * W + x);
    p[0] = static_cast<uint8_t>(raw);
    p[1] = static_cast<uint8_t>(raw >> 8);
  };
  const uint16_t hot = static_cast<uint16_t>(std::lround((100.0f + 273.15f) * 64));
  const uint16_t cold = static_cast<uint16_t>(std::lround((-10.0f + 273.15f) * 64));
  set(17, 3, hot);
  set(150, 110, cold);
  std::vector<float> t(PIXELS);
  esphome::video::FrameStats st;
  CHECK(decode(f.data(), 1.0f, 0.0f, t.data(), &st) == 0);
  CHECK(NEAR(t[0], 25.0f, 0.01f));
  CHECK(NEAR(t[3 * W + 17], 100.0f, 0.01f));
  CHECK(st.valid);
  CHECK(NEAR(st.max, 100.0f, 0.01f) && st.max_x == 17 && st.max_y == 3);
  CHECK(NEAR(st.min, -10.0f, 0.01f) && st.min_x == 150 && st.min_y == 110);
  CHECK(NEAR(raw_to_celsius(static_cast<uint16_t>(std::lround(273.15f * 64))), 0.0f, 0.01f));

  // Gain and offset.
  CHECK(decode(f.data(), 1.1f, -2.0f, t.data(), nullptr) == 0);
  CHECK(NEAR(t[0], 25.0f * 1.1f - 2.0f, 0.01f));

  // Pixels with no reading take a neighbour's value: the left one, at the left
  // edge the one above, at the top left the first reading.
  set(0, 0, 0);
  set(6, 0, 5);
  set(0, 3, 100);
  set(18, 3, 0);
  CHECK(decode(f.data(), 1.0f, 0.0f, t.data(), &st) == 4);
  CHECK(NEAR(t[0], 25.0f, 0.01f));
  CHECK(NEAR(t[6], 25.0f, 0.01f));
  CHECK(NEAR(t[3 * W], 25.0f, 0.01f));
  CHECK(NEAR(t[3 * W + 18], 100.0f, 0.01f));

  // No readings at all (a room-temperature scene in the low range reads
  // about 57 K): in the high range, not usable.
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      set(x, y, 57 * 64);
  CHECK(decode(f.data(), 1.0f, 0.0f, t.data(), &st) == PIXELS);
  CHECK(!st.valid);

  // In the low range, values under 150 C read 150 C and are counted.
  const uint16_t c300 = static_cast<uint16_t>(std::lround((300.0f + 273.15f) * 64));
  set(40, 50, c300);
  CHECK(decode(f.data(), 1.0f, 0.0f, t.data(), &st, true) == PIXELS - 1);
  CHECK(st.valid && NEAR(st.max, 300.0f, 0.02f) && st.max_x == 40 && st.max_y == 50);
  CHECK(NEAR(st.min, 150.0f, 0.001f) && NEAR(t[0], 150.0f, 0.001f));
  CHECK(decode(f.data(), 1.0f, 5.0f, t.data(), &st, true) == PIXELS - 1 && NEAR(t[0], 155.0f, 0.001f));

  // The metadata rows are readable.
  std::vector<uint8_t> g = camera_frame(9, 0);
  CHECK(metadata_word(g.data(), 0) == 0x8009);
  CHECK(metadata_word(g.data(), 2 * W - 1) == 0x8009);
}

static void test_info_rows() {
  std::vector<uint8_t> f = camera_frame(0, 0);
  uint8_t *p = f.data() + MARKER_BYTES + 2 * static_cast<size_t>(META_ROW) * W;
  CHECK(info_bytes(f.data()) == p);
  p[42] = 17;
  p[82] = 0;
  p[138] = 1;
  p[128] = 0x34;
  p[129] = 0x12;
  p[360] = 0x78;
  p[361] = 0x56;
  const InfoRows r = parse_info_rows(f.data());
  CHECK(r.ffc_countdown == 17 && r.sun_protect == 0 && r.hw_sun_protect == 1);
  CHECK(r.frame_count == 0x1234 && r.frame_count2 == 0x5678);
  CHECK(INFO_BYTES == 640);
}

// The vendor library's automatic range switching.
static void test_auto_range() {
  std::vector<uint8_t> cool = camera_frame(0, 0, 25.0f);
  std::vector<uint8_t> hot = camera_frame(0, 0, 25.0f);
  // 20 % of the pixels at 300 C.
  const uint16_t c300 = static_cast<uint16_t>(std::lround((300.0f + 273.15f) * 64));
  for (int y = 0; y < H / 5; y++)
    for (int x = 0; x < W; x++) {
      uint8_t *q = hot.data() + MARKER_BYTES + 2 * (static_cast<size_t>(TEMP_ROW + y) * W + x);
      q[0] = static_cast<uint8_t>(c300);
      q[1] = static_cast<uint8_t>(c300 >> 8);
    }
  // 7 % hot: neither more than 95 % under 170 C nor more than 10 % over.
  std::vector<uint8_t> some = cool;
  for (int i = 0; i < PIXELS * 7 / 100; i++) {
    uint8_t *q = some.data() + MARKER_BYTES + 2 * (static_cast<size_t>(TEMP_ROW) * W + i);
    q[0] = static_cast<uint8_t>(c300);
    q[1] = static_cast<uint8_t>(c300 >> 8);
  }
  CHECK(AutoRange::vote(cool.data()) == AutoRange::HIGH);
  CHECK(AutoRange::vote(hot.data()) == AutoRange::LOW);
  CHECK(AutoRange::vote(some.data()) == AutoRange::NONE);

  AutoRange a;
  // 125 hot frames in the high range: nothing yet; the 126th: the low range.
  int got = -1;
  for (uint32_t i = 0; i < AutoRange::SWITCH_FRAMES; i++)
    got = std::max(got, a.feed(hot.data(), 1));
  CHECK(got == -1);
  CHECK(a.feed(hot.data(), 1) == 0);
  // Then 176 frames are not looked at.
  got = -1;
  for (uint32_t i = 0; i <= AutoRange::WAIT_FRAMES; i++)
    got = std::max(got, a.feed(cool.data(), 0));
  CHECK(got == -1);
  // A frame with no vote starts the count again.
  a.reset();
  for (uint32_t i = 0; i < AutoRange::SWITCH_FRAMES; i++)
    a.feed(cool.data(), 0);
  a.feed(some.data(), 0);
  got = -1;
  for (uint32_t i = 0; i < AutoRange::SWITCH_FRAMES; i++)
    got = std::max(got, a.feed(cool.data(), 0));
  CHECK(got == -1);
  CHECK(a.feed(cool.data(), 0) == 1);
  // Already in the range the votes want: no change, but the wait still comes.
  a.reset();
  for (uint32_t i = 0; i <= AutoRange::SWITCH_FRAMES; i++)
    got = a.feed(cool.data(), 1);
  CHECK(got == -1);
  CHECK(a.feed(hot.data(), 1) == -1);
  // The range not known: the one the votes want.
  a.reset();
  for (uint32_t i = 0; i <= AutoRange::SWITCH_FRAMES; i++)
    got = a.feed(hot.data(), -1);
  CHECK(got == 0);
}

int main() {
  test_commands();
  test_command_set();
  test_picture();
  test_geometry();
  test_markers();
  test_assembly();
  test_temperatures();
  test_info_rows();
  test_auto_range();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
