// Thermal Master P1 on a Linux host, through libusb: the ESP32-P4 component's
// start-up sequence, with every bulk transfer fed to the same tm_core
// assembler. For checking a camera before (or without) a P4, and for saving
// real frames.
//
// Sends only what the component sends, with the vendor library's handshake
// (tm_core.h): the reads of its start-up sequence, the range read, the high
// range; with --shutter-at one shutter cycle; with --settings a read of every
// setting that has one, and the camera's limits where it has them; with
// --readings every reading, the device information and the palette names;
// with --set NAME=VALUE or NAME=FIRST,SECOND for a pair (repeatable) that
// setting written (as the camera takes it: tm_core's SETTINGS names and raw
// values), then read back. All of them come from tm_core; the maintenance
// commands (stored settings, calibration, dead pixels, firmware mode) cannot
// be sent.
//
// Build, from the repository root (libusb.h from libusb 1.0.2x if the
// system has no development package), as one command:
//   g++ -std=gnu++17 -O2 -Icomponents/video_core -Icomponents/thermal_master_p1
//       -I<directory holding libusb-1.0/libusb.h> tests/host/tm_capture.cpp
//       components/thermal_master_p1/tm_core.cpp -l:libusb-1.0.so.0 -o tm_capture
// Run (root, or a udev rule for 3474:45c2):
//   ./tm_capture [--seconds S] [--out DIR] [--shutter-at S] [--all-raw] [--transfers N]
//                [--settings] [--readings] [--set NAME=VALUE[,SECOND] ...]
// Prints the USB descriptors, the camera's answers, then one line a second.
// DIR gets frame.raw (the last whole frame as sent), temps.f32 (its 160x120
// temperatures, degrees C), temps.pgm (the same, 8-bit, stretched),
// picture.ppm (the camera's own picture, rows 0..119 as YUYV), frames.txt
// (one line a frame: marker frame counter, time and phase, start sync, the
// centre temperature, min, max, pixels without a reading, the metadata rows'
// shutter countdown, sun flags and frame counters, and the first 8
// metadata words), and with --all-raw every whole frame appended to all.raw.

#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "tm_core.h"

using namespace esphome::thermal_master_p1;
using Clock = std::chrono::steady_clock;

static volatile std::sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

static double since(Clock::time_point t0) {
  return std::chrono::duration<double>(Clock::now() - t0).count();
}

static void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

struct Camera {
  libusb_device_handle *h{nullptr};
  uint8_t last_status{0};
  double last_busy_s{0};  // how long the last status read said busy

  // One status read; with `poll`, read again while it is 1 (busy) for up to
  // `budget_ms`. True if it ends at `expect`.
  bool status(uint8_t expect, unsigned budget_ms) {
    const Clock::time_point t0 = Clock::now();
    for (;;) {
      uint8_t b = 0;
      const int r = libusb_control_transfer(this->h, 0xC1, 0x22, 0, 0, &b, 1, budget_ms);
      if (r != 1) {
        std::fprintf(stderr, "status read: %s\n", r < 0 ? libusb_error_name(r) : "short");
        return false;
      }
      this->last_status = b;
      this->last_busy_s = since(t0);
      if (b != STATUS_BUSY)
        break;
      if (since(t0) * 1000 > budget_ms) {
        std::fprintf(stderr, "status: still busy after %u ms\n", budget_ms);
        return false;
      }
      sleep_ms(1);
    }
    if (this->last_status != expect) {
      std::fprintf(stderr, "status %u, not %u\n", this->last_status, expect);
      return false;
    }
    return true;
  }
  bool run(Cmd c, std::vector<uint8_t> *resp = nullptr) { return this->run(command(c), resp); }
  // The component's handshake (the vendor library's): the command; a write's
  // status until not busy, then 3; a read's status (2), response, status (3);
  // a reset's nothing.
  bool run(const Command &cmd, std::vector<uint8_t> *resp = nullptr) {
    int r = libusb_control_transfer(this->h, 0x41, 0x20, 0, 0, const_cast<uint8_t *>(cmd.data()), cmd.size, 2000);
    if (r != static_cast<int>(cmd.size)) {
      std::fprintf(stderr, "command: %s\n", r < 0 ? libusb_error_name(r) : "short");
      return false;
    }
    if (cmd.no_status)
      return true;
    const uint16_t n = cmd.response_bytes();
    if (n == 0)
      return this->status(STATUS_DONE, cmd.budget_ms);
    if (!this->status(STATUS_READY, cmd.budget_ms))
      return false;
    std::vector<uint8_t> buf(n);
    r = libusb_control_transfer(this->h, 0xC1, 0x21, 0, 0, buf.data(), n, 2000);
    if (r < 0) {
      std::fprintf(stderr, "response: %s\n", libusb_error_name(r));
      return false;
    }
    buf.resize(r);
    if (resp != nullptr)
      *resp = buf;
    return this->status(STATUS_DONE, cmd.budget_ms);
  }
};

static std::string hexs(const std::vector<uint8_t> &b) {
  std::string s;
  char t[4];
  for (uint8_t c : b) {
    std::snprintf(t, sizeof(t), "%02x", c);
    s += t;
  }
  return s;
}

static std::string text(const std::vector<uint8_t> &b) {
  std::string s;
  for (uint8_t c : b) {
    if (c == 0)
      break;
    if (c >= 0x20 && c < 0x7F)
      s += static_cast<char>(c);
  }
  return s;
}

static void print_descriptors(libusb_device *dev) {
  libusb_device_descriptor dd{};
  libusb_get_device_descriptor(dev, &dd);
  const int speed = libusb_get_device_speed(dev);
  std::printf("USB %04x:%04x, class 0x%02x, speed %s, %u configuration(s)\n", dd.idVendor, dd.idProduct,
              dd.bDeviceClass,
              speed == LIBUSB_SPEED_HIGH ? "high (480 Mb/s)" : speed == LIBUSB_SPEED_FULL ? "full (12 Mb/s)" : "other",
              dd.bNumConfigurations);
  libusb_config_descriptor *cfg = nullptr;
  if (libusb_get_active_config_descriptor(dev, &cfg) != 0)
    return;
  for (int i = 0; i < cfg->bNumInterfaces; i++) {
    const libusb_interface &itf = cfg->interface[i];
    for (int a = 0; a < itf.num_altsetting; a++) {
      const libusb_interface_descriptor &d = itf.altsetting[a];
      std::printf("  interface %u alt %u: class 0x%02x/0x%02x, %u endpoint(s)", d.bInterfaceNumber,
                  d.bAlternateSetting, d.bInterfaceClass, d.bInterfaceSubClass, d.bNumEndpoints);
      for (int e = 0; e < d.bNumEndpoints; e++) {
        const libusb_endpoint_descriptor &ep = d.endpoint[e];
        const char *type[] = {"control", "isochronous", "bulk", "interrupt"};
        std::printf(" [0x%02x %s %u]", ep.bEndpointAddress, type[ep.bmAttributes & 3], ep.wMaxPacketSize);
      }
      std::printf("\n");
    }
  }
  libusb_free_config_descriptor(cfg);
}

struct Stream {
  Assembler assembler;
  std::vector<uint8_t> frame = std::vector<uint8_t>(FRAME_BYTES);
  std::vector<uint8_t> last = std::vector<uint8_t>(FRAME_BYTES);
  bool have_last{false};
  unsigned frames{0}, shorts{0}, longs{0}, bad{0}, errors{0}, total_frames{0};
  std::vector<size_t> sizes;  // the first transfers' sizes, for the log
  FILE *all_raw{nullptr};
  FILE *lines{nullptr};
  Clock::time_point t0;
  std::vector<float> temps = std::vector<float>(PIXELS);
  float centre{NAN};
  uint32_t bad_pixels{0};
};

static void on_frame(Stream *s) {
  s->frames++;
  s->total_frames++;
  std::copy(s->frame.begin(), s->frame.end(), s->last.begin());
  s->have_last = true;
  if (s->all_raw != nullptr)
    std::fwrite(s->frame.data(), 1, FRAME_BYTES, s->all_raw);
  esphome::video::FrameStats st;
  s->bad_pixels = decode(s->frame.data(), 1.0f, 0.0f, s->temps.data(), &st);
  s->centre = s->bad_pixels == PIXELS ? NAN : s->temps[(H / 2) * W + W / 2];
  if (s->lines != nullptr) {
    const Marker m = parse_marker(s->frame.data());
    std::fprintf(s->lines, "%.3f %u %u %u 0x%02x %.3f %.3f %.3f %u", since(s->t0), m.frame, m.time, m.phase, m.sync,
                 s->centre, st.min, st.max, static_cast<unsigned>(s->bad_pixels));
    const InfoRows ir = parse_info_rows(s->frame.data());
    std::fprintf(s->lines, " ffc %u sun %u/%u count %u/%u", ir.ffc_countdown, ir.sun_protect, ir.hw_sun_protect,
                 ir.frame_count, ir.frame_count2);
    for (int i = 0; i < 8; i++)
      std::fprintf(s->lines, " %u", metadata_word(s->frame.data(), i));
    std::fprintf(s->lines, "\n");
  }
}

static void LIBUSB_CALL transfer_cb(libusb_transfer *t) {
  Stream *s = static_cast<Stream *>(t->user_data);
  if (t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length > 0) {
    if (s->sizes.size() < 40)
      s->sizes.push_back(t->actual_length);
    switch (s->assembler.feed(t->buffer, t->actual_length, s->frame.data())) {
      case Assembler::Result::FRAME:
        on_frame(s);
        break;
      case Assembler::Result::SHORT:
        s->shorts++;
        break;
      case Assembler::Result::LONG:
        s->longs++;
        break;
      case Assembler::Result::BAD_MARKER:
        s->bad++;
        break;
      default:
        break;
    }
  } else if (t->status != LIBUSB_TRANSFER_COMPLETED && t->status != LIBUSB_TRANSFER_CANCELLED) {
    s->errors++;
  }
  if (!g_stop && t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_NO_DEVICE)
    libusb_submit_transfer(t);
}

static void save(const Stream &s, const std::string &dir) {
  if (!s.have_last)
    return;
  FILE *f = std::fopen((dir + "/frame.raw").c_str(), "wb");
  if (f) {
    std::fwrite(s.last.data(), 1, FRAME_BYTES, f);
    std::fclose(f);
  }
  std::vector<float> t(PIXELS);
  esphome::video::FrameStats st;
  if (decode(s.last.data(), 1.0f, 0.0f, t.data(), &st) == PIXELS)
    return;
  f = std::fopen((dir + "/temps.f32").c_str(), "wb");
  if (f) {
    std::fwrite(t.data(), sizeof(float), PIXELS, f);
    std::fclose(f);
  }
  std::vector<uint8_t> rgb(static_cast<size_t>(W) * H * 2);
  picture_to_rgb565(s.last.data(), rgb.data(), static_cast<size_t>(W) * 2);
  f = std::fopen((dir + "/picture.ppm").c_str(), "wb");
  if (f) {
    std::fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (size_t i = 0; i < rgb.size(); i += 2) {
      const uint16_t c = static_cast<uint16_t>(rgb[i] | (rgb[i + 1] << 8));
      std::fputc((c >> 8) & 0xF8, f);
      std::fputc((c >> 3) & 0xFC, f);
      std::fputc((c << 3) & 0xF8, f);
    }
    std::fclose(f);
  }
  f = std::fopen((dir + "/temps.pgm").c_str(), "wb");
  if (f) {
    std::fprintf(f, "P5\n%d %d\n255\n", W, H);
    const float span = std::max(st.max - st.min, 0.1f);
    for (float v : t)
      std::fputc(static_cast<int>(std::lround(255.0f * (v - st.min) / span)), f);
    std::fclose(f);
  }
}

int main(int argc, char **argv) {
  double seconds = 20, shutter_at = -1;
  std::string out = ".";
  bool all_raw = false, read_settings = false, read_readings = false;
  int transfers = 4;
  std::vector<std::pair<Setting, int32_t>> writes;
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    if (a == "--seconds" && i + 1 < argc)
      seconds = std::atof(argv[++i]);
    else if (a == "--out" && i + 1 < argc)
      out = argv[++i];
    else if (a == "--shutter-at" && i + 1 < argc)
      shutter_at = std::atof(argv[++i]);
    else if (a == "--all-raw")
      all_raw = true;
    else if (a == "--transfers" && i + 1 < argc)
      transfers = std::max(1, std::atoi(argv[++i]));
    else if (a == "--settings")
      read_settings = true;
    else if (a == "--readings")
      read_readings = true;
    else if (a == "--set" && i + 1 < argc) {
      const std::string kv = argv[++i];
      const size_t eq = kv.find('=');
      bool found = false;
      for (int k = 0; k < static_cast<int>(Setting::COUNT) && eq != std::string::npos; k++) {
        const Setting st = static_cast<Setting>(k);
        if (kv.substr(0, eq) == setting_info(st).name && !setting_info(st).maintenance) {
          const char *v = kv.c_str() + eq + 1;
          const char *comma = std::strchr(v, ',');
          if (setting_is_pair(st) != (comma != nullptr)) {
            std::fprintf(stderr, "--set %s: %s\n", setting_info(st).name,
                         setting_is_pair(st) ? "a pair: FIRST,SECOND" : "one value");
            return 2;
          }
          writes.emplace_back(st, comma ? pack_pair(st, std::atoi(v), std::atoi(comma + 1)) : std::atoi(v));
          found = true;
        }
      }
      if (!found) {
        std::fprintf(stderr, "--set: no such setting in %s\n", kv.c_str());
        return 2;
      }
    } else {
      std::fprintf(stderr,
                   "usage: %s [--seconds S] [--out DIR] [--shutter-at S] [--all-raw] [--transfers N] [--settings] "
                   "[--readings] [--set NAME=VALUE[,SECOND] ...]\n",
                   argv[0]);
      return 2;
    }
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  libusb_context *ctx = nullptr;
  if (libusb_init(&ctx) != 0)
    return 1;
  Camera cam;
  cam.h = libusb_open_device_with_vid_pid(ctx, VID, PID_P1);
  if (cam.h == nullptr) {
    std::fprintf(stderr, "No Thermal Master P1 (%04x:%04x)\n", VID, PID_P1);
    return 1;
  }
  print_descriptors(libusb_get_device(cam.h));
  libusb_set_auto_detach_kernel_driver(cam.h, 1);
  for (int i : {0, 1}) {
    const int r = libusb_claim_interface(cam.h, i);
    if (r != 0) {
      std::fprintf(stderr, "claim interface %d: %s\n", i, libusb_error_name(r));
      return 1;
    }
  }

  // The component's start-up sequence.
  int r = libusb_set_interface_alt_setting(cam.h, 1, 0);
  std::printf("interface 1 alt 0: %s\n", r == 0 ? "ok" : libusb_error_name(r));
  sleep_ms(100);
  std::vector<uint8_t> resp;
  if (!cam.run(Cmd::READ_MODEL, &resp))
    return 1;
  const std::string model = text(resp);
  if (!cam.run(Cmd::READ_FIRMWARE, &resp))
    return 1;
  const std::string firmware = text(resp);
  if (!cam.run(Cmd::READ_SYSTEM_VERSION, &resp))
    return 1;
  std::printf("model \"%s\", firmware \"%s\", system \"%s\"\n", model.c_str(), firmware.c_str(), text(resp).c_str());
  if (!cam.run(Cmd::READ_RANGE, &resp))
    return 1;
  std::printf("range read: 0x%02x\n", resp.empty() ? 0 : resp[0]);
  sleep_ms(1000);
  r = libusb_set_interface_alt_setting(cam.h, 1, 1);
  if (r != 0) {
    std::fprintf(stderr, "interface 1 alt 1: %s\n", libusb_error_name(r));
    return 1;
  }
  r = libusb_control_transfer(cam.h, 0x40, 0xEE, 0, 1, nullptr, 0, 1000);
  if (r < 0) {
    std::fprintf(stderr, "stream on (0xEE): %s\n", libusb_error_name(r));
    return 1;
  }
  sleep_ms(2000);

  Stream s;
  if (all_raw)
    s.all_raw = std::fopen((out + "/all.raw").c_str(), "wb");
  s.lines = std::fopen((out + "/frames.txt").c_str(), "w");
  std::vector<libusb_transfer *> xfers;
  std::vector<std::vector<uint8_t>> bufs(transfers, std::vector<uint8_t>(16 * 512));
  for (int i = 0; i < transfers; i++) {
    libusb_transfer *t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, cam.h, 0x81, bufs[i].data(), bufs[i].size(), transfer_cb, &s, 0);
    libusb_submit_transfer(t);
    xfers.push_back(t);
  }
  const Clock::time_point t_start = Clock::now();
  s.t0 = t_start;
  if (!cam.run(Cmd::READ_RANGE, &resp))
    return 1;
  std::printf("range read again: 0x%02x\n", resp.empty() ? 0 : resp[0]);
  if (!cam.run(Cmd::RANGE_HIGH))
    return 1;
  std::printf("high range: status %u after %.2f s (busy %.2f s)\n", cam.last_status, since(t_start), cam.last_busy_s);
  for (int i = 0; i < 30; i++) {
    if (!cam.run(reading_command(Reading::GAIN_SWITCH_STATE), &resp) || resp.empty() || resp[0] != 2)
      break;
    std::printf("  range change under way\n");
    sleep_ms(1000);
  }
  auto read_setting = [&](Setting st) {
    const SettingInfo &info = setting_info(st);
    if (info.get == 0)
      return;
    std::vector<uint8_t> r;
    const bool ok = cam.run(setting_read(st), &r);
    std::printf("  %-26s ", info.name);
    if (!ok) {
      std::printf("refused (status %u)\n", cam.last_status);
      return;
    }
    const int32_t v = setting_value(st, r.data(), r.size());
    if (setting_is_pair(st))
      std::printf("%d, %d", static_cast<int>(pair_part(st, v, 0)), static_cast<int>(pair_part(st, v, 1)));
    else
      std::printf("%d", static_cast<int>(v));
    std::printf("  (%s)", hexs(r).c_str());
    if (has_attributes(st)) {
      Attributes a{};
      std::vector<uint8_t> ar;
      if (cam.run(attribute_read(st), &ar) && parse_attributes(ar.data(), ar.size(), &a))
        std::printf("  the camera takes %u..%u, step %u", a.min, a.max, a.step);
      else
        std::printf("  limits refused (status %u)", cam.last_status);
    }
    std::printf("\n");
  };
  for (const auto &w : writes) {
    const bool ok = cam.run(setting_write(w.first, w.second));
    std::printf("set %s = %d: %s, status %u after %.2f s busy\n", setting_info(w.first).name,
                static_cast<int>(w.second), ok ? "ok" : "refused", cam.last_status, cam.last_busy_s);
    read_setting(w.first);
  }
  if (read_settings) {
    std::printf("settings:\n");
    for (int k = 0; k < static_cast<int>(Setting::COUNT); k++)
      read_setting(static_cast<Setting>(k));
  }
  if (read_readings) {
    static const char *const NAMES[] = {"device_temperature", "powered_time",   "device_status",
                                        "shutter_status",     "crg_value",      "sensor_vtemp",
                                        "stream_source",      "digital_video_output", "system_data_flag",
                                        "cursor_position",    "camera_point",   "camera_frame",
                                        "gain_switch_state"};
    static_assert(sizeof(NAMES) / sizeof(NAMES[0]) == static_cast<size_t>(Reading::COUNT), "a name a reading");
    std::printf("readings:\n");
    for (int k = 0; k < static_cast<int>(Reading::COUNT); k++) {
      const Reading rd = static_cast<Reading>(k);
      std::vector<uint8_t> r;
      const bool ok = cam.run(reading_command(rd), &r);
      std::printf("  %-22s ", NAMES[k]);
      if (!ok) {
        std::printf("refused (status %u)\n", cam.last_status);
        continue;
      }
      std::printf("%s", hexs(r).c_str());
      if (rd == Reading::DEVICE_STATUS && !r.empty())
        std::printf("  %s", device_status_name(r[0]));
      if (rd == Reading::SHUTTER_STATUS && !r.empty())
        std::printf("  %s", shutter_status_name(r[0]));
      if (rd == Reading::STREAM_SOURCE && !r.empty())
        std::printf("  %s", stream_source_name(r[0]));
      if (rd == Reading::DEVICE_TEMPERATURE && r.size() >= 2)
        std::printf("  %.2f C", (r[0] | (r[1] << 8)) / 100.0);
      if (rd == Reading::CAMERA_POINT && r.size() >= 2)
        std::printf("  %.2f C", raw_to_celsius(static_cast<uint16_t>(r[0] | (r[1] << 8))));
      FrameInfo fi{};
      if (rd == Reading::CAMERA_FRAME && parse_frame_info(r.data(), r.size(), &fi))
        std::printf("  max %.2f C at %u, %u; min %.2f C at %u, %u", fi.max_c, fi.max_x, fi.max_y, fi.min_c, fi.min_x,
                    fi.min_y);
      std::printf("\n");
    }
    std::printf("device information:\n");
    for (uint8_t t = 1; t <= static_cast<uint8_t>(Info::LAST); t++) {
      std::vector<uint8_t> r;
      const bool ok = cam.run(info_command(static_cast<Info>(t)), &r);
      std::printf("  %2u  %s\n", t, ok ? ("\"" + text(r) + "\"  " + hexs(r)).c_str() : "refused");
    }
    std::vector<uint8_t> r;
    if (cam.run(palette_count_command(), &r) && !r.empty()) {
      std::printf("palettes: %u\n", r[0]);
      for (uint8_t i = 0; i < r[0] && i < 64; i++) {
        std::vector<uint8_t> nm;
        const bool ok = cam.run(palette_name_command(i), &nm);
        std::printf("  %2u  %s\n", i, ok ? ("\"" + text(nm) + "\"").c_str() : "refused");
      }
    } else {
      std::printf("palettes: refused (status %u)\n", cam.last_status);
    }
  }

  double next_line = 1, first_frame = -1;
  bool shutter_done = shutter_at < 0;
  while (!g_stop && since(t_start) < seconds) {
    timeval tv{0, 50000};
    libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
    const double now = since(t_start);
    if (first_frame < 0 && s.total_frames > 0) {
      first_frame = now;
      std::printf("first frame after %.2f s\n", now);
    }
    if (!shutter_done && now >= shutter_at) {
      shutter_done = true;
      const Clock::time_point ts = Clock::now();
      const bool ok = cam.run(Cmd::SHUTTER);
      std::printf("shutter: %s, status %u after %.2f s (busy %.2f s)\n", ok ? "ok" : "failed", cam.last_status,
                  since(ts), cam.last_busy_s);
    }
    if (now >= next_line) {
      next_line += 1;
      std::printf("%5.1f s: %2u frames, %u short, %u long, %u bad markers, %u errors, %u skipped by the camera; "
                  "centre %.2f C, %u pixels without a reading\n",
                  now, s.frames, s.shorts, s.longs, s.bad, s.errors, s.assembler.camera_skipped(), s.centre,
                  static_cast<unsigned>(s.bad_pixels));
      s.frames = 0;
      std::fflush(stdout);
    }
  }
  g_stop = 1;
  for (auto *t : xfers)
    libusb_cancel_transfer(t);
  for (int i = 0; i < 20; i++) {
    timeval tv{0, 50000};
    libusb_handle_events_timeout_completed(ctx, &tv, nullptr);
  }
  std::printf("first transfer sizes:");
  for (size_t n : s.sizes)
    std::printf(" %zu", n);
  std::printf("\n%u whole frames in %.1f s\n", s.total_frames, since(t_start));
  save(s, out);
  if (s.all_raw)
    std::fclose(s.all_raw);
  if (s.lines)
    std::fclose(s.lines);
  libusb_set_interface_alt_setting(cam.h, 1, 0);
  for (auto *t : xfers)
    libusb_free_transfer(t);
  libusb_release_interface(cam.h, 1);
  libusb_release_interface(cam.h, 0);
  libusb_close(cam.h);
  libusb_exit(ctx);
  return 0;
}
