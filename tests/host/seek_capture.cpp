// Seek Compact Pro on a Linux host, through libusb: the ESP32-P4 component's
// start-up sequence and frame requests, with every frame fed to the same
// seek_core. For checking a camera and the temperature model before (or
// without) a P4, and for saving real frames.
//
// Sends only what the component sends: the vendor read requests of its
// start-up sequence, operation and image-processing mode, and one frame
// request per frame; with --shutter-at, one shutter cycle. Nothing that
// writes the camera's flash (those requests are refused below).
//
// Build, from the repository root (libusb.h from libusb 1.0.2x if the
// system has no development package), as one command:
//   g++ -std=gnu++17 -O2 -pthread -Icomponents/video_core -Icomponents/seek_compact_pro
//       -I<directory holding libusb-1.0/libusb.h> tests/host/seek_capture.cpp
//       components/seek_compact_pro/seek_core.cpp components/video_core/frame.cpp
//       -l:libusb-1.0.so.0 -o seek_capture
// Run (root, or a udev rule for 289d:0011):
//   ./seek_capture [--seconds S] [--out DIR] [--shutter-at S] [--all-raw]
// --all-raw appends every frame, as received, to DIR/all.raw; --means writes one
// line per frame to DIR/means.txt: counter, id, picture mean, row-2 mean,
// seconds. --bad-column X (repeatable): a picture column filled like dead pixels.
// At the end it lists the columns whose mean flickers at more than 3x the median,
// --rgb FIFO
// writes every image, through the radiometric and ISP stages with the P4's
// settings (55-100 F, ironbow, 65% equalization, bicubic x2; not mirrored), as
// 640x480 RGB24 to FIFO (a writer thread; images are dropped while it is
// busy), and keeps only the first three images on disk.

#include <libusb-1.0/libusb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "seek_core.h"
#include "isp_core.h"
#include "palette.h"
#include "radiometric_core.h"

#include <condition_variable>
#include <fcntl.h>
#include <mutex>
#include <thread>
#include <unistd.h>

using namespace esphome::seek_compact_pro;
using namespace esphome;
using Clock = std::chrono::steady_clock;

static constexpr uint16_t VID = 0x289D, PID = 0x0011;
static constexpr uint8_t EP_IN = 0x81, IFACE = 0;
static constexpr uint8_t TYPE_OUT = 0x41, TYPE_IN = 0xC1;
static constexpr int XFER_BYTES = 27 * 512;  // one 13680-byte piece fits; it ends in a short packet
static constexpr int NUM_XFERS = 3;

enum : uint8_t {
  REQ_TOGGLE_SHUTTER = 55,
  REQ_READ_CHIP_ID = 54,
  REQ_SET_OPERATION_MODE = 60,
  REQ_SET_IMAGE_PROCESSING_MODE = 62,
  REQ_GET_FIRMWARE_INFO = 78,
  REQ_START_GET_IMAGE_TRANSFER = 83,
  REQ_TARGET_PLATFORM = 84,
  REQ_SET_FIRMWARE_INFO_FEATURES = 85,
  REQ_SET_FACTORY_SETTINGS_FEATURES = 86,
  REQ_GET_FACTORY_SETTINGS = 88,
};

static bool allowed(uint8_t req) {
  switch (req) {
    case REQ_TOGGLE_SHUTTER:
    case REQ_READ_CHIP_ID:
    case REQ_SET_OPERATION_MODE:
    case REQ_SET_IMAGE_PROCESSING_MODE:
    case REQ_GET_FIRMWARE_INFO:
    case REQ_START_GET_IMAGE_TRANSFER:
    case REQ_TARGET_PLATFORM:
    case REQ_SET_FIRMWARE_INFO_FEATURES:
    case REQ_SET_FACTORY_SETTINGS_FEATURES:
    case REQ_GET_FACTORY_SETTINGS:
      return true;
    default:
      return false;  // flash writes (0x46, 0x50-0x52, 0x57), reset (0x59), polarity (0x38), ...
  }
}

static std::atomic<bool> g_stop{false};
static void on_signal(int) { g_stop = true; }

static double secs_since(Clock::time_point t) {
  return std::chrono::duration<double>(Clock::now() - t).count();
}

// ---------------------------------------------------------------- control requests

static libusb_device_handle *g_dev = nullptr;

static bool ctrl_out(uint8_t req, std::vector<uint8_t> data, bool quiet = false) {
  if (!allowed(req)) {
    std::fprintf(stderr, "refusing request 0x%02X\n", req);
    std::abort();
  }
  int r = libusb_control_transfer(g_dev, TYPE_OUT, req, 0, 0, data.data(), static_cast<uint16_t>(data.size()), 1000);
  if (r != static_cast<int>(data.size())) {
    if (!quiet)
      std::fprintf(stderr, "request 0x%02X out failed: %s\n", req, r < 0 ? libusb_error_name(r) : "short");
    return false;
  }
  return true;
}

static bool ctrl_in(uint8_t req, uint16_t len, std::vector<uint8_t> &out) {
  if (!allowed(req))
    std::abort();
  out.assign(len, 0);
  int r = libusb_control_transfer(g_dev, TYPE_IN, req, 0, 0, out.data(), len, 1000);
  if (r != len) {  // libusb-thermal treats a short reply as an error too
    std::fprintf(stderr, "request 0x%02X in (%u) failed: %s\n", req, len, r < 0 ? libusb_error_name(r) : "short");
    return false;
  }
  return true;
}

static std::string hex(const std::vector<uint8_t> &v) {
  std::string s;
  char b[4];
  for (uint8_t c : v) {
    std::snprintf(b, sizeof b, "%02X ", c);
    s += b;
  }
  if (!s.empty())
    s.pop_back();
  return s;
}

static std::string printable(const std::vector<uint8_t> &v) {
  std::string s;
  for (uint8_t c : v)
    s += (c >= 0x20 && c < 0x7F) ? static_cast<char>(c) : '.';
  return s;
}

static void save(const std::string &path, const void *data, size_t n) {
  if (FILE *f = std::fopen(path.c_str(), "wb")) {
    std::fwrite(data, 1, n, f);
    std::fclose(f);
  }
}

// ---------------------------------------------------------------- start-up

struct Info {
  std::vector<uint8_t> firmware, chip_id, serial, identity, fov, focus, geometry, table;
};

static bool start_up(Info &info, FILE *log) {
  ctrl_out(REQ_SET_OPERATION_MODE, {0x00, 0x00}, true);  // in case it was left running
  if (!ctrl_out(REQ_TARGET_PLATFORM, {0x01})) {
    for (int i = 0; i < 3; i++)
      ctrl_out(REQ_SET_OPERATION_MODE, {0x00, 0x00}, true);
    if (!ctrl_out(REQ_TARGET_PLATFORM, {0x01}))
      return false;
  }
  if (!ctrl_out(REQ_SET_OPERATION_MODE, {0x00, 0x00}))
    return false;
  if (!ctrl_in(REQ_GET_FIRMWARE_INFO, 4, info.firmware) || !ctrl_in(REQ_READ_CHIP_ID, 12, info.chip_id))
    return false;
  if (!ctrl_out(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x06, 0x00, 0x08, 0x00, 0x00, 0x00}) ||
      !ctrl_in(REQ_GET_FACTORY_SETTINGS, 12, info.serial))
    return false;
  if (!ctrl_out(REQ_SET_FIRMWARE_INFO_FEATURES, {0x17, 0x00}) || !ctrl_in(REQ_GET_FIRMWARE_INFO, 64, info.identity))
    return false;
  if (!ctrl_out(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x01, 0x00, 0x00, 0x06, 0x00, 0x00}) ||
      !ctrl_in(REQ_GET_FACTORY_SETTINGS, 2, info.fov))
    return false;
  if (!ctrl_out(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x01, 0x00, 0x01, 0x06, 0x00, 0x00}) ||
      !ctrl_in(REQ_GET_FACTORY_SETTINGS, 2, info.focus))
    return false;
  info.table.clear();
  for (uint16_t addr = 0; addr < 0xA00; addr += 0x20) {
    std::vector<uint8_t> chunk;
    if (!ctrl_out(REQ_SET_FACTORY_SETTINGS_FEATURES,
                  {0x20, 0x00, static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>(addr >> 8), 0x00, 0x00}) ||
        !ctrl_in(REQ_GET_FACTORY_SETTINGS, 64, chunk))
      return false;
    info.table.insert(info.table.end(), chunk.begin(), chunk.end());
  }
  if (!ctrl_out(REQ_SET_FIRMWARE_INFO_FEATURES, {0x15, 0x00}) || !ctrl_in(REQ_GET_FIRMWARE_INFO, 64, info.geometry))
    return false;

  auto u16 = [](const std::vector<uint8_t> &v, size_t i) { return v.size() >= 2 * i + 2 ? v[2 * i] | v[2 * i + 1] << 8 : 0; };
  std::fprintf(log, "firmware      %u.%u.%u.%u  (%s)\n", info.firmware[0], info.firmware[1], info.firmware[2],
               info.firmware[3], hex(info.firmware).c_str());
  std::fprintf(log, "chip id       %s\n", hex(info.chip_id).c_str());
  std::fprintf(log, "serial        %s\n", printable(info.serial).c_str());
  std::fprintf(log, "record 23     %s\n", printable(info.identity).c_str());
  std::fprintf(log, "lens          fov word 0x%04X, focus word 0x%04X\n", u16(info.fov, 0), u16(info.focus, 0));
  std::fprintf(log, "record 21     ");
  for (size_t i = 0; i < 16; i++)
    std::fprintf(log, "%u ", u16(info.geometry, i));
  Factory fac;
  const bool fac_ok = parse_factory(info.table.data(), info.table.size(), &fac);
  std::fprintf(log, "\nfactory       calibration %s (thermography version %d, sensor reference %.1f, warm-up %.1f C / %.0f s)\n",
               fac_ok ? "supported" : "NOT supported", fac.version, fac.sensor_ref, fac.warm_a, fac.warm_tau);
  return ctrl_out(REQ_SET_IMAGE_PROCESSING_MODE, {0x08, 0x00}) && ctrl_out(REQ_SET_OPERATION_MODE, {0x01, 0x00});
}

static void stop_camera() {
  for (int i = 0; i < 3; i++)
    ctrl_out(REQ_SET_OPERATION_MODE, {0x00, 0x00}, true);
}

// ---------------------------------------------------------------- streaming

struct Stream {
  std::vector<uint8_t> frame = std::vector<uint8_t>(RAW_BYTES);
  size_t fill{0};
  std::deque<std::vector<uint16_t>> done;  // complete frames, oldest first
  Clock::time_point last_piece, last_frame;
  libusb_transfer *credit{nullptr};
  uint8_t credit_buf[LIBUSB_CONTROL_SETUP_SIZE + 4];
  bool credit_busy{false};
  unsigned overflow{0}, credit_errors{0}, credit_late{0}, xfer_errors{0}, pieces{0};
  double credit_max_ms{0};
};
static Stream g_s;

static void credit_done(libusb_transfer *t) {
  g_s.credit_busy = false;
  if (t->status != LIBUSB_TRANSFER_COMPLETED)
    g_s.credit_errors++;
}

// START_GET_IMAGE_TRANSFER for one frame (88920 words).
static void send_credit() {
  if (g_s.credit_busy) {
    g_s.credit_late++;
    return;
  }
  libusb_fill_control_setup(g_s.credit_buf, TYPE_OUT, REQ_START_GET_IMAGE_TRANSFER, 0, 0, 4);
  const uint32_t words = RAW_WORDS;
  std::memcpy(g_s.credit_buf + LIBUSB_CONTROL_SETUP_SIZE, &words, 4);
  libusb_fill_control_transfer(g_s.credit, g_dev, g_s.credit_buf, credit_done, nullptr, 1000);
  if (libusb_submit_transfer(g_s.credit) == 0)
    g_s.credit_busy = true;
  else
    g_s.credit_errors++;
}

static void piece_done(libusb_transfer *t) {
  if (t->status == LIBUSB_TRANSFER_COMPLETED) {
    g_s.pieces++;
    g_s.last_piece = Clock::now();
    const size_t n = static_cast<size_t>(t->actual_length);
    if (g_s.fill + n > RAW_BYTES) {
      g_s.overflow++;
      g_s.fill = 0;  // resynchronise on the next frame
    } else {
      std::memcpy(g_s.frame.data() + g_s.fill, t->buffer, n);
      g_s.fill += n;
      if (g_s.fill == RAW_BYTES) {
        send_credit();  // at once: asked for late, the camera stops for good
        g_s.credit_max_ms = std::max(g_s.credit_max_ms, secs_since(g_s.last_piece) * 1000.0);
        std::vector<uint16_t> f(RAW_WORDS);
        std::memcpy(f.data(), g_s.frame.data(), RAW_BYTES);
        g_s.done.push_back(std::move(f));
        g_s.fill = 0;
        g_s.last_frame = Clock::now();
      }
    }
  } else if (t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_TIMED_OUT) {
    g_s.xfer_errors++;
  }
  if (!g_stop && t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_NO_DEVICE) {
    if (libusb_submit_transfer(t) != 0)
      g_s.xfer_errors++;
  } else {
    t->user_data = nullptr;  // finished
  }
}

// ---------------------------------------------------------------- output

static void write_pgm(const std::string &path, const float *t) {
  float lo = INFINITY, hi = -INFINITY;
  for (size_t i = 0; i < IMG_PIXELS; i++)
    if (std::isfinite(t[i])) {
      lo = std::min(lo, t[i]);
      hi = std::max(hi, t[i]);
    }
  std::vector<uint8_t> px(IMG_PIXELS, 0);
  const float span = hi > lo ? hi - lo : 1.0f;
  for (size_t i = 0; i < IMG_PIXELS; i++)
    px[i] = std::isfinite(t[i]) ? static_cast<uint8_t>(std::lround((t[i] - lo) / span * 255.0f)) : 0;
  if (FILE *f = std::fopen(path.c_str(), "wb")) {
    std::fprintf(f, "P5\n# %.2f..%.2f C\n%u %u\n255\n", lo, hi, IMG_W, IMG_H);
    std::fwrite(px.data(), 1, px.size(), f);
    std::fclose(f);
  }
}

static float centre_mean(const float *t) {  // centre 6x6, as Seek's spot
  double s = 0;
  int n = 0;
  for (int y = IMG_H / 2 - 3; y < IMG_H / 2 + 3; y++)
    for (int x = IMG_W / 2 - 3; x < IMG_W / 2 + 3; x++)
      if (std::isfinite(t[y * IMG_W + x])) {
        s += t[y * IMG_W + x];
        n++;
      }
  return n ? static_cast<float>(s / n) : NAN;
}

// Zeroed pixels in the first frame, by position modulo 15 over the raw
// frame: one dominant residue would be the original Compact's "patent pixel"
// grid.
// The first frame is the camera's pixel table: 0 marks a dead pixel.
static void report_table(const uint16_t *raw, FILE *log) {
  unsigned sensor_zeros = 0, picture_zeros = 0, ones = 0;
  std::vector<unsigned> col_zeros(324, 0);
  for (size_t y = 4; y < 258; y++) {
    for (size_t x = 0; x < 324; x++) {
      const uint16_t v = raw[y * RAW_W + x];
      if (v == 0xFFFF)
        ones++;
      if (v != 0)
        continue;
      sensor_zeros++;
      col_zeros[x]++;
      if (y >= IMG_ROW0 && y < IMG_ROW0 + IMG_H && x >= IMG_COL0 && x < IMG_COL0 + IMG_W)
        picture_zeros++;
    }
  }
  std::fprintf(log, "pixel table   %u zeros on the sensor (columns 0-323, rows 4-257), %u in the picture; %u read 0xFFFF",
               sensor_zeros, picture_zeros, ones);
  for (size_t x = 0; x < col_zeros.size(); x++)
    if (col_zeros[x] >= 200)
      std::fprintf(log, "; sensor column %zu all but %u zero", x, 254 - col_zeros[x]);
  std::fprintf(log, "\n");
}

int main(int argc, char **argv) {
  double seconds = 30, shutter_at = -1;
  std::string out = "seek_out";
  bool all_raw = false, means = false;
  std::string rgb_path;
  bool scene_filter = true;
  std::vector<uint16_t> bad_columns;
  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() { return i + 1 < argc ? argv[++i] : ""; };
    if (a == "--seconds")
      seconds = std::atof(next());
    else if (a == "--out")
      out = next();
    else if (a == "--shutter-at")
      shutter_at = std::atof(next());
    else if (a == "--all-raw")
      all_raw = true;
    else if (a == "--means")
      means = true;
    else if (a == "--no-scene-filter")
      scene_filter = false;
    else if (a == "--rgb")
      rgb_path = next();
    else if (a == "--bad-column")
      bad_columns.push_back(static_cast<uint16_t>(std::atoi(next())));
    else {
      std::fprintf(stderr, "usage: %s [--seconds S] [--out DIR] [--shutter-at S] [--all-raw] [--means] [--bad-column X]... [--rgb FIFO] [--no-scene-filter]\n", argv[0]);
      return 2;
    }
  }
  mkdir(out.c_str(), 0755);
  FILE *log = std::fopen((out + "/info.txt").c_str(), "w");
  FILE *all = all_raw ? std::fopen((out + "/all.raw").c_str(), "wb") : nullptr;
  FILE *mf = means ? std::fopen((out + "/means.txt").c_str(), "w") : nullptr;
  if (log == nullptr)
    return 1;
  auto both = [&](const char *fmt, auto... args) {
    std::printf(fmt, args...);
    std::fprintf(log, fmt, args...);
  };

  if (libusb_init(nullptr) != 0)
    return 1;
  g_dev = libusb_open_device_with_vid_pid(nullptr, VID, PID);
  if (g_dev == nullptr) {
    std::fprintf(stderr, "no 289d:0011 device (plugged in? permission?)\n");
    return 1;
  }
  libusb_device_descriptor dd{};
  libusb_get_device_descriptor(libusb_get_device(g_dev), &dd);
  unsigned char prod[128] = {0}, manu[128] = {0};
  if (dd.iProduct)
    libusb_get_string_descriptor_ascii(g_dev, dd.iProduct, prod, sizeof prod);
  if (dd.iManufacturer)
    libusb_get_string_descriptor_ascii(g_dev, dd.iManufacturer, manu, sizeof manu);
  const int speed = libusb_get_device_speed(libusb_get_device(g_dev));
  both("device        %04x:%04x \"%s\" \"%s\", bcdDevice %x.%02x, speed %s\n", dd.idVendor, dd.idProduct, manu, prod,
       dd.bcdDevice >> 8, dd.bcdDevice & 0xFF, speed == LIBUSB_SPEED_HIGH ? "high (480 Mbit/s)" : "NOT high speed");
  libusb_set_auto_detach_kernel_driver(g_dev, 1);
  if (libusb_claim_interface(g_dev, IFACE) != 0) {
    std::fprintf(stderr, "cannot claim interface 0\n");
    return 1;
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  Info info;
  if (!start_up(info, log)) {
    std::fprintf(stderr, "start-up failed\n");
    stop_camera();
    return 1;
  }
  std::fflush(log);
  if (FILE *f = std::fopen((out + "/info.txt").c_str(), "r")) {  // echo what start-up logged
    char line[256];
    while (std::fgets(line, sizeof line, f))
      if (std::strncmp(line, "device", 6) != 0)
        std::fputs(line, stdout);
    std::fclose(f);
  }
  save(out + "/table.bin", info.table.data(), info.table.size());

  Processor proc;
  proc.set_scene_filter(scene_filter);
  if (!proc.set_factory(info.table.data(), info.table.size()))
    both("%s", "factory calibration not supported: no temperatures\n");
  for (uint16_t x : bad_columns) {
    if (!proc.add_bad_column(x)) {
      std::fprintf(stderr, "bad column %u: out of range or too many\n", x);
      return 2;
    }
  }
  // Rendering for --rgb: the latest image waits for the writer thread.
  radiometric::Processor rproc;
  isp::CpuIsp cpu_isp;
  isp::Options io;
  std::vector<float> rtemps(IMG_PIXELS);
  std::vector<uint8_t> rgb_pending;
  bool rgb_ready = false;
  std::atomic<bool> rgb_failed{false};
  std::mutex rgb_mu;
  std::condition_variable rgb_cv;
  std::thread rgb_thread;
  uint32_t last_render_ms = 0;
  if (!rgb_path.empty()) {
    radiometric::Options ro;
    ro.temporal_frames = 1;
    ro.motion_threshold = 0.05f;
    ro.edge_smoothing = 0.1f;
    ro.outlier_level = 0.0f;
    rproc.set_options(ro);
    io.range_min = (55.0f - 32.0f) / 1.8f;
    io.range_max = (100.0f - 32.0f) / 1.8f;
    io.mirror_x = false;
    io.scale = 2;
    io.upscale = isp::Upscale::BICUBIC;
    io.format = video::ImageFormat::RGB888;
    io.equalize = 0.65f;
    cpu_isp.set_palette(isp::make_palette(isp::Palette::IRONBOW));
    signal(SIGPIPE, SIG_IGN);
    rgb_thread = std::thread([&]() {
      const int fd = open(rgb_path.c_str(), O_WRONLY);  // waits for the reader
      if (fd < 0) {
        std::fprintf(stderr, "cannot open %s\n", rgb_path.c_str());
        rgb_failed = true;
        g_stop = true;
        return;
      }
      std::vector<uint8_t> out;
      while (!g_stop) {
        {
          std::unique_lock<std::mutex> lk(rgb_mu);
          rgb_cv.wait_for(lk, std::chrono::milliseconds(200), [&] { return rgb_ready || g_stop.load(); });
          if (!rgb_ready)
            continue;
          out.swap(rgb_pending);
          rgb_ready = false;
        }
        size_t off = 0;
        while (off < out.size()) {
          const ssize_t n = write(fd, out.data() + off, out.size() - off);
          if (n <= 0) {
            std::fprintf(stderr, "the reader of %s went away\n", rgb_path.c_str());
            rgb_failed = true;
            g_stop = true;
            close(fd);
            return;
          }
          off += static_cast<size_t>(n);
        }
      }
      close(fd);
    });
  }

  // Column flicker: per picture column, the change of its mean from one image
  // to the next (the picture's own change taken out), squared and summed.
  std::vector<double> col_prev(IMG_W, 0.0), col_sq(IMG_W, 0.0);
  unsigned col_n = 0;
  int prev_image_counter = -1;

  std::vector<libusb_transfer *> xfers;
  std::vector<std::vector<uint8_t>> bufs(NUM_XFERS, std::vector<uint8_t>(XFER_BYTES));
  g_s.credit = libusb_alloc_transfer(0);
  for (int i = 0; i < NUM_XFERS; i++) {
    libusb_transfer *t = libusb_alloc_transfer(0);
    libusb_fill_bulk_transfer(t, g_dev, EP_IN, bufs[i].data(), XFER_BYTES, piece_done, &g_s, 0);
    if (libusb_submit_transfer(t) != 0) {
      std::fprintf(stderr, "cannot queue bulk transfers\n");
      stop_camera();
      return 1;
    }
    xfers.push_back(t);
  }
  const auto t0 = Clock::now();
  g_s.last_frame = g_s.last_piece = t0;
  send_credit();

  std::vector<float> temps(IMG_PIXELS);
  std::map<uint16_t, unsigned> ids, steps;
  std::map<uint16_t, bool> saved_id;
  unsigned frames = 0, images = 0, torn = 0, saved_images = 0, misses = 0, restarts = 0;
  int last_counter = -1;
  bool shutter_sent = false;
  auto report_at = t0 + std::chrono::seconds(1);
  unsigned frames_at_report = 0, images_at_report = 0;
  esphome::video::FrameStats st{};

  while (!g_stop && secs_since(t0) < seconds) {
    timeval tv{0, 20000};
    libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);

    bool tables_lost = false;
    while (!g_s.done.empty()) {
      std::vector<uint16_t> f = std::move(g_s.done.front());
      g_s.done.pop_front();
      const uint16_t *raw = f.data();
      if (frame_magic(raw) != FRAME_MAGIC) {
        torn++;
        continue;
      }
      frames++;
      if (all != nullptr)
        std::fwrite(raw, 1, RAW_BYTES, all);
      if (mf != nullptr) {
        double sum = 0;
        for (size_t i = 0; i < IMG_PIXELS; i++)
          sum += image_px(raw, i);
        double row2 = 0;
        for (size_t x = 0; x < 324; x++)
          row2 += raw[2 * RAW_W + x];
        std::fprintf(mf, "%u %u %.2f %.3f %.3f\n", frame_counter(raw), frame_id(raw), sum / IMG_PIXELS, row2 / 324,
                     secs_since(t0));
      }
      const uint16_t id = frame_id(raw), counter = frame_counter(raw);
      ids[id]++;
      if (last_counter >= 0)
        steps[static_cast<uint16_t>(counter - last_counter)]++;
      last_counter = counter;
      if (!saved_id[id]) {
        saved_id[id] = true;
        save(out + "/frame_id" + std::to_string(id) + ".raw", raw, RAW_BYTES);
      }
      if (id == FRAME_FIRST)
        report_table(raw, log);
      if (id == FRAME_IMAGE) {
        std::vector<double> cm(IMG_W, 0.0);
        double all = 0;
        for (size_t y = 0; y < IMG_H; y++)
          for (size_t x = 0; x < IMG_W; x++)
            cm[x] += raw[(IMG_ROW0 + y) * RAW_W + IMG_COL0 + x];
        for (auto &v : cm) {
          v /= IMG_H;
          all += v;
        }
        all /= IMG_W;
        for (auto &v : cm)
          v -= all;
        if (prev_image_counter >= 0 && static_cast<uint16_t>(counter - prev_image_counter) == 1) {
          for (size_t x = 0; x < IMG_W; x++)
            col_sq[x] += (cm[x] - col_prev[x]) * (cm[x] - col_prev[x]);
          col_n++;
        }
        col_prev = cm;
        prev_image_counter = counter;
      }
      // As the component: the tables come once per start, first; a shutter
      // frame or image before them means one was lost (a short first frame).
      if ((id == FRAME_IMAGE || id == FRAME_SHUTTER || id == FRAME_START_SHUTTER) && !proc.has_table()) {
        tables_lost = true;
        break;
      }
      const auto r = proc.feed(raw, temps.data(), &st);
      if (r == Processor::Result::TABLE && id == FRAME_FIRST)
        both("dead pixels   %zu (first frame)\n", proc.dead_pixels());
      if (r == Processor::Result::IMAGE) {
        images++;
        if (!rgb_path.empty()) {
          const uint32_t ms = static_cast<uint32_t>(secs_since(t0) * 1000);
          io.curve_blend = last_render_ms ? isp::curve_blend_for(ms - last_render_ms) : 1.0f;
          last_render_ms = ms;
          if (rproc.process(temps.data(), rtemps.data(), IMG_W, IMG_H) && cpu_isp.tone_map(rtemps.data(), IMG_W, IMG_H, io)) {
            std::vector<uint8_t> buf(static_cast<size_t>(IMG_W) * 2 * IMG_H * 2 * 3);
            cpu_isp.render(io, buf.data(), IMG_W * 2 * 3);
            std::lock_guard<std::mutex> lk(rgb_mu);
            rgb_pending.swap(buf);
            rgb_ready = true;
            rgb_cv.notify_one();
          }
        }
        if (saved_images < 3 || (rgb_path.empty() && images % 100 == 0)) {
          const std::string n = std::to_string(images);
          write_pgm(out + "/image" + n + ".pgm", temps.data());
          save(out + "/image" + n + ".raw", raw, RAW_BYTES);
          save(out + "/image" + n + ".f32", temps.data(), IMG_PIXELS * sizeof(float));
          saved_images++;
        }
      }
    }

    if (tables_lost) {
      if (++restarts > 5) {
        both("%s", "the camera's tables were lost 5 times: giving up\n");
        break;
      }
      both("tables lost (a frame came before them): starting the camera again (%u)\n", restarts);
      stop_camera();
      g_s.done.clear();
      proc.reset();
      if (!start_up(info, log)) {
        both("%s", "start-up failed\n");
        break;
      }
      g_s.fill = 0;
      g_s.last_frame = Clock::now();
      send_credit();
    }

    if (shutter_at >= 0 && !shutter_sent && secs_since(t0) >= shutter_at) {
      shutter_sent = true;
      ctrl_out(REQ_TOGGLE_SHUTTER, {0xFC, 0x00, 0x04, 0x00});
      both("shutter cycle requested at %.1f s\n", secs_since(t0));
    }

    if (secs_since(g_s.last_frame) > 1.0) {  // as the component: ask again, drop the partial frame
      misses++;
      both("no frame for 1 s (%u pieces so far, %zu bytes pending): asking again\n", g_s.pieces, g_s.fill);
      g_s.fill = 0;
      g_s.last_frame = Clock::now();
      send_credit();
      if (misses >= 5)
        break;
    }

    if (Clock::now() >= report_at) {
      report_at += std::chrono::seconds(1);
      std::string idstr, stepstr;
      for (auto &kv : ids)
        idstr += std::to_string(kv.first) + ":" + std::to_string(kv.second) + " ";
      for (auto &kv : steps)
        stepstr += "+" + std::to_string(kv.first) + ":" + std::to_string(kv.second) + " ";
      both("%5.1f s  %2u fps (%2u images)  ids [%s] counter steps [%s] torn %u overflow %u credit err/late %u/%u "
           "(max %.2f ms) | sensor %.1f  warm-up %.2f C  dead %zu | %s min %.2f max %.2f mean %.2f centre %.2f C\n",
           secs_since(t0), frames - frames_at_report, images - images_at_report, idstr.c_str(), stepstr.c_str(), torn,
           g_s.overflow, g_s.credit_errors, g_s.credit_late, g_s.credit_max_ms, proc.sensor_counts(), proc.warm_up_c(),
           proc.dead_pixels(), st.valid ? "T" : "no T", st.min, st.max, st.mean, centre_mean(temps.data()));
      frames_at_report = frames;
      images_at_report = images;
      std::fflush(log);
    }
  }

  g_stop = true;
  for (auto *t : xfers)
    libusb_cancel_transfer(t);
  for (int i = 0; i < 50; i++) {
    bool busy = g_s.credit_busy;
    for (auto *t : xfers)
      busy |= t->user_data != nullptr;
    if (!busy)
      break;
    timeval tv{0, 20000};
    libusb_handle_events_timeout_completed(nullptr, &tv, nullptr);
  }
  stop_camera();
  if (rgb_thread.joinable()) {
    g_stop = true;
    rgb_cv.notify_one();
    rgb_thread.join();
  }
  if (rgb_failed)
    both("%s", "rgb output failed\n");
  if (col_n > 10) {
    std::vector<double> rms(IMG_W);
    for (size_t x = 0; x < IMG_W; x++)
      rms[x] = std::sqrt(col_sq[x] / col_n / 2);
    std::vector<double> sorted = rms;
    std::nth_element(sorted.begin(), sorted.begin() + IMG_W / 2, sorted.end());
    const double med = sorted[IMG_W / 2];
    both("column flicker: median %.2f counts over %u image pairs; above 3x:", med, col_n);
    bool any = false;
    for (size_t x = 0; x < IMG_W; x++)
      if (rms[x] > 3 * med) {
        both(" picture column %zu (%.1fx)", x, rms[x] / med);
        any = true;
      }
    both("%s\n", any ? "" : " none");
  }
  both("done: %u frames, %u images in %.1f s; files in %s/\n", frames, images, secs_since(t0), out.c_str());
  std::fclose(log);
  if (all != nullptr)
    std::fclose(all);
  if (mf != nullptr)
    std::fclose(mf);
  libusb_release_interface(g_dev, IFACE);
  libusb_close(g_dev);
  libusb_exit(nullptr);
  return 0;
}
