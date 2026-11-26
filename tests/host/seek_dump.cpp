// Seek Compact Pro: reads (only reads) the camera's factory settings over the
// whole 16-bit word address range and its firmware information records, and
// saves them. Requests sent: SET_OPERATION_MODE 0 (stopped), then for factory
// settings SET_FACTORY_SETTINGS_FEATURES {count, 0, address lo, address hi, 0,
// 0} + GET_FACTORY_SETTINGS (as every driver's start-up does for words
// 0..2559), and for firmware information SET_FIRMWARE_INFO_FEATURES {record,
// 0} + GET_FIRMWARE_INFO (start-up reads records 21 and 23). Nothing that
// writes the camera's flash.
//
// Build: g++ -std=gnu++17 -O2 -I<dir with libusb-1.0/libusb.h> tests/host/seek_dump.cpp -l:libusb-1.0.so.0 -o seek_dump
// Run:   ./seek_dump OUT_DIR [last_word (default 65535)]

#include <libusb-1.0/libusb.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <sys/stat.h>
#include <vector>

static constexpr uint16_t VID = 0x289D, PID = 0x0011;
static constexpr uint8_t TYPE_OUT = 0x41, TYPE_IN = 0xC1;
enum : uint8_t {
  REQ_SET_OPERATION_MODE = 60,
  REQ_GET_FIRMWARE_INFO = 78,
  REQ_SET_FIRMWARE_INFO_FEATURES = 85,
  REQ_SET_FACTORY_SETTINGS_FEATURES = 86,
  REQ_GET_FACTORY_SETTINGS = 88,
};

static libusb_device_handle *g_dev = nullptr;

static int out(uint8_t req, std::vector<uint8_t> d) {
  return libusb_control_transfer(g_dev, TYPE_OUT, req, 0, 0, d.data(), static_cast<uint16_t>(d.size()), 1000);
}
static int in(uint8_t req, std::vector<uint8_t> &d, uint16_t len) {
  d.assign(len, 0);
  return libusb_control_transfer(g_dev, TYPE_IN, req, 0, 0, d.data(), len, 1000);
}

int main(int argc, char **argv) {
  if (argc < 2)
    return 2;
  const std::string dir = argv[1];
  const unsigned last = argc > 2 ? static_cast<unsigned>(std::atoi(argv[2])) : 65535;
  mkdir(dir.c_str(), 0755);
  if (libusb_init(nullptr) != 0 || !(g_dev = libusb_open_device_with_vid_pid(nullptr, VID, PID))) {
    std::fprintf(stderr, "no camera\n");
    return 1;
  }
  libusb_set_auto_detach_kernel_driver(g_dev, 1);
  if (libusb_claim_interface(g_dev, 0) != 0) {
    std::fprintf(stderr, "cannot claim interface 0\n");
    return 1;
  }
  out(REQ_SET_OPERATION_MODE, {0x00, 0x00});

  // Factory settings, 32 words at a time; a failed read is recorded as such.
  FILE *f = std::fopen((dir + "/factory.bin").c_str(), "wb");
  FILE *log = std::fopen((dir + "/factory_status.txt").c_str(), "w");
  std::vector<uint8_t> d;
  unsigned ok = 0, failed = 0;
  int last_state = -1;
  unsigned run_start = 0;
  for (unsigned a = 0; a <= last; a += 32) {
    const int r1 = out(REQ_SET_FACTORY_SETTINGS_FEATURES,
                       {0x20, 0x00, static_cast<uint8_t>(a & 0xFF), static_cast<uint8_t>(a >> 8), 0x00, 0x00});
    const int r2 = r1 == 6 ? in(REQ_GET_FACTORY_SETTINGS, d, 64) : -1;
    const int state = (r1 == 6 && r2 == 64) ? 1 : 0;
    if (!state)
      d.assign(64, 0);
    std::fwrite(d.data(), 1, 64, f);
    state ? ok++ : failed++;
    if (state != last_state) {
      if (last_state >= 0)
        std::fprintf(log, "words %u-%u: %s\n", run_start, a - 1, last_state ? "read" : "failed");
      run_start = a;
      last_state = state;
      if (!state)
        std::fprintf(log, "  first failure at %u: set %d (%s), get %d (%s)\n", a, r1, r1 < 0 ? libusb_error_name(r1) : "",
                     r2, r2 < 0 ? libusb_error_name(r2) : "");
    }
  }
  std::fprintf(log, "words %u-%u: %s\n", run_start, last, last_state ? "read" : "failed");
  std::fclose(f);
  std::fclose(log);
  std::printf("factory settings: %u chunks read, %u failed\n", ok, failed);

  // Firmware information records 0..63, 64 bytes each.
  FILE *fi = std::fopen((dir + "/firmware_info.txt").c_str(), "w");
  for (unsigned rec = 0; rec < 64; rec++) {
    const int r1 = out(REQ_SET_FIRMWARE_INFO_FEATURES, {static_cast<uint8_t>(rec), 0x00});
    const int r2 = r1 == 2 ? in(REQ_GET_FIRMWARE_INFO, d, 64) : -1;
    std::fprintf(fi, "record %2u: ", rec);
    if (r1 != 2 || r2 <= 0) {
      std::fprintf(fi, "failed (set %d, get %d)\n", r1, r2);
      continue;
    }
    for (int i = 0; i < r2; i++)
      std::fprintf(fi, "%02X", d[i]);
    std::fprintf(fi, "\n");
  }
  std::fclose(fi);

  out(REQ_SET_OPERATION_MODE, {0x00, 0x00});
  libusb_release_interface(g_dev, 0);
  libusb_close(g_dev);
  libusb_exit(nullptr);
  std::printf("files in %s\n", dir.c_str());
  return 0;
}
