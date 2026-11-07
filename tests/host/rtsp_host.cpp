// Host harness: runs RtspServerCore on a real socket and feeds it access units
// from an Annex-B file on a separate thread, as the encoder task does on the
// device.
// Usage: rtsp_host <port> <file.h264> <fps> <seconds> [stalled_timeout_ms] [max_client_buffer]
//                  [socket_send_buffer]
//
// The file must contain access unit delimiters (x264 aud=1); they mark frame
// boundaries here and are stripped, like the P4 encoder's output (one access
// unit per call, SPS/PPS before each IDR).
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <random>
#include <thread>
#include <vector>

#include "rtsp_server_core.h"

using namespace esphome::rtsp_server;

static uint32_t now_ms() {
  using namespace std::chrono;
  static const auto start = steady_clock::now();
  return static_cast<uint32_t>(duration_cast<milliseconds>(steady_clock::now() - start).count());
}

struct AccessUnit {
  std::vector<uint8_t> data;
  bool keyframe;
};

static std::vector<AccessUnit> load(const char *path) {
  std::ifstream f(path, std::ios::binary);
  std::vector<uint8_t> all((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  std::vector<NalUnit> nals;
  split_annexb(all.data(), all.size(), nals);
  std::vector<AccessUnit> aus;
  for (const auto &n : nals) {
    if (n.type() == 9) {
      aus.push_back({{}, false});
      continue;
    }
    if (aus.empty())
      aus.push_back({{}, false});
    static const uint8_t sc[4] = {0, 0, 0, 1};
    aus.back().data.insert(aus.back().data.end(), sc, sc + 4);
    aus.back().data.insert(aus.back().data.end(), n.data, n.data + n.len);
    if (n.type() == 5)
      aus.back().keyframe = true;
  }
  return aus;
}

int main(int argc, char **argv) {
  if (argc < 5) {
    std::fprintf(stderr, "usage: %s port file.h264 fps seconds\n", argv[0]);
    return 2;
  }
  const int port = std::atoi(argv[1]);
  auto aus = load(argv[2]);
  const int fps = std::atoi(argv[3]);
  const int seconds = std::atoi(argv[4]);
  if (aus.empty() || fps <= 0) {
    std::fprintf(stderr, "no access units\n");
    return 2;
  }
  std::fprintf(stderr, "loaded %zu access units\n", aus.size());

  std::mt19937 rng(12345);
  std::mutex rng_mutex;
  std::atomic<uint32_t> keyframe_requests{0};
  ServerConfig cfg;
  cfg.port = static_cast<uint16_t>(port);
  cfg.path = "thermal";
  cfg.session_name = "Thermal Camera";
  cfg.max_clients = 3;
  // Optional: stalled-client timeout and per-client backlog, for the slow-client test.
  if (argc > 5)
    cfg.stalled_timeout_ms = static_cast<uint32_t>(std::atoi(argv[5]));
  if (argc > 6)
    cfg.max_client_buffer = static_cast<size_t>(std::atoi(argv[6]));
  if (argc > 7)
    cfg.socket_send_buffer = static_cast<size_t>(std::atoi(argv[7]));
  cfg.now_ms = now_ms;
  cfg.random = [&]() {
    std::lock_guard<std::mutex> l(rng_mutex);
    return static_cast<uint32_t>(rng());
  };
  cfg.request_keyframe = [&]() { keyframe_requests++; };
  static const char *const LEVELS[] = {"E", "W", "I", "D"};
  cfg.log = [](LogLevel l, const std::string &m) {
    std::fprintf(stderr, "[%s] %s\n", LEVELS[static_cast<int>(l)], m.c_str());
  };
  RtspServerCore server(cfg);
  if (!server.start())
    return 1;
  std::printf("READY\n");
  std::fflush(stdout);

  std::atomic<bool> running{true};
  std::thread feeder([&]() {
    size_t i = 0;
    const uint32_t period = 1000 / static_cast<uint32_t>(fps);
    uint32_t next = now_ms();
    while (running) {
      const AccessUnit &au = aus[i % aus.size()];
      server.on_access_unit(au.data.data(), au.data.size(), now_ms(), au.keyframe);
      i++;
      next += period;
      int32_t wait = static_cast<int32_t>(next - now_ms());
      if (wait > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(wait));
    }
  });

  const uint32_t end = now_ms() + static_cast<uint32_t>(seconds) * 1000u;
  while (now_ms() < end)
    server.poll(20);
  running = false;
  feeder.join();
  std::fprintf(stderr, "keyframe requests: %u, dropped frames: %u, clients at end: %zu\n",
               keyframe_requests.load(), server.dropped_frames(), server.connected_clients());
  server.stop();
  return 0;
}
