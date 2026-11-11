// A client that falls behind gets one keyframe request per stall, made once
// it has room for a keyframe again, not one request per frame while it
// waits (which made every other frame a keyframe on a slow link).
#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include "rtsp_server_core.h"

using namespace esphome::rtsp_server;

static int g_failures = 0;
#define CHECK(cond)                                               \
  do {                                                            \
    if (!(cond)) {                                                \
      g_failures++;                                               \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
    }                                                             \
  } while (0)

// One Annex-B access unit: NAL units of the given types, `size` bytes each.
static std::vector<uint8_t> access_unit(std::initializer_list<uint8_t> types, size_t size) {
  std::vector<uint8_t> au;
  for (uint8_t t : types) {
    const uint8_t start[] = {0, 0, 0, 1, static_cast<uint8_t>(0x60 | t)};
    au.insert(au.end(), start, start + sizeof(start));
    au.insert(au.end(), size, 0x55);
  }
  return au;
}

static std::string drain(int fd) {
  std::string got;
  char buf[16384];
  ssize_t r;
  while ((r = ::recv(fd, buf, sizeof(buf), MSG_DONTWAIT)) > 0)
    got.append(buf, static_cast<size_t>(r));
  return got;
}

static std::string request(RtspServerCore &server, int fd, const std::string &req) {
  CHECK(::send(fd, req.data(), req.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(req.size()));
  std::string got;
  for (int i = 0; i < 20 && got.find("\r\n\r\n") == std::string::npos; i++) {
    server.poll(10);
    got += drain(fd);
  }
  return got;
}

int main() {
  uint32_t clock = 1000;
  unsigned requests = 0;
  ServerConfig cfg;
  cfg.port = 18598;
  cfg.path = "thermal";
  cfg.max_client_buffer = 32 * 1024;
  cfg.socket_send_buffer = 4096;
  cfg.now_ms = [&]() { return clock; };
  cfg.random = []() { return 0x12345678u; };
  cfg.request_keyframe = [&]() { requests++; };
  RtspServerCore server(cfg);
  CHECK(server.start());

  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  int small = 4096;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &small, sizeof(small));
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(cfg.port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  CHECK(::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0);

  const std::string url = "rtsp://127.0.0.1:18598/thermal";
  std::string r = request(server, fd,
                          "SETUP " + url + "/trackID=0 RTSP/1.0\r\nCSeq: 1\r\n"
                          "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n");
  const size_t s = r.find("Session: ");
  CHECK(s != std::string::npos);
  const std::string session = r.substr(s + 9, r.find_first_of(";\r", s + 9) - (s + 9));
  r = request(server, fd, "PLAY " + url + " RTSP/1.0\r\nCSeq: 2\r\nSession: " + session + "\r\n\r\n");
  CHECK(r.find("200 OK") != std::string::npos);
  CHECK(requests == 1);  // PLAY starts at a keyframe

  const auto key = access_unit({7, 8, 5}, 2000);
  const auto delta = access_unit({1}, 4000);
  uint32_t ts = 0;
  server.on_access_unit(key.data(), key.size(), ts += 40, true);

  // The client stops reading: frames pile up, then are dropped. While it
  // stays behind, no further keyframe is requested.
  for (int i = 0; i < 200; i++) {
    server.on_access_unit(delta.data(), delta.size(), ts += 40, false);
    server.poll(0);
  }
  const uint32_t dropped = server.dropped_frames();
  CHECK(dropped > 0);
  CHECK(requests == 1);

  // It catches up: the next frame asks for one keyframe, and only one.
  for (int i = 0; i < 200; i++) {
    drain(fd);
    server.poll(1);
  }
  drain(fd);
  server.on_access_unit(delta.data(), delta.size(), ts += 40, false);
  CHECK(requests == 2);
  server.on_access_unit(delta.data(), delta.size(), ts += 40, false);
  CHECK(requests == 2);

  // The keyframe is sent, and the frames after it follow without requests.
  server.on_access_unit(key.data(), key.size(), ts += 40, true);
  server.on_access_unit(delta.data(), delta.size(), ts += 40, false);
  CHECK(server.dropped_frames() == dropped);
  CHECK(requests == 2);

  ::close(fd);
  server.stop();
  std::printf("keyframe request test: %u requests, %u dropped, %d failures\n", requests, dropped, g_failures);
  return g_failures ? 1 : 0;
}
