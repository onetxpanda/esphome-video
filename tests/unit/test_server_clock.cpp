// Regression test: a client must never be closed as idle or stalled because
// the clock advanced between reading `now` and stamping its activity.
// The fake clock moves forward on every read, which made the original bug
// (`now - last_activity_ms` wrapping around) happen on every poll.
#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

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

int main() {
  uint32_t clock = 1000;
  std::string log;
  ServerConfig cfg;
  cfg.port = 18599;
  cfg.path = "thermal";
  cfg.now_ms = [&]() { return clock++; };  // advances on every read
  cfg.random = []() { return 0x12345678u; };
  cfg.log = [&](LogLevel, const std::string &m) { log += m + "\n"; };
  RtspServerCore server(cfg);
  CHECK(server.start());

  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(cfg.port);
  a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  CHECK(::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) == 0);

  std::string got;
  for (int i = 0; i < 5; i++) {
    std::string req = "OPTIONS rtsp://127.0.0.1:18599/thermal RTSP/1.0\r\nCSeq: " + std::to_string(i + 1) + "\r\n\r\n";
    // MSG_NOSIGNAL: if the server wrongly closed us, report it rather than die of SIGPIPE.
    CHECK(::send(fd, req.data(), req.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(req.size()));
    for (int p = 0; p < 5; p++)
      server.poll(20);
    char buf[2048];
    ssize_t r = ::recv(fd, buf, sizeof(buf), MSG_DONTWAIT);
    if (r > 0)
      got.append(buf, static_cast<size_t>(r));
  }
  CHECK(server.connected_clients() == 1);
  CHECK(log.find("idle, closing") == std::string::npos);
  CHECK(log.find("not accepting data") == std::string::npos);
  CHECK(got.find("CSeq: 5") != std::string::npos);

  // The idle timeout itself still works: jump the clock past it.
  clock += cfg.idle_timeout_ms + 10;
  server.poll(20);
  CHECK(log.find("idle, closing") != std::string::npos);
  CHECK(server.connected_clients() == 0);

  ::close(fd);
  server.stop();
  std::printf("server clock test: %d failures\n", g_failures);
  return g_failures ? 1 : 0;
}
