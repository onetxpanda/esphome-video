#pragma once

// A small RTSP server for one H.264 stream, RTP over TCP interleaved.
// No ESPHome dependency: BSD sockets + std::mutex, so it runs unchanged on
// ESP-IDF (lwIP) and on a host for testing.
//
// Threading: poll() runs on one thread (the server task); on_access_unit()
// may be called from another (the encoder task). Both take the same mutex.

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rtsp_core.h"

namespace esphome {
namespace rtsp_server {

enum class LogLevel : uint8_t { ERROR, WARN, INFO, DEBUG };

struct ServerConfig {
  uint16_t port{554};
  std::string path{"stream"};  // rtsp://host:port/<path>
  std::string session_name{"ESPHome"};
  size_t max_clients{2};
  // Per-client send backlog. Beyond it whole frames are dropped and the client
  // resumes at the next keyframe, so a slow client never stalls the others.
  size_t max_client_buffer{256 * 1024};
  // SO_SNDBUF for client sockets; 0 keeps the OS default. Caps how much video
  // the OS itself queues for a slow client (and so its latency), so the
  // frame-dropping above takes over sooner.
  size_t socket_send_buffer{0};
  size_t max_payload{1400};               // RTP payload bytes per packet
  uint32_t idle_timeout_ms{60000};        // connection that never starts playing
  uint32_t stalled_timeout_ms{30000};     // playing client whose socket accepts nothing

  std::function<uint32_t()> now_ms;                          // required
  std::function<uint32_t()> random;                          // required
  std::function<void()> request_keyframe;                    // optional
  std::function<void(LogLevel, const std::string &)> log;    // optional
};

class RtspServerCore {
 public:
  explicit RtspServerCore(ServerConfig config);
  ~RtspServerCore();
  RtspServerCore(const RtspServerCore &) = delete;
  RtspServerCore &operator=(const RtspServerCore &) = delete;

  bool start();
  void stop();
  bool is_running() const { return this->listen_fd_ >= 0; }

  // Waits up to `timeout_ms` for socket activity and handles it.
  void poll(uint32_t timeout_ms);

  // One encoded access unit (Annex-B). Thread-safe.
  void on_access_unit(const uint8_t *data, size_t length, uint32_t timestamp_ms, bool keyframe);

  size_t playing_clients() const { return this->playing_.load(); }
  size_t connected_clients() const;
  uint32_t dropped_frames() const { return this->dropped_frames_.load(); }

 private:
  // Headroom over max_client_buffer in a client's reserved send buffer: a
  // frame's packet overhead estimate is not exact, and responses go there too.
  static constexpr size_t SEND_SLACK = 64 * 1024;
  enum class State : uint8_t { INIT, READY, PLAYING };
  struct Client {
    int fd{-1};
    std::string peer;
    std::string in;
    SendBuffer out;  // reserved when the client connects; never grows past it
    size_t out_off{0};
    State state{State::INIT};
    std::string session;
    Transport transport;
    RtpState rtp;
    uint32_t ts_base{0};
    bool waiting_keyframe{true};
    bool reject{false};
    bool close_after_flush{false};
    bool dead{false};
    uint32_t last_activity_ms{0};
    uint32_t last_send_progress_ms{0};
  };

  void log_(LogLevel level, const std::string &msg) const;
  void accept_();
  void read_(Client &c);
  void handle_request_(Client &c, const RtspRequest &req);
  void send_response_(Client &c, const std::string &response);
  void make_room_(Client &c, size_t bytes);
  void flush_(Client &c);
  void set_state_(Client &c, State s);
  void close_client_(size_t index);
  std::string local_ip_(int fd) const;
  std::string new_session_id_();

  ServerConfig config_;
  int listen_fd_{-1};
  mutable std::mutex mutex_;
  std::vector<std::unique_ptr<Client>> clients_;
  std::vector<uint8_t> sps_;
  std::vector<uint8_t> pps_;
  std::vector<NalUnit> nal_scratch_;
  uint32_t sdp_session_id_{0};
  std::atomic<size_t> playing_{0};
  std::atomic<uint32_t> dropped_frames_{0};
};

}  // namespace rtsp_server
}  // namespace esphome
