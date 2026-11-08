#include "rtsp_server_core.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

namespace esphome {
namespace rtsp_server {

static constexpr size_t MAX_INPUT_BUFFER = MAX_REQUEST_SIZE + 2048;
static constexpr const char *PUBLIC_METHODS = "OPTIONS, DESCRIBE, SETUP, PLAY, PAUSE, TEARDOWN, GET_PARAMETER, SET_PARAMETER";

static bool set_nonblocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  return flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

RtspServerCore::RtspServerCore(ServerConfig config) : config_(std::move(config)) {}

RtspServerCore::~RtspServerCore() { this->stop(); }

void RtspServerCore::log_(LogLevel level, const std::string &msg) const {
  if (this->config_.log)
    this->config_.log(level, msg);
}

bool RtspServerCore::start() {
  if (this->listen_fd_ >= 0)
    return true;
  int fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    this->log_(LogLevel::ERROR, "socket() failed: " + std::to_string(errno));
    return false;
  }
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(this->config_.port);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 || ::listen(fd, 4) != 0 ||
      !set_nonblocking(fd)) {
    this->log_(LogLevel::ERROR, "Cannot listen on port " + std::to_string(this->config_.port) + ": errno " +
                                    std::to_string(errno));
    ::close(fd);
    return false;
  }
  this->sdp_session_id_ = this->config_.random();
  this->listen_fd_ = fd;
  this->log_(LogLevel::INFO, "Listening on port " + std::to_string(this->config_.port) + ", path /" +
                                 this->config_.path);
  return true;
}

void RtspServerCore::stop() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  while (!this->clients_.empty())
    this->close_client_(this->clients_.size() - 1);
  if (this->listen_fd_ >= 0) {
    ::close(this->listen_fd_);
    this->listen_fd_ = -1;
  }
}

size_t RtspServerCore::connected_clients() const {
  std::lock_guard<std::mutex> lock(this->mutex_);
  return this->clients_.size();
}

void RtspServerCore::poll(uint32_t timeout_ms) {
  if (this->listen_fd_ < 0)
    return;
  fd_set rfds, wfds;
  FD_ZERO(&rfds);
  FD_ZERO(&wfds);
  int maxfd = this->listen_fd_;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    // Accept only while there is room for another client in the fd set.
    FD_SET(this->listen_fd_, &rfds);
    for (auto &c : this->clients_) {
      if (c->dead)
        continue;
      FD_SET(c->fd, &rfds);
      if (c->out.size() > c->out_off)
        FD_SET(c->fd, &wfds);
      if (c->fd > maxfd)
        maxfd = c->fd;
    }
  }
  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;
  int n = ::select(maxfd + 1, &rfds, &wfds, nullptr, &tv);
  if (n < 0 && errno != EINTR) {
    this->log_(LogLevel::WARN, "select() failed: " + std::to_string(errno));
    return;
  }

  std::lock_guard<std::mutex> lock(this->mutex_);
  if (n > 0 && FD_ISSET(this->listen_fd_, &rfds))
    this->accept_();
  for (auto &cp : this->clients_) {
    Client &c = *cp;
    if (c.dead)
      continue;
    if (n > 0 && FD_ISSET(c.fd, &rfds))
      this->read_(c);
    if (!c.dead && c.out.size() > c.out_off)
      this->flush_(c);
    if (c.dead)
      continue;
    if (c.close_after_flush && c.out.size() == c.out_off) {
      c.dead = true;
      continue;
    }
    // Read the clock after this client's I/O: read_() and flush_() stamp it
    // with the current time, and an earlier `now` would make `now - stamp`
    // wrap around and close a healthy client as idle or stalled.
    const uint32_t now = this->config_.now_ms();
    if (c.state != State::PLAYING && now - c.last_activity_ms > this->config_.idle_timeout_ms) {
      this->log_(LogLevel::INFO, c.peer + ": idle, closing");
      c.dead = true;
    } else if (c.out.size() > c.out_off && now - c.last_send_progress_ms > this->config_.stalled_timeout_ms) {
      this->log_(LogLevel::INFO, c.peer + ": not accepting data, closing");
      c.dead = true;
    }
  }
  for (size_t i = this->clients_.size(); i-- > 0;) {
    if (this->clients_[i]->dead)
      this->close_client_(i);
  }
}

void RtspServerCore::accept_() {
  for (;;) {
    sockaddr_in addr{};
    socklen_t alen = sizeof(addr);
    int fd = ::accept(this->listen_fd_, reinterpret_cast<sockaddr *>(&addr), &alen);
    if (fd < 0)
      return;
    if (fd >= FD_SETSIZE || !set_nonblocking(fd)) {
      ::close(fd);
      continue;
    }
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
    if (this->config_.socket_send_buffer > 0) {
      int sz = static_cast<int>(this->config_.socket_send_buffer);
      setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sz, sizeof(sz));
    }
    auto c = std::make_unique<Client>();
    c->fd = fd;
    // All of this client's budget at once, so sending never reallocates (a
    // growing vector needs old and new copies at the same time).
    c->out.reserve(this->config_.max_client_buffer + SEND_SLACK);
    char ip[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
    c->peer = std::string(ip) + ":" + std::to_string(ntohs(addr.sin_port));
    c->last_activity_ms = c->last_send_progress_ms = this->config_.now_ms();
    // Over the limit: answer its first request with 503, then close.
    c->reject = this->clients_.size() >= this->config_.max_clients;
    this->log_(LogLevel::DEBUG, c->peer + ": connected" + (c->reject ? " (over client limit)" : ""));
    this->clients_.push_back(std::move(c));
  }
}

void RtspServerCore::read_(Client &c) {
  char buf[1024];
  for (;;) {
    ssize_t r = ::recv(c.fd, buf, sizeof(buf), 0);
    if (r > 0) {
      c.last_activity_ms = this->config_.now_ms();
      c.in.append(buf, static_cast<size_t>(r));
      if (c.in.size() > MAX_INPUT_BUFFER * 4) {
        c.dead = true;
        return;
      }
      continue;
    }
    if (r == 0) {
      this->log_(LogLevel::DEBUG, c.peer + ": disconnected");
      c.dead = true;
      return;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
      break;
    c.dead = true;
    return;
  }

  size_t pos = 0;
  while (!c.dead && !c.close_after_flush && pos < c.in.size()) {
    RtspRequest req;
    size_t consumed = 0;
    ParseResult pr = parse_request(c.in.data() + pos, c.in.size() - pos, req, consumed);
    if (pr == ParseResult::INCOMPLETE)
      break;
    if (pr == ParseResult::ERROR) {
      this->log_(LogLevel::WARN, c.peer + ": malformed request, closing");
      this->send_response_(c, build_response(400, -1, {}));
      c.close_after_flush = true;
      break;
    }
    pos += consumed;
    if (pr == ParseResult::REQUEST)
      this->handle_request_(c, req);
  }
  c.in.erase(0, pos);
  if (c.in.size() > MAX_INPUT_BUFFER) {
    this->log_(LogLevel::WARN, c.peer + ": request too large, closing");
    c.dead = true;
  }
}

void RtspServerCore::handle_request_(Client &c, const RtspRequest &req) {
  const int cseq = req.cseq;
  std::vector<std::pair<std::string, std::string>> h;
  h.emplace_back("Server", "ESPHome rtsp_server");
  this->log_(LogLevel::DEBUG, c.peer + ": " + req.method + " " + req.uri);

  if (c.reject) {
    this->send_response_(c, build_response(503, cseq, h));
    c.close_after_flush = true;
    return;
  }
  if (req.version != "RTSP/1.0") {
    this->send_response_(c, build_response(505, cseq, h));
    return;
  }
  if (cseq < 0) {
    this->send_response_(c, build_response(400, -1, h));
    return;
  }

  // Every request after SETUP must carry the session it was given.
  auto session_ok = [&]() -> bool {
    const std::string *s = req.header("session");
    if (s == nullptr || c.session.empty())
      return false;
    std::string id = s->substr(0, s->find(';'));
    return id == c.session;
  };
  auto session_header = [&]() { h.emplace_back("Session", c.session + ";timeout=60"); };

  const std::string &m = req.method;
  if (m == "OPTIONS") {
    h.emplace_back("Public", PUBLIC_METHODS);
    this->send_response_(c, build_response(200, cseq, h));
    return;
  }
  if (m == "DESCRIBE") {
    bool track = false;
    if (!uri_matches(req.uri, this->config_.path, &track) || track) {
      this->send_response_(c, build_response(404, cseq, h));
      return;
    }
    std::string base = req.uri;
    if (base.empty() || base.back() != '/')
      base += '/';
    std::string sdp = build_sdp(this->config_.session_name, this->local_ip_(c.fd), this->sdp_session_id_,
                                this->sps_, this->pps_);
    h.emplace_back("Content-Base", base);
    h.emplace_back("Content-Type", "application/sdp");
    this->send_response_(c, build_response(200, cseq, h, sdp));
    return;
  }
  if (m == "SETUP") {
    if (!uri_matches(req.uri, this->config_.path)) {
      this->send_response_(c, build_response(404, cseq, h));
      return;
    }
    if (!c.session.empty() && !session_ok()) {
      this->send_response_(c, build_response(454, cseq, h));
      return;
    }
    if (c.state == State::PLAYING) {
      this->send_response_(c, build_response(455, cseq, h));
      return;
    }
    const std::string *tv = req.header("transport");
    Transport t = tv ? parse_transport(*tv) : Transport{};
    if (!t.valid) {
      // Only TCP interleaved is offered; clients such as ffmpeg then retry with TCP.
      h.emplace_back("Transport", "RTP/AVP/TCP;unicast;interleaved=0-1");
      this->send_response_(c, build_response(461, cseq, h));
      return;
    }
    if (c.session.empty()) {
      c.session = this->new_session_id_();
      c.rtp.seq = static_cast<uint16_t>(this->config_.random());
      c.rtp.ssrc = this->config_.random();
      c.ts_base = this->config_.random();
    }
    c.transport = t;
    this->set_state_(c, State::READY);
    char ssrc[9];
    std::snprintf(ssrc, sizeof(ssrc), "%08X", static_cast<unsigned>(c.rtp.ssrc));
    h.emplace_back("Transport", "RTP/AVP/TCP;unicast;interleaved=" + std::to_string(t.rtp_channel) + "-" +
                                    std::to_string(t.rtcp_channel) + ";ssrc=" + ssrc);
    session_header();
    this->send_response_(c, build_response(200, cseq, h));
    return;
  }
  if (m == "PLAY") {
    if (!session_ok()) {
      this->send_response_(c, build_response(454, cseq, h));
      return;
    }
    if (c.state == State::INIT) {
      this->send_response_(c, build_response(455, cseq, h));
      return;
    }
    std::string base = req.uri;
    while (!base.empty() && base.back() == '/')
      base.pop_back();
    session_header();
    h.emplace_back("Range", "npt=now-");
    h.emplace_back("RTP-Info", "url=" + base + "/trackID=0;seq=" + std::to_string(c.rtp.seq));
    this->send_response_(c, build_response(200, cseq, h));
    // The response must precede the first RTP packet; it does, since both share
    // this client's ordered send buffer. Start at the next keyframe.
    c.waiting_keyframe = true;
    this->set_state_(c, State::PLAYING);
    if (this->config_.request_keyframe)
      this->config_.request_keyframe();
    this->log_(LogLevel::INFO, c.peer + ": playing");
    return;
  }
  if (m == "PAUSE") {
    if (!session_ok()) {
      this->send_response_(c, build_response(454, cseq, h));
      return;
    }
    if (c.state == State::PLAYING)
      this->set_state_(c, State::READY);
    session_header();
    this->send_response_(c, build_response(200, cseq, h));
    return;
  }
  if (m == "TEARDOWN") {
    if (!c.session.empty() && !session_ok()) {
      this->send_response_(c, build_response(454, cseq, h));
      return;
    }
    this->set_state_(c, State::INIT);
    this->send_response_(c, build_response(200, cseq, h));
    c.close_after_flush = true;
    return;
  }
  if (m == "GET_PARAMETER" || m == "SET_PARAMETER") {
    // Used as keepalives. A session is optional for GET_PARAMETER with no body.
    if (!c.session.empty() && req.header("session") != nullptr && !session_ok()) {
      this->send_response_(c, build_response(454, cseq, h));
      return;
    }
    if (!c.session.empty())
      session_header();
    this->send_response_(c, build_response(200, cseq, h));
    return;
  }
  h.emplace_back("Allow", PUBLIC_METHODS);
  this->send_response_(c, build_response(405, cseq, h));
}

void RtspServerCore::send_response_(Client &c, const std::string &response) {
  this->make_room_(c, response.size());
  c.out.insert(c.out.end(), response.begin(), response.end());
}

// Drops what has been sent from the front if `bytes` more would not fit in
// the reserved capacity, so appending does not reallocate.
void RtspServerCore::make_room_(Client &c, size_t bytes) {
  if (c.out_off == 0 || c.out.size() + bytes <= c.out.capacity())
    return;
  c.out.erase(c.out.begin(), c.out.begin() + static_cast<std::ptrdiff_t>(c.out_off));
  c.out_off = 0;
}

void RtspServerCore::set_state_(Client &c, State s) {
  if (c.state == State::PLAYING && s != State::PLAYING)
    this->playing_--;
  else if (c.state != State::PLAYING && s == State::PLAYING)
    this->playing_++;
  c.state = s;
}

void RtspServerCore::flush_(Client &c) {
  while (c.out.size() > c.out_off) {
    ssize_t w = ::send(c.fd, c.out.data() + c.out_off, c.out.size() - c.out_off, MSG_DONTWAIT | MSG_NOSIGNAL);
    if (w > 0) {
      c.out_off += static_cast<size_t>(w);
      c.last_send_progress_ms = this->config_.now_ms();
      continue;
    }
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
      break;
    c.dead = true;
    return;
  }
  if (c.out_off == c.out.size()) {
    c.out.clear();
    c.out_off = 0;
  } else if (c.out_off > 16 * 1024 && c.out_off * 2 > c.out.size()) {
    c.out.erase(c.out.begin(), c.out.begin() + static_cast<std::ptrdiff_t>(c.out_off));
    c.out_off = 0;
  }
}

void RtspServerCore::close_client_(size_t index) {
  Client &c = *this->clients_[index];
  this->set_state_(c, State::INIT);
  if (c.fd >= 0)
    ::close(c.fd);
  this->log_(LogLevel::DEBUG, c.peer + ": closed");
  this->clients_.erase(this->clients_.begin() + static_cast<std::ptrdiff_t>(index));
}

void RtspServerCore::on_access_unit(const uint8_t *data, size_t length, uint32_t timestamp_ms, bool keyframe) {
  if (data == nullptr || length == 0)
    return;
  std::lock_guard<std::mutex> lock(this->mutex_);
  split_annexb(data, length, this->nal_scratch_);
  for (const auto &nal : this->nal_scratch_) {
    if (nal.type() == 7)
      this->sps_.assign(nal.data, nal.data + nal.len);
    else if (nal.type() == 8)
      this->pps_.assign(nal.data, nal.data + nal.len);
  }
  if (this->playing_.load() == 0)
    return;

  bool want_keyframe = false;
  for (auto &cp : this->clients_) {
    Client &c = *cp;
    if (c.dead || c.state != State::PLAYING)
      continue;
    if (c.waiting_keyframe) {
      if (!keyframe) {
        want_keyframe = true;
        continue;
      }
      c.waiting_keyframe = false;
    }
    // Room for this frame plus packet overhead (16 bytes per packet)?
    size_t estimate = length + 16 * (length / this->config_.max_payload + this->nal_scratch_.size() + 1);
    if (c.out.size() - c.out_off + estimate > this->config_.max_client_buffer) {
      // Never send part of a frame: drop it and resume at the next keyframe.
      c.waiting_keyframe = true;
      want_keyframe = true;
      this->dropped_frames_++;
      continue;
    }
    uint32_t rtp_ts = c.ts_base + timestamp_ms * 90u;
    this->make_room_(c, estimate);
    append_access_unit(c.out, static_cast<uint8_t>(c.transport.rtp_channel), this->nal_scratch_, rtp_ts, c.rtp,
                       this->config_.max_payload);
    this->flush_(c);
  }
  if (want_keyframe && this->config_.request_keyframe)
    this->config_.request_keyframe();
}

std::string RtspServerCore::local_ip_(int fd) const {
  sockaddr_in addr{};
  socklen_t alen = sizeof(addr);
  if (::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &alen) != 0)
    return "";
  char ip[INET_ADDRSTRLEN] = {0};
  inet_ntop(AF_INET, &addr.sin_addr, ip, sizeof(ip));
  return ip;
}

std::string RtspServerCore::new_session_id_() {
  char buf[17];
  std::snprintf(buf, sizeof(buf), "%08X%08X", static_cast<unsigned>(this->config_.random()),
                static_cast<unsigned>(this->config_.random()));
  return buf;
}

}  // namespace rtsp_server
}  // namespace esphome
