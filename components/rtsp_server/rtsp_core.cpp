#include "rtsp_core.h"

#ifdef USE_ESP32
#include <esp_heap_caps.h>
#endif

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace esphome {
namespace rtsp_server {

void *send_buffer_alloc(size_t bytes) {
#ifdef USE_ESP32
  void *p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM);
  return p != nullptr ? p : heap_caps_malloc(bytes, MALLOC_CAP_DEFAULT);
#else
  return std::malloc(bytes);
#endif
}

void send_buffer_free(void *p) {
#ifdef USE_ESP32
  heap_caps_free(p);
#else
  std::free(p);
#endif
}

static std::string to_lower(std::string s) {
  for (auto &c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

static std::string trim(const std::string &s) {
  size_t b = 0, e = s.size();
  while (b < e && (s[b] == ' ' || s[b] == '\t'))
    b++;
  while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\r'))
    e--;
  return s.substr(b, e - b);
}

const std::string *RtspRequest::header(const char *name) const {
  std::string n = to_lower(name);
  for (const auto &h : this->headers) {
    if (h.first == n)
      return &h.second;
  }
  return nullptr;
}

ParseResult parse_request(const char *data, size_t len, RtspRequest &out, size_t &consumed) {
  consumed = 0;
  if (len == 0)
    return ParseResult::INCOMPLETE;

  // Interleaved binary frame (e.g. RTCP receiver reports over TCP).
  if (data[0] == '$') {
    if (len < 4)
      return ParseResult::INCOMPLETE;
    size_t frame_len = (static_cast<uint8_t>(data[2]) << 8) | static_cast<uint8_t>(data[3]);
    if (len < 4 + frame_len)
      return ParseResult::INCOMPLETE;
    consumed = 4 + frame_len;
    return ParseResult::INTERLEAVED;
  }

  // Find the end of the header block.
  const char *end = nullptr;
  for (size_t i = 0; i + 3 < len; i++) {
    if (data[i] == '\r' && data[i + 1] == '\n' && data[i + 2] == '\r' && data[i + 3] == '\n') {
      end = data + i;
      break;
    }
  }
  if (end == nullptr)
    return len > MAX_REQUEST_SIZE ? ParseResult::ERROR : ParseResult::INCOMPLETE;

  size_t header_len = static_cast<size_t>(end - data) + 4;
  std::string block(data, static_cast<size_t>(end - data));

  out = RtspRequest{};
  size_t pos = 0;
  bool first = true;
  while (pos <= block.size()) {
    size_t nl = block.find("\r\n", pos);
    std::string line = block.substr(pos, nl == std::string::npos ? std::string::npos : nl - pos);
    pos = (nl == std::string::npos) ? block.size() + 1 : nl + 2;
    if (first) {
      first = false;
      size_t s1 = line.find(' ');
      size_t s2 = (s1 == std::string::npos) ? std::string::npos : line.find(' ', s1 + 1);
      if (s1 == std::string::npos || s2 == std::string::npos)
        return ParseResult::ERROR;
      out.method = line.substr(0, s1);
      out.uri = line.substr(s1 + 1, s2 - s1 - 1);
      out.version = line.substr(s2 + 1);
      if (out.method.empty() || out.uri.empty() || out.version.compare(0, 5, "RTSP/") != 0)
        return ParseResult::ERROR;
      continue;
    }
    if (line.empty())
      continue;
    size_t colon = line.find(':');
    if (colon == std::string::npos)
      return ParseResult::ERROR;
    out.headers.emplace_back(to_lower(trim(line.substr(0, colon))), trim(line.substr(colon + 1)));
  }

  const std::string *cseq = out.header("cseq");
  if (cseq != nullptr) {
    char *e = nullptr;
    long v = std::strtol(cseq->c_str(), &e, 10);
    if (e != cseq->c_str() && v >= 0 && v <= 0x7FFFFFFF)
      out.cseq = static_cast<int>(v);
  }

  size_t content_length = 0;
  const std::string *cl = out.header("content-length");
  if (cl != nullptr) {
    char *e = nullptr;
    long v = std::strtol(cl->c_str(), &e, 10);
    if (e == cl->c_str() || v < 0 || static_cast<size_t>(v) > MAX_REQUEST_SIZE)
      return ParseResult::ERROR;
    content_length = static_cast<size_t>(v);
  }
  if (header_len + content_length > MAX_REQUEST_SIZE)
    return ParseResult::ERROR;
  if (len < header_len + content_length)
    return ParseResult::INCOMPLETE;
  out.body.assign(data + header_len, content_length);
  consumed = header_len + content_length;
  return ParseResult::REQUEST;
}

const char *reason_phrase(int code) {
  switch (code) {
    case 200:
      return "OK";
    case 400:
      return "Bad Request";
    case 404:
      return "Not Found";
    case 405:
      return "Method Not Allowed";
    case 453:
      return "Not Enough Bandwidth";
    case 454:
      return "Session Not Found";
    case 455:
      return "Method Not Valid in This State";
    case 459:
      return "Aggregate Operation Not Allowed";
    case 461:
      return "Unsupported Transport";
    case 500:
      return "Internal Server Error";
    case 503:
      return "Service Unavailable";
    case 505:
      return "RTSP Version Not Supported";
    default:
      return "Error";
  }
}

std::string build_response(int code, int cseq, const std::vector<std::pair<std::string, std::string>> &headers,
                           const std::string &body) {
  std::string r = "RTSP/1.0 " + std::to_string(code) + " " + reason_phrase(code) + "\r\n";
  if (cseq >= 0)
    r += "CSeq: " + std::to_string(cseq) + "\r\n";
  for (const auto &h : headers)
    r += h.first + ": " + h.second + "\r\n";
  if (!body.empty())
    r += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  r += "\r\n";
  r += body;
  return r;
}

std::string base64_encode(const uint8_t *data, size_t len) {
  static const char *const TABLE = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string out;
  out.reserve((len + 2) / 3 * 4);
  size_t i = 0;
  while (i + 2 < len) {
    uint32_t v = (data[i] << 16) | (data[i + 1] << 8) | data[i + 2];
    out += TABLE[(v >> 18) & 63];
    out += TABLE[(v >> 12) & 63];
    out += TABLE[(v >> 6) & 63];
    out += TABLE[v & 63];
    i += 3;
  }
  if (i < len) {
    uint32_t v = data[i] << 16;
    if (i + 1 < len)
      v |= data[i + 1] << 8;
    out += TABLE[(v >> 18) & 63];
    out += TABLE[(v >> 12) & 63];
    out += (i + 1 < len) ? TABLE[(v >> 6) & 63] : '=';
    out += '=';
  }
  return out;
}

std::string uri_path(const std::string &uri) {
  if (uri == "*")
    return uri;
  std::string p = uri;
  size_t scheme = p.find("://");
  if (scheme != std::string::npos) {
    size_t slash = p.find('/', scheme + 3);
    p = (slash == std::string::npos) ? std::string() : p.substr(slash);
  }
  size_t q = p.find('?');
  if (q != std::string::npos)
    p = p.substr(0, q);
  while (!p.empty() && p.front() == '/')
    p.erase(0, 1);
  while (!p.empty() && p.back() == '/')
    p.pop_back();
  return p;
}

bool uri_matches(const std::string &uri, const std::string &path, bool *is_track) {
  std::string p = uri_path(uri);
  if (is_track != nullptr)
    *is_track = false;
  if (p == path)
    return true;
  if (p == path + "/trackID=0") {
    if (is_track != nullptr)
      *is_track = true;
    return true;
  }
  return false;
}

Transport parse_transport(const std::string &value) {
  // A client may offer several specs separated by commas, in preference order.
  size_t start = 0;
  while (start <= value.size()) {
    size_t comma = value.find(',', start);
    std::string spec = trim(value.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
    start = (comma == std::string::npos) ? value.size() + 1 : comma + 1;

    Transport t;
    std::string lower = to_lower(spec);
    if (lower.compare(0, 11, "rtp/avp/tcp") != 0)
      continue;
    // Multicast is never offered over TCP; reject anything that asks for it.
    if (lower.find("multicast") != std::string::npos)
      continue;
    t.tcp = true;
    size_t il = lower.find("interleaved=");
    if (il != std::string::npos) {
      const char *s = lower.c_str() + il + 12;
      char *e = nullptr;
      long a = std::strtol(s, &e, 10);
      if (e == s || a < 0 || a > 254)
        continue;
      long b = a + 1;
      if (*e == '-') {
        const char *s2 = e + 1;
        char *e2 = nullptr;
        b = std::strtol(s2, &e2, 10);
        if (e2 == s2 || b < 0 || b > 255)
          continue;
      }
      t.rtp_channel = static_cast<int>(a);
      t.rtcp_channel = static_cast<int>(b);
    }
    t.valid = true;
    return t;
  }
  return Transport{};
}

void split_annexb(const uint8_t *data, size_t len, std::vector<NalUnit> &out) {
  out.clear();
  size_t i = 0;
  // Locate the first start code.
  auto find_start = [&](size_t from, size_t &sc_len) -> size_t {
    for (size_t j = from; j + 2 < len; j++) {
      if (data[j] == 0 && data[j + 1] == 0) {
        if (data[j + 2] == 1) {
          sc_len = 3;
          return j;
        }
        if (j + 3 < len && data[j + 2] == 0 && data[j + 3] == 1) {
          sc_len = 4;
          return j;
        }
      }
    }
    sc_len = 0;
    return len;
  };
  size_t sc = 0;
  i = find_start(0, sc);
  while (i < len) {
    size_t nal_start = i + sc;
    size_t next_sc = 0;
    size_t next = find_start(nal_start, next_sc);
    size_t nal_end = next;
    // Trailing zero bytes belong to the next start code (00 00 00 01) or are padding.
    while (nal_end > nal_start && data[nal_end - 1] == 0)
      nal_end--;
    if (nal_end > nal_start)
      out.push_back(NalUnit{data + nal_start, nal_end - nal_start});
    i = next;
    sc = next_sc;
  }
}

std::string build_sdp(const std::string &session_name, const std::string &server_ip, uint32_t session_id,
                      const std::vector<uint8_t> &sps, const std::vector<uint8_t> &pps) {
  std::string ip = server_ip.empty() ? std::string("0.0.0.0") : server_ip;
  std::string sdp;
  sdp += "v=0\r\n";
  sdp += "o=- " + std::to_string(session_id) + " 1 IN IP4 " + ip + "\r\n";
  sdp += "s=" + (session_name.empty() ? std::string("H.264") : session_name) + "\r\n";
  sdp += "c=IN IP4 0.0.0.0\r\n";
  sdp += "t=0 0\r\n";
  sdp += "a=control:*\r\n";
  sdp += "a=range:npt=now-\r\n";
  sdp += "m=video 0 RTP/AVP 96\r\n";
  sdp += "a=rtpmap:96 H264/90000\r\n";
  std::string fmtp = "a=fmtp:96 packetization-mode=1";
  if (sps.size() >= 4) {
    char pli[8];
    std::snprintf(pli, sizeof(pli), "%02X%02X%02X", sps[1], sps[2], sps[3]);
    fmtp += std::string(";profile-level-id=") + pli;
    if (!pps.empty()) {
      fmtp += ";sprop-parameter-sets=" + base64_encode(sps.data(), sps.size()) + "," +
              base64_encode(pps.data(), pps.size());
    }
  }
  sdp += fmtp + "\r\n";
  sdp += "a=control:trackID=0\r\n";
  return sdp;
}

static void put_rtp_header(SendBuffer &out, uint8_t channel, size_t payload_len, bool marker,
                           uint32_t ts, RtpState &state) {
  size_t rtp_len = 12 + payload_len;
  out.push_back('$');
  out.push_back(channel);
  out.push_back(static_cast<uint8_t>(rtp_len >> 8));
  out.push_back(static_cast<uint8_t>(rtp_len & 0xFF));
  out.push_back(0x80);  // V=2, P=0, X=0, CC=0
  out.push_back(static_cast<uint8_t>((marker ? 0x80 : 0x00) | 96));
  out.push_back(static_cast<uint8_t>(state.seq >> 8));
  out.push_back(static_cast<uint8_t>(state.seq & 0xFF));
  state.seq++;
  out.push_back(static_cast<uint8_t>(ts >> 24));
  out.push_back(static_cast<uint8_t>(ts >> 16));
  out.push_back(static_cast<uint8_t>(ts >> 8));
  out.push_back(static_cast<uint8_t>(ts));
  out.push_back(static_cast<uint8_t>(state.ssrc >> 24));
  out.push_back(static_cast<uint8_t>(state.ssrc >> 16));
  out.push_back(static_cast<uint8_t>(state.ssrc >> 8));
  out.push_back(static_cast<uint8_t>(state.ssrc));
}

size_t append_access_unit(SendBuffer &out, uint8_t channel, const std::vector<NalUnit> &nals,
                          uint32_t rtp_timestamp, RtpState &state, size_t max_payload) {
  // The interleaved length field is 16 bits; keep every packet well inside it.
  if (max_payload < 16)
    max_payload = 16;
  if (max_payload > 65000)
    max_payload = 65000;

  // Index of the last NAL unit that will be sent, for the marker bit.
  int last = -1;
  for (size_t i = 0; i < nals.size(); i++) {
    if (nals[i].len > 0 && nals[i].type() != 9)
      last = static_cast<int>(i);
  }
  size_t packets = 0;
  for (size_t i = 0; i < nals.size(); i++) {
    const NalUnit &nal = nals[i];
    if (nal.len == 0 || nal.type() == 9)
      continue;
    bool last_nal = static_cast<int>(i) == last;
    if (nal.len <= max_payload) {
      put_rtp_header(out, channel, nal.len, last_nal, rtp_timestamp, state);
      out.insert(out.end(), nal.data, nal.data + nal.len);
      packets++;
      continue;
    }
    // FU-A: the NAL header is replaced by an indicator + FU header per fragment.
    const uint8_t indicator = static_cast<uint8_t>((nal.data[0] & 0xE0) | 28);
    const uint8_t type = static_cast<uint8_t>(nal.data[0] & 0x1F);
    const size_t chunk = max_payload - 2;
    size_t off = 1;
    while (off < nal.len) {
      size_t n = nal.len - off;
      if (n > chunk)
        n = chunk;
      bool start = off == 1;
      bool end = off + n == nal.len;
      put_rtp_header(out, channel, n + 2, last_nal && end, rtp_timestamp, state);
      out.push_back(indicator);
      out.push_back(static_cast<uint8_t>((start ? 0x80 : 0) | (end ? 0x40 : 0) | type));
      out.insert(out.end(), nal.data + off, nal.data + off + n);
      off += n;
      packets++;
    }
  }
  return packets;
}

}  // namespace rtsp_server
}  // namespace esphome
