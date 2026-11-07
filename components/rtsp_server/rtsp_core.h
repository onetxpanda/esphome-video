#pragma once

// RTSP/RTP protocol helpers with no ESPHome or ESP-IDF dependency, so they can
// be unit-tested on a host. Covers what an H.264 unicast server needs:
// request parsing (with interleaved binary frames skipped), responses, SDP,
// the Transport header, and RFC 6184 packetization (single NAL unit and FU-A)
// into RTSP-interleaved TCP frames (RFC 2326 section 10.12).

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace esphome {
namespace rtsp_server {

// Longest RTSP request accepted; anything longer is a protocol error.
static constexpr size_t MAX_REQUEST_SIZE = 4096;

struct RtspRequest {
  std::string method;
  std::string uri;
  std::string version;
  int cseq{-1};
  // Header names are stored lower-case.
  std::vector<std::pair<std::string, std::string>> headers;
  std::string body;

  // Value of a header (case-insensitive name), or nullptr.
  const std::string *header(const char *name) const;
};

enum class ParseResult {
  INCOMPLETE,   // need more bytes
  REQUEST,      // a request was parsed; `consumed` bytes used
  INTERLEAVED,  // an interleaved binary frame ('$') was skipped; `consumed` bytes used
  ERROR,        // malformed or oversized; the connection should be closed
};

// Parses one request (or skips one interleaved binary frame) from the start of
// `data`. On REQUEST / INTERLEAVED, `consumed` is the number of bytes used.
ParseResult parse_request(const char *data, size_t len, RtspRequest &out, size_t &consumed);

std::string build_response(int code, int cseq, const std::vector<std::pair<std::string, std::string>> &headers,
                           const std::string &body = "");
const char *reason_phrase(int code);

std::string base64_encode(const uint8_t *data, size_t len);

// The path part of an RTSP URL, without a leading or trailing '/'.
// "rtsp://host:554/thermal/trackID=0" -> "thermal/trackID=0"; "*" -> "*".
std::string uri_path(const std::string &uri);

// True if `uri` addresses the stream `path` (the presentation, or its single
// track `trackID=0`).
bool uri_matches(const std::string &uri, const std::string &path, bool *is_track = nullptr);

struct Transport {
  bool valid{false};
  bool tcp{false};
  int rtp_channel{0};
  int rtcp_channel{1};
};
// Parses the client's Transport header and picks the first acceptable spec
// (RTP/AVP/TCP with interleaved channels). `valid` is false if none is.
Transport parse_transport(const std::string &value);

struct NalUnit {
  const uint8_t *data;
  size_t len;
  uint8_t type() const { return len ? (data[0] & 0x1F) : 0; }
};
// Splits an Annex-B byte stream (start codes 00 00 01 / 00 00 00 01) into NAL
// units, without the start codes. Empty NAL units are skipped.
void split_annexb(const uint8_t *data, size_t len, std::vector<NalUnit> &out);

// SDP for one H.264 video track. `sps`/`pps` may be empty (no
// sprop-parameter-sets; parameter sets then arrive in-band with each IDR).
std::string build_sdp(const std::string &session_name, const std::string &server_ip, uint32_t session_id,
                      const std::vector<uint8_t> &sps, const std::vector<uint8_t> &pps);

struct RtpState {
  uint16_t seq{0};
  uint32_t ssrc{0};
};

// Appends one access unit as RTP packets, each wrapped in an interleaved frame
// ('$', channel, 16-bit length). NAL units larger than `max_payload` are split
// into FU-A fragments; the marker bit is set on the access unit's last packet.
// Access unit delimiters (type 9) are dropped. Returns the number of packets.
size_t append_access_unit(std::vector<uint8_t> &out, uint8_t channel, const std::vector<NalUnit> &nals,
                          uint32_t rtp_timestamp, RtpState &state, size_t max_payload);

}  // namespace rtsp_server
}  // namespace esphome
