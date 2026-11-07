// Unit tests for rtsp_core (no sockets). Built with sanitizers by run_tests.py.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "rtsp_core.h"

using namespace esphome::rtsp_server;

static int g_checks = 0, g_failures = 0;
#define CHECK(cond)                                                     \
  do {                                                                  \
    g_checks++;                                                         \
    if (!(cond)) {                                                      \
      g_failures++;                                                     \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
    }                                                                   \
  } while (0)

static ParseResult parse(const std::string &s, RtspRequest &r, size_t &consumed) {
  return parse_request(s.data(), s.size(), r, consumed);
}

static void test_parse() {
  RtspRequest r;
  size_t n = 0;
  std::string opt = "OPTIONS rtsp://10.0.0.5:554/thermal RTSP/1.0\r\nCSeq: 2\r\nUser-Agent: x\r\n\r\n";
  CHECK(parse(opt, r, n) == ParseResult::REQUEST);
  CHECK(n == opt.size());
  CHECK(r.method == "OPTIONS" && r.uri == "rtsp://10.0.0.5:554/thermal" && r.version == "RTSP/1.0");
  CHECK(r.cseq == 2);
  CHECK(r.header("USER-AGENT") != nullptr && *r.header("user-agent") == "x");

  // Every prefix is incomplete, never an error.
  bool all_incomplete = true;
  for (size_t i = 0; i < opt.size(); i++) {
    if (parse_request(opt.data(), i, r, n) != ParseResult::INCOMPLETE)
      all_incomplete = false;
  }
  CHECK(all_incomplete);

  // Two pipelined requests: the first is consumed exactly.
  std::string two = opt + "DESCRIBE rtsp://h/thermal RTSP/1.0\r\nCSeq: 3\r\n\r\n";
  CHECK(parse(two, r, n) == ParseResult::REQUEST && n == opt.size());
  CHECK(parse(two.substr(n), r, n) == ParseResult::REQUEST && r.method == "DESCRIBE" && r.cseq == 3);

  // Body with Content-Length.
  std::string body = "SET_PARAMETER rtsp://h/s RTSP/1.0\r\nCSeq: 9\r\nContent-Length: 5\r\n\r\nhello";
  CHECK(parse(body, r, n) == ParseResult::REQUEST && r.body == "hello" && n == body.size());
  CHECK(parse(body.substr(0, body.size() - 1), r, n) == ParseResult::INCOMPLETE);

  // Interleaved binary frames (RTCP over TCP) are skipped whole.
  std::string il("$\x01\x00\x04" "abcd", 8);
  CHECK(parse(il, r, n) == ParseResult::INTERLEAVED && n == 8);
  CHECK(parse(il.substr(0, 6), r, n) == ParseResult::INCOMPLETE);

  // Malformed / oversized.
  CHECK(parse("GARBAGE\r\n\r\n", r, n) == ParseResult::ERROR);
  CHECK(parse("OPTIONS * HTTP/1.1\r\nCSeq: 1\r\n\r\n", r, n) == ParseResult::ERROR);
  CHECK(parse("OPTIONS * RTSP/1.0\r\nNoColonHere\r\n\r\n", r, n) == ParseResult::ERROR);
  CHECK(parse("OPTIONS * RTSP/1.0\r\nCSeq: 1\r\nContent-Length: 999999\r\n\r\n", r, n) == ParseResult::ERROR);
  CHECK(parse(std::string(MAX_REQUEST_SIZE + 10, 'A'), r, n) == ParseResult::ERROR);
  // Missing / bad CSeq parses, but cseq stays -1 (the server answers 400).
  CHECK(parse("OPTIONS * RTSP/1.0\r\n\r\n", r, n) == ParseResult::REQUEST && r.cseq == -1);
  CHECK(parse("OPTIONS * RTSP/1.0\r\nCSeq: abc\r\n\r\n", r, n) == ParseResult::REQUEST && r.cseq == -1);
}

static void test_uri() {
  CHECK(uri_path("rtsp://10.0.0.5:554/thermal") == "thermal");
  CHECK(uri_path("rtsp://10.0.0.5:554/thermal/") == "thermal");
  CHECK(uri_path("rtsp://10.0.0.5/thermal/trackID=0") == "thermal/trackID=0");
  CHECK(uri_path("rtsp://10.0.0.5:554") == "");
  CHECK(uri_path("rtsp://h/thermal?x=1") == "thermal");
  CHECK(uri_path("*") == "*");
  bool track = true;
  CHECK(uri_matches("rtsp://h:554/thermal", "thermal", &track) && !track);
  CHECK(uri_matches("rtsp://h:554/thermal/trackID=0", "thermal", &track) && track);
  CHECK(!uri_matches("rtsp://h:554/other", "thermal"));
  CHECK(!uri_matches("rtsp://h:554/thermal2", "thermal"));
}

static void test_transport() {
  Transport t = parse_transport("RTP/AVP/TCP;unicast;interleaved=0-1");
  CHECK(t.valid && t.tcp && t.rtp_channel == 0 && t.rtcp_channel == 1);
  t = parse_transport("RTP/AVP/TCP;unicast;interleaved=4-5");
  CHECK(t.valid && t.rtp_channel == 4 && t.rtcp_channel == 5);
  t = parse_transport("RTP/AVP/TCP;unicast");
  CHECK(t.valid && t.rtp_channel == 0 && t.rtcp_channel == 1);
  t = parse_transport("RTP/AVP;unicast;client_port=5000-5001");
  CHECK(!t.valid);
  t = parse_transport("RTP/AVP;unicast;client_port=5000-5001,RTP/AVP/TCP;unicast;interleaved=2-3");
  CHECK(t.valid && t.rtp_channel == 2);
  t = parse_transport("RTP/AVP/TCP;interleaved=x-1");
  CHECK(!t.valid);
  t = parse_transport("RTP/AVP/TCP;interleaved=300-301");
  CHECK(!t.valid);
}

static void test_base64() {
  auto b = [](const char *s) { return base64_encode(reinterpret_cast<const uint8_t *>(s), std::strlen(s)); };
  CHECK(b("") == "");
  CHECK(b("f") == "Zg==");
  CHECK(b("fo") == "Zm8=");
  CHECK(b("foo") == "Zm9v");
  CHECK(b("foobar") == "Zm9vYmFy");
}

static void test_split() {
  const uint8_t s[] = {0, 0, 0, 1, 0x67, 1, 2, 0, 0, 1, 0x68, 3, 0, 0, 0, 1, 0x65, 4, 5, 6, 0, 0};
  std::vector<NalUnit> v;
  split_annexb(s, sizeof(s), v);
  CHECK(v.size() == 3);
  CHECK(v[0].type() == 7 && v[0].len == 3);
  CHECK(v[1].type() == 8 && v[1].len == 2);
  CHECK(v[2].type() == 5 && v[2].len == 4);  // trailing zeros trimmed
  split_annexb(s, 0, v);
  CHECK(v.empty());
  const uint8_t junk[] = {1, 2, 3};
  split_annexb(junk, sizeof(junk), v);
  CHECK(v.empty());
}

struct Packet {
  uint8_t channel;
  bool marker;
  uint16_t seq;
  uint32_t ts, ssrc;
  std::vector<uint8_t> payload;
};

static std::vector<Packet> unframe(const std::vector<uint8_t> &out, bool &ok) {
  std::vector<Packet> p;
  ok = true;
  size_t i = 0;
  while (i < out.size()) {
    if (out[i] != '$' || i + 4 > out.size()) {
      ok = false;
      return p;
    }
    size_t len = (out[i + 2] << 8) | out[i + 3];
    if (i + 4 + len > out.size() || len < 12) {
      ok = false;
      return p;
    }
    const uint8_t *r = out.data() + i + 4;
    Packet k;
    k.channel = out[i + 1];
    if (r[0] != 0x80 || (r[1] & 0x7F) != 96)
      ok = false;
    k.marker = (r[1] & 0x80) != 0;
    k.seq = static_cast<uint16_t>((r[2] << 8) | r[3]);
    k.ts = (uint32_t(r[4]) << 24) | (r[5] << 16) | (r[6] << 8) | r[7];
    k.ssrc = (uint32_t(r[8]) << 24) | (r[9] << 16) | (r[10] << 8) | r[11];
    k.payload.assign(r + 12, r + len);
    p.push_back(k);
    i += 4 + len;
  }
  return p;
}

static void test_packetize() {
  // AUD, SPS, PPS, a 5000-byte IDR slice.
  std::vector<uint8_t> aud = {0x09, 0xF0};
  std::vector<uint8_t> sps = {0x67, 0x42, 0xC0, 0x1E, 0xAA};
  std::vector<uint8_t> pps = {0x68, 0xCE, 0x38, 0x80};
  std::vector<uint8_t> idr(5000);
  idr[0] = 0x65;
  for (size_t i = 1; i < idr.size(); i++)
    idr[i] = static_cast<uint8_t>(i * 7 + 3);
  std::vector<NalUnit> nals = {{aud.data(), aud.size()}, {sps.data(), sps.size()}, {pps.data(), pps.size()},
                               {idr.data(), idr.size()}};
  RtpState st;
  st.seq = 65534;  // exercises wrap-around
  st.ssrc = 0xDEADBEEF;
  std::vector<uint8_t> out;
  size_t n = append_access_unit(out, 2, nals, 123456, st, 1400);
  bool ok = false;
  auto pk = unframe(out, ok);
  CHECK(ok);
  // SPS + PPS single, IDR in ceil(4999 / 1398) = 4 FU-A fragments; AUD dropped.
  CHECK(n == 6 && pk.size() == 6);
  CHECK(st.seq == static_cast<uint16_t>(65534 + 6));
  bool seq_ok = true, marker_ok = true, hdr_ok = true;
  for (size_t i = 0; i < pk.size(); i++) {
    if (pk[i].seq != static_cast<uint16_t>(65534 + i))
      seq_ok = false;
    if (pk[i].marker != (i == pk.size() - 1))
      marker_ok = false;
    if (pk[i].channel != 2 || pk[i].ts != 123456 || pk[i].ssrc != 0xDEADBEEF)
      hdr_ok = false;
    if (pk[i].payload.size() > 1400)
      hdr_ok = false;
  }
  CHECK(seq_ok && marker_ok && hdr_ok);
  CHECK(pk[0].payload == sps && pk[1].payload == pps);
  // Reassemble the FU-A fragments into the original NAL unit.
  std::vector<uint8_t> re;
  bool fu_ok = true;
  for (size_t i = 2; i < pk.size(); i++) {
    const auto &p = pk[i].payload;
    if ((p[0] & 0x1F) != 28 || (p[0] & 0xE0) != (0x65 & 0xE0) || (p[1] & 0x1F) != 5)
      fu_ok = false;
    bool s = p[1] & 0x80, e = p[1] & 0x40;
    if (s != (i == 2) || e != (i == pk.size() - 1))
      fu_ok = false;
    if (i == 2)
      re.push_back(static_cast<uint8_t>((p[0] & 0xE0) | (p[1] & 0x1F)));
    re.insert(re.end(), p.begin() + 2, p.end());
  }
  CHECK(fu_ok);
  CHECK(re == idr);

  // A NAL of exactly max_payload bytes goes in one packet; one byte more fragments.
  std::vector<uint8_t> exact(1400, 0x41);
  std::vector<NalUnit> one = {{exact.data(), exact.size()}};
  out.clear();
  CHECK(append_access_unit(out, 0, one, 0, st, 1400) == 1);
  exact.push_back(0x41);
  one = {{exact.data(), exact.size()}};
  out.clear();
  CHECK(append_access_unit(out, 0, one, 0, st, 1400) == 2);
}

static void test_sdp() {
  std::vector<uint8_t> sps = {0x67, 0x42, 0xC0, 0x1E, 0xAA};
  std::vector<uint8_t> pps = {0x68, 0xCE, 0x38, 0x80};
  std::string sdp = build_sdp("Thermal", "10.0.0.5", 42, sps, pps);
  CHECK(sdp.find("m=video 0 RTP/AVP 96\r\n") != std::string::npos);
  CHECK(sdp.find("a=rtpmap:96 H264/90000\r\n") != std::string::npos);
  CHECK(sdp.find("profile-level-id=42C01E") != std::string::npos);
  CHECK(sdp.find("sprop-parameter-sets=Z0LAHqo=,aM44gA==") != std::string::npos);
  CHECK(sdp.find("o=- 42 1 IN IP4 10.0.0.5\r\n") != std::string::npos);
  CHECK(sdp.find("a=control:trackID=0\r\n") != std::string::npos);
  std::string bare = build_sdp("", "", 1, {}, {});
  CHECK(bare.find("sprop") == std::string::npos && bare.find("packetization-mode=1") != std::string::npos);
}

static void test_response() {
  std::string r = build_response(200, 7, {{"Public", "OPTIONS"}});
  CHECK(r == "RTSP/1.0 200 OK\r\nCSeq: 7\r\nPublic: OPTIONS\r\n\r\n");
  r = build_response(461, 3, {}, "");
  CHECK(r.find("461 Unsupported Transport") != std::string::npos);
  r = build_response(200, 1, {}, "abc");
  CHECK(r.find("Content-Length: 3\r\n\r\nabc") != std::string::npos);
}

int main() {
  test_parse();
  test_uri();
  test_transport();
  test_base64();
  test_split();
  test_packetize();
  test_sdp();
  test_response();
  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures ? 1 : 0;
}
