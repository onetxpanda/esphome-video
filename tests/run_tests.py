#!/usr/bin/env python3
"""Test suite for esphome-video.

1. Unit tests (ASan/UBSan, and TSan for the frame pool): frames and pools,
   radiometric processing, the CPU ISP, overlay drawing, MLX90640 sub-page
   assembly, the HTI-301 core (descriptors, packet assembly, temperatures,
   YUYV), and the RTSP/RTP protocol helpers.
2. Host RTSP server (AddressSanitizer + UBSan build) against real clients:
   ffprobe, ffmpeg (decode with errors fatal, TCP and default transport),
   GStreamer rtspsrc, three concurrent clients, the client limit, raw-socket
   protocol checks, and a client that stops reading.
3. The same server under ThreadSanitizer with concurrent clients.
4. Optional: ESPHome configs (valid compile for ESP32-P4, invalid ones rejected).

Usage: tests/run_tests.py [--esphome PATH] [--skip-esphome]
Needs: g++, ffmpeg/ffprobe (with libx264, used only to make the test stream),
gst-launch-1.0 with rtspsrc/rtph264depay/h264parse/avdec_h264.
"""

import argparse
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
BUILD = os.path.join(ROOT, "build")
COMP = os.path.join(ROOT, "components")
RTSP = os.path.join(COMP, "rtsp_server")
FIXTURE = os.path.join(HERE, "host", "fixtures", "thermal_128x96_8fps.h264")
FAILURES = []


def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond:
        FAILURES.append(msg)
    return cond


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def build(name, sources, flags):
    out = os.path.join(BUILD, name)
    r = run(["g++", "-std=gnu++17", "-Wall", "-Wextra", "-Werror", "-g", "-O1", "-pthread", f"-I{RTSP}"]
            + flags + sources + ["-o", out])
    if r.returncode != 0:
        print(r.stderr[-4000:])
    return out if r.returncode == 0 else None


def make_fixture():
    if os.path.exists(FIXTURE):
        return True
    os.makedirs(os.path.dirname(FIXTURE), exist_ok=True)
    r = run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "lavfi", "-i",
             "testsrc2=size=128x96:rate=8", "-t", "6", "-c:v", "libx264", "-profile:v", "baseline",
             "-pix_fmt", "yuv420p", "-x264-params",
             "aud=1:keyint=8:min-keyint=8:scenecut=0:repeat-headers=1", "-f", "h264", FIXTURE])
    return r.returncode == 0


ASAN = ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]


# name, sources, include dirs
PIPELINE_TESTS = [
    ("test_frame", ["unit/test_frame.cpp", "video_core/frame.cpp"], ["video_core"]),
    ("test_processing", ["unit/test_processing.cpp", "video_core/frame.cpp", "radiometric/radiometric_core.cpp",
                         "isp/isp_core.cpp"], ["video_core", "radiometric", "isp"]),
    ("test_overlay", ["unit/test_overlay.cpp", "video_core/frame.cpp", "overlay/overlay_core.cpp"],
     ["video_core", "overlay"]),
    ("test_assembler", ["unit/test_assembler.cpp", "mlx90640/subpage_assembler.cpp"], ["mlx90640"]),
    ("test_hti301", ["unit/test_hti301.cpp", "hti301/hti301_core.cpp", "video_core/frame.cpp"],
     ["video_core", "hti301"]),
]


def pipeline_unit_tests():
    print("== pipeline unit tests")
    for name, srcs, incs in PIPELINE_TESTS:
        paths = [os.path.join(HERE, s) if s.startswith("unit/") else os.path.join(COMP, s) for s in srcs]
        flags = [f"-I{os.path.join(COMP, i)}" for i in incs]
        for label, san in (("ASan/UBSan", ASAN), ("TSan", ["-fsanitize=thread"])):
            if label == "TSan" and name != "test_frame":
                continue  # only the frame pool is shared between threads
            exe = build(f"{name}_{label[:4].lower()}", paths, flags + san)
            if not check(exe is not None, f"{name} builds ({label})"):
                continue
            r = run([exe])
            print("  " + (r.stdout.strip().splitlines() or ["(no output)"])[-1])
            check(r.returncode == 0, f"{name} passes ({label})")


def unit_tests():
    print("== unit tests")
    exe = build("test_rtsp_core", [os.path.join(HERE, "unit", "test_rtsp_core.cpp"),
                                   os.path.join(RTSP, "rtsp_core.cpp")], ASAN)
    if not check(exe is not None, "unit tests build"):
        return
    r = run([exe])
    print("  " + (r.stdout.strip().splitlines() or ["(no output)"])[-1])
    check(r.returncode == 0, "protocol helper unit tests pass")
    exe = build("test_server_clock", [os.path.join(HERE, "unit", "test_server_clock.cpp"),
                                      os.path.join(RTSP, "rtsp_core.cpp"),
                                      os.path.join(RTSP, "rtsp_server_core.cpp")], ASAN)
    if check(exe is not None, "server clock test builds"):
        r = run([exe], timeout=60)
        if r.returncode != 0:
            print(r.stdout[-2000:])
        check(r.returncode == 0, "a client is never closed as idle because the clock ticked mid-poll")
    exe = build("test_keyframe_requests", [os.path.join(HERE, "unit", "test_keyframe_requests.cpp"),
                                           os.path.join(RTSP, "rtsp_core.cpp"),
                                           os.path.join(RTSP, "rtsp_server_core.cpp")], ASAN)
    if check(exe is not None, "keyframe request test builds"):
        r = run([exe], timeout=60)
        print("  " + (r.stdout.strip().splitlines() or ["(no output)"])[-1])
        check(r.returncode == 0, "a client that falls behind gets one keyframe request per stall")


class Server:
    def __init__(self, exe, port, seconds, *extra, fps=8):
        self.port = port
        self.err_path = os.path.join(BUILD, f"server_{port}.err")
        self.err = open(self.err_path, "w")
        self.p = subprocess.Popen([exe, str(port), FIXTURE, str(fps), str(seconds)] + [str(e) for e in extra],
                                  stdout=subprocess.PIPE, stderr=self.err, text=True)
        line = self.p.stdout.readline()
        self.ready = line.strip() == "READY"

    def url(self, path="thermal"):
        return f"rtsp://127.0.0.1:{self.port}/{path}"

    def finish(self):
        try:
            self.p.wait(timeout=90)
        except subprocess.TimeoutExpired:
            self.p.kill()
        self.err.close()
        with open(self.err_path) as f:
            return self.p.returncode, f.read()


def rtsp_exchange(port, requests, read_timeout=2.0):
    """Sends raw requests on one connection; returns all response bytes."""
    s = socket.create_connection(("127.0.0.1", port), timeout=read_timeout)
    data = b""
    try:
        for req in requests:
            s.sendall(req.encode() if isinstance(req, str) else req)
            time.sleep(0.15)
        end = time.time() + read_timeout
        while time.time() < end:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                break
            if not chunk:
                break
            data += chunk
    finally:
        s.close()
    return data.decode("latin-1")


def statuses(text):
    return [int(m) for m in re.findall(r"^RTSP/1\.0 (\d{3})", text, re.M)]


def ffmpeg_decode(url, frames, transport="tcp", timeout=40):
    cmd = ["ffmpeg", "-hide_banner", "-v", "error", "-xerror"]
    if transport:
        cmd += ["-rtsp_transport", transport]
    cmd += ["-i", url, "-frames:v", str(frames), "-f", "framecrc", "-"]
    r = run(cmd, timeout=timeout)
    n = len([ln for ln in r.stdout.splitlines() if ln and not ln.startswith("#")])
    return r.returncode, n, r.stderr


def protocol_checks(port):
    base = f"rtsp://127.0.0.1:{port}/thermal"
    t = rtsp_exchange(port, [f"OPTIONS {base} RTSP/1.0\r\nCSeq: 1\r\n\r\n"])
    check(statuses(t) == [200] and "DESCRIBE" in t and "CSeq: 1" in t, "OPTIONS lists methods and echoes CSeq")
    t = rtsp_exchange(port, [f"DESCRIBE {base} RTSP/1.0\r\nCSeq: 2\r\nAccept: application/sdp\r\n\r\n"])
    check(statuses(t) == [200] and "a=rtpmap:96 H264/90000" in t and "sprop-parameter-sets=" in t
          and "profile-level-id=" in t, "DESCRIBE returns SDP with SPS/PPS once frames have flowed")
    t = rtsp_exchange(port, [f"DESCRIBE rtsp://127.0.0.1:{port}/nope RTSP/1.0\r\nCSeq: 3\r\n\r\n"])
    check(statuses(t) == [404], "DESCRIBE of an unknown path is 404")
    t = rtsp_exchange(port, [f"PLAY {base} RTSP/1.0\r\nCSeq: 4\r\nSession: 1234\r\n\r\n"])
    check(statuses(t) == [454], "PLAY without a valid session is 454")
    t = rtsp_exchange(port, [f"SETUP {base}/trackID=0 RTSP/1.0\r\nCSeq: 5\r\n"
                             "Transport: RTP/AVP;unicast;client_port=5000-5001\r\n\r\n"])
    check(statuses(t) == [461], "UDP-only SETUP is 461 Unsupported Transport")
    t = rtsp_exchange(port, [f"FROB {base} RTSP/1.0\r\nCSeq: 6\r\n\r\n"])
    check(statuses(t) == [405] and "Allow:" in t, "unknown method is 405 with Allow")
    t = rtsp_exchange(port, [f"OPTIONS {base} RTSP/1.0\r\n\r\n"])
    check(statuses(t) == [400], "request without CSeq is 400")
    t = rtsp_exchange(port, ["THIS IS NOT RTSP\r\n\r\n", f"OPTIONS {base} RTSP/1.0\r\nCSeq: 7\r\n\r\n"])
    check(statuses(t) == [400], "malformed request: 400, then the connection is closed")
    t = rtsp_exchange(port, [f"OPTIONS {base} RTSP/1.0\r\nCSeq: 8\r\n\r\nOPTIONS {base} RTSP/1.0\r\nCSeq: 9\r\n\r\n"])
    check(statuses(t) == [200, 200] and "CSeq: 9" in t, "pipelined requests are both answered")
    t = rtsp_exchange(port, [b"$\x01\x00\x04abcd" + f"OPTIONS {base} RTSP/1.0\r\nCSeq: 10\r\n\r\n".encode()])
    check(statuses(t) == [200], "interleaved RTCP bytes before a request are skipped")
    t = rtsp_exchange(port, [b"OPTIONS " + b"A" * 6000])
    check(statuses(t) == [400] or t == "", "oversized request is rejected")
    # Full session over a raw socket: SETUP -> PLAY -> interleaved RTP -> TEARDOWN.
    s = socket.create_connection(("127.0.0.1", port), timeout=3)
    s.sendall(f"SETUP {base}/trackID=0 RTSP/1.0\r\nCSeq: 1\r\nTransport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n\r\n".encode())
    resp = s.recv(4096).decode("latin-1")
    sess = re.search(r"Session: ([0-9A-F]+)", resp)
    ok = statuses(resp) == [200] and sess is not None and "interleaved=0-1" in resp
    check(ok, "SETUP over TCP returns a session and the interleaved channels")
    if ok:
        s.sendall(f"PLAY {base} RTSP/1.0\r\nCSeq: 2\r\nSession: {sess.group(1)}\r\n\r\n".encode())
        buf = b""
        end = time.time() + 4
        while time.time() < end and buf.count(b"$\x00") < 5:
            buf += s.recv(65536)
        head, _, rest = buf.partition(b"\r\n\r\n")
        check(head.startswith(b"RTSP/1.0 200") and b"RTP-Info" in head, "PLAY is answered before any RTP data")
        first_nal = rest[4 + 12] & 0x1F if len(rest) > 17 and rest[:1] == b"$" else -1
        check(first_nal in (7, 5) or (first_nal == 28 and (rest[4 + 13] & 0x1F) in (5, 7)),
              "first packet after PLAY starts a keyframe (SPS or IDR)")
        s.sendall(f"TEARDOWN {base} RTSP/1.0\r\nCSeq: 3\r\nSession: {sess.group(1)}\r\n\r\n".encode())
        s.settimeout(3)
        tail = b""
        try:
            while True:
                c = s.recv(65536)
                if not c:
                    break
                tail += c
        except socket.timeout:
            pass
        check(b"RTSP/1.0 200 OK\r\nCSeq: 3" in tail, "TEARDOWN is answered and the server closes the connection")
    s.close()


def stalled_client(port):
    """Starts a session, then never reads. Returns the socket (caller closes)."""
    base = f"rtsp://127.0.0.1:{port}/thermal"
    # The receive buffer must be set before connecting to limit the TCP window.
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
    s.settimeout(3)
    s.connect(("127.0.0.1", port))
    s.sendall(f"SETUP {base}/trackID=0 RTSP/1.0\r\nCSeq: 1\r\nTransport: RTP/AVP/TCP;interleaved=0-1\r\n\r\n".encode())
    resp = s.recv(4096).decode("latin-1")
    sess = re.search(r"Session: ([0-9A-F]+)", resp).group(1)
    s.sendall(f"PLAY {base} RTSP/1.0\r\nCSeq: 2\r\nSession: {sess}\r\n\r\n".encode())
    return s


def host_tests(exe, label):
    print(f"== host RTSP server ({label})")
    srv = Server(exe, 18554, 75)
    if not check(srv.ready, "server starts and listens"):
        print(srv.finish()[1][-2000:])
        return
    try:
        r = run(["ffprobe", "-hide_banner", "-v", "error", "-rtsp_transport", "tcp", "-show_entries",
                 "stream=codec_name,profile,width,height,avg_frame_rate", "-of", "compact", srv.url()], timeout=30)
        check("codec_name=h264" in r.stdout and "width=128" in r.stdout and "height=96" in r.stdout,
              f"ffprobe sees H.264 128x96 ({r.stdout.strip()})")
        rc, n, err = ffmpeg_decode(srv.url(), 40)
        check(rc == 0 and n == 40, f"ffmpeg decodes 40 frames over TCP with errors fatal (got {n}) {err[-300:]}")
        rc, n, err = ffmpeg_decode(srv.url(), 16, transport=None)
        check(rc == 0 and n == 16, f"ffmpeg default transport falls back to TCP after 461 (got {n}) {err[-600:]}")
        r = run(["gst-launch-1.0", "-q", "rtspsrc", f"location={srv.url()}", "protocols=tcp", "latency=0", "!",
                 "rtph264depay", "!", "h264parse", "!", "avdec_h264", "!", "fakesink", "num-buffers=24"], timeout=40)
        check(r.returncode == 0, f"GStreamer rtspsrc decodes 24 frames {r.stderr[-300:]}")

        # Three clients at once; a fourth is refused with 503.
        results = []

        def client():
            results.append(ffmpeg_decode(srv.url(), 32, timeout=60))

        threads = [threading.Thread(target=client) for _ in range(3)]
        for t in threads:
            t.start()
        time.sleep(2.0)
        t4 = rtsp_exchange(srv.port, [f"OPTIONS {srv.url()} RTSP/1.0\r\nCSeq: 1\r\n\r\n"])
        for t in threads:
            t.join()
        check(len(results) == 3 and all(rc == 0 and n == 32 for rc, n, _ in results),
              f"three concurrent clients each decode 32 frames ({[n for _, n, _ in results]})")
        check(statuses(t4) == [503], "a fourth client is refused with 503 while three are connected")

        protocol_checks(srv.port)
    finally:
        rc, err = srv.finish()
    check(rc == 0, f"server exits cleanly (rc={rc})")
    check("AddressSanitizer" not in err and "runtime error" not in err and "ThreadSanitizer" not in err,
          "no sanitizer reports")
    if "Sanitizer" in err or "runtime error" in err:
        print(err[-3000:])

    # A client that stops reading: frames for it are dropped whole, it is closed
    # after the stall timeout, and a healthy client is unaffected meanwhile.
    print(f"== slow client ({label})")
    # Linux auto-grows send buffers to MBs, so cap it (as lwIP's small buffer
    # does on the device) and feed fast, so the server's own backlog fills.
    srv = Server(exe, 18555, 30, 4000, 32768, 16384, fps=100)
    if not check(srv.ready, "server starts"):
        return
    slow = stalled_client(srv.port)
    time.sleep(1.0)
    rc, n, err = ffmpeg_decode(srv.url(), 40, timeout=40)
    check(rc == 0 and n == 40, f"healthy client decodes 40 frames while another is stalled (got {n})")
    time.sleep(6.0)
    slow.close()
    rc, err = srv.finish()
    dropped = re.search(r"dropped frames: (\d+)", err)
    check(dropped is not None and int(dropped.group(1)) > 0, f"stalled client's frames were dropped ({dropped and dropped.group(1)})")
    check("not accepting data, closing" in err, "stalled client is closed after the stall timeout")
    check(rc == 0 and "Sanitizer" not in err and "runtime error" not in err, "no sanitizer reports")


def guard_tests():
    """The build guard must accept only the register set matching the chip."""
    print("== build guard (esp_h264 register definitions)")
    if not check(shutil.which("cmake") is not None, "cmake available"):
        return
    script = os.path.join(HERE, "cmake", "test_guard.cmake")
    p = "/x/esp_h264/hw/soc/esp32p4"
    cases = [
        ("301", f"/a;{p}/hw_ver3;/b", True, "rev 3.1 with v3 registers is accepted"),
        ("301", f"/a;{p}/hw_ver1;/b", False, "rev 3.1 with v1 registers stops the build"),
        ("301", f"{p}/hw_ver1;{p}/hw_ver3", False, "both register sets stop the build"),
        ("301", "/a;/b", False, "no register set stops the build"),
        ("100", f"{p}/hw_ver1", True, "rev 1.0 with v1 registers is accepted"),
        ("100", f"{p}/hw_ver3", False, "rev 1.0 with v3 registers stops the build"),
        ("", f"{p}/hw_ver3", False, "unknown chip revision stops the build"),
    ]
    for rev, dirs, ok, msg in cases:
        r = run(["cmake", f"-DREV={rev}", f"-DDIRS={dirs}", "-P", script])
        check((r.returncode == 0) == ok, msg)


def esphome_tests(esphome):
    print("== ESPHome configs")
    d = os.path.join(HERE, "esphome")
    if not os.path.isdir(d):
        check(False, "tests/esphome exists")
        return
    with open(os.path.join(d, "invalid", "expected.txt")) as f:
        cases = [ln.strip().split("|", 1) for ln in f if ln.strip()]
    for name, expect in cases:
        r = run([esphome, "config", name], cwd=os.path.join(d, "invalid"))
        out = r.stdout + r.stderr
        check(r.returncode != 0 and expect in out, f"{name}: rejected with \"{expect}\"")
    for yaml in sorted(f for f in os.listdir(d) if f.endswith(".yaml")):
        # Force our sources to recompile, so their warnings are always seen
        # (an incremental build would skip unchanged files).
        name = yaml[:-5]
        with open(os.path.join(d, yaml)) as f:
            m = re.search(r"^\s+name:\s*(\S+)", f.read(), re.M)
        if m:
            name = m.group(1)
        shutil.rmtree(os.path.join(d, ".esphome", "build", name, "build", "esp-idf", "src"), ignore_errors=True)
        r = run([esphome, "compile", yaml], cwd=d, timeout=1800)
        ours = [ln for ln in (r.stdout + r.stderr).splitlines() if "warning:" in ln and re.search(
            r"components/(assert_capture|video_core|video_controls|mlx90640|hti301|radiometric|isp|overlay|video_camera|"
            r"video_test_pattern|video_switch|h264_encoder|rtsp_server|task_stats)/", ln)]
        if not check(not ours, f"{yaml}: no compiler warnings in our components"):
            print("\n".join(ours[:10]))
        if not check(r.returncode == 0, f"{yaml} compiles"):
            print((r.stdout + r.stderr)[-4000:])
        check("esp_h264 uses hw_ver3 register definitions for chip revision 301 (checked)" in r.stdout + r.stderr,
              f"{yaml}: build guard confirmed v3 registers for the rev 3.01 target")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--esphome", default=shutil.which("esphome") or "esphome")
    ap.add_argument("--skip-esphome", action="store_true")
    a = ap.parse_args()
    # The ESPHome tests run from other directories, so resolve a relative path now.
    if os.sep in a.esphome:
        a.esphome = os.path.abspath(a.esphome)
    os.makedirs(BUILD, exist_ok=True)
    if not check(make_fixture(), "test stream fixture available"):
        sys.exit(1)
    pipeline_unit_tests()
    unit_tests()
    guard_tests()
    srcs =[os.path.join(RTSP, "rtsp_core.cpp"), os.path.join(RTSP, "rtsp_server_core.cpp"),
            os.path.join(HERE, "host", "rtsp_host.cpp")]
    asan = build("rtsp_host_asan", srcs, ASAN)
    if check(asan is not None, "host server builds (ASan/UBSan)"):
        host_tests(asan, "ASan/UBSan")
    tsan = build("rtsp_host_tsan", srcs, ["-fsanitize=thread"])
    if check(tsan is not None, "host server builds (TSan)"):
        host_tests(tsan, "TSan")
    if not a.skip_esphome:
        esphome_tests(a.esphome)
    print(f"\n{'FAILED' if FAILURES else 'PASSED'}: {len(FAILURES)} failure(s)")
    for f in FAILURES:
        print("  - " + f)
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
