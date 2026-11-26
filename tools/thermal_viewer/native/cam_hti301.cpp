// HTI-301 (InfiRay T3-317-13) through libusb: the component's UVC start
// (idle alternate setting, probe, commit, the streaming alternate setting),
// its commands on the Zoom control, isochronous packets fed to hti301_core's
// UvcAssembler, temperatures from hti301_core, and its calibration schedule.
//
// The camera is a UVC device, so the system's own camera driver holds it:
// Linux lets libusb take it over; macOS does not without root.

#include "tv_camera.h"

#ifndef TV_NO_USB

#include <chrono>
#include <cstring>

#include "hti301/hti301_core.h"
#include "tv_usb.h"

using namespace esphome::hti301;

namespace tv {
namespace {

using Clock = std::chrono::steady_clock;
constexpr uint8_t REQ_SET_CUR = 0x01, REQ_GET_CUR = 0x81;
constexpr uint8_t CLASS_OUT = 0x21, CLASS_IN = 0xA1;
constexpr uint16_t VS_PROBE = 0x01 << 8, VS_COMMIT = 0x02 << 8, CT_ZOOM_ABSOLUTE = 0x0B << 8;
constexpr uint16_t ZOOM_ENTITY = 0;  // as the camera's own driver addresses it
constexpr int NUM_TRANSFERS = 4, PACKETS = 32;
constexpr uint8_t SETTLE_FRAMES = 5, CALIBRATE_SETTLE_FRAMES = 25;

class Hti301 : public Camera {
 public:
  void run(Host &host, const std::string &) override {
    std::string err;
    if (!this->dev_.open(VID, PID, "HTI-301", &err)) {
      host.status(err);
      return;
    }
    if (this->start_(host))
      this->stream_(host);
    this->stop_();
  }

 protected:
  bool ctrl_(uint8_t type, uint8_t req, uint16_t value, uint16_t index, uint8_t *data, uint16_t len) {
    const int r = libusb_control_transfer(this->dev_.h, type, req, value, index, data, len, 1000);
    if (r != len) {
      this->error_ = usb_error("control request", r);
      return false;
    }
    return true;
  }
  bool command_(uint16_t code) {
    uint8_t b[2] = {static_cast<uint8_t>(code & 0xFF), static_cast<uint8_t>(code >> 8)};
    return this->ctrl_(CLASS_OUT, REQ_SET_CUR, CT_ZOOM_ABSOLUTE,
                       static_cast<uint16_t>((ZOOM_ENTITY << 8) | this->layout_.vc_interface), b, 2);
  }

  bool fail_(Host &host, const std::string &what) {
    host.status("HTI-301: " + what + (this->error_.empty() ? "" : " (" + this->error_ + ")"));
    return false;
  }

  bool start_(Host &host) {
    host.status("HTI-301: starting");
    uint8_t head[9];
    int r = libusb_get_descriptor(this->dev_.h, LIBUSB_DT_CONFIG, 0, head, sizeof(head));
    if (r < 9)
      return this->fail_(host, usb_error("cannot read the configuration descriptor", r));
    std::vector<uint8_t> cfg(static_cast<size_t>(head[2] | (head[3] << 8)));
    r = libusb_get_descriptor(this->dev_.h, LIBUSB_DT_CONFIG, 0, cfg.data(), static_cast<int>(cfg.size()));
    if (r < static_cast<int>(cfg.size()))
      return this->fail_(host, usb_error("cannot read the configuration descriptor", r));
    this->layout_ = parse_descriptors(cfg.data(), cfg.size());
    const UvcLayout &l = this->layout_;
    if (!l.ok)
      return this->fail_(host, "its descriptors are not the HTI-301's");
    std::string err;
    if (!this->dev_.claim(l.vc_interface, &err) || !this->dev_.claim(l.vs_interface, &err)) {
      host.status("HTI-301: " + err);
      return false;
    }
    const uint8_t vs = l.vs_interface;
    libusb_set_interface_alt_setting(this->dev_.h, vs, 0);
    if (!host.wait(100))
      return false;
    Probe p;
    p.hint = 1;  // keep the frame interval
    p.format_index = l.format_index;
    p.frame_index = l.frame_index;
    p.frame_interval = l.frame_interval;
    std::vector<uint8_t> buf(l.probe_size());
    encode_probe(p, buf.data(), buf.size());
    if (!this->ctrl_(CLASS_OUT, REQ_SET_CUR, VS_PROBE, vs, buf.data(), static_cast<uint16_t>(buf.size())))
      return this->fail_(host, "probe refused");
    if (!host.wait(100))
      return false;
    if (!this->ctrl_(CLASS_IN, REQ_GET_CUR, VS_PROBE, vs, buf.data(), static_cast<uint16_t>(buf.size())))
      return this->fail_(host, "probe not read back");
    Probe got;
    if (!decode_probe(buf.data(), buf.size(), &got))
      return this->fail_(host, "short probe reply");
    const size_t expect = static_cast<size_t>(l.width) * l.height * 2;
    if (got.max_frame_size == 0)
      got.max_frame_size = l.max_frame_size;
    if (got.format_index != l.format_index || got.frame_index != l.frame_index || got.max_frame_size != expect)
      return this->fail_(host, "unexpected stream parameters");
    if (!this->ctrl_(CLASS_OUT, REQ_SET_CUR, VS_COMMIT, vs, buf.data(), static_cast<uint16_t>(buf.size())))
      return this->fail_(host, "commit refused");
    if (!host.wait(200))
      return false;
    this->alt_ = l.alt_for(got.max_payload);
    if (this->alt_ == nullptr)
      return this->fail_(host, "no streaming alternate setting carries its payload");
    r = libusb_set_interface_alt_setting(this->dev_.h, vs, this->alt_->alt);
    if (r != 0)
      return this->fail_(host, usb_error("cannot select the streaming alternate setting", r));
    if (!host.wait(200))
      return false;

    this->width_ = l.width;
    this->rows_ = l.height - METADATA_ROWS;
    this->frame_bytes_ = got.max_frame_size;
    this->frame_.assign(this->frame_bytes_, 0);
    this->ready_.assign(this->frame_bytes_, 0);
    this->asm_buf_ = this->frame_.data();
    this->assembler_.set_frame_size(this->frame_bytes_);
    const int mps = this->alt_->packet_size;
    this->bufs_.assign(NUM_TRANSFERS, std::vector<uint8_t>(static_cast<size_t>(PACKETS) * mps));
    for (int i = 0; i < NUM_TRANSFERS; i++) {
      libusb_transfer *t = libusb_alloc_transfer(PACKETS);
      libusb_fill_iso_transfer(t, this->dev_.h, this->alt_->endpoint, this->bufs_[i].data(), PACKETS * mps, PACKETS,
                               Hti301::transfer_cb_, this, 0);
      libusb_set_iso_packet_lengths(t, mps);
      this->xfers_.push_back(t);
      if (libusb_submit_transfer(t) == 0)
        this->inflight_++;
    }
    if (this->inflight_ == 0)
      return this->fail_(host, "cannot queue isochronous transfers");
    // Radiometric mode, then the range (frames just after a command are not used).
    if (!this->command_(CMD_RADIOMETRIC_MODE))
      return this->fail_(host, "radiometric mode refused");
    this->settle_ = SETTLE_FRAMES;
    if (!this->command_(CMD_RANGE_NORMAL))
      return this->fail_(host, "range refused");
    return true;
  }

  static void LIBUSB_CALL transfer_cb_(libusb_transfer *t) { static_cast<Hti301 *>(t->user_data)->on_transfer_(t); }
  void on_transfer_(libusb_transfer *t) {
    if (t->status == LIBUSB_TRANSFER_COMPLETED) {
      const uint8_t *p = t->buffer;
      for (int i = 0; i < t->num_iso_packets; i++) {
        const libusb_iso_packet_descriptor &d = t->iso_packet_desc[i];
        if (d.status == LIBUSB_TRANSFER_COMPLETED) {
          this->assembler_.feed(p, d.actual_length, this->asm_buf_, this->frame_bytes_,
                                [this](bool complete, size_t) { this->assembled_(complete); });
        } else {
          this->assembler_.lost();
          this->lost_++;
        }
        p += d.length;
      }
    } else if (t->status != LIBUSB_TRANSFER_CANCELLED) {
      this->assembler_.lost();
      this->errors_++;
    }
    if (!this->stopping_ && t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_NO_DEVICE &&
        libusb_submit_transfer(t) == 0)
      return;
    if (t->status == LIBUSB_TRANSFER_NO_DEVICE)
      this->gone_ = true;
    this->inflight_--;
  }
  // From the assembler; leaves asm_buf_ at the next frame's buffer.
  void assembled_(bool complete) {
    if (!complete) {
      this->incomplete_++;
      return;
    }
    this->frames_++;
    if (this->settle_ > 0) {
      this->settle_--;
      return;
    }
    this->frame_.swap(this->ready_);
    this->asm_buf_ = this->frame_.data();
    this->have_ready_ = true;
  }

  static uint32_t ms_(Clock::time_point t0) {
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count()) + 1;
  }

  void stream_(Host &host) {
    const Clock::time_point t0 = Clock::now();
    CalibrationSchedule schedule;
    schedule.start(ms_(t0));
    TemperatureTable table;
    std::vector<float> temps(static_cast<size_t>(this->width_) * this->rows_);
    float detector = NAN;
    bool calibrate = false, high = false;
    unsigned bad_calibration = 0;
    Clock::time_point report = Clock::now(), last_frame = Clock::now();
    while (!host.stopping() && !this->gone_) {
      this->dev_.pump(20);
      if (this->have_ready_) {
        this->have_ready_ = false;
        last_frame = Clock::now();
        const auto *words = reinterpret_cast<const uint16_t *>(this->ready_.data());
        const Metadata m = parse_metadata(words, this->width_, this->rows_);
        if (m.valid)
          detector = m.fpa_temp;
        const Environment want{NAN, NAN, NAN, NAN, NAN, NAN};  // the camera's own, else the defaults
        const Environment env = resolve_environment(want, m);
        esphome::video::FrameStats st;
        if (raw_to_temperatures(words, this->width_, this->rows_, m, env, table, temps.data(), &st))
          host.frame(temps.data(), this->width_, this->rows_, &st, env.emissivity, env.reflected);
        else
          bad_calibration++;
      }
      std::string action;
      while (host.take_action(&action)) {
        if (action == "Calibrate (shutter)")
          calibrate = true;
        else if (action == "Normal range (-20..120 C)" && this->command_(CMD_RANGE_NORMAL))
          high = false, this->settle_ = SETTLE_FRAMES;
        else if (action == "High range (-20..450 C)" && this->command_(CMD_RANGE_HIGH))
          high = true, this->settle_ = SETTLE_FRAMES;
      }
      const uint32_t now = ms_(t0);
      if (calibrate || schedule.due(now, detector)) {
        calibrate = false;
        if (this->command_(CMD_CALIBRATE)) {
          schedule.done(now, detector);
          this->settle_ = CALIBRATE_SETTLE_FRAMES;
        }
      }
      if (Clock::now() >= report) {
        report = Clock::now() + std::chrono::seconds(1);
        char line[220];
        std::snprintf(line, sizeof(line),
                      "HTI-301 %u x %u, %s range%s - %u frames, %u incomplete, %u packets lost, %u transfer errors, "
                      "%u without calibration, detector %.1f C",
                      this->width_, this->rows_, high ? "high" : "normal",
                      Clock::now() - last_frame > std::chrono::seconds(3) ? " - no frames for 3 s" : "", this->frames_,
                      this->incomplete_, this->lost_, this->errors_, bad_calibration, detector);
        host.status(line);
      }
    }
    host.status(this->gone_ ? "HTI-301: unplugged" : "Stopped");
  }

  void stop_() {
    this->stopping_ = true;
    for (auto *t : this->xfers_)
      libusb_cancel_transfer(t);
    for (int i = 0; i < 40 && this->inflight_ > 0; i++)
      this->dev_.pump(50);
    if (this->dev_.h != nullptr && !this->gone_ && this->layout_.ok)
      libusb_set_interface_alt_setting(this->dev_.h, this->layout_.vs_interface, 0);
    if (this->inflight_ == 0)
      for (auto *t : this->xfers_)
        libusb_free_transfer(t);
    this->xfers_.clear();
    this->dev_.close();
  }

  UsbDevice dev_;
  std::string error_;
  UvcLayout layout_;
  const StreamAlt *alt_{nullptr};
  UvcAssembler assembler_;
  uint16_t width_{0}, rows_{0};
  size_t frame_bytes_{0};
  std::vector<uint8_t> frame_, ready_;
  uint8_t *asm_buf_{nullptr};
  std::vector<std::vector<uint8_t>> bufs_;
  std::vector<libusb_transfer *> xfers_;
  int inflight_{0};
  uint8_t settle_{0};
  bool have_ready_{false}, stopping_{false}, gone_{false};
  unsigned frames_{0}, incomplete_{0}, lost_{0}, errors_{0};
};

}  // namespace

CameraType hti301_type() {
  return {"hti301", "HTI-301 (USB)", "", USB_UNAVAILABLE,
          {"Calibrate (shutter)", "Normal range (-20..120 C)", "High range (-20..450 C)"}, 2, 10.0f,
          []() -> Camera * { return new Hti301(); }};
}

}  // namespace tv

#else

namespace tv {
CameraType hti301_type() {
  return {"hti301", "HTI-301 (USB)", "", USB_UNAVAILABLE, {}, 2, 10.0f, []() -> Camera * { return nullptr; }};
}
}  // namespace tv

#endif
