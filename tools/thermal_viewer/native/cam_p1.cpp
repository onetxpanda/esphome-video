// Thermal Master P1 through libusb: the component's start-up sequence and the
// vendor library's handshake (as tests/host/tm_capture.cpp), every bulk
// transfer fed to tm_core's Assembler, temperatures from tm_core's decode().

#include "tv_camera.h"

#ifndef TV_NO_USB

#include <chrono>
#include <cstring>
#include <thread>

#include "thermal_master_p1/tm_core.h"
#include "tv_usb.h"

using namespace esphome::thermal_master_p1;

namespace tv {
namespace {

using Clock = std::chrono::steady_clock;
constexpr int TRANSFERS = 4;
constexpr int TRANSFER_BYTES = 16 * 512;
constexpr uint8_t EP_IN = 0x81;
constexpr uint8_t REQ_STREAM_ON = 0xEE;

class P1 : public Camera {
 public:
  void run(Host &host, const std::string &) override {
    std::string err;
    if (!this->dev_.open(VID, PID_P1, "Thermal Master P1", &err) || !this->dev_.claim(0, &err) ||
        !this->dev_.claim(1, &err)) {
      host.status(err);
      return;
    }
    if (this->start_(host))
      this->stream_(host);
    this->stop_();
  }

 protected:
  // One status read; read again while it is busy, for up to `budget_ms`.
  bool status_(uint8_t expect, unsigned budget_ms) {
    const Clock::time_point t0 = Clock::now();
    for (;;) {
      uint8_t b = 0;
      const int r = libusb_control_transfer(this->dev_.h, 0xC1, 0x22, 0, 0, &b, 1, budget_ms);
      if (r != 1) {
        this->error_ = usb_error("status read", r);
        return false;
      }
      if (b != STATUS_BUSY) {
        if (b != expect)
          this->error_ = "status " + std::to_string(b) + ", not " + std::to_string(expect);
        return b == expect;
      }
      if (std::chrono::duration<double, std::milli>(Clock::now() - t0).count() > budget_ms) {
        this->error_ = "still busy after " + std::to_string(budget_ms) + " ms";
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  bool cmd_(const Command &cmd, std::vector<uint8_t> *resp = nullptr) {
    int r = libusb_control_transfer(this->dev_.h, 0x41, 0x20, 0, 0, const_cast<uint8_t *>(cmd.data()), cmd.size, 2000);
    if (r != static_cast<int>(cmd.size)) {
      this->error_ = usb_error("command", r);
      return false;
    }
    if (cmd.no_status)
      return true;
    const uint16_t n = cmd.response_bytes();
    if (n == 0)
      return this->status_(STATUS_DONE, cmd.budget_ms);
    if (!this->status_(STATUS_READY, cmd.budget_ms))
      return false;
    std::vector<uint8_t> buf(n);
    r = libusb_control_transfer(this->dev_.h, 0xC1, 0x21, 0, 0, buf.data(), n, 2000);
    if (r < 0) {
      this->error_ = usb_error("response", r);
      return false;
    }
    buf.resize(r);
    if (resp != nullptr)
      *resp = buf;
    return this->status_(STATUS_DONE, cmd.budget_ms);
  }
  static std::string text_(const std::vector<uint8_t> &b) {
    std::string s;
    for (uint8_t c : b) {
      if (c == 0)
        break;
      if (c >= 0x20 && c < 0x7F)
        s += static_cast<char>(c);
    }
    return s;
  }

  bool start_(Host &host) {
    host.status("Thermal Master P1: starting");
    libusb_set_interface_alt_setting(this->dev_.h, 1, 0);
    if (!host.wait(100))
      return false;
    std::vector<uint8_t> resp;
    if (!this->cmd_(command(Cmd::READ_MODEL), &resp))
      return this->fail_(host, "read the model");
    this->name_ = text_(resp);
    if (!this->cmd_(command(Cmd::READ_FIRMWARE), &resp))
      return this->fail_(host, "read the firmware version");
    this->name_ += ", firmware " + text_(resp);
    if (!this->cmd_(command(Cmd::READ_SYSTEM_VERSION), &resp) || !this->cmd_(command(Cmd::READ_RANGE), &resp))
      return this->fail_(host, "start-up reads");
    if (!host.wait(1000))
      return false;
    int r = libusb_set_interface_alt_setting(this->dev_.h, 1, 1);
    if (r != 0) {
      host.status(usb_error("Thermal Master P1: streaming interface", r));
      return false;
    }
    r = libusb_control_transfer(this->dev_.h, 0x40, REQ_STREAM_ON, 0, 1, nullptr, 0, 1000);
    if (r < 0) {
      host.status(usb_error("Thermal Master P1: stream on", r));
      return false;
    }
    if (!host.wait(2000))
      return false;
    this->bufs_.assign(TRANSFERS, std::vector<uint8_t>(TRANSFER_BYTES));
    for (int i = 0; i < TRANSFERS; i++) {
      libusb_transfer *t = libusb_alloc_transfer(0);
      libusb_fill_bulk_transfer(t, this->dev_.h, EP_IN, this->bufs_[i].data(), TRANSFER_BYTES, P1::transfer_cb_, this,
                                0);
      this->xfers_.push_back(t);
      if (libusb_submit_transfer(t) == 0)
        this->inflight_++;
    }
    if (!this->cmd_(command(Cmd::READ_RANGE), &resp) || !this->set_range_(host, 1))
      return this->fail_(host, "set the high range");
    return true;
  }

  // 0 the low range (150..600 C), 1 the high range (-20..150 C).
  bool set_range_(Host &host, int range) {
    if (!this->cmd_(range == 1 ? command(Cmd::RANGE_HIGH) : setting_write(Setting::RANGE, 0)))
      return false;
    this->low_ = range == 0;
    std::vector<uint8_t> resp;
    for (int i = 0; i < 30 && !host.stopping(); i++) {
      if (!this->cmd_(reading_command(Reading::GAIN_SWITCH_STATE), &resp) || resp.empty() || resp[0] != 2)
        break;
      host.status("Thermal Master P1: range change under way");
      for (int k = 0; k < 20; k++)
        this->service_(host, 50);
    }
    return true;
  }

  bool fail_(Host &host, const char *what) {
    host.status(std::string("Thermal Master P1: cannot ") + what + " (" + this->error_ + ")");
    return false;
  }

  static void LIBUSB_CALL transfer_cb_(libusb_transfer *t) { static_cast<P1 *>(t->user_data)->on_transfer_(t); }
  void on_transfer_(libusb_transfer *t) {
    if (t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length > 0) {
      switch (this->assembler_.feed(t->buffer, t->actual_length, this->frame_.data())) {
        case Assembler::Result::FRAME:
          this->frame_.swap(this->ready_);  // the next transfer starts a new frame
          this->have_ready_ = true;
          this->frames_++;
          break;
        case Assembler::Result::SHORT:
        case Assembler::Result::LONG:
        case Assembler::Result::BAD_MARKER:
          this->damaged_++;
          break;
        default:
          break;
      }
    } else if (t->status != LIBUSB_TRANSFER_COMPLETED && t->status != LIBUSB_TRANSFER_CANCELLED) {
      this->errors_++;
    }
    if (!this->stopping_ && t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_NO_DEVICE &&
        libusb_submit_transfer(t) == 0)
      return;
    if (t->status == LIBUSB_TRANSFER_NO_DEVICE)
      this->gone_ = true;
    this->inflight_--;
  }

  // Runs USB events for `ms` and passes on a frame that completed.
  void service_(Host &host, int ms) {
    this->dev_.pump(ms);
    if (!this->have_ready_)
      return;
    this->have_ready_ = false;
    esphome::video::FrameStats st;
    const uint32_t bad = decode(this->ready_.data(), 1.0f, 0.0f, this->temps_.data(), &st, this->low_);
    this->last_frame_ = Clock::now();
    if (!this->low_ && bad == static_cast<uint32_t>(PIXELS))
      return;  // no pixel held a reading
    // The camera's temperatures are for a black body: emissivity and
    // reflected temperature are not known (as the component).
    host.frame(this->temps_.data(), W, H, &st, NAN, NAN);
  }

  void stream_(Host &host) {
    this->last_frame_ = Clock::now();
    Clock::time_point report = Clock::now();
    while (!host.stopping() && !this->gone_) {
      this->service_(host, 20);
      std::string action;
      while (host.take_action(&action)) {
        bool ok = true;
        if (action == "Shutter")
          ok = this->cmd_(command(Cmd::SHUTTER));
        else if (action == "High range (-20..150 C)")
          ok = this->set_range_(host, 1);
        else if (action == "Low range (150..600 C)")
          ok = this->set_range_(host, 0);
        if (!ok) {
          host.status("Thermal Master P1: " + action + " failed (" + this->error_ + ")");
          report = Clock::now() + std::chrono::seconds(3);
        }
      }
      const Clock::time_point now = Clock::now();
      if (now >= report) {
        report = now + std::chrono::seconds(1);
        const bool stalled = now - this->last_frame_ > std::chrono::seconds(3);
        host.status(this->name_ + (this->low_ ? ", low range" : ", high range") +
                    (stalled ? " - no frames for 3 s" : "") + " - " + std::to_string(this->frames_) + " frames, " +
                    std::to_string(this->damaged_) + " damaged, " + std::to_string(this->errors_) +
                    " transfer errors, " + std::to_string(this->assembler_.camera_skipped()) + " skipped by the camera");
      }
    }
    host.status(this->gone_ ? "Thermal Master P1: unplugged" : "Stopped");
  }

  void stop_() {
    this->stopping_ = true;
    for (auto *t : this->xfers_)
      libusb_cancel_transfer(t);
    for (int i = 0; i < 40 && this->inflight_ > 0; i++)
      this->dev_.pump(50);
    if (this->dev_.h != nullptr && !this->gone_)
      libusb_set_interface_alt_setting(this->dev_.h, 1, 0);
    if (this->inflight_ == 0)
      for (auto *t : this->xfers_)
        libusb_free_transfer(t);
    this->xfers_.clear();
    this->dev_.close();
  }

  UsbDevice dev_;
  std::string error_, name_;
  Assembler assembler_;
  std::vector<uint8_t> frame_ = std::vector<uint8_t>(FRAME_BYTES);
  std::vector<uint8_t> ready_ = std::vector<uint8_t>(FRAME_BYTES);
  std::vector<float> temps_ = std::vector<float>(PIXELS);
  std::vector<std::vector<uint8_t>> bufs_;
  std::vector<libusb_transfer *> xfers_;
  int inflight_{0};
  bool have_ready_{false}, stopping_{false}, gone_{false}, low_{false};
  unsigned frames_{0}, damaged_{0}, errors_{0};
  Clock::time_point last_frame_;
};

}  // namespace

CameraType thermal_master_p1_type() {
  return {"thermal_master_p1", "Thermal Master P1 (USB)", "", USB_UNAVAILABLE,
          {"Shutter", "High range (-20..150 C)", "Low range (150..600 C)"}, 4, 0.0f,
          []() -> Camera * { return new P1(); }};
}

}  // namespace tv

#else

namespace tv {
CameraType thermal_master_p1_type() {
  return {"thermal_master_p1", "Thermal Master P1 (USB)", "", USB_UNAVAILABLE, {}, 4, 0.0f,
          []() -> Camera * { return nullptr; }};
}
}  // namespace tv

#endif
