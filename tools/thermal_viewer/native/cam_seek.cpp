// Seek Compact Pro through libusb: the component's start-up sequence and one
// frame request per frame (as tests/host/seek_capture.cpp), every frame fed
// to seek_core's Processor. Only the requests the component sends; nothing
// that writes the camera's flash.

#include "tv_camera.h"

#ifndef TV_NO_USB

#include <chrono>
#include <cstring>
#include <deque>

#include "seek_compact_pro/seek_core.h"
#include "tv_usb.h"

using namespace esphome::seek_compact_pro;

namespace tv {
namespace {

using Clock = std::chrono::steady_clock;
constexpr uint16_t VID = 0x289D, PID = 0x0011;
constexpr uint8_t EP_IN = 0x81, IFACE = 0;
constexpr uint8_t TYPE_OUT = 0x41, TYPE_IN = 0xC1;
constexpr int XFER_BYTES = 27 * 512;  // one 13680-byte piece fits; it ends in a short packet
constexpr int NUM_XFERS = 3;

enum : uint8_t {
  REQ_TOGGLE_SHUTTER = 55,
  REQ_READ_CHIP_ID = 54,
  REQ_SET_OPERATION_MODE = 60,
  REQ_SET_IMAGE_PROCESSING_MODE = 62,
  REQ_GET_FIRMWARE_INFO = 78,
  REQ_START_GET_IMAGE_TRANSFER = 83,
  REQ_TARGET_PLATFORM = 84,
  REQ_SET_FIRMWARE_INFO_FEATURES = 85,
  REQ_SET_FACTORY_SETTINGS_FEATURES = 86,
  REQ_GET_FACTORY_SETTINGS = 88,
};

class Seek : public Camera {
 public:
  void run(Host &host, const std::string &) override {
    std::string err;
    if (!this->dev_.open(VID, PID, "Seek Compact Pro", &err) || !this->dev_.claim(IFACE, &err)) {
      host.status(err);
      return;
    }
    host.status("Seek Compact Pro: starting");
    if (!this->start_up_()) {
      host.status("Seek Compact Pro: start-up failed (" + this->error_ + ")");
    } else {
      if (!this->proc_.set_factory(this->table_.data(), this->table_.size()))
        host.status("Seek Compact Pro: factory calibration not supported, no temperatures");
      else
        this->stream_(host);
    }
    this->stop_();
  }

 protected:
  bool out_(uint8_t req, std::vector<uint8_t> data) {
    const int r = libusb_control_transfer(this->dev_.h, TYPE_OUT, req, 0, 0, data.data(),
                                          static_cast<uint16_t>(data.size()), 1000);
    if (r != static_cast<int>(data.size())) {
      this->error_ = usb_error(("request " + std::to_string(req)).c_str(), r);
      return false;
    }
    return true;
  }
  bool in_(uint8_t req, uint16_t len, std::vector<uint8_t> &out) {
    out.assign(len, 0);
    const int r = libusb_control_transfer(this->dev_.h, TYPE_IN, req, 0, 0, out.data(), len, 1000);
    if (r != len) {
      this->error_ = usb_error(("request " + std::to_string(req)).c_str(), r);
      return false;
    }
    return true;
  }

  bool start_up_() {
    std::vector<uint8_t> v;
    this->out_(REQ_SET_OPERATION_MODE, {0x00, 0x00});  // in case it was left running
    if (!this->out_(REQ_TARGET_PLATFORM, {0x01})) {
      for (int i = 0; i < 3; i++)
        this->out_(REQ_SET_OPERATION_MODE, {0x00, 0x00});
      if (!this->out_(REQ_TARGET_PLATFORM, {0x01}))
        return false;
    }
    if (!this->out_(REQ_SET_OPERATION_MODE, {0x00, 0x00}))
      return false;
    if (!this->in_(REQ_GET_FIRMWARE_INFO, 4, v) || !this->in_(REQ_READ_CHIP_ID, 12, v))
      return false;
    if (!this->out_(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x06, 0x00, 0x08, 0x00, 0x00, 0x00}) ||
        !this->in_(REQ_GET_FACTORY_SETTINGS, 12, v))
      return false;
    if (!this->out_(REQ_SET_FIRMWARE_INFO_FEATURES, {0x17, 0x00}) || !this->in_(REQ_GET_FIRMWARE_INFO, 64, v))
      return false;
    if (!this->out_(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x01, 0x00, 0x00, 0x06, 0x00, 0x00}) ||
        !this->in_(REQ_GET_FACTORY_SETTINGS, 2, v))
      return false;
    if (!this->out_(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x01, 0x00, 0x01, 0x06, 0x00, 0x00}) ||
        !this->in_(REQ_GET_FACTORY_SETTINGS, 2, v))
      return false;
    this->table_.clear();
    for (uint16_t addr = 0; addr < 0xA00; addr += 0x20) {
      if (!this->out_(REQ_SET_FACTORY_SETTINGS_FEATURES,
                      {0x20, 0x00, static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>(addr >> 8), 0x00, 0x00}) ||
          !this->in_(REQ_GET_FACTORY_SETTINGS, 64, v))
        return false;
      this->table_.insert(this->table_.end(), v.begin(), v.end());
    }
    if (!this->out_(REQ_SET_FIRMWARE_INFO_FEATURES, {0x15, 0x00}) || !this->in_(REQ_GET_FIRMWARE_INFO, 64, v))
      return false;
    return this->out_(REQ_SET_IMAGE_PROCESSING_MODE, {0x08, 0x00}) && this->out_(REQ_SET_OPERATION_MODE, {0x01, 0x00});
  }

  void stop_camera_() {
    if (this->dev_.h == nullptr || this->gone_)
      return;
    for (int i = 0; i < 3; i++)
      this->out_(REQ_SET_OPERATION_MODE, {0x00, 0x00});
  }

  static void LIBUSB_CALL credit_cb_(libusb_transfer *t) {
    auto *s = static_cast<Seek *>(t->user_data);
    s->credit_busy_ = false;
    if (t->status != LIBUSB_TRANSFER_COMPLETED)
      s->credit_errors_++;
  }
  // START_GET_IMAGE_TRANSFER for one frame.
  void send_credit_() {
    if (this->credit_busy_)
      return;
    libusb_fill_control_setup(this->credit_buf_, TYPE_OUT, REQ_START_GET_IMAGE_TRANSFER, 0, 0, 4);
    const uint32_t words = RAW_WORDS;
    std::memcpy(this->credit_buf_ + LIBUSB_CONTROL_SETUP_SIZE, &words, 4);
    libusb_fill_control_transfer(this->credit_, this->dev_.h, this->credit_buf_, Seek::credit_cb_, this, 1000);
    if (libusb_submit_transfer(this->credit_) == 0)
      this->credit_busy_ = true;
    else
      this->credit_errors_++;
  }

  static void LIBUSB_CALL piece_cb_(libusb_transfer *t) { static_cast<Seek *>(t->user_data)->on_piece_(t); }
  void on_piece_(libusb_transfer *t) {
    if (t->status == LIBUSB_TRANSFER_COMPLETED) {
      const size_t n = static_cast<size_t>(t->actual_length);
      if (this->fill_ + n > RAW_BYTES) {
        this->fill_ = 0;  // resynchronise on the next frame
      } else {
        std::memcpy(this->frame_.data() + this->fill_, t->buffer, n);
        this->fill_ += n;
        if (this->fill_ == RAW_BYTES) {
          this->send_credit_();  // at once: asked for late, the camera stops for good
          std::vector<uint16_t> f(RAW_WORDS);
          std::memcpy(f.data(), this->frame_.data(), RAW_BYTES);
          this->done_.push_back(std::move(f));
          this->fill_ = 0;
          this->last_frame_ = Clock::now();
        }
      }
    } else if (t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_TIMED_OUT) {
      this->errors_++;
    }
    if (!this->stopping_ && t->status != LIBUSB_TRANSFER_CANCELLED && t->status != LIBUSB_TRANSFER_NO_DEVICE &&
        libusb_submit_transfer(t) == 0)
      return;
    if (t->status == LIBUSB_TRANSFER_NO_DEVICE)
      this->gone_ = true;
    this->inflight_--;
  }

  void stream_(Host &host) {
    this->bufs_.assign(NUM_XFERS, std::vector<uint8_t>(XFER_BYTES));
    this->credit_ = libusb_alloc_transfer(0);
    for (int i = 0; i < NUM_XFERS; i++) {
      libusb_transfer *t = libusb_alloc_transfer(0);
      libusb_fill_bulk_transfer(t, this->dev_.h, EP_IN, this->bufs_[i].data(), XFER_BYTES, Seek::piece_cb_, this, 0);
      this->xfers_.push_back(t);
      if (libusb_submit_transfer(t) == 0)
        this->inflight_++;
    }
    this->last_frame_ = Clock::now();
    this->send_credit_();
    std::vector<float> temps(IMG_PIXELS);
    unsigned images = 0, torn = 0, misses = 0, restarts = 0;
    Clock::time_point report = Clock::now();
    while (!host.stopping() && !this->gone_) {
      this->dev_.pump(20);
      bool tables_lost = false;
      while (!this->done_.empty()) {
        const std::vector<uint16_t> f = std::move(this->done_.front());
        this->done_.pop_front();
        const uint16_t *raw = f.data();
        if (frame_magic(raw) != FRAME_MAGIC) {
          torn++;
          continue;
        }
        const uint16_t id = frame_id(raw);
        // The tables come once per start, first; a shutter frame or image
        // before them means one was lost.
        if ((id == FRAME_IMAGE || id == FRAME_SHUTTER || id == FRAME_START_SHUTTER) && !this->proc_.has_table()) {
          tables_lost = true;
          break;
        }
        esphome::video::FrameStats st{};
        if (this->proc_.feed(raw, temps.data(), &st) == Processor::Result::IMAGE) {
          images++;
          misses = 0;
          // For a black body; the reflected temperature is not known (as the component).
          host.frame(temps.data(), IMG_W, IMG_H, &st, NAN, NAN);
        }
      }
      if (tables_lost) {
        if (++restarts > 5) {
          host.status("Seek Compact Pro: the camera's tables were lost 5 times, giving up");
          return;
        }
        this->stop_camera_();
        this->done_.clear();
        this->proc_.reset();
        if (!this->start_up_()) {
          host.status("Seek Compact Pro: restart failed (" + this->error_ + ")");
          return;
        }
        this->fill_ = 0;
        this->last_frame_ = Clock::now();
        this->send_credit_();
      }
      std::string action;
      while (host.take_action(&action))
        if (action == "Shutter")
          this->out_(REQ_TOGGLE_SHUTTER, {0xFC, 0x00, 0x04, 0x00});
      const Clock::time_point now = Clock::now();
      if (now - this->last_frame_ > std::chrono::seconds(1)) {  // ask again, drop the partial frame
        this->fill_ = 0;
        this->last_frame_ = now;
        this->send_credit_();
        if (++misses >= 5) {
          host.status("Seek Compact Pro: no frames for 5 s");
          return;
        }
      }
      if (now >= report) {
        report = now + std::chrono::seconds(1);
        char line[200];
        std::snprintf(line, sizeof(line),
                      "Seek Compact Pro%s - %u images, %u torn, %u transfer errors, %zu dead pixels, warm-up %.2f C",
                      this->proc_.ready() ? "" : " (waiting for its tables and a shutter frame)", images, torn,
                      this->errors_ + this->credit_errors_, this->proc_.dead_pixels(), this->proc_.warm_up_c());
        host.status(line);
      }
    }
    host.status(this->gone_ ? "Seek Compact Pro: unplugged" : "Stopped");
  }

  void stop_() {
    this->stopping_ = true;
    for (auto *t : this->xfers_)
      libusb_cancel_transfer(t);
    for (int i = 0; i < 50 && (this->inflight_ > 0 || this->credit_busy_); i++)
      this->dev_.pump(20);
    this->stop_camera_();
    if (this->inflight_ == 0 && !this->credit_busy_) {
      for (auto *t : this->xfers_)
        libusb_free_transfer(t);
      if (this->credit_ != nullptr)
        libusb_free_transfer(this->credit_);
    }
    this->xfers_.clear();
    this->credit_ = nullptr;
    this->dev_.close();
  }

  UsbDevice dev_;
  std::string error_;
  std::vector<uint8_t> table_;
  Processor proc_;
  std::vector<uint8_t> frame_ = std::vector<uint8_t>(RAW_BYTES);
  size_t fill_{0};
  std::deque<std::vector<uint16_t>> done_;
  std::vector<std::vector<uint8_t>> bufs_;
  std::vector<libusb_transfer *> xfers_;
  libusb_transfer *credit_{nullptr};
  uint8_t credit_buf_[LIBUSB_CONTROL_SETUP_SIZE + 4];
  bool credit_busy_{false}, stopping_{false}, gone_{false};
  int inflight_{0};
  unsigned errors_{0}, credit_errors_{0};
  Clock::time_point last_frame_;
};

}  // namespace

CameraType seek_compact_pro_type() {
  return {"seek_compact_pro", "Seek Compact Pro (USB)", "", USB_UNAVAILABLE, {"Shutter"}, 2, 0.0f,
          []() -> Camera * { return new Seek(); }};
}

}  // namespace tv

#else

namespace tv {
CameraType seek_compact_pro_type() {
  return {"seek_compact_pro", "Seek Compact Pro (USB)", "", USB_UNAVAILABLE, {}, 2, 0.0f,
          []() -> Camera * { return nullptr; }};
}
}  // namespace tv

#endif
