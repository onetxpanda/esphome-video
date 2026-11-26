#pragma once

// What the USB backends share: a device opened by its ids, with its
// interfaces claimed, closed again when it goes out of scope.

#ifndef TV_NO_USB

#include <libusb.h>

#include <cstdio>
#include <string>
#include <vector>

namespace tv {

inline std::string usb_error(const char *what, int r) {
  return std::string(what) + ": " + (r < 0 ? libusb_error_name(r) : "short transfer");
}

class UsbDevice {
 public:
  ~UsbDevice() { this->close(); }
  // False with `err` set when the camera is not there or cannot be opened.
  bool open(uint16_t vid, uint16_t pid, const char *name, std::string *err) {
    if (libusb_init(&this->ctx) != 0) {
      *err = "libusb did not start";
      return false;
    }
    this->h = libusb_open_device_with_vid_pid(this->ctx, vid, pid);
    if (this->h == nullptr) {
      char ids[16];
      std::snprintf(ids, sizeof(ids), "%04x:%04x", vid, pid);
      *err = std::string("No ") + name + " (" + ids + ") found, or it cannot be opened (permissions)";
      return false;
    }
    libusb_set_auto_detach_kernel_driver(this->h, 1);  // not supported everywhere; claiming says if it matters
    return true;
  }
  bool claim(int interface, std::string *err) {
    const int r = libusb_claim_interface(this->h, interface);
    if (r != 0) {
      *err = usb_error(("Cannot claim interface " + std::to_string(interface)).c_str(), r) +
             " (another program or the system's own driver has the camera)";
      return false;
    }
    this->claimed_.push_back(interface);
    return true;
  }
  // Runs libusb's events for up to `ms`.
  void pump(int ms) {
    timeval tv{ms / 1000, (ms % 1000) * 1000};
    libusb_handle_events_timeout_completed(this->ctx, &tv, nullptr);
  }
  void close() {
    if (this->h != nullptr) {
      for (int i : this->claimed_)
        libusb_release_interface(this->h, i);
      libusb_close(this->h);
    }
    if (this->ctx != nullptr)
      libusb_exit(this->ctx);
    this->h = nullptr;
    this->ctx = nullptr;
    this->claimed_.clear();
  }

  libusb_context *ctx{nullptr};
  libusb_device_handle *h{nullptr};

 protected:
  std::vector<int> claimed_;
};

}  // namespace tv

#endif  // TV_NO_USB
