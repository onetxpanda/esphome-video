#pragma once

// Camera backends of the desktop viewer. Each one talks to its camera from a
// PC (libusb, or Linux i2c-dev) and turns what arrives into temperatures with
// the same core the ESP32-P4 component uses (tm_core, seek_core, hti301_core,
// the Melexis API and SubpageAssembler). A backend sends only what its
// component sends.

#include <string>
#include <vector>

#include "video_core/frame.h"

namespace tv {

// What a backend talks to: the viewer's pipeline and its controls.
class Host {
 public:
  virtual ~Host() = default;
  // One frame of temperatures (degrees C, row-major). `stats` may be null;
  // emissivity and reflected are NaN when the source does not know them.
  virtual void frame(const float *temps, uint16_t w, uint16_t h, const esphome::video::FrameStats *stats,
                     float emissivity, float reflected) = 0;
  // A line for the status bar.
  virtual void status(const std::string &text) = 0;
  // True once the camera should stop: run() returns soon after.
  virtual bool stopping() const = 0;
  // The next action the user asked for (one of the type's `actions`), if any.
  virtual bool take_action(std::string *name) = 0;
  // Sleeps up to `ms`; false if stopping.
  bool wait(unsigned ms);
};

class Camera {
 public:
  virtual ~Camera() = default;
  // Opens the camera and delivers frames until host.stopping(), or until the
  // camera fails (the last status says why).
  virtual void run(Host &host, const std::string &arg) = 0;
};

struct CameraType {
  const char *key;
  const char *label;
  const char *arg_hint;        // what `arg` means; "" when there is none
  const char *unavailable;     // why this build cannot use it; "" when it can
  std::vector<const char *> actions;
  int scale;                   // the scale that gives about 640 x 480
  float outlier_level;         // the radiometric default that suits it
  Camera *(*make)();
};

const std::vector<CameraType> &camera_types();

// Each backend's entry (in its own file).
CameraType synthetic_type();
CameraType thermal_master_p1_type();
CameraType seek_compact_pro_type();
CameraType hti301_type();
CameraType mlx90640_type();

#ifndef TV_NO_USB
static constexpr const char *USB_UNAVAILABLE = "";
#else
static constexpr const char *USB_UNAVAILABLE = "built without libusb (install libusb 1.0 and start the viewer again)";
#endif

}  // namespace tv
