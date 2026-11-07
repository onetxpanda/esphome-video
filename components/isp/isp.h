#pragma once

// Turns radiometric frames into images: tone mapping onto a palette,
// mirroring, upscaling and output format. Runs on its own task; when frames
// arrive faster than they are processed, only the newest waits (older ones
// are dropped). Scaling uses the CPU or, on the ESP32-P4, the PPA.

#include <atomic>
#include <memory>
#include <mutex>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/core/component.h"

#include "../video_core/frame.h"
#include "isp_core.h"
#include "palette.h"

#ifdef USE_ISP_PPA
#include "driver/ppa.h"
#endif

namespace esphome {
namespace isp {

enum class Scaler : uint8_t { CPU = 0, PPA = 1 };

class Isp : public Component, public video::RadiometricSink, public video::ImageSource {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  // RadiometricSink (source's task)
  void on_radiometric_frame(const video::RadiometricFrame &frame) override;
  bool wants_frames() const override { return this->sinks_.any_wants(); }
  // ImageSource
  void add_image_sink(video::ImageSink *sink) override { this->sinks_.add(sink); }

  // Configuration (before setup)
  void set_scale(uint8_t s) { this->opt_.scale = s; }
  void set_format(video::ImageFormat f) { this->opt_.format = f; }
  void set_palette(Palette p) { this->palette_ = p; }
  void set_log_timing(bool v) { this->log_timing_ = v; }

  // Runtime options; thread-safe.
  void set_range_min(float v);
  void set_range_max(float v);
  void set_mirror_x(bool v);
  void set_mirror_y(bool v);
  void set_upscale(Upscale u);
  void set_scaler(Scaler s);

 protected:
  static void task_entry_(void *arg);
  void task_loop_();
  void process_(const video::RadiometricFrame &in, const Options &o, Scaler scaler);
  bool render_ppa_(const Options &o, video::ImageFrame &out);

  std::mutex mutex_;  // guards opt_, scaler_ and pending_
  Options opt_;
  Scaler scaler_{Scaler::CPU};
  Palette palette_{Palette::IRON};
  video::RadiometricFrame pending_;
  bool has_pending_{false};

  CpuIsp cpu_;
  video::SinkList<video::ImageSink> sinks_;
  std::unique_ptr<video::FramePool> pool_;
  size_t pool_size_{0};
  TaskHandle_t task_{nullptr};

#ifdef USE_ISP_PPA
  ppa_client_handle_t ppa_{nullptr};
  uint8_t *ppa_in_{nullptr};  // input-resolution colour image
  size_t ppa_in_cap_{0};
  bool ppa_error_logged_{false};
#endif

  bool log_timing_{false};
  std::atomic<uint32_t> frames_{0}, dropped_{0}, no_buffer_{0};
  std::atomic<uint32_t> tone_us_{0}, scale_us_{0}, sink_us_{0};
  uint32_t last_report_ms_{0};
};

}  // namespace isp
}  // namespace esphome
