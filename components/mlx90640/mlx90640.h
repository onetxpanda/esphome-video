#pragma once

// MLX90640 thermal sensor. Reads sub-pages on its own task, timed to the
// sensor's data-ready flag, assembles them into 32x24 frames of temperatures
// and hands each frame to its radiometric sinks (on that task). It does no
// image processing.

#include <atomic>
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/component.h"

#include "../video_core/frame.h"
#include "MLX90640_API.h"
#include "subpage_assembler.h"

namespace esphome {
namespace mlx90640 {

class MLX90640 : public i2c::I2CDevice, public Component, public video::RadiometricSource {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  void add_radiometric_sink(video::RadiometricSink *sink) override { this->sinks_.add(sink); }

  // Refresh-rate register value 0..7 (0.5 Hz .. 64 Hz). Thread-safe; applied
  // by the capture task.
  void set_refresh_rate(uint8_t r) { this->refresh_rate_ = r > 7 ? 7 : r; }
  uint8_t get_refresh_rate() const { return this->refresh_rate_; }
  // Thread-safe; applied by the capture task.
  void set_frame_mode(FrameMode m) { this->frame_mode_ = static_cast<uint8_t>(m); }
  FrameMode get_frame_mode() const { return static_cast<FrameMode>(this->frame_mode_.load()); }
  // Emissivity used in the temperature calculation (0.1..1).
  void set_emissivity(float e) { this->emissivity_ = e; }
  // Logs per-stage timing every 10 s.
  void set_log_timing(bool v) { this->log_timing_ = v; }

 protected:
  static void task_entry_(void *arg);
  void task_loop_();
  bool apply_refresh_rate_(uint8_t r);
  uint32_t subpage_period_ms_(uint8_t r) const { return 2000u >> r; }

  video::SinkList<video::RadiometricSink> sinks_;
  std::unique_ptr<video::FramePool> pool_;
  TaskHandle_t task_{nullptr};

  paramsMLX90640 params_{};
  int chess_{1};
  float emissivity_{0.95f};
  std::atomic<uint8_t> refresh_rate_{5};
  std::atomic<uint8_t> frame_mode_{0};
  bool log_timing_{false};

  // Capture task state.
  SubpageAssembler assembler_;
  uint16_t raw_[834]{};
  float temps_[SENSOR_PIXELS]{};
  uint32_t sequence_{0};

  // Statistics, written by the task, read by loop().
  std::atomic<uint32_t> subpages_{0}, frames_{0}, errors_{0}, dropped_{0};
  std::atomic<uint32_t> settle_dropped_{0};  // sub-pages discarded after a rate change
  std::atomic<uint32_t> read_us_{0}, calc_us_{0}, deliver_us_{0}, wait_us_{0};
  uint32_t last_report_ms_{0};
  uint32_t last_errors_{0};
};

}  // namespace mlx90640
}  // namespace esphome
