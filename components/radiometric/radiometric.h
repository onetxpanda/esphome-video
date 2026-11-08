#pragma once

// Temperature-domain processing between a radiometric source (a thermal
// sensor) and its consumers: filtering, statistics and their sensors. Frames
// are processed on the source's task; sensors publish from the main loop
// every update_interval.

#include <atomic>
#include <memory>
#include <mutex>

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

#include "../video_core/frame.h"
#include "radiometric_core.h"

namespace esphome {
namespace radiometric {

class Radiometric : public PollingComponent, public video::RadiometricSink, public video::RadiometricSource {
 public:
  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void update() override;
  void dump_config() override;

  // RadiometricSink (source's task)
  void on_radiometric_frame(const video::RadiometricFrame &frame) override;
  bool wants_frames() const override;
  // RadiometricSource
  void add_radiometric_sink(video::RadiometricSink *sink) override { this->sinks_.add(sink); }

  // Options; thread-safe.
  void set_temporal_frames(float n);
  void set_median_filter(bool v);
  void set_outlier_level(float v);

  void set_min_sensor(sensor::Sensor *s) { this->min_sensor_ = s; }
  void set_max_sensor(sensor::Sensor *s) { this->max_sensor_ = s; }
  void set_mean_sensor(sensor::Sensor *s) { this->mean_sensor_ = s; }
  void set_median_sensor(sensor::Sensor *s) { this->median_sensor_ = s; }
  void set_center_sensor(sensor::Sensor *s) { this->center_sensor_ = s; }
  // Logs the processing time per frame every 10 s.
  void set_log_timing(bool v) { this->log_timing_ = v; }

 protected:
  bool has_sensors_() const {
    return this->min_sensor_ || this->max_sensor_ || this->mean_sensor_ || this->median_sensor_ ||
           this->center_sensor_;
  }

  // Guards options_, options_changed_, latest_ and latest_new_ only: never
  // held while a frame is processed, so the main loop is not kept waiting.
  std::mutex mutex_;
  Processor processor_;  // the source's task only
  Options options_;
  bool options_changed_{false};
  Stats latest_;
  bool log_timing_{false};
  std::atomic<uint32_t> process_us_{0}, processed_{0};
  uint32_t last_timing_ms_{0};
  bool memory_error_logged_{false};
  bool latest_new_{false};

  video::SinkList<video::RadiometricSink> sinks_;
  std::unique_ptr<video::FramePool> pool_;
  size_t pool_size_{0};

  sensor::Sensor *min_sensor_{nullptr};
  sensor::Sensor *max_sensor_{nullptr};
  sensor::Sensor *mean_sensor_{nullptr};
  sensor::Sensor *median_sensor_{nullptr};
  sensor::Sensor *center_sensor_{nullptr};
};

}  // namespace radiometric
}  // namespace esphome
