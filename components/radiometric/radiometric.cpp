#include "radiometric.h"

#include <cmath>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace radiometric {

static const char *const TAG = "radiometric";
static constexpr size_t POOL_FRAMES = 3;

void Radiometric::setup() {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->options_.median_stat = this->median_sensor_ != nullptr;  // only computed for its sensor
  this->options_changed_ = true;  // applied by the source's task with the first frame
}

void Radiometric::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Radiometric processing:\n"
                "  Temporal average: %u frames\n"
                "  Median filter: %s\n"
                "  Outlier level: %.1f",
                this->options_.temporal_frames, YESNO(this->options_.median_filter), this->options_.outlier_level);
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Min", this->min_sensor_);
  LOG_SENSOR("  ", "Max", this->max_sensor_);
  LOG_SENSOR("  ", "Mean", this->mean_sensor_);
  LOG_SENSOR("  ", "Median", this->median_sensor_);
  LOG_SENSOR("  ", "Center", this->center_sensor_);
}

void Radiometric::set_temporal_frames(float n) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  const long v = std::lround(n);
  this->options_.temporal_frames = static_cast<uint8_t>(v < 1 ? 1 : (v > MAX_TEMPORAL ? MAX_TEMPORAL : v));
  this->options_changed_ = true;
}

void Radiometric::set_median_filter(bool v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->options_.median_filter = v;
  this->options_changed_ = true;
}

void Radiometric::set_outlier_level(float v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->options_.outlier_level = v;
  this->options_changed_ = true;
}

bool Radiometric::wants_frames() const { return this->has_sensors_() || this->sinks_.any_wants(); }

void Radiometric::on_radiometric_frame(const video::RadiometricFrame &in) {
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->options_changed_) {
      this->processor_.set_options(this->options_);
      this->options_changed_ = false;
    }
  }
  const uint32_t t0 = micros();
  video::RadiometricFrame out;
  if (!this->processor_.filters_active() && this->processor_.analyze(in.temps(), in.width, in.height)) {
    // Nothing to change: pass the input on (sinks only read frames).
    out = in;
    out.annotations = video::Annotations();
  } else {
    const size_t bytes = sizeof(float) * in.width * in.height;
    if (this->pool_ == nullptr || this->pool_size_ < bytes) {
      // First frame (or a larger sensor): size the pool to the source.
      this->pool_.reset();
#ifdef USE_ESP32
      this->pool_ = std::make_unique<video::FramePool>(POOL_FRAMES, bytes, 64, video::dma_allocator());
#else
      this->pool_ = std::make_unique<video::FramePool>(POOL_FRAMES, bytes);
#endif
      this->pool_size_ = this->pool_->ok() ? bytes : 0;
    }
    out.buffer = this->pool_->acquire();
    if (!out.buffer)
      return;  // all buffers held downstream: drop this frame
    out.width = in.width;
    out.height = in.height;
    out.timestamp_ms = in.timestamp_ms;
    out.sequence = in.sequence;
    if (!this->processor_.process(in.temps(), out.temps(), in.width, in.height)) {
      if (!this->memory_error_logged_) {
        ESP_LOGE(TAG, "Out of memory for %ux%u processing buffers; frames are dropped", in.width, in.height);
        this->memory_error_logged_ = true;
      }
      return;
    }
  }
  this->process_us_ += micros() - t0;
  this->processed_++;
  const Stats st = this->processor_.stats();
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->latest_ = st;
    this->latest_new_ = true;
  }
  if (st.valid) {
    out.annotations.add({static_cast<float>(st.min_x), static_cast<float>(st.min_y), st.min,
                         video::AnnotationKind::MIN});
    out.annotations.add({static_cast<float>(st.max_x), static_cast<float>(st.max_y), st.max,
                         video::AnnotationKind::MAX});
    out.annotations.add({static_cast<float>(in.width / 2), static_cast<float>(in.height / 2), st.center,
                         video::AnnotationKind::CENTER});
  }
  this->sinks_.deliver(out, [](video::RadiometricSink *s, const video::RadiometricFrame &f) {
    s->on_radiometric_frame(f);
  });
}

void Radiometric::update() {
  if (this->log_timing_) {
    const uint32_t now = millis();
    if (this->last_timing_ms_ == 0) {
      this->last_timing_ms_ = now;
    } else if (now - this->last_timing_ms_ >= 10000) {
      const float s = (now - this->last_timing_ms_) / 1000.0f;
      this->last_timing_ms_ = now;
      const uint32_t us = this->process_us_.exchange(0), n = this->processed_.exchange(0);
      ESP_LOGI(TAG, "Over %.1f s: %.2f frames/s processed, %.2f ms per frame", s, n / s, n ? us / 1000.0f / n : 0.0f);
    }
  }
  Stats st;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (!this->latest_new_)
      return;
    st = this->latest_;
    this->latest_new_ = false;
  }
  if (!st.valid)
    return;
  if (this->min_sensor_ != nullptr)
    this->min_sensor_->publish_state(st.min);
  if (this->max_sensor_ != nullptr)
    this->max_sensor_->publish_state(st.max);
  if (this->mean_sensor_ != nullptr)
    this->mean_sensor_->publish_state(st.mean);
  if (this->median_sensor_ != nullptr)
    this->median_sensor_->publish_state(st.median);
  if (this->center_sensor_ != nullptr)
    this->center_sensor_->publish_state(st.center);
}

}  // namespace radiometric
}  // namespace esphome
