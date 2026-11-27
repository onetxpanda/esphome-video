#pragma once

// Turns radiometric frames into images: tone mapping onto a palette,
// mirroring, upscaling and output format. Runs on its own task; when frames
// arrive faster than they are processed, only the newest waits (older ones
// are dropped). Scaling uses the CPU or, on the ESP32-P4, the PPA.

#include <array>
#include <atomic>
#include <memory>
#include <mutex>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include "../video_core/frame.h"
#include "isp_core.h"
#include "palette.h"

#ifdef USE_ISP_PPA
#include "driver/ppa.h"
#endif

namespace esphome {
namespace isp {

enum class Scaler : uint8_t { CPU = 0, PPA = 1 };

class Isp : public Component,
            public video::RadiometricSink,
            public video::ImageSource,
            public video::PaletteTarget {
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
  // Draws the last frame again at once (thread-safe). Every option change
  // does this too, so a change shows without waiting for the next frame,
  // however slow the sensor's frame rate.
  void request_refresh() override;

  // Configuration (before setup)
  void set_scale(uint8_t s) { this->opt_.scale = s; }
  // Fit each frame within fit_w x fit_h instead (see fit_scale); 0 x 0: off.
  void set_fit(uint16_t w, uint16_t h) {
    this->fit_w_ = w;
    this->fit_h_ = h;
  }
  void set_format(video::ImageFormat f) { this->opt_.format = f; }
  void set_palette(Palette p) { this->palette_ = static_cast<uint8_t>(p); }
  // PaletteTarget (main loop): applied by the ISP task from the next frame.
  void set_palette_index(uint8_t index) override {
    this->palette_ = index < PALETTE_COUNT ? index : static_cast<uint8_t>(Palette::IRONBOW);
    this->request_refresh();
  }
  void set_log_timing(bool v) { this->log_timing_ = v; }

  // Runtime options; thread-safe.
  void set_range_min(float v);
  void set_range_max(float v);
  void set_mirror_x(bool v);
  void set_mirror_y(bool v);
  void set_rotate(Rotation r);
  void set_upscale(Upscale u);
  void set_scaler(Scaler s);
  // Equalization in percent (0 = linear, 100 = fully equalized).
  void set_equalize(float percent);
  void set_detail(float gain);

  // Spot meters, placed at run time (e.g. by clicks on the image, through API
  // actions): each adds an annotation with the temperature under it,
  // corrected for its material, for an overlay. Up to MAX_SPOTS, kept across
  // reboots under `key`. Main loop (API actions, buttons).
  // Removes the spot at (x, y), in percent of the image as shown from the top
  // left, or adds one of `material` (a key or label of MATERIALS) there.
  void toggle_spot(float x_percent, float y_percent, const std::string &material);
  // Spots are numbered from 1, in the order placed.
  void set_spot_material(int spot, const std::string &material);
  void remove_spot(int spot);
  void clear_spots();
  void set_spots_key(uint32_t key) { this->spots_key_ = key; }
#ifdef USE_TEXT_SENSOR
  // The spots as "x,y,material,color;..." (see format_spots).
  void set_spots_text_sensor(text_sensor::TextSensor *s) { this->spots_sensor_ = s; }
#endif

 protected:
  static void task_entry_(void *arg);
  void task_loop_();
  void process_(const video::RadiometricFrame &in, const Options &o, Scaler scaler, const Spot *spots,
                uint8_t spot_count, bool refresh);
  void wake_();  // the ISP task
  bool render_ppa_(const Options &o, video::ImageFrame &out);

  std::mutex mutex_;  // guards opt_, scaler_, spots_, pending_, last_ and refresh_
  Options opt_;
  uint16_t fit_w_{0}, fit_h_{0};
  video::LargeBuffer<float> oriented_;  // ISP task: a rotated frame
  Scaler scaler_{Scaler::CPU};
  std::array<Spot, MAX_SPOTS> spots_{};
  uint8_t spot_count_{0};
  float aspect_{4.0f / 3.0f};  // of the last frame (width / height), for toggle_spot
  struct SpotStore {
    uint8_t count;
    Spot spots[MAX_SPOTS];
  };
  void spots_changed_();  // main loop: saves and publishes them
  uint32_t spots_key_{0};
  ESPPreferenceObject spots_pref_;
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *spots_sensor_{nullptr};
#endif
  std::atomic<uint8_t> palette_{static_cast<uint8_t>(Palette::IRONBOW)};
  uint8_t palette_applied_{0xFF};  // ISP task only
  video::RadiometricFrame pending_;
  bool has_pending_{false};
  video::RadiometricFrame last_;  // the last frame drawn, to draw again on a change
  bool refresh_{false};           // draw last_ again
  uint32_t last_ts_{0};           // ISP task: the last new frame's time, for curve_blend
  bool have_last_ts_{false};

  CpuIsp cpu_;
  video::SinkList<video::ImageSink> sinks_;
  std::unique_ptr<video::FramePool> pool_;
  size_t pool_size_{0};
  TaskHandle_t task_{nullptr};

#ifdef USE_ISP_PPA
  ppa_client_handle_t ppa_{nullptr};
  uint8_t *ppa_in_{nullptr};  // the index image, for the PPA to read
  size_t ppa_in_cap_{0};
  uint8_t *ppa_out_{nullptr};  // the scaled index image
  size_t ppa_out_cap_{0};
  bool ppa_failed_{false};  // the PPA rejected an operation: scale on the CPU from then on
  bool memory_error_logged_{false};
#endif

  bool log_timing_{false};
  std::atomic<uint32_t> frames_{0}, dropped_{0}, no_buffer_{0};
  std::atomic<uint32_t> tone_us_{0}, scale_us_{0}, sink_us_{0};
  uint32_t last_report_ms_{0};
};

}  // namespace isp
}  // namespace esphome
