#include "isp.h"

#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#ifdef USE_ISP_PPA
#include <esp_heap_caps.h>
#endif

namespace esphome {
namespace isp {

static const char *const TAG = "isp";
static constexpr size_t POOL_FRAMES = 3;

static const char *format_name(video::ImageFormat f) {
  switch (f) {
    case video::ImageFormat::RGB565_LE:
      return "RGB565 little-endian";
    case video::ImageFormat::RGB565_BE:
      return "RGB565 big-endian";
    case video::ImageFormat::RGB888:
      return "RGB888";
    case video::ImageFormat::GRAY8:
      return "8-bit grey";
  }
  return "?";
}

void Isp::setup() {
  if (!this->sinks_.modifying_sink_is_exclusive()) {
    this->mark_failed(LOG_STR("An overlay must be the only consumer of its source; chain other consumers after it"));
    return;
  }
  this->cpu_.set_palette(make_palette(this->palette_));
#ifdef USE_ISP_PPA
  ppa_client_config_t cfg{};
  cfg.oper_type = PPA_OPERATION_SRM;
  cfg.max_pending_trans_num = 1;
  if (ppa_register_client(&cfg, &this->ppa_) != ESP_OK) {
    ESP_LOGW(TAG, "PPA unavailable; scaling on the CPU");
    this->ppa_ = nullptr;
  }
#endif
  // Processing waits for frames on this task; tone mapping and CPU scaling
  // need little stack, sinks (overlay, encoder hand-off) a little more.
  if (xTaskCreate(Isp::task_entry_, "isp", 6144, this, 4, &this->task_) != pdPASS)
    this->mark_failed(LOG_STR("Cannot start the ISP task"));
}

void Isp::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ISP:\n"
                "  Scale: %ux, format: %s, palette: %s\n"
                "  Range: %.1f .. %.1f\n"
                "  Mirror: x %s, y %s; upscale: %s; scaler: %s",
                this->opt_.scale, format_name(this->opt_.format), this->palette_ == Palette::IRON ? "iron" : "grey",
                this->opt_.range_min, this->opt_.range_max, YESNO(this->opt_.mirror_x), YESNO(this->opt_.mirror_y),
                this->opt_.upscale == Upscale::BILINEAR ? "bilinear" : "nearest",
                this->scaler_ == Scaler::PPA ? "PPA" : "CPU");
#ifndef USE_ISP_PPA
  if (this->scaler_ == Scaler::PPA)
    ESP_LOGW(TAG, "  The PPA exists only on the ESP32-P4; scaling on the CPU");
#endif
}

void Isp::set_range_min(float v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.range_min = v;
}
void Isp::set_range_max(float v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.range_max = v;
}
void Isp::set_mirror_x(bool v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.mirror_x = v;
}
void Isp::set_mirror_y(bool v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.mirror_y = v;
}
void Isp::set_upscale(Upscale u) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.upscale = u;
}
void Isp::set_scaler(Scaler s) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->scaler_ = s;
}

void Isp::on_radiometric_frame(const video::RadiometricFrame &frame) {
  if (this->task_ == nullptr)
    return;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (this->has_pending_)
      this->dropped_++;
    this->pending_ = frame;  // holds the buffer until processed
    this->has_pending_ = true;
  }
  xTaskNotifyGive(this->task_);
}

void Isp::task_entry_(void *arg) { static_cast<Isp *>(arg)->task_loop_(); }

void Isp::task_loop_() {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    video::RadiometricFrame f;
    Options o;
    Scaler scaler;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (!this->has_pending_)
        continue;
      f = std::move(this->pending_);
      this->pending_ = video::RadiometricFrame();
      this->has_pending_ = false;
      o = this->opt_;
      scaler = this->scaler_;
    }
    this->process_(f, o, scaler);
  }
}

void Isp::process_(const video::RadiometricFrame &in, const Options &o, Scaler scaler) {
  const uint32_t t0 = micros();
  if (!this->cpu_.tone_map(in.temps(), in.width, in.height, o)) {
    if (!this->memory_error_logged_) {
      ESP_LOGE(TAG, "Out of memory for a %ux%u index buffer; frames are dropped", in.width, in.height);
      this->memory_error_logged_ = true;
    }
    return;
  }
  const uint32_t t1 = micros();

  const int s = o.scale < 1 ? 1 : o.scale;
  video::ImageFrame out;
  out.width = in.width * s;
  out.height = in.height * s;
  out.format = o.format;
  out.stride = out.width * video::bytes_per_pixel(o.format);
  out.timestamp_ms = in.timestamp_ms;
  out.sequence = in.sequence;
  const size_t bytes = out.length();
  if (this->pool_ == nullptr || this->pool_size_ < bytes) {
    this->pool_.reset();
#ifdef USE_ESP32
    this->pool_ = std::make_unique<video::FramePool>(POOL_FRAMES, bytes, video::dma_alignment(),
                                                     video::dma_allocator());
#else
    this->pool_ = std::make_unique<video::FramePool>(POOL_FRAMES, bytes);
#endif
    this->pool_size_ = this->pool_->ok() ? bytes : 0;
    if (!this->pool_->ok())
      ESP_LOGE(TAG, "Out of memory for %ux%u image buffers", out.width, out.height);
  }
  out.buffer = this->pool_->acquire();
  if (!out.buffer) {
    this->no_buffer_++;
    return;
  }

  bool done = false;
  if (scaler == Scaler::PPA)
    done = this->render_ppa_(o, out);
  if (!done)
    this->cpu_.render(o, out.buffer.data(), out.stride);
  const uint32_t t2 = micros();

  for (uint8_t i = 0; i < in.annotations.count; i++) {
    video::Annotation a = in.annotations.items[i];
    const Point p = map_point(o, in.width, in.height, a.x, a.y);
    a.x = static_cast<float>(p.x);
    a.y = static_cast<float>(p.y);
    out.annotations.add(a);
  }
  this->sinks_.deliver(out, [](video::ImageSink *sk, const video::ImageFrame &fr) { sk->on_image_frame(fr); });
  const uint32_t t3 = micros();
  this->frames_++;
  this->tone_us_ += t1 - t0;
  this->scale_us_ += t2 - t1;
  this->sink_us_ += t3 - t2;
}

bool Isp::render_ppa_(const Options &o, video::ImageFrame &out) {
#ifdef USE_ISP_PPA
  if (this->ppa_ == nullptr)
    return false;
  ppa_srm_color_mode_t cm;
  if (o.format == video::ImageFormat::RGB565_LE) {
    cm = PPA_SRM_COLOR_MODE_RGB565;
  } else if (o.format == video::ImageFormat::RGB888) {
    cm = PPA_SRM_COLOR_MODE_RGB888;  // byte order is kept; scaling is per channel
  } else {
    if (!this->ppa_error_logged_) {
      ESP_LOGW(TAG, "The PPA cannot output %s; scaling on the CPU", format_name(o.format));
      this->ppa_error_logged_ = true;
    }
    return false;
  }
  const uint16_t w = this->cpu_.width(), h = this->cpu_.height();
  const size_t bpp = video::bytes_per_pixel(o.format);
  const size_t need = static_cast<size_t>(w) * h * bpp;
  if (this->ppa_in_cap_ < need) {
    heap_caps_free(this->ppa_in_);
    this->ppa_in_ = static_cast<uint8_t *>(
        heap_caps_aligned_calloc(video::dma_alignment(), 1, need, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    if (this->ppa_in_ == nullptr)  // large frames: PSRAM (the PPA driver writes the cache back)
      this->ppa_in_ = static_cast<uint8_t *>(
          heap_caps_aligned_calloc(video::dma_alignment(), 1, need, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
    this->ppa_in_cap_ = this->ppa_in_ ? need : 0;
    if (this->ppa_in_ == nullptr)
      return false;
  }
  this->cpu_.colorize(o, this->ppa_in_, w * bpp);

  ppa_srm_oper_config_t c{};
  c.in.buffer = this->ppa_in_;
  c.in.pic_w = w;
  c.in.pic_h = h;
  c.in.block_w = w;
  c.in.block_h = h;
  c.in.srm_cm = cm;
  c.out.buffer = out.buffer.data();
  c.out.buffer_size = this->pool_->buffer_size();
  c.out.pic_w = out.width;
  c.out.pic_h = out.height;
  c.out.srm_cm = cm;
  c.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
  c.scale_x = o.scale;
  c.scale_y = o.scale;
  c.mode = PPA_TRANS_MODE_BLOCKING;
  const esp_err_t err = ppa_do_scale_rotate_mirror(this->ppa_, &c);
  if (err != ESP_OK) {
    if (!this->ppa_error_logged_) {
      ESP_LOGW(TAG, "PPA scaling failed (%s); scaling on the CPU", esp_err_to_name(err));
      this->ppa_error_logged_ = true;
    }
    return false;
  }
  return true;
#else
  (void) o;
  (void) out;
  return false;
#endif
}

void Isp::loop() {
  if (!this->log_timing_)
    return;
  const uint32_t now = millis();
  if (this->last_report_ms_ == 0) {
    this->last_report_ms_ = now;
    return;
  }
  const uint32_t elapsed = now - this->last_report_ms_;
  if (elapsed < 10000)
    return;
  this->last_report_ms_ = now;
  const uint32_t n = this->frames_.exchange(0), d = this->dropped_.exchange(0), nb = this->no_buffer_.exchange(0);
  const uint32_t tone = this->tone_us_.exchange(0), scale = this->scale_us_.exchange(0),
                 sink = this->sink_us_.exchange(0);
  auto avg = [n](uint32_t us) { return n ? us / 1000.0f / n : 0.0f; };
  Scaler scaler;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    scaler = this->scaler_;
  }
  ESP_LOGI(TAG,
           "Over %.1f s: %.2f frames/s, %u superseded, %u without a free buffer. Per frame: tone map %.2f ms, "
           "scale (%s) %.2f ms, sinks %.2f ms",
           elapsed / 1000.0f, n * 1000.0f / elapsed, static_cast<unsigned>(d), static_cast<unsigned>(nb), avg(tone),
           scaler == Scaler::PPA ? "PPA" : "CPU", avg(scale), avg(sink));
}

}  // namespace isp
}  // namespace esphome
