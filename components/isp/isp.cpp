#include "isp.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <esp_heap_caps.h>
#include <freertos/idf_additions.h>

// The task's stack is in PSRAM: internal RAM is needed by the network stack,
// USB and (with Matter) the WebRTC stack.
static constexpr UBaseType_t TASK_STACK_CAPS = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

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
  if (this->spots_key_ != 0) {
    this->spots_pref_ = global_preferences->make_preference<SpotStore>(this->spots_key_);
    SpotStore st{};
    if (this->spots_pref_.load(&st) && st.count <= MAX_SPOTS) {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->spot_count_ = 0;
      for (uint8_t i = 0; i < st.count; i++) {
        Spot sp = st.spots[i];
        if (!(std::isfinite(sp.x) && std::isfinite(sp.y) && sp.x >= 0 && sp.x <= 1 && sp.y >= 0 && sp.y <= 1 &&
              sp.material < MATERIAL_COUNT))
          continue;
        // Each spot keeps its colour; a missing or repeated one gets a free one.
        bool taken = sp.color >= MAX_SPOTS;
        for (uint8_t j = 0; j < this->spot_count_ && !taken; j++)
          taken = this->spots_[j].color == sp.color;
        if (taken)
          sp.color = free_spot_color(this->spots_.data(), this->spot_count_);
        this->spots_[this->spot_count_++] = sp;
      }
    }
  }
#ifdef USE_TEXT_SENSOR
  if (this->spots_sensor_ != nullptr)
    this->spots_sensor_->publish_state(format_spots(this->spots_.data(), this->spot_count_));
#endif
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
  if (xTaskCreatePinnedToCoreWithCaps(Isp::task_entry_, "isp", 6144, this, 4, &this->task_, tskNO_AFFINITY,
                                      TASK_STACK_CAPS) != pdPASS)
    this->mark_failed(LOG_STR("Cannot start the ISP task"));
}

void Isp::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ISP:\n"
                "  Scale: %ux%s, format: %s, palette: %s\n"
                "  Range: %.1f .. %.1f\n"
                "  Mirror: x %s, y %s; upscale: %s; scaler: %s\n"
                "  Equalization: %.0f%%, detail: %.2f",
                this->opt_.scale, this->fit_w_ ? " (fitted to the size below)" : "", format_name(this->opt_.format),
                video::PALETTE_NAMES[this->palette_.load()],
                this->opt_.range_min, this->opt_.range_max, YESNO(this->opt_.mirror_x), YESNO(this->opt_.mirror_y),
                this->opt_.upscale == Upscale::BICUBIC    ? "bicubic"
                : this->opt_.upscale == Upscale::BILINEAR ? "bilinear"
                                                          : "nearest",
                this->scaler_ == Scaler::PPA ? "PPA" : "CPU", this->opt_.equalize * 100.0f, this->opt_.detail);
  if (this->fit_w_ != 0)
    ESP_LOGCONFIG(TAG, "  Fit: within %ux%u", this->fit_w_, this->fit_h_);
  static const uint16_t DEGREES[] = {0, 90, 180, 270};
  ESP_LOGCONFIG(TAG, "  Rotation: %u degrees clockwise", DEGREES[static_cast<uint8_t>(this->opt_.rotate) & 3]);
  ESP_LOGCONFIG(TAG, "  Spots: %u of %u", this->spot_count_, MAX_SPOTS);
  for (uint8_t i = 0; i < this->spot_count_; i++)
    ESP_LOGCONFIG(TAG, "    %u: %.1f%%, %.1f%%, %s, colour %u", i + 1, this->spots_[i].x * 100.0f,
                  this->spots_[i].y * 100.0f, MATERIALS[this->spots_[i].material].label, this->spots_[i].color);
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Spots", this->spots_sensor_);
#endif
#ifndef USE_ISP_PPA
  if (this->scaler_ == Scaler::PPA)
    ESP_LOGW(TAG, "  The PPA exists only on the ESP32-P4; scaling on the CPU");
#endif
}

void Isp::set_range_min(float v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.range_min = v;
  this->refresh_ = true;
  this->wake_();
}
void Isp::set_range_max(float v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.range_max = v;
  this->refresh_ = true;
  this->wake_();
}
void Isp::set_mirror_x(bool v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.mirror_x = v;
  this->refresh_ = true;
  this->wake_();
}
void Isp::set_mirror_y(bool v) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.mirror_y = v;
  this->refresh_ = true;
  this->wake_();
}
void Isp::set_rotate(Rotation r) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.rotate = r;
  this->refresh_ = true;
  this->wake_();
}
void Isp::set_upscale(Upscale u) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.upscale = u;
  this->refresh_ = true;
  this->wake_();
}
void Isp::set_scaler(Scaler s) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->scaler_ = s;
  this->refresh_ = true;
  this->wake_();
}
void Isp::set_equalize(float percent) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.equalize = std::min(std::max(percent, 0.0f), 100.0f) / 100.0f;
  this->refresh_ = true;
  this->wake_();
}
void Isp::set_detail(float gain) {
  std::lock_guard<std::mutex> lock(this->mutex_);
  this->opt_.detail = std::max(gain, 0.0f);
  this->refresh_ = true;
  this->wake_();
}

void Isp::wake_() {
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

void Isp::request_refresh() {
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->refresh_ = true;
  }
  this->wake_();
}

void Isp::toggle_spot(float x_percent, float y_percent, const std::string &material) {
  bool changed, full;
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    changed = isp::toggle_spot(this->spots_.data(), this->spot_count_, x_percent / 100.0f, y_percent / 100.0f,
                               this->aspect_, material_index(material));
    full = this->spot_count_ >= MAX_SPOTS;
  }
  if (changed) {
    this->spots_changed_();
  } else if (full) {
    ESP_LOGW(TAG, "Already %u spots; remove one first", MAX_SPOTS);
  }
}

void Isp::set_spot_material(int spot, const std::string &material) {
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (spot < 1 || spot > this->spot_count_)
      return;
    this->spots_[spot - 1].material = material_index(material);
  }
  this->spots_changed_();
}

void Isp::remove_spot(int spot) {
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    if (spot < 1 || spot > this->spot_count_)
      return;
    for (int i = spot - 1; i + 1 < this->spot_count_; i++)
      this->spots_[i] = this->spots_[i + 1];
    this->spot_count_--;
  }
  this->spots_changed_();
}

void Isp::clear_spots() {
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    this->spot_count_ = 0;
  }
  this->spots_changed_();
}

void Isp::spots_changed_() {
  SpotStore st{};
  {
    std::lock_guard<std::mutex> lock(this->mutex_);
    st.count = this->spot_count_;
    for (uint8_t i = 0; i < this->spot_count_; i++)
      st.spots[i] = this->spots_[i];
  }
  if (this->spots_key_ != 0)
    this->spots_pref_.save(&st);
#ifdef USE_TEXT_SENSOR
  if (this->spots_sensor_ != nullptr)
    this->spots_sensor_->publish_state(format_spots(st.spots, st.count));
#endif
  this->request_refresh();
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
    std::array<Spot, MAX_SPOTS> spots;
    uint8_t spot_count;
    bool refresh = false;
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      if (this->has_pending_) {
        f = std::move(this->pending_);
        this->pending_ = video::RadiometricFrame();
        this->has_pending_ = false;
        // Kept (the same buffer, so nothing more is held) to draw again when
        // an option changes before the next frame.
        this->last_ = f;
      } else if (this->refresh_ && this->last_.buffer) {
        f = this->last_;
        refresh = true;
      } else {
        this->refresh_ = false;
        continue;
      }
      this->refresh_ = false;  // a new frame shows the changes too
      o = this->opt_;
      scaler = this->scaler_;
      spots = this->spots_;
      spot_count = this->spot_count_;
      if (f.width > 0 && f.height > 0)
        this->aspect_ = rotation_swaps(this->opt_.rotate) ? static_cast<float>(f.height) / f.width
                                                          : static_cast<float>(f.width) / f.height;
    }
    const uint8_t palette = this->palette_.load();
    if (palette != this->palette_applied_) {
      this->cpu_.set_palette(make_palette(static_cast<Palette>(palette)));
      this->palette_applied_ = palette;
    }
    // Equalization follows the scene over time, not frames; drawn again after
    // a change, a frame takes its curve at once.
    if (refresh) {
      o.curve_blend = 1.0f;
    } else {
      o.curve_blend = this->have_last_ts_ ? curve_blend_for(f.timestamp_ms - this->last_ts_) : 1.0f;
      this->last_ts_ = f.timestamp_ms;
      this->have_last_ts_ = true;
    }
    this->process_(f, o, scaler, spots.data(), spot_count, refresh);
  }
}

void Isp::process_(const video::RadiometricFrame &frame, const Options &opt, Scaler scaler, const Spot *spots,
                   uint8_t spot_count, bool refresh) {
  // A rotated frame is turned (with its mirroring) into a frame of its own
  // first; everything below then works on the picture as shown, so spots and
  // marks land where they are seen. Unrotated frames are mirrored on the way.
  Options o = opt;
  const float *temps = frame.temps();
  uint16_t in_w = frame.width, in_h = frame.height;
  const bool turned = o.rotate != Rotation::R0;
  if (turned) {
    const size_t n = static_cast<size_t>(in_w) * in_h;
    if (!this->oriented_.ensure(n)) {
      if (!this->memory_error_logged_) {
        ESP_LOGE(TAG, "Out of memory to rotate a %ux%u frame; frames are dropped", in_w, in_h);
        this->memory_error_logged_ = true;
      }
      return;
    }
    orient(temps, in_w, in_h, o.mirror_x, o.mirror_y, o.rotate, this->oriented_.data());
    temps = this->oriented_.data();
    if (rotation_swaps(o.rotate))
      std::swap(in_w, in_h);
    o.mirror_x = o.mirror_y = false;
  }
  if (this->fit_w_ != 0)
    o.scale = fit_scale(in_w, in_h, this->fit_w_, this->fit_h_);
  const int s = o.scale < 1 ? 1 : o.scale;
  // Unscaled linear output is tone-mapped straight into the output frame,
  // below; everything else goes through the index image.
  const bool direct = s == 1 && linear_tone(o);
  uint32_t tone_us = 0;
  if (!direct) {
    const uint32_t t0 = micros();
    if (!this->cpu_.tone_map(temps, in_w, in_h, o)) {
      if (!this->memory_error_logged_) {
        ESP_LOGE(TAG, "Out of memory for a %ux%u index buffer; frames are dropped", in_w, in_h);
        this->memory_error_logged_ = true;
      }
      return;
    }
    tone_us = micros() - t0;
  }

  video::ImageFrame out;
  out.width = in_w * s;
  out.height = in_h * s;
  out.format = o.format;
  out.stride = out.width * video::bytes_per_pixel(o.format);
  // Drawn again after a change: now, so encoders see time move on.
  out.timestamp_ms = refresh ? millis() : frame.timestamp_ms;
  out.sequence = frame.sequence;
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

  const uint32_t t1 = micros();
  uint32_t scale_us = 0;
  if (direct) {
    this->cpu_.map_direct(temps, in_w, in_h, o, out.buffer.data(), out.stride);
    tone_us = micros() - t1;
  } else {
    bool done = false;
    if (scaler == Scaler::PPA)
      done = this->render_ppa_(o, out);
    if (!done)
      this->cpu_.render(o, out.buffer.data(), out.stride);
    scale_us = micros() - t1;
  }
  const uint32_t t2 = micros();

  // Marks (min, max, centre) are in the source's pixels: turned like the frame.
  for (uint8_t i = 0; i < frame.annotations.count; i++) {
    video::Annotation a = frame.annotations.items[i];
    if (turned)
      orient_point(a.x, a.y, frame.width, frame.height, opt.mirror_x, opt.mirror_y, opt.rotate, &a.x, &a.y);
    const Point p = map_point(o, in_w, in_h, a.x, a.y);
    a.x = static_cast<float>(p.x);
    a.y = static_cast<float>(p.y);
    out.annotations.add(a);
  }
  // Spots: placed on the image as shown, measured on the input under them
  // and corrected for their material.
  for (uint8_t i = 0; i < spot_count; i++) {
    const Point p = spot_point(out.width, out.height, spots[i].x, spots[i].y);
    const float v = correct_emissivity(spot_value(temps, in_w, in_h, o, spots[i].x, spots[i].y), frame.emissivity,
                                       MATERIALS[spots[i].material].emissivity, frame.reflected);
    out.annotations.add(
        {static_cast<float>(p.x), static_cast<float>(p.y), v, video::AnnotationKind::SPOT, spots[i].color});
  }
  this->sinks_.deliver(out, [](video::ImageSink *sk, const video::ImageFrame &fr) { sk->on_image_frame(fr); });
  const uint32_t t3 = micros();
  this->frames_++;
  this->tone_us_ += tone_us;
  this->scale_us_ += scale_us;
  this->sink_us_ += t3 - t2;
}

// The PPA scales the index image (GRAY8; chip revision 3 or later), so it
// blends values, as the CPU path does: blending two palette colours instead
// gives colours between them that are not on the palette. The palette is then
// looked up on the CPU (ESP-IDF 5.5 has no driver for the PPA's colour
// look-up table). The PPA scales bilinearly only; nearest is left to the CPU,
// which only copies.
bool Isp::render_ppa_(const Options &o, video::ImageFrame &out) {
#ifdef USE_ISP_PPA
  if (this->ppa_ == nullptr || this->ppa_failed_ || o.upscale != Upscale::BILINEAR)
    return false;
  const uint16_t w = this->cpu_.width(), h = this->cpu_.height();
  const size_t align = video::dma_alignment();
  const size_t in_bytes = static_cast<size_t>(w) * h;
  if (this->ppa_in_cap_ < in_bytes) {
    heap_caps_free(this->ppa_in_);
    this->ppa_in_ = static_cast<uint8_t *>(
        heap_caps_aligned_calloc(align, 1, in_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
    if (this->ppa_in_ == nullptr)  // large frames: PSRAM (the PPA driver writes the cache back)
      this->ppa_in_ = static_cast<uint8_t *>(
          heap_caps_aligned_calloc(align, 1, in_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
    this->ppa_in_cap_ = this->ppa_in_ ? in_bytes : 0;
    if (this->ppa_in_ == nullptr)
      return false;
  }
  // A GRAY8 frame is the scaled index image itself; other formats take it
  // from a buffer of its own. The PPA needs the output buffer and its size
  // aligned to the cache line.
  const bool into_frame = o.format == video::ImageFormat::GRAY8 && out.stride == out.width;
  uint8_t *scaled = out.buffer.data();
  size_t scaled_size = this->pool_->buffer_size();
  if (!into_frame) {
    const size_t need = (static_cast<size_t>(out.width) * out.height + align - 1) / align * align;
    if (this->ppa_out_cap_ < need) {
      heap_caps_free(this->ppa_out_);
      this->ppa_out_ =
          static_cast<uint8_t *>(heap_caps_aligned_calloc(align, 1, need, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA));
      if (this->ppa_out_ == nullptr)
        this->ppa_out_ =
            static_cast<uint8_t *>(heap_caps_aligned_calloc(align, 1, need, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA));
      this->ppa_out_cap_ = this->ppa_out_ ? need : 0;
      if (this->ppa_out_ == nullptr)
        return false;
    }
    scaled = this->ppa_out_;
    scaled_size = this->ppa_out_cap_;
  }
  std::memcpy(this->ppa_in_, this->cpu_.indices(), in_bytes);

  ppa_srm_oper_config_t c{};
  c.in.buffer = this->ppa_in_;
  c.in.pic_w = w;
  c.in.pic_h = h;
  c.in.block_w = w;
  c.in.block_h = h;
  c.in.srm_cm = PPA_SRM_COLOR_MODE_GRAY8;
  c.out.buffer = scaled;
  c.out.buffer_size = scaled_size;
  c.out.pic_w = out.width;
  c.out.pic_h = out.height;
  c.out.srm_cm = PPA_SRM_COLOR_MODE_GRAY8;
  c.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
  c.scale_x = o.scale;
  c.scale_y = o.scale;
  c.mode = PPA_TRANS_MODE_BLOCKING;
  const esp_err_t err = ppa_do_scale_rotate_mirror(this->ppa_, &c);
  if (err != ESP_OK) {
    // An older chip revision cannot scale GRAY8; nothing changes that while running.
    this->ppa_failed_ = true;
    ESP_LOGW(TAG, "PPA scaling failed (%s); scaling on the CPU from now on", esp_err_to_name(err));
    return false;
  }
  if (!into_frame)
    this->cpu_.lookup(scaled, out.width, out.height, o.format, out.buffer.data(), out.stride);
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
