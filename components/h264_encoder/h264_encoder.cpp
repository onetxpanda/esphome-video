#include "h264_encoder.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <esp_heap_caps.h>
#include <freertos/idf_additions.h>

// The task's stack is in PSRAM: internal RAM is needed by the network stack,
// USB and (with Matter) the WebRTC stack.
static constexpr UBaseType_t TASK_STACK_CAPS = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

#include <cmath>
#include <cstring>

#include <esp_cache.h>

// esp_h264_alloc.h (esp_h264 1.5.0) has no extern "C" guard of its own.
extern "C" {
#include "esp_h264_alloc.h"
}
#include "esp_h264_enc_param_hw.h"

namespace esphome {
namespace h264_encoder {

static const char *const TAG = "h264_encoder";

// Hardware limits (esp_h264_types.h).
static constexpr uint16_t MIN_DIM = 80;
static constexpr uint16_t MAX_WIDTH = 1920;
static constexpr uint16_t MAX_HEIGHT = 2032;
// Conversion buffers up to this size use internal DMA RAM; larger ones PSRAM.
static constexpr uint32_t INTERNAL_BUFFER_LIMIT = 96 * 1024;
// With a measured frame rate, reopen the encoder when the rate has been off by
// more than this fraction for DRIFT_MS (a short hiccup, such as the source
// switching modes, does not count), at most once per REOPEN_MIN_MS.
static constexpr float FPS_TOLERANCE = 0.25f;
static constexpr uint32_t DRIFT_MS = 3000;
static constexpr uint32_t REOPEN_MIN_MS = 5000;
static constexpr uint8_t DEFAULT_FPS = 10;  // before a rate has been measured

using video::ImageFormat;

static const char *format_name(ImageFormat f) {
  switch (f) {
    case ImageFormat::RGB565_BE:
      return "RGB565 (big-endian)";
    case ImageFormat::RGB565_LE:
      return "RGB565 (little-endian)";
    case ImageFormat::RGB888:
      return "RGB888";
    case ImageFormat::GRAY8:
      return "8-bit grey";
  }
  return "unknown";
}

static uint32_t align16(uint32_t v) { return (v + 15u) & ~15u; }

static void *alloc_frame_buffer(uint32_t size, uint32_t *actual) {
  void *p = nullptr;
  if (size <= INTERNAL_BUFFER_LIMIT)
    p = esp_h264_aligned_calloc(16, 1, size, actual, ESP_H264_MEM_INTERNAL_DMA);
  if (p == nullptr)
    p = esp_h264_aligned_calloc(16, 1, size, actual, ESP_H264_MEM_SPIRAM);
  if (p == nullptr && size > INTERNAL_BUFFER_LIMIT)
    p = esp_h264_aligned_calloc(16, 1, size, actual, ESP_H264_MEM_INTERNAL_DMA);
  return p;
}

void H264Encoder::setup() {
  if (this->qp_min_ > this->qp_max_)
    this->qp_min_ = this->qp_max_;
  // The task also runs the sinks (e.g. RTSP packetization), hence 6 KB.
  if (xTaskCreatePinnedToCoreWithCaps(H264Encoder::task_entry_, "h264_enc", 6144, this, 5, &this->task_,
                                      tskNO_AFFINITY, TASK_STACK_CAPS) != pdPASS)
    this->mark_failed(LOG_STR("Could not start the encoder task"));
}

void H264Encoder::dump_config() {
  ESP_LOGCONFIG(TAG, "H.264 encoder (ESP32-P4 hardware):");
  if (this->fps_ != 0) {
    ESP_LOGCONFIG(TAG, "  Frame rate: %u fps", this->fps_);
  } else {
    ESP_LOGCONFIG(TAG, "  Frame rate: measured from the source");
  }
  if (this->gop_ != 0) {
    ESP_LOGCONFIG(TAG, "  Keyframe every %u frames", this->gop_);
  } else {
    ESP_LOGCONFIG(TAG, "  Keyframe every second");
  }
  if (this->always_on_)
    ESP_LOGCONFIG(TAG, "  Always on: encodes with no viewer, so a new one gets a picture at once");
  if (this->repeat_.enabled()) {
    ESP_LOGCONFIG(TAG, "  Minimum frame rate: %u fps (a slower source's last frame is repeated)",
                  this->repeat_.min_fps);
  }
  ESP_LOGCONFIG(TAG, "  QP range: %u-%u", this->qp_min_, this->qp_max_);
  if (this->bitrate_ != 0) {
    ESP_LOGCONFIG(TAG, "  Bitrate: %u bit/s", static_cast<unsigned>(this->bitrate_));
  } else {
    ESP_LOGCONFIG(TAG, "  Bitrate: automatic (from frame size and rate)");
  }
  for (size_t i = 0; i < this->source_names_.size(); i++)
    ESP_LOGCONFIG(TAG, "  Source: %s%s", this->source_names_[i], i == this->active_source_ ? " (active)" : "");
  if (this->sinks_.empty())
    ESP_LOGW(TAG, "  No consumer is attached (e.g. rtsp_server); nothing will be encoded");
}

void H264Encoder::loop() {
  // Periodic statistics, so a field problem shows up in the logs.
  const uint32_t now = millis();
  if (now - this->last_stats_ms_ < 60000)
    return;
  this->last_stats_ms_ = now;
  const uint32_t n = this->encoded_frames_.load();
  if (n > 0 || this->dropped_frames_.load() > 0) {
    const uint32_t iv = this->interval_x16_.load();
    ESP_LOGD(TAG,
             "Encoded %u frames (%u keyframes, %u repeats, %u without a copy, %u bytes total), dropped %u, "
             "errors %u; source %.2f fps",
             static_cast<unsigned>(n), static_cast<unsigned>(this->keyframes_.load()),
             static_cast<unsigned>(this->repeats_.load()), static_cast<unsigned>(this->zero_copy_.load()),
             static_cast<unsigned>(this->encoded_bytes_.load()),
             static_cast<unsigned>(this->dropped_frames_.load()), static_cast<unsigned>(this->encode_errors_.load()),
             iv ? 16000.0f / iv : 0.0f);
  }
}

void SourceInput::on_image_frame(const video::ImageFrame &frame) { this->encoder_->handle_frame(this->index_, frame); }
bool SourceInput::wants_frames() const { return this->encoder_->source_wanted(this->index_); }
void SourceInput::on_source_change() { this->encoder_->source_changed(this->index_); }

bool H264Encoder::consumers_want_frames_() const {
  if (this->is_failed())
    return false;
  if (this->always_on_)
    return true;
  for (auto *s : this->sinks_) {
    if (s->wants_frames())
      return true;
  }
  return false;
}

void H264Encoder::set_active_source(uint8_t index) {
  if (index >= this->source_names_.size() || index == this->active_source_)
    return;
  ESP_LOGI(TAG, "Source: %s -> %s", this->source_names_[this->active_source_], this->source_names_[index]);
  this->active_source_ = index;
  this->source_changed(index);
}

void H264Encoder::source_changed(uint8_t index) {
  if (index != this->active_source_)
    return;
  // Start the new picture cleanly, measure its rate afresh, and report its
  // problems afresh. The old picture is not repeated.
  this->repeatable_ = false;
  this->keyframe_requested_ = true;
  this->interval_x16_ = 0;
  this->format_error_logged_ = false;
}

uint8_t H264Encoder::target_fps_() const {
  if (this->fps_ != 0)
    return this->fps_;
  const uint32_t iv = this->interval_x16_.load();
  if (iv == 0)
    return DEFAULT_FPS;
  const long fps = std::lround(16000.0f / iv);
  // With repeats the stream runs at min_fps or more, whatever the source does.
  return this->repeat_.declared_fps(static_cast<uint8_t>(fps < 1 ? 1 : (fps > 60 ? 60 : fps)));
}

void H264Encoder::note_interval_(uint32_t ts) {
  // Mean interval between the active source's frames (exponential, 1/8 per
  // frame). Gaps longer than a repeated frame may stand in for (a pause)
  // restart the measurement.
  if (this->have_last_ts_) {
    const uint32_t d = ts - this->last_ts_;
    if (d > 0 && d <= RepeatPolicy::MAX_REPEAT_MS) {
      const uint32_t x = d * 16, iv = this->interval_x16_.load();
      this->interval_x16_ = iv == 0 ? x : iv + (static_cast<int32_t>(x - iv) / 8);
    }
  }
  this->last_ts_ = ts;
  this->have_last_ts_ = true;
}

uint32_t H264Encoder::raw_size_(uint16_t width, uint16_t height, esp_h264_raw_format_t type) {
  return static_cast<uint32_t>(static_cast<float>(align16(width)) * align16(height) *
                                   ESP_H264_GET_BPP_BY_PIC_TYPE(type) +
                               0.5f);
}

// Copies the frame into dst in the encoder's layout, padding to whole 16x16
// macroblocks by repeating the last column and row.
void H264Encoder::convert_(const video::ImageFrame &f, uint8_t *dst) {
  const uint16_t w = f.width, h = f.height, aw = align16(w), ah = align16(h);
  const uint32_t bpp = (f.format == ImageFormat::RGB888) ? 3 : 2;
  const uint32_t dst_stride = static_cast<uint32_t>(aw) * bpp;
  for (uint16_t y = 0; y < h; y++) {
    uint8_t *d = dst + y * dst_stride;
    const uint8_t *s = f.buffer.data() + static_cast<size_t>(y) * f.stride;
    switch (f.format) {
      case ImageFormat::RGB565_LE:
        std::memcpy(d, s, static_cast<size_t>(w) * 2);
        break;
      case ImageFormat::RGB565_BE:
        for (uint16_t x = 0; x < w; x++) {
          d[2 * x] = s[2 * x + 1];
          d[2 * x + 1] = s[2 * x];
        }
        break;
      case ImageFormat::GRAY8:
        for (uint16_t x = 0; x < w; x++) {
          const uint8_t v = s[x];
          const uint16_t px = static_cast<uint16_t>(((v >> 3) << 11) | ((v >> 2) << 5) | (v >> 3));
          d[2 * x] = static_cast<uint8_t>(px & 0xFF);
          d[2 * x + 1] = static_cast<uint8_t>(px >> 8);
        }
        break;
      case ImageFormat::RGB888:
        // R G B -> the encoder's B G R.
        for (uint16_t x = 0; x < w; x++) {
          d[3 * x] = s[3 * x + 2];
          d[3 * x + 1] = s[3 * x + 1];
          d[3 * x + 2] = s[3 * x];
        }
        break;
    }
    for (uint16_t x = w; x < aw; x++)
      std::memcpy(d + x * bpp, d + (w - 1) * bpp, bpp);
  }
  for (uint16_t y = h; y < ah; y++)
    std::memcpy(dst + y * dst_stride, dst + (h - 1) * dst_stride, dst_stride);
}

void H264Encoder::handle_frame(uint8_t source, const video::ImageFrame &f) {
  if (this->is_failed() || this->task_ == nullptr || source != this->active_source_)
    return;
  if (!this->consumers_want_frames_()) {
    this->was_active_ = false;
    this->have_last_ts_ = false;
    this->repeatable_ = false;
    return;
  }
  if (!this->was_active_.exchange(true))
    this->keyframe_requested_ = true;  // resuming after idle: start with a keyframe
  this->note_interval_(f.timestamp_ms);

  const esp_h264_raw_format_t type =
      f.format == ImageFormat::RGB888 ? ESP_H264_RAW_FMT_BGR888 : ESP_H264_RAW_FMT_RGB565_LE;
  const char *problem = nullptr;
  if (f.width < MIN_DIM || f.height < MIN_DIM) {
    problem = "smaller than the hardware minimum of 80x80";
  } else if (f.width > MAX_WIDTH || f.height > MAX_HEIGHT) {
    problem = "larger than the hardware maximum of 1920x2032";
  } else if (!f.buffer || f.stride < f.width * video::bytes_per_pixel(f.format)) {
    problem = "missing its pixels";
  } else if (!ESP_H264_HW_IS_SUPPORTED_PIC_TYPE(type)) {
    problem = "not supported by this chip revision (RGB input needs ESP32-P4 rev 3)";
  }
  if (problem != nullptr) {
    if (!this->format_error_logged_.exchange(true))
      ESP_LOGE(TAG, "Cannot encode %ux%u %s frame: %s", f.width, f.height, format_name(f.format), problem);
    this->encode_errors_++;
    return;
  }

  // A slot the encoder is not using; only this task touches it until
  // finish_write().
  const int index = this->exchange_.begin_write();
  if (index == SlotExchange::NONE) {
    this->dropped_frames_++;
    return;
  }
  Slot &slot = this->slots_[index];
  slot.held.reset();
  // A slow source's frame is copied even when it could be encoded in place:
  // the source reuses its buffer, and the copy is what gets repeated.
  const bool direct = f.format == ImageFormat::RGB565_LE && (f.width & 15) == 0 && (f.height & 15) == 0 &&
                      f.stride == static_cast<uint32_t>(f.width) * 2 &&
                      (reinterpret_cast<uintptr_t>(f.buffer.data()) & 15) == 0 &&
                      !this->repeat_.source_slow(this->interval_x16_.load());
  if (direct) {
    // The hardware reads the source's buffer: write the CPU's view of it back
    // to memory, and hold the buffer until the encode is done.
    esp_cache_msync(f.buffer.data(), f.length(), ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    slot.held = f.buffer;
    slot.data = f.buffer.data();
    slot.len = f.length();
    slot.copied = false;
    this->zero_copy_++;
  } else {
    const uint32_t need = raw_size_(f.width, f.height, type);
    if (slot.cap < need) {
      if (slot.buf != nullptr)
        esp_h264_free(slot.buf);
      slot.cap = 0;
      slot.buf = static_cast<uint8_t *>(alloc_frame_buffer(need, &slot.cap));
      if (slot.buf == nullptr) {
        ESP_LOGE(TAG, "Out of memory for a %ux%u frame buffer (%u bytes)", f.width, f.height,
                 static_cast<unsigned>(need));
        slot.cap = 0;
        this->encode_errors_++;
        this->exchange_.abandon_write();
        return;
      }
    }
    convert_(f, slot.buf);
    slot.data = slot.buf;
    slot.len = need;
    slot.copied = true;
  }
  slot.width = f.width;
  slot.height = f.height;
  slot.type = type;
  slot.timestamp_ms = f.timestamp_ms;
  this->last_source_ms_ = millis();
  this->repeatable_ = true;
  const int replaced = this->exchange_.finish_write(index);
  if (replaced != SlotExchange::NONE) {
    // The encoder had not taken the previous frame: drop it, and give back
    // the source buffer it may hold.
    this->slots_[replaced].held.reset();
    this->exchange_.release_done();
    this->dropped_frames_++;
  }
  xTaskNotifyGive(this->task_);
}

// Encoder task: whether its slot may be encoded again now or later.
bool H264Encoder::repeat_ready_() const {
  return this->repeat_.enabled() && this->front_ != SlotExchange::NONE && this->slots_[this->front_].copied &&
         this->repeatable_.load() && this->enc_ != nullptr &&
         this->repeat_.may_repeat(millis(), this->last_source_ms_.load()) && this->consumers_want_frames_();
}

// Encoder task: an output buffer for the slot's size (an encoded frame is
// far smaller than the raw one, so the raw size is a safe bound).
bool H264Encoder::prepare_output_(const Slot &slot) {
  const uint32_t need = raw_size_(slot.width, slot.height, slot.type);
  if (this->out_cap_ >= need)
    return true;
  if (this->out_buf_ != nullptr)
    esp_h264_free(this->out_buf_);
  this->out_cap_ = 0;
  this->out_buf_ = static_cast<uint8_t *>(alloc_frame_buffer(need, &this->out_cap_));
  if (this->out_buf_ == nullptr) {
    ESP_LOGE(TAG, "Out of memory for the %ux%u output buffer (%u bytes)", slot.width, slot.height,
             static_cast<unsigned>(need));
    this->out_cap_ = 0;
    return false;
  }
  return true;
}

bool H264Encoder::open_encoder_(const Slot &slot) {
  const uint8_t fps = this->target_fps_();
  esp_h264_enc_cfg_hw_t cfg{};
  cfg.pic_type = slot.type;
  cfg.gop = this->gop_ != 0 ? this->gop_ : fps;
  cfg.fps = fps;
  cfg.res.width = slot.width;
  cfg.res.height = slot.height;
  // Automatic: about 1/8 bit per pixel per frame, at least 64 kbit/s.
  uint32_t bitrate = this->bitrate_;
  if (bitrate == 0) {
    bitrate = static_cast<uint32_t>(slot.width) * slot.height * fps / 8;
    if (bitrate < 64000)
      bitrate = 64000;
  }
  cfg.rc.bitrate = bitrate;
  cfg.rc.qp_min = this->qp_min_;
  cfg.rc.qp_max = this->qp_max_;
  esp_h264_err_t err = esp_h264_enc_hw_new(&cfg, &this->enc_);
  if (err == ESP_H264_ERR_OK)
    err = esp_h264_enc_open(this->enc_);
  if (err == ESP_H264_ERR_OK)
    err = esp_h264_enc_hw_get_param_hd(this->enc_, &this->param_);
  if (err != ESP_H264_ERR_OK) {
    ESP_LOGE(TAG, "Hardware encoder setup failed (%d) for %ux%u", static_cast<int>(err), slot.width, slot.height);
    this->close_encoder_();
    return false;
  }
  this->enc_w_ = slot.width;
  this->enc_h_ = slot.height;
  this->enc_type_ = slot.type;
  this->enc_fps_ = fps;
  this->enc_opened_ms_ = millis();
  this->drifting_ = false;
  ESP_LOGI(TAG, "Encoding %ux%u at %u fps%s, keyframe every %u frames, %u bit/s", slot.width, slot.height, fps,
           this->fps_ ? "" : " (measured)", static_cast<unsigned>(cfg.gop), static_cast<unsigned>(bitrate));
  return true;
}

void H264Encoder::close_encoder_() {
  if (this->enc_ != nullptr) {
    esp_h264_enc_close(this->enc_);
    esp_h264_enc_del(this->enc_);
  }
  this->enc_ = nullptr;
  this->param_ = nullptr;
}

void H264Encoder::task_entry_(void *arg) { static_cast<H264Encoder *>(arg)->task_loop_(); }

void H264Encoder::task_loop_() {
  for (;;) {
    // Sleep until a source frame is ready or a repeat falls due. Nothing here
    // waits for the source: a newer frame, if any, is simply taken.
    TickType_t wait = portMAX_DELAY;
    if (this->repeat_ready_()) {
      wait = pdMS_TO_TICKS(this->repeat_.wait_ms(millis(), this->last_out_ms_));
      if (wait == 0)
        wait = 1;  // never spin: let lower-priority tasks run between frames
    }
    ulTaskNotifyTake(pdTRUE, wait);

    bool fresh = false;
    this->front_ = this->exchange_.take(&fresh);
    if (this->front_ == SlotExchange::NONE)
      continue;
    Slot &slot = this->slots_[this->front_];
    bool repeat = false;
    uint32_t ts;
    if (fresh) {
      ts = slot.timestamp_ms;
    } else {
      // No new frame: encode the last one again if a repeat is due.
      if (!this->repeat_ready_() || this->repeat_.wait_ms(millis(), this->last_out_ms_) != 0)
        continue;
      repeat = true;
      ts = this->last_out_ts_ + (millis() - this->last_out_ms_);
    }

    if (this->enc_ != nullptr &&
        (this->enc_w_ != slot.width || this->enc_h_ != slot.height || this->enc_type_ != slot.type))
      this->close_encoder_();
    if (this->enc_ != nullptr && this->fps_ == 0) {
      // Measured rate drifted from the one the stream declares, and stayed
      // off: restart the stream description (a new IDR with new timing).
      const float measured = this->target_fps_();
      const uint32_t now = millis();
      if (std::fabs(measured - this->enc_fps_) <= FPS_TOLERANCE * this->enc_fps_) {
        this->drifting_ = false;
      } else if (!this->drifting_) {
        this->drifting_ = true;
        this->drift_since_ms_ = now;
      } else if (now - this->drift_since_ms_ >= DRIFT_MS && now - this->enc_opened_ms_ >= REOPEN_MIN_MS) {
        this->close_encoder_();
      }
    }
    if (!this->prepare_output_(slot)) {
      this->encode_errors_++;
      this->repeatable_ = false;
      slot.held.reset();
      continue;
    }
    if (this->enc_ == nullptr) {
      if (!this->open_encoder_(slot)) {
        this->repeatable_ = false;
        this->encode_errors_++;
        slot.held.reset();
        continue;
      }
      this->keyframe_requested_ = false;  // a new encoder starts with an IDR anyway
    }
    if (this->keyframe_requested_.exchange(false) && this->param_ != nullptr)
      esp_h264_enc_force_idr(&this->param_->base);
    ts = next_timestamp(ts, this->last_out_ts_, this->have_out_ts_);

    esp_h264_enc_in_frame_t in{};
    in.raw_data.buffer = const_cast<uint8_t *>(slot.data);
    in.raw_data.len = slot.len;
    in.pts = ts;
    esp_h264_enc_out_frame_t out{};
    out.raw_data.buffer = this->out_buf_;
    out.raw_data.len = this->out_cap_;

    const esp_h264_err_t err = esp_h264_enc_process(this->enc_, &in, &out);
    if (!slot.copied) {
      // A held source buffer goes back to the source; it cannot be repeated.
      slot.held.reset();
      slot.data = nullptr;
    }
    if (err != ESP_H264_ERR_OK || out.length == 0) {
      // No repeats of a frame that failed: they would come back at once.
      this->repeatable_ = false;
      this->encode_errors_++;
      if (this->encode_errors_.load() <= 3)
        ESP_LOGW(TAG, "Encode failed (%d)", static_cast<int>(err));
      continue;
    }
    const bool keyframe = out.frame_type == ESP_H264_FRAME_TYPE_IDR;
    this->last_out_ms_ = millis();
    this->last_out_ts_ = ts;
    this->have_out_ts_ = true;
    if (repeat)
      this->repeats_++;
    this->encoded_frames_++;
    this->encoded_bytes_ += out.length;
    if (keyframe)
      this->keyframes_++;
    for (auto *s : this->sinks_)
      s->on_encoded_frame(out.raw_data.buffer, out.length, ts, keyframe);
  }
}

}  // namespace h264_encoder
}  // namespace esphome
