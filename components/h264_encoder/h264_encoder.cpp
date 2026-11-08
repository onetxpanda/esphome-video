#include "h264_encoder.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

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
// With a measured frame rate, reopen the encoder when the rate drifts by more
// than this fraction, at most once per REOPEN_MIN_MS.
static constexpr float FPS_TOLERANCE = 0.25f;
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
  if (xTaskCreate(H264Encoder::task_entry_, "h264_enc", 6144, this, 5, &this->task_) != pdPASS)
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
             "Encoded %u frames (%u keyframes, %u without a copy, %u bytes total), dropped %u, errors %u; "
             "source %.2f fps",
             static_cast<unsigned>(n), static_cast<unsigned>(this->keyframes_.load()),
             static_cast<unsigned>(this->zero_copy_.load()), static_cast<unsigned>(this->encoded_bytes_.load()),
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
  // problems afresh.
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
  return static_cast<uint8_t>(fps < 1 ? 1 : (fps > 60 ? 60 : fps));
}

void H264Encoder::note_interval_(uint32_t ts) {
  // Mean interval between the active source's frames (exponential, 1/8 per
  // frame). Gaps over 2 s (a pause) restart the measurement.
  if (this->have_last_ts_) {
    const uint32_t d = ts - this->last_ts_;
    if (d > 0 && d < 2000) {
      const uint32_t x = d * 16, iv = this->interval_x16_.load();
      this->interval_x16_ = iv == 0 ? x : iv + (static_cast<int32_t>(x - iv) / 8);
    }
  }
  this->last_ts_ = ts;
  this->have_last_ts_ = true;
}

void H264Encoder::free_buffers_() {
  if (this->in_buf_ != nullptr)
    esp_h264_free(this->in_buf_);
  if (this->out_buf_ != nullptr)
    esp_h264_free(this->out_buf_);
  this->in_buf_ = this->out_buf_ = nullptr;
  this->in_cap_ = this->out_cap_ = 0;
}

bool H264Encoder::prepare_buffers_(uint16_t width, uint16_t height, esp_h264_raw_format_t type, bool convert) {
  const uint16_t aw = align16(width), ah = align16(height);
  const uint32_t need =
      static_cast<uint32_t>(static_cast<float>(aw) * ah * ESP_H264_GET_BPP_BY_PIC_TYPE(type) + 0.5f);
  if (this->width_ != width || this->height_ != height || this->type_ != type || this->out_cap_ < need)
    this->free_buffers_();
  // An encoded frame is far smaller than the raw one; the raw size is a safe bound.
  if (this->out_buf_ == nullptr)
    this->out_buf_ = static_cast<uint8_t *>(alloc_frame_buffer(need, &this->out_cap_));
  // The conversion buffer only exists while frames need converting.
  if (convert && this->in_buf_ == nullptr)
    this->in_buf_ = static_cast<uint8_t *>(alloc_frame_buffer(need, &this->in_cap_));
  if (this->out_buf_ == nullptr || (convert && this->in_buf_ == nullptr)) {
    ESP_LOGE(TAG, "Out of memory for %ux%u frame buffers (%u bytes each)", width, height,
             static_cast<unsigned>(need));
    this->free_buffers_();
    return false;
  }
  this->width_ = width;
  this->height_ = height;
  this->aligned_w_ = aw;
  this->aligned_h_ = ah;
  this->type_ = type;
  this->in_len_ = need;
  return true;
}

// Copies the frame into the conversion buffer in the encoder's layout,
// padding to whole 16x16 macroblocks by repeating the last column and row.
bool H264Encoder::convert_(const video::ImageFrame &f) {
  const uint16_t w = f.width, h = f.height, aw = this->aligned_w_, ah = this->aligned_h_;
  uint8_t *dst = this->in_buf_;
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
  return true;
}

void H264Encoder::handle_frame(uint8_t source, const video::ImageFrame &f) {
  if (this->is_failed() || this->task_ == nullptr || source != this->active_source_)
    return;
  if (!this->consumers_want_frames_()) {
    this->was_active_ = false;
    this->have_last_ts_ = false;
    return;
  }
  if (!this->was_active_.exchange(true))
    this->keyframe_requested_ = true;  // resuming after idle: start with a keyframe
  this->note_interval_(f.timestamp_ms);

  bool expected = false;
  if (!this->busy_.compare_exchange_strong(expected, true)) {
    this->dropped_frames_++;
    return;
  }
  // From here the encoder task is idle and the hand-over fields are ours.

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
    this->busy_ = false;
    return;
  }
  const bool direct = f.format == ImageFormat::RGB565_LE && (f.width & 15) == 0 && (f.height & 15) == 0 &&
                      f.stride == static_cast<uint32_t>(f.width) * 2 &&
                      (reinterpret_cast<uintptr_t>(f.buffer.data()) & 15) == 0;
  if (!this->prepare_buffers_(f.width, f.height, type, !direct)) {
    this->encode_errors_++;
    this->busy_ = false;
    return;
  }
  if (direct) {
    // The hardware reads the source's buffer: write the CPU's view of it back
    // to memory, and hold the buffer until the encode is done.
    esp_cache_msync(f.buffer.data(), f.length(), ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
    this->held_ = f.buffer;
    this->in_ptr_ = f.buffer.data();
    this->in_len_ = f.length();
    this->zero_copy_++;
  } else {
    this->convert_(f);
    this->in_ptr_ = this->in_buf_;
  }
  this->frame_ts_ = f.timestamp_ms;
  xTaskNotifyGive(this->task_);
}

bool H264Encoder::open_encoder_() {
  const uint8_t fps = this->target_fps_();
  esp_h264_enc_cfg_hw_t cfg{};
  cfg.pic_type = this->type_;
  cfg.gop = this->gop_ != 0 ? this->gop_ : fps;
  cfg.fps = fps;
  cfg.res.width = this->width_;
  cfg.res.height = this->height_;
  // Automatic: about 1/8 bit per pixel per frame, at least 64 kbit/s.
  uint32_t bitrate = this->bitrate_;
  if (bitrate == 0) {
    bitrate = static_cast<uint32_t>(this->width_) * this->height_ * fps / 8;
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
    ESP_LOGE(TAG, "Hardware encoder setup failed (%d) for %ux%u", static_cast<int>(err), this->width_,
             this->height_);
    this->close_encoder_();
    return false;
  }
  this->enc_w_ = this->width_;
  this->enc_h_ = this->height_;
  this->enc_type_ = this->type_;
  this->enc_fps_ = fps;
  this->enc_opened_ms_ = millis();
  ESP_LOGI(TAG, "Encoding %ux%u at %u fps%s, keyframe every %u frames, %u bit/s", this->width_, this->height_, fps,
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
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (!this->busy_.load())
      continue;

    if (this->enc_ != nullptr &&
        (this->enc_w_ != this->width_ || this->enc_h_ != this->height_ || this->enc_type_ != this->type_))
      this->close_encoder_();
    if (this->enc_ != nullptr && this->fps_ == 0 && millis() - this->enc_opened_ms_ >= REOPEN_MIN_MS) {
      // Measured rate drifted from the one the stream declares: restart the
      // stream description (a new IDR with new timing).
      const float measured = this->target_fps_();
      if (std::fabs(measured - this->enc_fps_) > FPS_TOLERANCE * this->enc_fps_)
        this->close_encoder_();
    }
    if (this->enc_ == nullptr) {
      if (!this->open_encoder_()) {
        this->encode_errors_++;
        this->held_.reset();
        this->busy_ = false;
        continue;
      }
      this->keyframe_requested_ = false;  // a new encoder starts with an IDR anyway
    }
    if (this->keyframe_requested_.exchange(false) && this->param_ != nullptr)
      esp_h264_enc_force_idr(&this->param_->base);

    esp_h264_enc_in_frame_t in{};
    in.raw_data.buffer = const_cast<uint8_t *>(this->in_ptr_);
    in.raw_data.len = this->in_len_;
    in.pts = this->frame_ts_;
    esp_h264_enc_out_frame_t out{};
    out.raw_data.buffer = this->out_buf_;
    out.raw_data.len = this->out_cap_;

    const esp_h264_err_t err = esp_h264_enc_process(this->enc_, &in, &out);
    this->held_.reset();  // the source may reuse its buffer now
    if (err != ESP_H264_ERR_OK || out.length == 0) {
      this->encode_errors_++;
      if (this->encode_errors_.load() <= 3)
        ESP_LOGW(TAG, "Encode failed (%d)", static_cast<int>(err));
      this->busy_ = false;
      continue;
    }
    const bool keyframe = out.frame_type == ESP_H264_FRAME_TYPE_IDR;
    this->encoded_frames_++;
    this->encoded_bytes_ += out.length;
    if (keyframe)
      this->keyframes_++;
    for (auto *s : this->sinks_)
      s->on_encoded_frame(out.raw_data.buffer, out.length, this->frame_ts_, keyframe);
    this->busy_ = false;
  }
}

}  // namespace h264_encoder
}  // namespace esphome
