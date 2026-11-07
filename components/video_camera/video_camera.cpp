#include "video_camera.h"

#include "esphome/core/log.h"

namespace esphome {
namespace video_camera {

static const char *const TAG = "video_camera";

void VideoCamera::setup() {
  jpeg_encode_engine_cfg_t cfg{};
  cfg.timeout_ms = 200;
  if (jpeg_new_encoder_engine(&cfg, &this->encoder_) != ESP_OK)
    this->mark_failed(LOG_STR("Cannot start the hardware JPEG encoder"));
}

void VideoCamera::dump_config() {
  ESP_LOGCONFIG(TAG, "Video camera (hardware JPEG):\n  Quality: %u", this->quality_);
}

void VideoCamera::start_stream(camera::CameraRequester r) {
  for (auto *l : this->listeners_)
    l->on_stream_start();
  this->stream_ |= 1u << r;
}

void VideoCamera::stop_stream(camera::CameraRequester r) {
  for (auto *l : this->listeners_)
    l->on_stream_stop();
  this->stream_ &= static_cast<uint8_t>(~(1u << r));
}

bool VideoCamera::wants_frames() const {
  return !this->is_failed() && (this->single_ | this->stream_) != 0 && this->state_ == IDLE;
}

bool VideoCamera::ensure_output_(size_t raw_size) {
  // A JPEG is far smaller than the raw image; the raw size plus the header
  // is a safe bound.
  const size_t need = raw_size + 1024;
  if (this->out_cap_ >= need)
    return true;
  if (this->out_ != nullptr)
    free(this->out_);
  jpeg_encode_memory_alloc_cfg_t mem{};
  mem.buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER;
  size_t got = 0;
  this->out_ = static_cast<uint8_t *>(jpeg_alloc_encoder_mem(need, &mem, &got));
  this->out_cap_ = this->out_ ? got : 0;
  return this->out_ != nullptr;
}

void VideoCamera::on_image_frame(const video::ImageFrame &f) {
  const uint8_t requesters = this->single_ | this->stream_;
  if (requesters == 0)
    return;
  uint8_t expected = IDLE;
  if (!this->state_.compare_exchange_strong(expected, ENCODING))
    return;  // the previous image is still being sent

  jpeg_encode_cfg_t cfg{};
  cfg.width = f.width;
  cfg.height = f.height;
  cfg.image_quality = this->quality_;
  cfg.sub_sample = this->subsampling_;
  switch (f.format) {
    case video::ImageFormat::RGB565_LE:
      cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB565;
      break;
    case video::ImageFormat::RGB888:
      cfg.src_type = JPEG_ENCODE_IN_FORMAT_RGB888;
      break;
    case video::ImageFormat::GRAY8:
      cfg.src_type = JPEG_ENCODE_IN_FORMAT_GRAY;
      cfg.sub_sample = JPEG_DOWN_SAMPLING_GRAY;
      break;
    default:
      if (!this->format_logged_.exchange(true))
        ESP_LOGE(TAG, "The hardware JPEG encoder takes RGB565 little-endian, RGB888 or grey; set the source's format");
      this->state_ = IDLE;
      return;
  }
  // Packed rows only (the encoder has no stride setting).
  if (f.stride != f.width * video::bytes_per_pixel(f.format) || !this->ensure_output_(f.length())) {
    this->errors_++;
    this->state_ = IDLE;
    return;
  }
  uint32_t len = 0;
  const esp_err_t err =
      jpeg_encoder_process(this->encoder_, &cfg, f.buffer.data(), f.length(), this->out_, this->out_cap_, &len);
  if (err != ESP_OK || len == 0) {
    this->errors_++;
    this->state_ = IDLE;
    return;
  }
  this->out_len_ = len;
  this->out_requesters_ = requesters;
  this->state_ = ENCODED;
}

void VideoCamera::loop() {
  const uint32_t errors = this->errors_.load();
  if (errors != this->last_errors_) {
    ESP_LOGW(TAG, "%u JPEG encode errors", static_cast<unsigned>(errors - this->last_errors_));
    this->last_errors_ = errors;
  }
  if (this->state_ == PUBLISHED && this->current_ && this->current_.use_count() == 1) {
    this->current_.reset();
    this->state_ = IDLE;
  }
  if (this->state_ == ENCODED) {
    // Single requests are answered once; streams keep asking.
    this->single_ &= static_cast<uint8_t>(~this->out_requesters_);
    this->current_ = std::make_shared<JpegImage>(this->out_, this->out_len_, this->out_requesters_);
    this->state_ = PUBLISHED;
    for (auto *l : this->listeners_)
      l->on_camera_image(this->current_);
  }
}

}  // namespace video_camera
}  // namespace esphome
