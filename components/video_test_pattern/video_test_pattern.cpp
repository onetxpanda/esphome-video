#include "video_test_pattern.h"

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace video_test_pattern {

static const char *const TAG = "video_test_pattern";

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

void TestPattern::setup() {
  const size_t bytes = static_cast<size_t>(this->width_) * this->height_ * 2;
#ifdef USE_ESP32
  this->pool_ = std::make_unique<video::FramePool>(2, bytes, video::dma_alignment(), video::dma_allocator());
#else
  this->pool_ = std::make_unique<video::FramePool>(2, bytes);
#endif
  if (!this->pool_->ok()) {
    this->mark_failed(LOG_STR("Could not allocate frame buffers"));
    return;
  }
  this->next_ms_ = millis();
}

void TestPattern::dump_config() {
  ESP_LOGCONFIG(TAG, "Test pattern: %ux%u RGB565 at %u fps", this->width_, this->height_, this->fps_);
}

void TestPattern::render_(uint8_t *dst, uint32_t frame) {
  static const uint16_t BARS[8] = {
      rgb565(255, 255, 255), rgb565(255, 255, 0), rgb565(0, 255, 255), rgb565(0, 255, 0),
      rgb565(255, 0, 255),   rgb565(255, 0, 0),   rgb565(0, 0, 255),   rgb565(0, 0, 0),
  };
  const uint16_t w = this->width_, h = this->height_;
  const uint16_t bar_w = w / 8 ? w / 8 : 1;
  const uint32_t shift = frame * 2;  // bars scroll 2 px per frame
  const uint16_t box = h / 4;
  const uint16_t box_x = static_cast<uint16_t>((frame * 3) % (w > box ? w - box : 1));
  const uint16_t box_y = static_cast<uint16_t>((h - box) / 2);
  const uint16_t counter_h = h / 12 ? h / 12 : 1;
  for (uint16_t y = 0; y < h; y++) {
    uint8_t *p = dst + static_cast<size_t>(y) * w * 2;
    for (uint16_t x = 0; x < w; x++) {
      uint16_t c;
      if (y >= h - counter_h) {
        // Bottom strip: frame counter in binary, 16 cells, so dropped or
        // repeated frames are visible.
        const uint16_t bit = static_cast<uint16_t>(x * 16 / w);
        c = ((frame >> (15 - bit)) & 1) ? rgb565(255, 255, 255) : rgb565(40, 40, 40);
      } else if (x >= box_x && x < box_x + box && y >= box_y && y < box_y + box) {
        c = rgb565(255, 128, 0);
      } else {
        c = BARS[((x + shift) / bar_w) % 8];
      }
      *p++ = static_cast<uint8_t>(c & 0xFF);
      *p++ = static_cast<uint8_t>(c >> 8);
    }
  }
}

void TestPattern::loop() {
  if (this->pool_ == nullptr)
    return;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - this->next_ms_) < 0)
    return;
  const uint32_t period = 1000u / this->fps_;
  this->next_ms_ += period;
  if (static_cast<int32_t>(now - this->next_ms_) > static_cast<int32_t>(period))
    this->next_ms_ = now + period;  // fell behind: don't burst to catch up
  if (!this->sinks_.any_wants())
    return;

  video::ImageFrame f;
  f.buffer = this->pool_->acquire();
  if (!f.buffer)
    return;  // both buffers still held downstream
  f.width = this->width_;
  f.height = this->height_;
  f.stride = this->width_ * 2;
  f.format = video::ImageFormat::RGB565_LE;
  f.timestamp_ms = now;
  f.sequence = ++this->frame_;
  this->render_(f.buffer.data(), this->frame_);
  this->sinks_.deliver(f, [](video::ImageSink *s, const video::ImageFrame &fr) { s->on_image_frame(fr); });
}

}  // namespace video_test_pattern
}  // namespace esphome
