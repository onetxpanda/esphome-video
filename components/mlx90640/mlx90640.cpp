#include "mlx90640.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "MLX90640_I2C_Driver.h"

namespace esphome {
namespace mlx90640 {

static const char *const TAG = "mlx90640";
static constexpr int TA_SHIFT = 8;  // reflected temperature offset for open air (Melexis)
static constexpr size_t POOL_FRAMES = 3;
// Sub-pages to throw away after start-up or a refresh-rate change. The one
// being measured when the rate changes comes out with nonsense temperatures
// across the whole frame (seen as single readings of 150-500 F); two
// sub-pages is one full frame measured entirely with the new setting.
static constexpr uint8_t SETTLE_SUBPAGES = 2;

static const char *extract_error(int status) {
  switch (status) {
    case MLX90640_BROKEN_PIXELS_NUM_ERROR:
      return "too many broken pixels";
    case MLX90640_OUTLIER_PIXELS_NUM_ERROR:
      return "too many outlier pixels";
    case MLX90640_BAD_PIXELS_NUM_ERROR:
      return "too many bad pixels";
    case MLX90640_ADJACENT_BAD_PIXELS_ERROR:
      return "adjacent bad pixels";
    case MLX90640_EEPROM_DATA_ERROR:
      return "EEPROM data error";
    default:
      return "unknown error";
  }
}

void MLX90640::setup() {
  MLX90640_I2CInit(this);
  uint16_t ee[832];
  if (MLX90640_DumpEE(this->address_, ee) != 0) {
    this->mark_failed(LOG_STR("Cannot read the sensor EEPROM"));
    return;
  }
  const int status = MLX90640_ExtractParameters(ee, &this->params_);
  if (status != 0) {
    ESP_LOGE(TAG, "Calibration data unusable: %s (%d)", extract_error(status), status);
    this->mark_failed();
    return;
  }
  // Chess mode and the calibrated ADC resolution, set explicitly: a control
  // register left wrong by earlier firmware gives wildly wrong temperatures.
  MLX90640_SetChessMode(this->address_);
  MLX90640_SetResolution(this->address_, this->params_.resolutionEE);
  if (!this->apply_refresh_rate_(this->refresh_rate_)) {
    this->mark_failed(LOG_STR("Cannot write the sensor control register"));
    return;
  }
  this->chess_ = MLX90640_GetCurMode(this->address_);
  this->assembler_.set_mode(this->get_frame_mode());

#ifdef USE_ESP32
  this->pool_ = std::make_unique<video::FramePool>(POOL_FRAMES, sizeof(float) * SENSOR_PIXELS, 64,
                                                   video::dma_allocator());
#else
  this->pool_ = std::make_unique<video::FramePool>(POOL_FRAMES, sizeof(float) * SENSOR_PIXELS);
#endif
  if (!this->pool_->ok()) {
    this->mark_failed(LOG_STR("Out of memory for frame buffers"));
    return;
  }
  // 8 KB: the I2C driver reads a sub-page through a 1.5 KB stack buffer, and
  // sinks (radiometric processing) run on this task.
  if (xTaskCreate(MLX90640::task_entry_, "mlx90640", 8192, this, 5, &this->task_) != pdPASS) {
    this->mark_failed(LOG_STR("Cannot start the capture task"));
    return;
  }
}

void MLX90640::dump_config() {
  ESP_LOGCONFIG(TAG, "MLX90640:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed())
    return;
  ESP_LOGCONFIG(TAG,
                "  Refresh rate: %.1f Hz sub-pages (register %u)\n"
                "  Pattern: %s, frames: %s\n"
                "  Emissivity: %.2f",
                0.5f * (1u << this->refresh_rate_.load()), this->refresh_rate_.load(), this->chess_ ? "chess" : "interleaved",
                this->get_frame_mode() == FrameMode::MERGE ? "merge sub-page pairs" : "interpolate each sub-page",
                this->emissivity_);
}

bool MLX90640::apply_refresh_rate_(uint8_t r) {
  if (MLX90640_SetRefreshRate(this->address_, r) != 0)
    return false;
  ESP_LOGD(TAG, "Refresh rate register set to %u", r);
  return true;
}

void MLX90640::loop() {
  const uint32_t settled = this->settle_dropped_.exchange(0);
  if (settled != 0)
    ESP_LOGD(TAG, "Discarded %u sub-pages while the sensor settled on a new refresh rate", static_cast<unsigned>(settled));
  const uint32_t errors = this->errors_.load();
  if (errors != this->last_errors_) {
    ESP_LOGW(TAG, "%u sensor read errors", static_cast<unsigned>(errors - this->last_errors_));
    this->last_errors_ = errors;
  }
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
  const uint32_t sp = this->subpages_.exchange(0), fr = this->frames_.exchange(0), dr = this->dropped_.exchange(0);
  const uint32_t rd = this->read_us_.exchange(0), ca = this->calc_us_.exchange(0), de = this->deliver_us_.exchange(0),
                 wa = this->wait_us_.exchange(0);
  const float s = elapsed / 1000.0f;
  auto avg = [](uint32_t us, uint32_t n) { return n ? us / 1000.0f / n : 0.0f; };
  ESP_LOGI(TAG,
           "Over %.1f s: %.2f sub-pages/s, %.2f frames/s, %u dropped (no free buffer). Per sub-page: "
           "wait %.2f ms, read %.2f ms, calculate %.2f ms; per frame: sinks %.2f ms",
           s, sp / s, fr / s, static_cast<unsigned>(dr), avg(wa, sp), avg(rd, sp), avg(ca, sp), avg(de, fr));
}

void MLX90640::task_entry_(void *arg) { static_cast<MLX90640 *>(arg)->task_loop_(); }

void MLX90640::task_loop_() {
  uint8_t rate = this->refresh_rate_;
  uint8_t mode = this->frame_mode_;
  uint8_t settle = SETTLE_SUBPAGES;
  for (;;) {
    // Settings changed from the main loop.
    if (this->refresh_rate_ != rate) {
      rate = this->refresh_rate_;
      if (!this->apply_refresh_rate_(rate))
        this->errors_++;
      this->assembler_.set_mode(this->get_frame_mode());  // don't merge across rates
      settle = SETTLE_SUBPAGES;
    }
    if (this->frame_mode_ != mode) {
      mode = this->frame_mode_;
      this->assembler_.set_mode(static_cast<FrameMode>(mode));
    }

    // Wait for data-ready: poll in steps of 1/8 of the sub-page period (at
    // least one tick), so a sub-page is picked up soon after it is complete.
    uint32_t t0 = micros();
    const TickType_t step = std::max<TickType_t>(1, pdMS_TO_TICKS(this->subpage_period_ms_(rate) / 8));
    for (;;) {
      uint16_t status;
      if (MLX90640_I2CRead(this->address_, MLX90640_STATUS_REG, 1, &status) != 0) {
        this->errors_++;
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }
      if (MLX90640_GET_DATA_READY(status))
        break;
      vTaskDelay(step);
    }
    uint32_t t1 = micros();
    this->wait_us_ += t1 - t0;

    if (MLX90640_GetFrameData(this->address_, this->raw_) < 0) {
      this->errors_++;
      this->assembler_.set_mode(static_cast<FrameMode>(mode));
      continue;
    }
    uint32_t t2 = micros();
    this->read_us_ += t2 - t1;
    this->subpages_++;
    if (settle > 0) {
      settle--;
      this->settle_dropped_++;
      continue;
    }

    const float ta = MLX90640_GetTa(this->raw_, &this->params_);
    MLX90640_CalculateTo(this->raw_, &this->params_, this->emissivity_, ta - TA_SHIFT, this->temps_);
    MLX90640_BadPixelsCorrection(this->params_.brokenPixels, this->temps_, this->chess_, &this->params_);
    MLX90640_BadPixelsCorrection(this->params_.outlierPixels, this->temps_, this->chess_, &this->params_);
    const int subpage = MLX90640_GetSubPageNumber(this->raw_);
    const bool frame = this->assembler_.add(this->temps_, subpage, this->chess_ == 1);
    uint32_t t3 = micros();
    this->calc_us_ += t3 - t2;
    if (!frame || !this->sinks_.any_wants())
      continue;

    video::RadiometricFrame f;
    f.buffer = this->pool_->acquire();
    if (!f.buffer) {
      this->dropped_++;
      continue;
    }
    std::memcpy(f.temps(), this->assembler_.frame(), sizeof(float) * SENSOR_PIXELS);
    f.width = SENSOR_COLS;
    f.height = SENSOR_ROWS;
    f.timestamp_ms = millis();
    f.sequence = ++this->sequence_;
    this->sinks_.deliver(f, [](video::RadiometricSink *s, const video::RadiometricFrame &fr) {
      s->on_radiometric_frame(fr);
    });
    this->frames_++;
    this->deliver_us_ += micros() - t3;
  }
}

}  // namespace mlx90640
}  // namespace esphome
