#include "seek_compact_pro.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <esp_heap_caps.h>
#include <freertos/idf_additions.h>

// The task's stack is in PSRAM: internal RAM is needed by the network stack and USB.
static constexpr UBaseType_t TASK_STACK_CAPS = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

namespace esphome {
namespace seek_compact_pro {

static const char *const TAG = "seek_compact_pro";

// Vendor requests (bRequest), to interface 0.
static constexpr uint8_t REQ_TOGGLE_SHUTTER = 55;
static constexpr uint8_t REQ_READ_CHIP_ID = 54;
static constexpr uint8_t REQ_SET_OPERATION_MODE = 60;
static constexpr uint8_t REQ_SET_IMAGE_PROCESSING_MODE = 62;
static constexpr uint8_t REQ_GET_FIRMWARE_INFO = 78;
static constexpr uint8_t REQ_START_GET_IMAGE_TRANSFER = 83;
static constexpr uint8_t REQ_TARGET_PLATFORM = 84;
static constexpr uint8_t REQ_SET_FIRMWARE_INFO_FEATURES = 85;
static constexpr uint8_t REQ_SET_FACTORY_SETTINGS_FEATURES = 86;
static constexpr uint8_t REQ_GET_FACTORY_SETTINGS = 88;
static constexpr uint8_t TYPE_OUT = 0x41;  // vendor, interface, host to device
static constexpr uint8_t TYPE_IN = 0xC1;   // vendor, interface, device to host
static constexpr uint8_t BULK_IN_EP = 0x81;
static constexpr uint8_t INTERFACE = 0;
// Each 13680-byte piece ends with a short packet; a transfer is a whole
// number of 512-byte packets that holds one.
static constexpr size_t TRANSFER_BYTES = 27 * 512;
static constexpr size_t RAW_FRAMES = 6;  // assembling, the special queue, an image, processing
static constexpr size_t RAD_FRAMES = 4;  // stages after this one may hold frames
static constexpr uint32_t CTRL_TIMEOUT_MS = 1000;
static constexpr uint32_t FRAME_TIMEOUT_MS = 1000;
static constexpr uint16_t TABLE_WORDS = 2560, TABLE_CHUNK_WORDS = 32;

void SeekCompactPro::setup() {
  usb_host::USBClient::setup();
  if (this->is_failed())
    return;
  // 8 KB: sinks (radiometric processing) run on this task.
  if (xTaskCreatePinnedToCoreWithCaps(SeekCompactPro::task_entry_, "seek", 8192, this, 4, &this->task_, tskNO_AFFINITY,
                                      TASK_STACK_CAPS) != pdPASS) {
    this->mark_failed(LOG_STR("Cannot start the processing task"));
    return;
  }
  this->build_steps_();
  this->publish_streaming_(false);
  this->set_status_("No camera");
}

void SeekCompactPro::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Seek Compact Pro:\n"
                "  %u bulk transfers of %u bytes\n"
                "  Correction: x %.3f %+.2f C\n"
                "  Scene filter: %s",
                this->num_transfers_, static_cast<unsigned>(TRANSFER_BYTES), this->gain_.load(), this->offset_.load(),
                this->processor_.scene_filter() ? "on" : "off");
  if (!this->firmware_.empty())
    ESP_LOGCONFIG(TAG, "  Firmware: %s\n  Chip ID: %s", this->firmware_.c_str(), this->chip_id_.c_str());
  if (!this->table_.empty())
    ESP_LOGCONFIG(TAG, "  Factory calibration: %s (thermography version %d)",
                  this->factory_.valid ? "supported" : "not supported: no temperatures", this->factory_.version);
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Streaming", this->streaming_sensor_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Status", this->status_sensor_);
#endif
}

// The start-up sequence of the vendor's software (libseek-thermal, MIT): the
// target platform, firmware and chip ID, factory settings, the factory table
// (2560 words, 32 at a time), image processing mode 8, then running.
void SeekCompactPro::build_steps_() {
  auto out = [this](uint8_t req, std::vector<uint8_t> d, bool optional = false) {
    this->steps_.push_back({req, false, std::move(d), 0, KEEP_NONE, optional});
  };
  auto in = [this](uint8_t req, uint16_t len, uint8_t keep = KEEP_NONE) {
    this->steps_.push_back({req, true, {}, len, keep, false});
  };
  this->steps_.clear();
  // A camera left running (this board restarted, the camera kept its power)
  // refuses the start; libseek stops it first. A stopped one may refuse this.
  out(REQ_SET_OPERATION_MODE, {0x00, 0x00}, true);
  out(REQ_TARGET_PLATFORM, {0x01});
  out(REQ_SET_OPERATION_MODE, {0x00, 0x00});
  in(REQ_GET_FIRMWARE_INFO, 4, KEEP_FIRMWARE);
  in(REQ_READ_CHIP_ID, 12, KEEP_CHIP_ID);
  out(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x06, 0x00, 0x08, 0x00, 0x00, 0x00});
  in(REQ_GET_FACTORY_SETTINGS, 12);
  out(REQ_SET_FIRMWARE_INFO_FEATURES, {0x17, 0x00});
  in(REQ_GET_FIRMWARE_INFO, 64, KEEP_FIRMWARE_LONG);
  out(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x01, 0x00, 0x00, 0x06, 0x00, 0x00});
  in(REQ_GET_FACTORY_SETTINGS, 2);
  out(REQ_SET_FACTORY_SETTINGS_FEATURES, {0x01, 0x00, 0x01, 0x06, 0x00, 0x00});
  in(REQ_GET_FACTORY_SETTINGS, 2);
  for (uint16_t addr = 0; addr < TABLE_WORDS; addr += TABLE_CHUNK_WORDS) {
    out(REQ_SET_FACTORY_SETTINGS_FEATURES,
        {0x20, 0x00, static_cast<uint8_t>(addr & 0xFF), static_cast<uint8_t>(addr >> 8), 0x00, 0x00});
    in(REQ_GET_FACTORY_SETTINGS, TABLE_CHUNK_WORDS * 2, KEEP_TABLE);
  }
  out(REQ_SET_FIRMWARE_INFO_FEATURES, {0x15, 0x00});
  in(REQ_GET_FIRMWARE_INFO, 64);
  out(REQ_SET_IMAGE_PROCESSING_MODE, {0x08, 0x00});
  out(REQ_SET_OPERATION_MODE, {0x01, 0x00});
}

void SeekCompactPro::on_connected() {
  // Bulk transfers need the interface claimed.
  const esp_err_t err = usb_host_interface_claim(this->handle_, this->device_handle_, INTERFACE, 0);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Cannot claim interface %u: %s", INTERFACE, esp_err_to_name(err));
    this->status_set_error(LOG_STR("Cannot claim the camera's interface"));
    this->disconnect();
    return;
  }
  this->interface_claimed_ = true;
  ESP_LOGI(TAG, "Camera connected; starting it");
  this->status_clear_error();
  this->failures_ = 0;
  this->set_status_("Camera connected; starting it");
  this->goto_stage_(Stage::INIT, 0);
}

void SeekCompactPro::on_disconnected() {
  const bool was_camera = this->stage_ != Stage::IDLE;
  this->stop_stream_();
  // The host cannot close a device with an interface still claimed.
  if (this->interface_claimed_ && this->device_handle_ != nullptr) {
    const esp_err_t err = usb_host_interface_release(this->handle_, this->device_handle_, INTERFACE);
    if (err != ESP_OK)
      ESP_LOGW(TAG, "Cannot release interface %u: %s", INTERFACE, esp_err_to_name(err));
  }
  this->stage_ = Stage::IDLE;
  this->ctrl_gen_++;  // a request still in flight is abandoned
  this->ctrl_state_.store(CTRL_IDLE);
  this->interface_claimed_ = false;  // gone with the device
  this->publish_streaming_(false);
  {
    std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
    this->reset_pending_ = true;  // a new pixel table on the next start
  }
  if (was_camera) {
    ESP_LOGI(TAG, "Camera disconnected");
    this->set_status_("Camera disconnected");
  }
  usb_host::USBClient::on_disconnected();
}

void SeekCompactPro::goto_stage_(Stage s, uint32_t delay_ms) {
  this->stage_ = s;
  this->stage_at_ = millis() + delay_ms;
  this->step_ = 0;
  this->step_sent_ = false;
}

void SeekCompactPro::fail_(const char *what) {
  this->failures_++;
  const uint32_t delay = std::min<uint32_t>(1000u << std::min<uint8_t>(this->failures_, 4), 15000);
  ESP_LOGW(TAG, "%s; starting the camera again in %u s", what, static_cast<unsigned>(delay / 1000));
  this->set_status_(std::string(what) + "; retrying");
  this->stop_stream_();
  this->ctrl_gen_++;
  this->ctrl_state_.store(CTRL_IDLE);
  this->publish_streaming_(false);
  this->goto_stage_(Stage::BACKOFF, delay);
}

bool SeekCompactPro::send_control_(bool in, uint8_t request, const std::vector<uint8_t> &data, uint16_t length) {
  const uint32_t gen = ++this->ctrl_gen_;
  this->ctrl_state_.store(CTRL_PENDING);
  this->ctrl_sent_ms_ = millis();
  // An IN request's length is the length of `data`.
  const std::vector<uint8_t> payload = in ? std::vector<uint8_t>(length) : data;
  const bool ok = this->control_transfer(
      in ? TYPE_IN : TYPE_OUT, request, 0, INTERFACE,
      [this, gen](const usb_host::TransferStatus &st) {  // the USB task
        if (gen != this->ctrl_gen_.load())
          return;
        size_t n = 0;
        if (st.success && st.data_len > usb_host::SETUP_PACKET_SIZE) {
          n = std::min(st.data_len - usb_host::SETUP_PACKET_SIZE, sizeof(this->ctrl_resp_));
          std::memcpy(this->ctrl_resp_, st.data + usb_host::SETUP_PACKET_SIZE, n);
        }
        this->ctrl_resp_len_ = n;
        this->ctrl_state_.store(st.success ? CTRL_OK : CTRL_FAILED);
        App.wake_loop_threadsafe();
      },
      payload);
  if (!ok)
    this->ctrl_state_.store(CTRL_FAILED);
  return ok;
}

static std::string hex(const uint8_t *p, size_t n) {
  std::string s;
  char b[4];
  for (size_t i = 0; i < n; i++) {
    std::snprintf(b, sizeof(b), "%02X", p[i]);
    s += b;
  }
  return s;
}

static std::string printable(const uint8_t *p, size_t n) {
  std::string s;
  for (size_t i = 0; i < n; i++)
    if (p[i] >= 0x20 && p[i] < 0x7F)
      s += static_cast<char>(p[i]);
  return s;
}

void SeekCompactPro::run_init_() {
  const uint32_t now = millis();
  if (!this->step_sent_) {
    if (this->step_ == 0) {
      this->table_.clear();
      this->table_.reserve(TABLE_WORDS * 2);
    }
    const Step &s = this->steps_[this->step_];
    if (!this->send_control_(s.in, s.request, s.data, s.length)) {
      this->fail_("A start-up request could not be sent");
      return;
    }
    this->step_sent_ = true;
    return;
  }
  const uint8_t st = this->ctrl_state_.load();
  if (st == CTRL_PENDING) {
    if (now - this->ctrl_sent_ms_ > CTRL_TIMEOUT_MS)
      this->fail_("The camera did not answer a start-up request");
    return;
  }
  const Step &s = this->steps_[this->step_];
  if (st == CTRL_FAILED && s.optional) {
    this->ctrl_resp_len_ = 0;
  } else if (st == CTRL_FAILED) {
    char what[64];
    std::snprintf(what, sizeof(what), "The camera refused start-up request %u (step %u)", s.request,
                  static_cast<unsigned>(this->step_));
    this->fail_(what);
    return;
  }
  // Done: keep what was read.
  const uint8_t *r = this->ctrl_resp_;
  const size_t n = this->ctrl_resp_len_;
  switch (s.keep) {
    case KEEP_FIRMWARE:
      this->firmware_ = hex(r, n);
      break;
    case KEEP_FIRMWARE_LONG: {
      const std::string text = printable(r, n);
      if (!text.empty())
        this->firmware_ += " \"" + text + "\"";
      break;
    }
    case KEEP_CHIP_ID:
      this->chip_id_ = hex(r, n);
      break;
    case KEEP_TABLE:
      this->table_.insert(this->table_.end(), r, r + n);
      break;
    default:
      break;
  }
  this->ctrl_state_.store(CTRL_IDLE);
  this->step_++;
  this->step_sent_ = false;
  if (this->step_ < this->steps_.size())
    return;

  // Started: the factory calibration, then stream.
  parse_factory(this->table_.data(), this->table_.size(), &this->factory_);
  if (this->factory_.valid) {
    ESP_LOGI(TAG, "Camera started. Firmware %s, chip %s, factory calibration version %d", this->firmware_.c_str(),
             this->chip_id_.c_str(), this->factory_.version);
  } else {
    ESP_LOGE(TAG, "Camera started, but its factory calibration (%u bytes) is not one this component supports: "
                  "no temperatures", static_cast<unsigned>(this->table_.size()));
    this->set_status_("Unsupported factory calibration");
  }
  {
    std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
    this->factory_for_task_ = this->table_;
    this->factory_pending_ = true;
    this->reset_pending_ = true;  // the first frames bring the tables again
    for (auto &f : this->pending_special_)
      f.reset();  // frames of the last start must not reach the new one
    this->special_head_ = 0;
    this->special_count_ = 0;
    this->pending_image_.reset();
    this->table_lost_.store(false);
  }
  if (!this->start_transfers_()) {
    this->fail_("Cannot start the bulk transfers");
    return;
  }
  this->stage_ = Stage::STREAM;
  this->missed_ = 0;
  this->request_frame_();
  this->set_status_("Streaming");
}

bool SeekCompactPro::start_transfers_() {
  auto make_pool = [](size_t count, size_t bytes) {
    return std::make_unique<video::FramePool>(count, bytes, 64, video::dma_allocator());
  };
  if (!this->raw_pool_)
    this->raw_pool_ = make_pool(RAW_FRAMES, RAW_BYTES);
  if (!this->sinks_.empty() && !this->rad_pool_)
    this->rad_pool_ = make_pool(RAD_FRAMES, IMG_PIXELS * sizeof(float));
  if (!this->raw_pool_->ok() || (this->rad_pool_ && !this->rad_pool_->ok())) {
    ESP_LOGE(TAG, "No memory for frame buffers");
    return false;
  }
  for (uint8_t i = 0; i < this->num_transfers_; i++) {
    usb_transfer_t *t;
    if (usb_host_transfer_alloc(TRANSFER_BYTES, 0, &t) != ESP_OK) {
      ESP_LOGE(TAG, "No memory for %u bulk transfers of %u bytes", this->num_transfers_,
               static_cast<unsigned>(TRANSFER_BYTES));
      for (auto *x : this->transfers_)
        usb_host_transfer_free(x);
      this->transfers_.clear();
      return false;
    }
    t->device_handle = this->device_handle_;
    t->bEndpointAddress = BULK_IN_EP;
    t->callback = SeekCompactPro::transfer_done_;
    t->context = this;
    t->num_bytes = TRANSFER_BYTES;
    this->transfers_.push_back(t);
  }
  this->assembling_ = this->raw_pool_->acquire();
  this->asm_bytes_ = 0;
  this->frame_done_.store(false);
  this->last_frame_ms_.store(millis());
  this->streaming_.store(true);
  for (auto *t : this->transfers_) {
    this->inflight_++;
    const esp_err_t err = usb_host_transfer_submit(t);
    if (err != ESP_OK) {
      this->inflight_--;
      ESP_LOGE(TAG, "Cannot submit a bulk transfer: %s", esp_err_to_name(err));
      return false;
    }
  }
  return true;
}

void SeekCompactPro::stop_stream_() {
  this->streaming_.store(false);  // completions stop resubmitting
  if (!this->transfers_.empty() && this->device_handle_ != nullptr) {
    usb_host_endpoint_halt(this->device_handle_, BULK_IN_EP);
    usb_host_endpoint_flush(this->device_handle_, BULK_IN_EP);
    usb_host_endpoint_clear(this->device_handle_, BULK_IN_EP);
  }
  // Flushed transfers come back on the USB task (higher priority than this loop).
  const uint32_t t0 = millis();
  while (this->inflight_.load() > 0 && millis() - t0 < 500)
    vTaskDelay(pdMS_TO_TICKS(2));
  if (this->inflight_.load() > 0) {
    // Freeing them would let the controller write into freed memory.
    ESP_LOGE(TAG, "%d bulk transfers did not come back; leaving them allocated", this->inflight_.load());
    this->status_set_error(LOG_STR("USB transfers stuck"));
    this->inflight_.store(0);
  } else {
    for (auto *t : this->transfers_)
      usb_host_transfer_free(t);
  }
  this->transfers_.clear();
  this->assembling_.reset();
  this->asm_bytes_ = 0;
}

// The main loop asks for the first frame (and again after a timeout); then
// the USB task asks for each next one as the last one's last byte arrives.
bool SeekCompactPro::request_frame_() {
  this->requested_ms_ = millis();
  this->send_credit_();
  return true;
}

void SeekCompactPro::send_credit_() {
  static const std::vector<uint8_t> WORDS = {static_cast<uint8_t>(RAW_WORDS), static_cast<uint8_t>(RAW_WORDS >> 8),
                                             static_cast<uint8_t>(RAW_WORDS >> 16), static_cast<uint8_t>(RAW_WORDS >> 24)};
  const bool ok = this->control_transfer(
      TYPE_OUT, REQ_START_GET_IMAGE_TRANSFER, 0, INTERFACE,
      [this](const usb_host::TransferStatus &st) {
        if (!st.success)
          this->credit_errors_++;
      },
      WORDS);
  if (!ok)
    this->credit_errors_++;
}

void SeekCompactPro::trigger_shutter() { this->shutter_want_ = true; }

void SeekCompactPro::loop() {
  // Connect / disconnect events. Not USBClient::loop(): that switches this
  // component's loop off whenever no USB event is waiting, and everything
  // below (commands, timeouts, the stream) runs from this loop.
  this->process_usb_events_();
  const uint32_t now = millis();
  switch (this->stage_) {
    case Stage::IDLE:
      break;
    case Stage::BACKOFF:
      if (static_cast<int32_t>(now - this->stage_at_) >= 0 && this->device_handle_ != nullptr)
        this->goto_stage_(Stage::INIT, 0);
      break;
    case Stage::INIT:
      if (static_cast<int32_t>(now - this->stage_at_) >= 0)
        this->run_init_();
      break;
    case Stage::STREAM: {
      const uint8_t st = this->ctrl_state_.load();
      if (st == CTRL_PENDING) {
        if (now - this->ctrl_sent_ms_ > CTRL_TIMEOUT_MS)
          this->fail_("The camera stopped answering");
        break;
      }
      if (st == CTRL_FAILED) {
        this->fail_("The camera refused a request while streaming");
        break;
      }
      this->ctrl_state_.store(CTRL_IDLE);
      if (this->table_lost_.exchange(false)) {
        this->fail_("The camera's pixel table (its first frame) was lost");
        break;
      }
      if (this->shutter_want_) {
        this->shutter_want_ = false;
        // {address 0xFC, value 4}: a shutter cycle now (openseekthermal).
        ESP_LOGI(TAG, "Shutter");
        this->send_control_(false, REQ_TOGGLE_SHUTTER, {0xFC, 0x00, 0x04, 0x00}, 0);
        break;
      }
      if (this->frame_done_.exchange(false)) {
        this->missed_ = 0;
        this->requested_ms_ = now;  // the USB task has asked for the next one
        this->publish_streaming_(true);
      } else if (now - this->requested_ms_ > FRAME_TIMEOUT_MS) {
        if (++this->missed_ >= 3) {
          this->fail_("No frames from the camera");
          break;
        }
        ESP_LOGW(TAG, "No frame within %u ms; asking again", static_cast<unsigned>(FRAME_TIMEOUT_MS));
        this->discard_.store(true);  // what came of the last one would misalign the next
        this->request_frame_();
      }
      break;
    }
  }
  if (this->stage_ == Stage::STREAM && now - this->last_frame_ms_.load() > 5000)
    this->publish_streaming_(false);
  this->report_(now);
}

// ---------------------------------------------------------------- USB task

void SeekCompactPro::transfer_done_(usb_transfer_t *t) { static_cast<SeekCompactPro *>(t->context)->on_transfer_(t); }

void SeekCompactPro::on_transfer_(usb_transfer_t *t) {
  const usb_transfer_status_t st = t->status;
  const bool gone = st == USB_TRANSFER_STATUS_CANCELED || st == USB_TRANSFER_STATUS_NO_DEVICE;
  if (this->discard_.exchange(false) && this->asm_bytes_ != 0) {
    this->resyncs_++;
    this->asm_bytes_ = 0;
  }
  if (st == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
    const size_t n = static_cast<size_t>(t->actual_num_bytes);
    if (!this->assembling_ && this->raw_pool_)
      this->assembling_ = this->raw_pool_->acquire();
    if (!this->assembling_) {
      this->no_buffer_++;  // all frames held: this one is lost
    } else {
      if (this->asm_bytes_ + n > RAW_BYTES) {
        this->resyncs_++;  // more than a frame: start again at this piece
        this->asm_bytes_ = 0;
      }
      std::memcpy(this->assembling_.data() + this->asm_bytes_, t->data_buffer, n);
      this->asm_bytes_ += n;
      if (this->asm_bytes_ == RAW_BYTES) {
        // The next frame at once: asked for late, the camera stops for good.
        if (this->streaming_.load())
          this->send_credit_();
        const uint32_t ms = millis();
        const uint16_t *w = reinterpret_cast<const uint16_t *>(this->assembling_.data());
        if (frame_magic(w) != FRAME_MAGIC) {
          this->torn_++;  // not the start of a frame: keep the buffer, start again
        } else {
          {
            std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
            if (frame_id(w) == FRAME_IMAGE) {
              if (this->pending_image_)
                this->dropped_++;
              this->pending_image_ = std::move(this->assembling_);
              this->pending_image_ms_ = ms;
            } else if (frame_used(frame_id(w))) {
              // Tables and shutter frames queue in order. With the queue full
              // the oldest goes; a table lost that way shows as a frame
              // before the tables, and the camera is started again.
              if (this->special_count_ == SPECIAL_SLOTS) {
                this->special_head_ = (this->special_head_ + 1) % SPECIAL_SLOTS;
                this->special_count_--;
                this->dropped_++;
              }
              const uint8_t slot = (this->special_head_ + this->special_count_) % SPECIAL_SLOTS;
              this->pending_special_[slot] = std::move(this->assembling_);
              this->pending_special_ms_[slot] = ms;
              this->special_count_++;
            }
          }
          this->assembling_ = this->raw_pool_->acquire();
          this->frames_++;
          this->last_frame_ms_.store(ms);
          xTaskNotifyGive(this->task_);
        }
        this->asm_bytes_ = 0;
        this->frame_done_.store(true);
        App.wake_loop_threadsafe();
      }
    }
  } else if (st != USB_TRANSFER_STATUS_COMPLETED && !gone) {
    this->errors_++;
  }
  if (this->streaming_.load() && !gone) {
    if (usb_host_transfer_submit(t) == ESP_OK)
      return;
    this->errors_++;
  }
  this->inflight_--;
}

// ---------------------------------------------------------------- processing task

void SeekCompactPro::task_entry_(void *arg) { static_cast<SeekCompactPro *>(arg)->task_loop_(); }

void SeekCompactPro::task_loop_() {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    video::FrameRef special[SPECIAL_SLOTS], image;
    uint32_t special_ms[SPECIAL_SLOTS]{}, image_ms = 0;
    uint8_t n = 0;
    {
      std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
      if (this->reset_pending_) {
        this->processor_.reset();
        this->reset_pending_ = false;
      }
      if (this->factory_pending_) {
        this->processor_.set_factory(this->factory_for_task_.data(), this->factory_for_task_.size());
        this->factory_pending_ = false;
      }
      for (; this->special_count_ > 0; this->special_count_--) {
        special[n] = std::move(this->pending_special_[this->special_head_]);
        special_ms[n++] = this->pending_special_ms_[this->special_head_];
        this->special_head_ = (this->special_head_ + 1) % SPECIAL_SLOTS;
      }
      image = std::move(this->pending_image_);
      image_ms = this->pending_image_ms_;
    }
    this->processor_.set_correction(this->gain_.load(), this->offset_.load());
    // In the order they came: a shutter frame applies to the images after it.
    bool image_done = !image;
    for (uint8_t k = 0; k < n; k++) {
      if (!image_done && static_cast<int32_t>(special_ms[k] - image_ms) > 0) {
        this->process_(image, image_ms);
        image_done = true;
      }
      this->process_(special[k], special_ms[k]);
    }
    if (!image_done)
      this->process_(image, image_ms);
  }
}

void SeekCompactPro::process_(const video::FrameRef &raw, uint32_t ts) {
  const uint16_t *w = reinterpret_cast<const uint16_t *>(raw.data());
  const uint32_t t0 = micros();
  // The camera sends its tables (frames 4, 9 and 14) once, first after a
  // start. A shutter frame or image before them means one was lost (a short
  // first frame, discarded): only a new start brings them again.
  const uint16_t id = frame_id(w);
  if ((id == FRAME_IMAGE || id == FRAME_SHUTTER || id == FRAME_START_SHUTTER) && !this->processor_.has_table()) {
    this->table_lost_.store(true);
    return;
  }
  if (frame_id(w) == FRAME_IMAGE) {
    if (this->rad_pool_ == nullptr || !this->sinks_.any_wants())
      return;
    video::RadiometricFrame f;
    f.buffer = this->rad_pool_->acquire();
    if (!f.buffer) {
      this->no_buffer_++;
      return;
    }
    if (this->processor_.feed(w, f.temps(), &f.stats) != Processor::Result::IMAGE)
      return;  // no shutter frame yet
    f.width = IMG_W;
    f.height = IMG_H;
    f.timestamp_ms = ts;
    f.sequence = ++this->sequence_;
    // The camera's conversion is for a surface it sees as a black body; the
    // reflected temperature is not known.
    this->images_++;
    this->sensor_.store(this->processor_.sensor_counts());
    this->warm_up_.store(this->processor_.warm_up_c());
    this->process_us_ += micros() - t0;
    this->sinks_.deliver(f, [](video::RadiometricSink *s, const video::RadiometricFrame &fr) {
      s->on_radiometric_frame(fr);
    });
    return;
  }
  switch (this->processor_.feed(w, nullptr)) {
    case Processor::Result::TABLE:
      if (id == FRAME_GAIN) {
        this->dead_pixels_.store(this->processor_.dead_pixels());
        ESP_LOGI(TAG, "Pixel table: %u bad pixels (bad columns included)",
                 static_cast<unsigned>(this->processor_.dead_pixels()));
      } else if (id == FRAME_ADJUST) {
        ESP_LOGI(TAG, "Temperature correction tables: %u", this->processor_.adjust_tables());
      }
      break;
    case Processor::Result::SHUTTER:
      this->shutters_++;
      break;
    case Processor::Result::NO_MEMORY:
      ESP_LOGE(TAG, "No memory for the camera's tables or shutter frame");
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------- status

void SeekCompactPro::set_status_(const std::string &s) {
  if (s == this->status_)
    return;
  this->status_ = s;
#ifdef USE_TEXT_SENSOR
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(s);
#endif
}

void SeekCompactPro::publish_streaming_(bool on) {
  if (this->streaming_known_ && this->streaming_published_ == on)
    return;
  this->streaming_known_ = true;
  this->streaming_published_ = on;
#ifdef USE_BINARY_SENSOR
  if (this->streaming_sensor_ != nullptr)
    this->streaming_sensor_->publish_state(on);
#endif
}

void SeekCompactPro::report_(uint32_t now) {
  if (this->last_report_ms_ == 0) {
    this->last_report_ms_ = now;
    return;
  }
  const uint32_t elapsed = now - this->last_report_ms_;
  if (elapsed < 10000)
    return;
  this->last_report_ms_ = now;
  const uint32_t frames = this->frames_.exchange(0), images = this->images_.exchange(0),
                 shutters = this->shutters_.exchange(0), dropped = this->dropped_.exchange(0),
                 errors = this->errors_.exchange(0), resyncs = this->resyncs_.exchange(0),
                 nobuf = this->no_buffer_.exchange(0), us = this->process_us_.exchange(0),
                 torn = this->torn_.exchange(0), credit = this->credit_errors_.exchange(0);
  if (!this->log_timing_ || this->stage_ != Stage::STREAM)
    return;
  const float s = elapsed / 1000.0f;
  ESP_LOGI(TAG,
           "Over %.1f s: %.2f frames/s from the camera, %.2f images/s passed on (%.2f ms each), %u shutter frames, "
           "%u images dropped, %u torn, %u transfer errors, %u frame requests failed, %u resyncs, %u without a "
           "buffer; sensor %.1f counts, warm-up %.2f C, %u bad pixels",
           s, frames / s, images / s, images ? us / 1000.0f / images : 0.0f, static_cast<unsigned>(shutters),
           static_cast<unsigned>(dropped), static_cast<unsigned>(torn), static_cast<unsigned>(errors),
           static_cast<unsigned>(credit), static_cast<unsigned>(resyncs), static_cast<unsigned>(nobuf),
           this->sensor_.load(), this->warm_up_.load(), static_cast<unsigned>(this->dead_pixels_.load()));
}

}  // namespace seek_compact_pro
}  // namespace esphome
