#include "hti301.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace hti301 {

static const char *const TAG = "hti301";

// USB / UVC requests.
static constexpr uint8_t REQ_SET_CUR = 0x01;
static constexpr uint8_t REQ_GET_CUR = 0x81;
static constexpr uint8_t REQ_SET_INTERFACE = 0x0B;
static constexpr uint8_t CLASS_OUT = usb_host::USB_TYPE_CLASS | usb_host::USB_RECIP_INTERFACE | usb_host::USB_DIR_OUT;
static constexpr uint8_t CLASS_IN = usb_host::USB_TYPE_CLASS | usb_host::USB_RECIP_INTERFACE | usb_host::USB_DIR_IN;
static constexpr uint8_t STD_INTERFACE_OUT =
    usb_host::USB_TYPE_STANDARD | usb_host::USB_RECIP_INTERFACE | usb_host::USB_DIR_OUT;
static constexpr uint16_t VS_PROBE = 0x01 << 8;
static constexpr uint16_t VS_COMMIT = 0x02 << 8;
static constexpr uint16_t CT_ZOOM_ABSOLUTE = 0x0B << 8;
// The camera's own driver addresses the Zoom control with entity 0 (wIndex is
// just the VideoControl interface), not the camera terminal's ID. Follow it.
static constexpr uint16_t ZOOM_ENTITY = 0;

// Pauses the camera's own driver leaves between set-up steps (ms).
static constexpr uint32_t DELAY_AFTER_IDLE_ALT = 100;
static constexpr uint32_t DELAY_AFTER_PROBE = 100;
static constexpr uint32_t DELAY_AFTER_COMMIT = 200;
static constexpr uint32_t DELAY_AFTER_ALT = 200;
static constexpr uint32_t DELAY_BEFORE_RESTART = 500;

static constexpr size_t RAW_FRAMES = 3;  // being assembled, waiting, being processed
static constexpr size_t RAD_FRAMES = 3;
static constexpr size_t IMG_FRAMES = 4;      // encoders keep a frame while they work on it
static constexpr uint32_t STALL_MS = 3000;   // no complete frame for this long: restart the stream
static constexpr uint32_t STREAMING_MS = 2000;  // the streaming sensor is on while frames arrive this often
// Complete frames dropped after a mode or range change, while the camera
// switches over. A guess (5 frames = 200 ms); not yet measured on a camera.
static constexpr uint8_t SETTLE_FRAMES = 5;
static constexpr uint8_t COMMAND_ATTEMPTS = 3;
// ESP-IDF schedules an isochronous transfer in a 64-entry list less a
// 3-entry margin: packets x interval must fit in 61.
static constexpr int MAX_ISOC_SLOTS = 61;

// ---------------------------------------------------------------- set-up

void HTI301::setup() {
  usb_host::USBClient::setup();
  if (this->is_failed())
    return;
  // 8 KB: sinks (radiometric processing) run on this task.
  if (xTaskCreate(HTI301::task_entry_, "hti301", 8192, this, 4, &this->task_) != pdPASS) {
    this->mark_failed(LOG_STR("Cannot start the processing task"));
    return;
  }
  this->publish_streaming_(false);
}

static const char *range_name(uint8_t r) { return r == static_cast<uint8_t>(Range::HIGH) ? "high" : "normal"; }
static const char *mode_name(uint8_t m) {
  return m == static_cast<uint8_t>(Mode::PROCESSED) ? "processed" : "radiometric";
}

static void log_env(const char *name, float v, const char *unit) {
  if (std::isnan(v)) {
    ESP_LOGCONFIG(TAG, "  %s: camera's stored value", name);
  } else {
    ESP_LOGCONFIG(TAG, "  %s: %.2f%s", name, v, unit);
  }
}

void HTI301::dump_config() {
  ESP_LOGCONFIG(TAG,
                "HTI-301:\n"
                "  USB ID: %04X:%04X\n"
                "  Mode: %s, range: %s, palette: %s\n"
                "  Isochronous transfers: %u x %u packets",
                this->vid_, this->pid_, mode_name(this->mode_want_), range_name(this->range_want_),
                PALETTE_NAMES[this->palette_want_], this->num_transfers_, this->packets_per_transfer_);
  log_env("Emissivity", this->env_emissivity_.load(), "");
  log_env("Reflected temperature", this->env_reflected_.load(), " C");
  log_env("Ambient temperature", this->env_ambient_.load(), " C");
  const float h = this->env_humidity_.load();
  log_env("Humidity", std::isnan(h) ? h : h * 100.0f, " %");
  log_env("Distance", this->env_distance_.load(), " m");
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Streaming", this->streaming_sensor_);
#endif
}

void HTI301::loop() {
  this->process_usb_events_();  // connect / disconnect (never disables this loop)
  this->run_stage_();
  this->run_commands_();
  const uint32_t now = millis();
  const uint32_t last = this->last_frame_ms_.load();
  this->publish_streaming_(this->stage_ == Stage::STREAMING && this->out_mode_.load() != UNKNOWN && last != 0 &&
                           now - last < STREAMING_MS);
  this->report_(now);
}

void HTI301::on_connected() {
  const usb_config_desc_t *cfg;
  if (usb_host_get_active_config_descriptor(this->device_handle_, &cfg) != ESP_OK) {
    ESP_LOGE(TAG, "Cannot read the configuration descriptor");
    this->status_set_error(LOG_STR("Cannot read the camera's descriptors"));
    this->disconnect();
    return;
  }
  this->layout_ = parse_descriptors(reinterpret_cast<const uint8_t *>(cfg), cfg->wTotalLength);
  const UvcLayout &l = this->layout_;
  if (!l.ok) {
    ESP_LOGE(TAG, "Not a usable camera: no uncompressed video format with an isochronous endpoint");
    this->status_set_error(LOG_STR("Unsupported camera"));
    this->disconnect();
    return;
  }
  ESP_LOGI(TAG, "Camera connected: UVC %x.%02x, %ux%u frames (%u image rows), %u bytes, %.1f fps", l.uvc_version >> 8,
           l.uvc_version & 0xFF, l.width, l.height, l.height - METADATA_ROWS, static_cast<unsigned>(l.max_frame_size),
           l.frame_interval ? 1e7f / l.frame_interval : 0.0f);
  for (uint8_t i = 0; i < l.alt_count; i++)
    ESP_LOGD(TAG, "  Alternate setting %u: endpoint 0x%02X, %u x %u bytes, bInterval %u", l.alts[i].alt,
             l.alts[i].endpoint, l.alts[i].transactions, l.alts[i].packet_size, l.alts[i].interval);
  this->status_clear_error();
  this->failures_ = 0;
  this->goto_stage_(Stage::IDLE_ALT, 0);
}

void HTI301::on_disconnected() {
  // Also called for a device that was opened and turned out not to be the camera.
  const bool was_camera = this->stage_ != Stage::IDLE;
  this->stop_stream_();
  this->stage_ = Stage::IDLE;
  this->ctrl_gen_++;  // a request still in flight is abandoned
  this->ctrl_state_.store(CTRL_IDLE);
  this->publish_streaming_(false);
  if (was_camera)
    ESP_LOGI(TAG, "Camera disconnected");
  usb_host::USBClient::on_disconnected();
}

const char *HTI301::stage_name_(Stage s) {
  switch (s) {
    case Stage::IDLE_ALT:
      return "select alternate setting 0";
    case Stage::PROBE:
      return "probe stream parameters";
    case Stage::PROBE_READ:
      return "read negotiated stream parameters";
    case Stage::COMMIT:
      return "commit stream parameters";
    case Stage::SELECT_ALT:
      return "select the streaming alternate setting";
    case Stage::START:
      return "start the isochronous transfers";
    default:
      return "set-up";
  }
}

void HTI301::goto_stage_(Stage s, uint32_t delay_ms) {
  this->stage_ = s;
  this->stage_at_ = millis() + delay_ms;
  this->stage_sent_ = false;
}

void HTI301::fail_stage_(const char *what) {
  this->failures_++;
  const uint32_t delay = std::min<uint32_t>(30000, 1000u << std::min<uint8_t>(this->failures_, 5));
  ESP_LOGW(TAG, "Stream set-up failed (%s); retrying in %u s", what, static_cast<unsigned>(delay / 1000));
  this->status_set_warning();
  this->stop_stream_();
  this->goto_stage_(Stage::BACKOFF, delay);
}

void HTI301::run_stage_() {
  const uint32_t now = millis();
  switch (this->stage_) {
    case Stage::IDLE:
      return;
    case Stage::STREAMING: {
      const uint32_t last = this->last_frame_ms_.load();
      const uint32_t quiet = now - (last != 0 ? last : this->stream_start_ms_);
      const bool dead = this->inflight_.load() <= 0;
      if (dead || quiet > STALL_MS) {
        if (dead) {
          ESP_LOGW(TAG, "Every isochronous transfer has ended; restarting the stream");
        } else {
          ESP_LOGW(TAG, "No complete frame for %u ms; restarting the stream", static_cast<unsigned>(quiet));
        }
        this->restarts_++;
        this->status_set_warning();
        this->stop_stream_();
        this->goto_stage_(Stage::IDLE_ALT, DELAY_BEFORE_RESTART);
      }
      return;
    }
    case Stage::BACKOFF:
      if (static_cast<int32_t>(now - this->stage_at_) >= 0)
        this->goto_stage_(Stage::IDLE_ALT, 0);
      return;
    case Stage::START:
      if (static_cast<int32_t>(now - this->stage_at_) < 0)
        return;
      if (this->start_transfers_()) {
        this->stage_ = Stage::STREAMING;
        this->failures_ = 0;
        this->status_clear_warning();
      } else {
        this->fail_stage_(stage_name_(Stage::START));
      }
      return;
    default:
      break;
  }

  // Steps that send one control request, then wait for it.
  if (static_cast<int32_t>(now - this->stage_at_) < 0)
    return;
  const uint8_t cs = this->ctrl_state_.load();
  if (cs == CTRL_PENDING) {
    if (!this->ctrl_slow_logged_ && now - this->ctrl_sent_ms_ > 5000) {
      ESP_LOGW(TAG, "The camera has not answered a control request for 5 s (%s)", stage_name_(this->stage_));
      this->ctrl_slow_logged_ = true;
    }
    return;
  }
  if (!this->stage_sent_) {
    this->ctrl_state_.store(CTRL_IDLE);  // e.g. a command answered after its stream stopped
    if (!this->send_stage_request_()) {
      this->fail_stage_(stage_name_(this->stage_));
      return;
    }
    this->stage_sent_ = true;
    return;
  }
  this->ctrl_state_.store(CTRL_IDLE);
  if (cs == CTRL_FAILED) {
    this->fail_stage_(stage_name_(this->stage_));
    return;
  }
  this->stage_succeeded_(now);
}

bool HTI301::send_stage_request_() {
  const UvcLayout &l = this->layout_;
  const uint8_t vs = l.vs_interface;
  switch (this->stage_) {
    case Stage::IDLE_ALT:
      return this->send_control_(STD_INTERFACE_OUT, REQ_SET_INTERFACE, 0, vs, {});
    case Stage::PROBE: {
      Probe p;
      p.hint = 1;  // keep the frame interval
      p.format_index = l.format_index;
      p.frame_index = l.frame_index;
      p.frame_interval = l.frame_interval;
      std::vector<uint8_t> buf(l.probe_size());
      encode_probe(p, buf.data(), buf.size());
      return this->send_control_(CLASS_OUT, REQ_SET_CUR, VS_PROBE, vs, std::move(buf));
    }
    case Stage::PROBE_READ:
      return this->send_control_(CLASS_IN, REQ_GET_CUR, VS_PROBE, vs, std::vector<uint8_t>(l.probe_size()));
    case Stage::COMMIT:
      return this->send_control_(CLASS_OUT, REQ_SET_CUR, VS_COMMIT, vs,
                                 std::vector<uint8_t>(this->probe_buf_, this->probe_buf_ + l.probe_size()));
    case Stage::SELECT_ALT: {
      this->alt_ = l.alt_for(this->probe_.max_payload);
      if (this->alt_ == nullptr) {
        ESP_LOGE(TAG, "No streaming alternate setting carries %u bytes per interval",
                 static_cast<unsigned>(this->probe_.max_payload));
        return false;
      }
      const esp_err_t err = usb_host_interface_claim(this->handle_, this->device_handle_, vs, this->alt_->alt);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot claim interface %u alternate setting %u: %s", vs, this->alt_->alt, esp_err_to_name(err));
        return false;
      }
      this->vs_claimed_ = true;
      return this->send_control_(STD_INTERFACE_OUT, REQ_SET_INTERFACE, this->alt_->alt, vs, {});
    }
    default:
      return false;
  }
}

void HTI301::stage_succeeded_(uint32_t now) {
  switch (this->stage_) {
    case Stage::IDLE_ALT:
      this->goto_stage_(Stage::PROBE, DELAY_AFTER_IDLE_ALT);
      break;
    case Stage::PROBE:
      this->goto_stage_(Stage::PROBE_READ, DELAY_AFTER_PROBE);
      break;
    case Stage::PROBE_READ: {
      const UvcLayout &l = this->layout_;
      Probe got;
      if (this->ctrl_resp_len_ < l.probe_size() || !decode_probe(this->ctrl_resp_, this->ctrl_resp_len_, &got)) {
        this->fail_stage_("short probe reply");
        return;
      }
      const size_t expect = static_cast<size_t>(l.width) * l.height * 2;
      if (got.max_frame_size == 0)
        got.max_frame_size = l.max_frame_size;
      ESP_LOGD(TAG, "Negotiated: format %u, frame %u, interval %u, %u-byte frames, %u-byte payloads",
               got.format_index, got.frame_index, static_cast<unsigned>(got.frame_interval),
               static_cast<unsigned>(got.max_frame_size), static_cast<unsigned>(got.max_payload));
      if (got.format_index != l.format_index || got.frame_index != l.frame_index || got.max_frame_size != expect) {
        ESP_LOGE(TAG, "The camera negotiated format %u frame %u with %u-byte frames; expected %u, %u, %u bytes",
                 got.format_index, got.frame_index, static_cast<unsigned>(got.max_frame_size), l.format_index,
                 l.frame_index, static_cast<unsigned>(expect));
        this->fail_stage_("unexpected stream parameters");
        return;
      }
      std::memset(this->probe_buf_, 0, sizeof(this->probe_buf_));
      std::memcpy(this->probe_buf_, this->ctrl_resp_, l.probe_size());
      this->probe_ = got;
      this->goto_stage_(Stage::COMMIT, 0);
      break;
    }
    case Stage::COMMIT:
      this->goto_stage_(Stage::SELECT_ALT, DELAY_AFTER_COMMIT);
      break;
    case Stage::SELECT_ALT:
      this->goto_stage_(Stage::START, DELAY_AFTER_ALT);
      break;
    default:
      break;
  }
}

bool HTI301::send_control_(uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                           std::vector<uint8_t> data) {
  const uint32_t gen = ++this->ctrl_gen_;
  this->ctrl_state_.store(CTRL_PENDING);
  this->ctrl_sent_ms_ = millis();
  this->ctrl_slow_logged_ = false;
  // Completes on the USB task.
  const bool ok = this->control_transfer(
      type, request, value, index,
      [this, gen](const usb_host::TransferStatus &st) {
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
      data);
  if (!ok)
    this->ctrl_state_.store(CTRL_FAILED);
  return ok;
}

// ---------------------------------------------------------------- streaming

void HTI301::reset_camera_state_() {
  this->out_mode_.store(UNKNOWN);
  this->settle_.store(0);
  this->mode_set_ = this->range_set_ = this->palette_set_ = UNKNOWN;
  this->cmd_kind_ = Cmd::NONE;
  this->cmd_failures_ = 0;
  this->cmd_retry_ms_ = millis();
  std::fill(std::begin(this->gave_up_), std::end(this->gave_up_), UNKNOWN);
}

bool HTI301::start_transfers_() {
  const UvcLayout &l = this->layout_;
  this->width_ = l.width;
  this->image_rows_ = l.height - METADATA_ROWS;
  this->frame_bytes_ = this->probe_.max_frame_size;
  const size_t pixels = static_cast<size_t>(this->width_) * this->image_rows_;

  // Frame buffers, allocated on the first stream and kept.
  auto make_pool = [](size_t count, size_t bytes) {
    return std::make_unique<video::FramePool>(count, bytes, video::dma_alignment(), video::dma_allocator());
  };
  if (!this->raw_pool_)
    this->raw_pool_ = make_pool(RAW_FRAMES, this->frame_bytes_);
  if (!this->rad_sinks_.empty() && !this->rad_pool_)
    this->rad_pool_ = make_pool(RAD_FRAMES, pixels * sizeof(float));
  if (!this->img_sinks_.empty() && !this->img_pool_)
    this->img_pool_ = make_pool(IMG_FRAMES, pixels * 2);
  auto usable = [](const std::unique_ptr<video::FramePool> &p, size_t bytes) {
    return p == nullptr || (p->ok() && p->buffer_size() >= bytes);
  };
  if (!usable(this->raw_pool_, this->frame_bytes_) || !usable(this->rad_pool_, pixels * sizeof(float)) ||
      !usable(this->img_pool_, pixels * 2)) {
    ESP_LOGE(TAG, "No memory for %ux%u frame buffers", this->width_, this->image_rows_);
    return false;
  }

  // Isochronous transfers: interval in microframes is 2^(bInterval-1).
  const uint16_t mps = this->alt_->packet_size;
  const int interval = 1 << std::min<int>(this->alt_->interval > 0 ? this->alt_->interval - 1 : 0, 5);
  const int packets = std::max(1, std::min<int>(this->packets_per_transfer_, MAX_ISOC_SLOTS / interval));
  for (uint8_t i = 0; i < this->num_transfers_; i++) {
    usb_transfer_t *t;
    if (usb_host_transfer_alloc(static_cast<size_t>(packets) * mps, packets, &t) != ESP_OK) {
      ESP_LOGE(TAG, "No memory for %u isochronous transfers of %u bytes", this->num_transfers_, packets * mps);
      for (auto *x : this->transfers_)
        usb_host_transfer_free(x);
      this->transfers_.clear();
      return false;
    }
    t->device_handle = this->device_handle_;
    t->bEndpointAddress = this->alt_->endpoint;
    t->callback = HTI301::transfer_done_;
    t->context = this;
    t->num_bytes = packets * mps;
    for (int k = 0; k < packets; k++)
      t->isoc_packet_desc[k].num_bytes = mps;
    this->transfers_.push_back(t);
  }

  this->reset_camera_state_();
  this->assembler_.set_frame_size(this->frame_bytes_);
  this->assembling_ = this->raw_pool_->acquire();
  this->asm_buf_ = this->assembling_.data();
  this->asm_cap_ = this->raw_pool_->buffer_size();
  this->last_frame_ms_.store(0);
  this->stream_start_ms_ = millis();
  this->streaming_.store(true);
  for (auto *t : this->transfers_) {
    this->inflight_++;
    const esp_err_t err = usb_host_transfer_submit(t);
    if (err != ESP_OK) {
      this->inflight_--;
      ESP_LOGE(TAG, "Cannot submit an isochronous transfer: %s", esp_err_to_name(err));
      this->stop_stream_();
      return false;
    }
  }
  ESP_LOGI(TAG, "Streaming: alternate setting %u, %u-byte packets, %u transfers of %d packets", this->alt_->alt, mps,
           this->num_transfers_, packets);
  return true;
}

void HTI301::stop_stream_() {
  this->streaming_.store(false);  // completions stop resubmitting
  if (!this->transfers_.empty() && this->device_handle_ != nullptr && this->alt_ != nullptr) {
    usb_host_endpoint_halt(this->device_handle_, this->alt_->endpoint);
    usb_host_endpoint_flush(this->device_handle_, this->alt_->endpoint);
  }
  // Flushed transfers come back on the USB task (higher priority than this loop).
  const uint32_t t0 = millis();
  while (this->inflight_.load() > 0 && millis() - t0 < 500)
    vTaskDelay(pdMS_TO_TICKS(2));
  if (this->inflight_.load() > 0) {
    // Freeing them would let the controller write into freed memory.
    ESP_LOGE(TAG, "%d isochronous transfers did not come back; leaving them allocated", this->inflight_.load());
    this->status_set_error(LOG_STR("USB transfers stuck"));
    this->inflight_.store(0);
  } else {
    for (auto *t : this->transfers_)
      usb_host_transfer_free(t);
  }
  this->transfers_.clear();
  if (this->vs_claimed_ && this->device_handle_ != nullptr) {
    const esp_err_t err = usb_host_interface_release(this->handle_, this->device_handle_, this->layout_.vs_interface);
    if (err != ESP_OK)
      ESP_LOGW(TAG, "Cannot release the streaming interface: %s", esp_err_to_name(err));
  }
  this->vs_claimed_ = false;
  this->assembling_.reset();
  this->asm_buf_ = nullptr;
  this->reset_camera_state_();
}

void HTI301::transfer_done_(usb_transfer_t *t) { static_cast<HTI301 *>(t->context)->on_transfer_(t); }

// USB task.
void HTI301::on_transfer_(usb_transfer_t *t) {
  const usb_transfer_status_t st = t->status;
  const bool gone = st == USB_TRANSFER_STATUS_CANCELED || st == USB_TRANSFER_STATUS_NO_DEVICE;
  if (st == USB_TRANSFER_STATUS_COMPLETED) {
    const uint8_t *p = t->data_buffer;
    for (int i = 0; i < t->num_isoc_packets; i++) {
      const usb_isoc_packet_desc_t &d = t->isoc_packet_desc[i];
      if (d.status == USB_TRANSFER_STATUS_COMPLETED) {
        this->assembler_.feed(p, static_cast<size_t>(d.actual_num_bytes), this->asm_buf_, this->asm_cap_,
                              [this](bool complete, size_t) { this->frame_assembled_(complete); });
      } else {
        this->assembler_.lost();
        this->packets_lost_++;
      }
      p += d.num_bytes;
    }
  } else if (!gone) {
    this->assembler_.lost();
    this->transfer_errors_++;
  }
  if (this->streaming_.load() && !gone) {
    if (usb_host_transfer_submit(t) == ESP_OK)
      return;
    this->transfer_errors_++;
  }
  this->inflight_--;
}

// USB task, from the assembler. Must leave asm_buf_ at the next frame's buffer.
void HTI301::frame_assembled_(bool complete) {
  if (!complete) {
    this->incomplete_++;
  } else {
    this->frames_++;
    this->last_frame_ms_.store(millis());
    // Mode first, then the settle count: the main loop sets them the other way round.
    const uint8_t mode = this->out_mode_.load();
    uint8_t settle = this->settle_.load();
    while (settle > 0 && !this->settle_.compare_exchange_weak(settle, settle - 1)) {
    }
    if (settle > 0) {
      this->settled_++;
    } else if (mode != UNKNOWN && this->assembling_) {
      {
        std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
        if (this->pending_)
          this->replaced_++;  // the processing task is behind: keep only the newest
        this->pending_ = std::move(this->assembling_);
        this->pending_mode_ = mode;
        this->pending_ms_ = millis();
      }
      xTaskNotifyGive(this->task_);
    }
  }
  if (!this->assembling_) {
    this->assembling_ = this->raw_pool_->acquire();
    if (!this->assembling_)
      this->no_buffer_++;
  }
  this->asm_buf_ = this->assembling_ ? this->assembling_.data() : nullptr;
}

// ---------------------------------------------------------------- processing

void HTI301::task_entry_(void *arg) { static_cast<HTI301 *>(arg)->task_loop_(); }

void HTI301::task_loop_() {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    video::FrameRef raw;
    uint8_t mode;
    uint32_t ts;
    {
      std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
      raw = std::move(this->pending_);
      mode = this->pending_mode_;
      ts = this->pending_ms_;
    }
    if (!raw)
      continue;
    const uint32_t t0 = micros();
    if (mode == static_cast<uint8_t>(Mode::RADIOMETRIC)) {
      this->process_radiometric_(raw, ts);
    } else {
      this->process_image_(raw, ts);
    }
    this->process_us_ += micros() - t0;
    this->processed_++;
  }
}

void HTI301::process_radiometric_(const video::FrameRef &raw, uint32_t ts) {
  if (this->rad_pool_ == nullptr || !this->rad_sinks_.any_wants())
    return;
  const auto *words = reinterpret_cast<const uint16_t *>(raw.data());
  const Metadata m = parse_metadata(words, this->width_, this->image_rows_);
  video::RadiometricFrame f;
  f.buffer = this->rad_pool_->acquire();
  if (!f.buffer) {
    this->no_buffer_++;
    return;
  }
  const Environment want{this->env_emissivity_.load(), this->env_reflected_.load(), this->env_ambient_.load(),
                         this->env_humidity_.load(),   this->env_distance_.load(),  NAN};
  const Environment env = resolve_environment(want, m);
  if (!raw_to_temperatures(words, static_cast<size_t>(this->width_) * this->image_rows_, m, env, this->table_,
                           f.temps())) {
    this->bad_calibration_++;
    return;
  }
  f.width = this->width_;
  f.height = this->image_rows_;
  f.timestamp_ms = ts;
  f.sequence = ++this->sequence_;
  this->rad_sinks_.deliver(f, [](video::RadiometricSink *s, const video::RadiometricFrame &fr) {
    s->on_radiometric_frame(fr);
  });
}

void HTI301::process_image_(const video::FrameRef &raw, uint32_t ts) {
  if (this->img_pool_ == nullptr || !this->img_sinks_.any_wants())
    return;
  video::ImageFrame f;
  f.buffer = this->img_pool_->acquire();
  if (!f.buffer) {
    this->no_buffer_++;
    return;
  }
  f.width = this->width_;
  f.height = this->image_rows_;
  f.stride = static_cast<uint32_t>(this->width_) * 2;
  f.format = video::ImageFormat::RGB565_LE;
  f.timestamp_ms = ts;
  f.sequence = ++this->sequence_;
  yuyv_to_rgb565(raw.data(), this->width_, this->image_rows_, f.buffer.data(), f.stride);
  this->img_sinks_.deliver(f, [](video::ImageSink *s, const video::ImageFrame &fr) { s->on_image_frame(fr); });
}

// ---------------------------------------------------------------- commands

const char *HTI301::cmd_name_(Cmd kind) {
  switch (kind) {
    case Cmd::MODE:
      return "mode";
    case Cmd::RANGE:
      return "range";
    case Cmd::PALETTE:
      return "palette";
    case Cmd::CALIBRATE:
      return "calibrate";
    default:
      return "?";
  }
}

void HTI301::calibrate() {
  if (this->stage_ != Stage::STREAMING) {
    ESP_LOGW(TAG, "Calibration not sent: the camera is not streaming");
    return;
  }
  this->calibrate_want_ = true;
}

bool HTI301::send_command_(Cmd kind, uint8_t value, uint16_t code) {
  this->cmd_kind_ = kind;
  this->cmd_value_ = value;
  ESP_LOGD(TAG, "Sending %s command 0x%04X", cmd_name_(kind), code);
  return this->send_control_(CLASS_OUT, REQ_SET_CUR, CT_ZOOM_ABSOLUTE, (ZOOM_ENTITY << 8) | this->layout_.vc_interface,
                             {static_cast<uint8_t>(code & 0xFF), static_cast<uint8_t>(code >> 8)});
}

void HTI301::command_succeeded_(Cmd kind, uint8_t value) {
  switch (kind) {
    case Cmd::MODE:
      this->settle_.store(SETTLE_FRAMES);  // before the mode: see frame_assembled_()
      this->out_mode_.store(value);
      this->mode_set_ = value;
      this->palette_set_ = UNKNOWN;  // send the palette again in processed mode
      ESP_LOGI(TAG, "Camera mode: %s", mode_name(value));
      break;
    case Cmd::RANGE:
      this->settle_.store(SETTLE_FRAMES);
      this->range_set_ = value;
      ESP_LOGI(TAG, "Camera range: %s", range_name(value));
      break;
    case Cmd::PALETTE:
      this->palette_set_ = value;
      ESP_LOGI(TAG, "Camera palette: %s", PALETTE_NAMES[value]);
      break;
    case Cmd::CALIBRATE:
      this->calibrate_want_ = false;
      ESP_LOGI(TAG, "Calibration sent");
      break;
    default:
      break;
  }
}

void HTI301::run_commands_() {
  if (this->stage_ != Stage::STREAMING)
    return;
  const uint8_t cs = this->ctrl_state_.load();
  if (cs == CTRL_PENDING)
    return;
  const uint32_t now = millis();
  if (this->cmd_kind_ != Cmd::NONE) {
    const Cmd kind = this->cmd_kind_;
    this->cmd_kind_ = Cmd::NONE;
    this->ctrl_state_.store(CTRL_IDLE);
    if (cs == CTRL_OK) {
      this->cmd_failures_ = 0;
      this->command_succeeded_(kind, this->cmd_value_);
    } else if (++this->cmd_failures_ < COMMAND_ATTEMPTS) {
      ESP_LOGW(TAG, "The camera rejected the %s command; trying again", cmd_name_(kind));
      this->cmd_retry_ms_ = now + 1000;
    } else {
      ESP_LOGE(TAG, "The camera rejected the %s command %u times; not sending it again until the setting changes",
               cmd_name_(kind), COMMAND_ATTEMPTS);
      this->cmd_failures_ = 0;
      this->gave_up_[static_cast<uint8_t>(kind)] = this->cmd_value_;
      if (kind == Cmd::CALIBRATE)
        this->calibrate_want_ = false;
      this->status_set_warning();
    }
    return;
  }
  if (static_cast<int32_t>(now - this->cmd_retry_ms_) < 0)
    return;
  auto wanted = [this](Cmd k, uint8_t set, uint8_t want) {
    return set != want && this->gave_up_[static_cast<uint8_t>(k)] != want;
  };
  if (wanted(Cmd::MODE, this->mode_set_, this->mode_want_)) {
    this->send_command_(Cmd::MODE, this->mode_want_,
                        this->mode_want_ == static_cast<uint8_t>(Mode::PROCESSED) ? CMD_PROCESSED_MODE
                                                                                   : CMD_RADIOMETRIC_MODE);
  } else if (wanted(Cmd::RANGE, this->range_set_, this->range_want_)) {
    this->send_command_(Cmd::RANGE, this->range_want_,
                        this->range_want_ == static_cast<uint8_t>(Range::HIGH) ? CMD_RANGE_HIGH : CMD_RANGE_NORMAL);
  } else if (this->mode_set_ == static_cast<uint8_t>(Mode::PROCESSED) &&
             wanted(Cmd::PALETTE, this->palette_set_, this->palette_want_)) {
    this->send_command_(Cmd::PALETTE, this->palette_want_, CMD_PALETTE_FIRST + this->palette_want_);
  } else if (this->calibrate_want_) {
    this->send_command_(Cmd::CALIBRATE, 0, CMD_CALIBRATE);
  }
}

// ---------------------------------------------------------------- reporting

void HTI301::publish_streaming_(bool on) {
#ifdef USE_BINARY_SENSOR
  if (this->streaming_sensor_ == nullptr || (this->streaming_state_known_ && this->streaming_published_ == on))
    return;
  this->streaming_sensor_->publish_state(on);
  this->streaming_published_ = on;
  this->streaming_state_known_ = true;
#endif
}

void HTI301::report_(uint32_t now) {
  if (this->last_report_ms_ == 0) {
    this->last_report_ms_ = now;
    return;
  }
  const uint32_t elapsed = now - this->last_report_ms_;
  if (elapsed < 10000)
    return;
  this->last_report_ms_ = now;
  const uint32_t fr = this->frames_.exchange(0), inc = this->incomplete_.exchange(0), set = this->settled_.exchange(0),
                 rep = this->replaced_.exchange(0), lost = this->packets_lost_.exchange(0),
                 err = this->transfer_errors_.exchange(0), nob = this->no_buffer_.exchange(0),
                 bad = this->bad_calibration_.exchange(0), us = this->process_us_.exchange(0),
                 n = this->processed_.exchange(0);
  if (this->stage_ != Stage::STREAMING)
    return;
  const float s = elapsed / 1000.0f;
  if (bad != 0)
    ESP_LOGW(TAG, "%u frames had unusable calibration data", static_cast<unsigned>(bad));
  if (this->log_timing_ || lost != 0 || err != 0) {
    ESP_LOGI(TAG,
             "Over %.1f s: %.2f frames/s, %u incomplete, %u skipped after a setting change, %u superseded, "
             "%u packets lost, %u transfer errors, %u without a buffer; processing %.2f ms/frame; %u restarts",
             s, fr / s, static_cast<unsigned>(inc), static_cast<unsigned>(set), static_cast<unsigned>(rep),
             static_cast<unsigned>(lost), static_cast<unsigned>(err), static_cast<unsigned>(nob),
             n ? us / 1000.0f / n : 0.0f, static_cast<unsigned>(this->restarts_));
  }
}

}  // namespace hti301
}  // namespace esphome
