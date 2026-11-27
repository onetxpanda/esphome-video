#include "thermal_master_p1.h"

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
namespace thermal_master_p1 {

static const char *const TAG = "thermal_master_p1";

// Control requests.
static constexpr uint8_t TYPE_VENDOR_OUT = 0x41;         // vendor, interface, host to device
static constexpr uint8_t TYPE_VENDOR_IN = 0xC1;          // vendor, interface, device to host
static constexpr uint8_t TYPE_VENDOR_DEVICE_OUT = 0x40;  // vendor, device, host to device
static constexpr uint8_t TYPE_STD_INTERFACE_OUT = 0x01;
static constexpr uint8_t REQ_COMMAND = 0x20;
static constexpr uint8_t REQ_RESPONSE = 0x21;
static constexpr uint8_t REQ_STATUS = 0x22;
static constexpr uint8_t REQ_STREAM_ON = 0xEE;
static constexpr uint8_t REQ_SET_INTERFACE = 0x0B;
static constexpr uint8_t STREAM_INTERFACE = 1;
static constexpr uint8_t STREAM_ALT = 1;
static constexpr uint8_t BULK_IN_EP = 0x81;
// A whole frame (start marker and rows) per transfer, as the vendor app reads
// it, rounded up to whole 512-byte packets: the rows end with a short packet,
// which ends the transfer, and the end marker comes as a transfer of its own.
static constexpr size_t TRANSFER_BYTES = (BODY_BYTES + 511) / 512 * 512;  // 77,824
static constexpr size_t RAW_FRAMES = 3;  // assembling, waiting, processing
static constexpr size_t RAD_FRAMES = 4;  // stages after this one may hold frames
static constexpr size_t IMG_FRAMES = 4;
// The vendor library's time allowed for one control transfer.
static constexpr uint32_t CTRL_TIMEOUT_MS = 2000;
// A status read the camera holds until it is done may take the command's
// whole budget; then this much more.
static constexpr uint32_t STATUS_SLACK_MS = 1000;
// While the status says busy, it is read again no sooner than this.
static constexpr uint32_t STATUS_POLL_GAP_MS = 5;
// After a range change the library reads the camera's state every second
// while it says the change is under way.
static constexpr int RANGE_SETTLE_TRIES = 30;
// The first frame comes about 1.5 s after the last start command; then one
// every 40 ms, with short gaps while the shutter is closed.
static constexpr uint32_t FIRST_FRAME_TIMEOUT_MS = 5000;
// A shutter cycle pauses the camera's frames for several seconds (a shutter
// cycle sent right after each start was followed by a pause of over 3 s every
// time); only a longer silence is taken as the stream having stopped.
static constexpr uint32_t FRAME_TIMEOUT_MS = 10000;
static constexpr uint32_t PAUSE_LOG_MS = 1000;  // pauses at least this long are logged
// restart_usb(): how long the port stays off at least, how often the power-on
// is retried, how long the bus must be empty first, and when to give up.
static constexpr uint32_t USB_OFF_MS = 1000;
static constexpr uint32_t USB_RETRY_MS = 100;
static constexpr uint32_t USB_SETTLE_MS = 100;
static constexpr uint32_t USB_RESTART_TIMEOUT_MS = 10000;
static constexpr size_t MAX_QUEUED_JOBS = 64;

// ---------------------------------------------------------------- entities

void SettingNumber::control(float value) {
  this->parent_->write_setting(this->setting_, lroundf((value - this->offset_) * this->scale_));
}

void SettingPartNumber::control(float value) {
  this->parent_->write_setting_part(this->setting_, this->part_, lroundf(value));
}

void SettingSelect::control(size_t index) {
  if (index < this->values_.size())
    this->parent_->write_setting(this->setting_, this->values_[index]);
}

void SettingSelect::show(int32_t raw) {
  if (raw == UNKNOWN)
    return;
  for (size_t i = 0; i < this->values_.size(); i++) {
    if (this->values_[i] == raw) {
      this->publish_state(i);
      return;
    }
  }
  ESP_LOGW(TAG, "The camera reports %s %d, which is not one of the options", setting_info(this->setting_).name,
           static_cast<int>(raw));
}

void SettingSwitch::write_state(bool state) { this->parent_->write_setting(this->setting_, state ? 1 : 0); }

void SettingSwitch::show(int32_t raw) {
  if (raw == 0 || raw == 1)
    this->publish_state(raw == 1);
  else if (raw != UNKNOWN)
    ESP_LOGW(TAG, "The camera reports %s %d, neither off nor on", setting_info(this->setting_).name,
             static_cast<int>(raw));
}

// ---------------------------------------------------------------- set-up

ThermalMasterP1::ThermalMasterP1(uint16_t vid, uint16_t pid) : usb_host::USBClient(vid, pid) {
  for (auto &k : this->known_)
    k = UNKNOWN;
  // As the vendor app does at every start: the camera's own shutter cycles
  // and its sun detection on (the configuration can say otherwise).
  this->set_initial_setting(Setting::AUTO_SHUTTER, 1);
  this->set_initial_setting(Setting::SUN_DETECT, 1);
}

void ThermalMasterP1::set_initial_setting(Setting s, int32_t v) {
  this->initial_[static_cast<size_t>(s)] = v;
  this->initial_set_[static_cast<size_t>(s)] = true;
  if (s == Setting::RANGE)
    this->auto_range_ = v == RANGE_AUTO;
}

void ThermalMasterP1::setup() {
  usb_host::USBClient::setup();
  if (this->is_failed())
    return;
  // 8 KB: sinks (radiometric processing) run on this task.
  if (xTaskCreatePinnedToCoreWithCaps(ThermalMasterP1::task_entry_, "tm_p1", 8192, this, 4, &this->task_,
                                      tskNO_AFFINITY, TASK_STACK_CAPS) != pdPASS) {
    this->mark_failed(LOG_STR("Cannot start the processing task"));
    return;
  }
  this->publish_streaming_(false);
  this->set_status_("No camera");
}

void ThermalMasterP1::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Thermal Master P1:\n"
                "  %u bulk transfers of %u bytes\n"
                "  Correction: x %.3f %+.2f C\n"
                "  Readings every %u s; heartbeat %s; the camera's point %u, %u",
                this->num_transfers_, static_cast<unsigned>(TRANSFER_BYTES), this->gain_.load(), this->offset_.load(),
                static_cast<unsigned>(this->poll_ms_ / 1000), this->heartbeat_ms_ ? "on" : "off", this->point_x_,
                this->point_y_);
  for (size_t i = 0; i < static_cast<size_t>(Setting::COUNT); i++)
    if (this->initial_set_[i])
      ESP_LOGCONFIG(TAG, "  At every start: %s = %d%s", setting_info(static_cast<Setting>(i)).name,
                    static_cast<int>(this->initial_[i]),
                    i == static_cast<size_t>(Setting::RANGE) && this->initial_[i] == RANGE_AUTO ? " (automatic)" : "");
  if (!this->model_.empty())
    ESP_LOGCONFIG(TAG, "  Model: %s, firmware %s, system %s", this->model_.c_str(), this->firmware_.c_str(),
                  this->system_version_.c_str());
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Streaming", this->streaming_sensor_);
#endif
  LOG_TEXT_SENSOR("  ", "Status", this->status_sensor_);
}

// ---------------------------------------------------------------- jobs

// The vendor library's handshake: the command; a write's status read until
// it is not busy, which must then be 3; a read's status (2), its response and
// the status again (3); a reset's nothing.
void ThermalMasterP1::add_command_(std::vector<Step> &v, const Command &c) {
  v.push_back({Op::COMMAND, c});
  if (c.no_status)
    return;
  const uint16_t n = c.response_bytes();
  if (n > 0) {
    v.push_back({Op::STATUS, {}, 0, c.budget_ms, STATUS_READY});
    v.push_back({Op::RESPONSE, {}, n});
    v.push_back({Op::STATUS, {}, 0, c.budget_ms, STATUS_DONE});
  } else {
    v.push_back({Op::STATUS, {}, 0, c.budget_ms, STATUS_DONE});
  }
}

ThermalMasterP1::Job ThermalMasterP1::make_job_(std::string what, const Command &c, Done done) {
  Job j;
  j.what = std::move(what);
  add_command_(j.steps, c);
  j.done = std::move(done);
  return j;
}

bool ThermalMasterP1::queue_(Job j) {
  if (this->jobs_.size() >= MAX_QUEUED_JOBS) {
    ESP_LOGW(TAG, "Too many camera commands queued; not sent: %s", j.what.c_str());
    return false;
  }
  this->jobs_.push_back(std::move(j));
  return true;
}

static std::string text(const std::vector<uint8_t> &r) {
  std::string s;
  for (uint8_t c : r) {
    if (c == 0)
      break;
    if (c >= 0x20 && c < 0x7F)
      s += static_cast<char>(c);
  }
  return s;
}

static std::string hex(const std::vector<uint8_t> &r) {
  std::string s;
  char b[4];
  for (uint8_t c : r) {
    std::snprintf(b, sizeof(b), "%02X", c);
    s += b;
  }
  return s;
}

// A string up to its first zero, if it is printable; else the bytes in hex.
static std::string text_or_hex(const std::vector<uint8_t> &r) {
  size_t n = 0;
  while (n < r.size() && r[n] != 0)
    n++;
  for (size_t i = 0; i < n; i++)
    if (r[i] < 0x20 || r[i] >= 0x7F)
      return hex(r);
  return std::string(r.begin(), r.begin() + n);
}

// The start-up sequence of the vendor's Windows tool, with its delays
// (without them the camera takes every command and sends nothing): stop
// streaming (left on by an earlier start), the camera's name and versions,
// the range read, the streaming interface, the vendor request 0xEE, bulk
// reads waiting, the range read again, then the range (the high one unless
// configured otherwise). The vendor's Android app does less: the streaming
// interface, 0xEE and the bulk reads, with no delays.
void ThermalMasterP1::queue_init_() {
  this->jobs_.clear();
  this->step_ = 0;
  this->step_sent_ = false;
  this->status_polling_ = false;
  auto critical = [this](Job j) {
    j.critical = true;
    this->jobs_.push_back(std::move(j));
  };
  Job stop;
  stop.what = "stop streaming";
  stop.steps = {{Op::IDLE_ALT}, {Op::WAIT, {}, 0, 100}};
  critical(std::move(stop));
  critical(this->make_job_("read the model", command(Cmd::READ_MODEL),
                           [this](bool, const std::vector<uint8_t> &r) { this->model_ = text(r); }));
  critical(this->make_job_("read the firmware version", command(Cmd::READ_FIRMWARE),
                           [this](bool, const std::vector<uint8_t> &r) { this->firmware_ = text(r); }));
  critical(this->make_job_("read the system version", command(Cmd::READ_SYSTEM_VERSION),
                           [this](bool, const std::vector<uint8_t> &r) { this->system_version_ = text(r); }));
  auto range_answer = [this](bool, const std::vector<uint8_t> &r) {
    this->start_answer_ = r.empty() ? 0 : r[0];
    this->note_range_(r.empty() ? UNKNOWN : r[0]);
  };
  critical(this->make_job_("read the range", command(Cmd::READ_RANGE), range_answer));
  Job iface;
  iface.what = "start the stream";
  iface.steps = {{Op::WAIT, {}, 0, 1000}, {Op::CLAIM_STREAM}, {Op::STREAM_ON}, {Op::WAIT, {}, 0, 2000},
                 {Op::START_TRANSFERS}};
  critical(std::move(iface));
  critical(this->make_job_("read the range again", command(Cmd::READ_RANGE), range_answer));
  const size_t ri = static_cast<size_t>(Setting::RANGE);
  int32_t range = this->initial_set_[ri] ? this->initial_[ri] : 1;
  if (range == RANGE_AUTO)
    range = 1;  // the votes take it from there
  // The range is written only when the camera is not in it already: a range
  // change makes the camera reinitialise its measurement (the vendor app gives
  // it 22 s), and the app never sends one on connecting.
  Job started;
  started.what = "start";
  started.steps = {{Op::WAIT, {}, 0, 0}};
  started.done = [this, range](bool, const std::vector<uint8_t> &) {
    ESP_LOGI(TAG, "Camera started: model \"%s\", firmware %s, system %s; range read 0x%02X", this->model_.c_str(),
             this->firmware_.c_str(), this->system_version_.c_str(), this->start_answer_);
    if (this->model_ != "P1")
      ESP_LOGW(TAG, "The camera calls itself \"%s\", not P1; frames are read as a P1's", this->model_.c_str());
    this->stage_ = Stage::STREAM;
    this->stream_started_ms_ = millis();
    this->set_status_("Starting the stream");
    if (this->range_now_.load() != range) {
      ESP_LOGI(TAG, "Setting the range: %s", range == 1 ? "high" : "low");
      this->range_changing_ = true;
      this->jobs_.push_back(this->make_job_("set the range", setting_write(Setting::RANGE, range),
                                            [this, range](bool ok, const std::vector<uint8_t> &) {
                                              if (ok)
                                                this->note_range_(range);
                                              this->queue_range_settle_(0);
                                            }));
    } else {
      this->range_changing_ = false;
    }
    this->queue_after_start_();
  };
  critical(std::move(started));
}

// After a start: the configured settings, what the entities show, the
// camera's limits and texts, the heartbeat, the first readings.
void ThermalMasterP1::queue_after_start_() {
  if (this->shutter_after_start_) {
    // A shutter cycle (offset update, the vendor app's shutter button) first:
    // after a start the temperatures read several degrees low until the
    // camera's own next shutter cycle. Queued directly, so a full queue
    // cannot drop it.
    ESP_LOGI(TAG, "Command: %s (after the start)", action_name(Action::SHUTTER));
    this->jobs_.push_back(this->make_job_(action_name(Action::SHUTTER), action_command(Action::SHUTTER)));
  }
  this->publish_text_(this->info_texts_[static_cast<size_t>(Info::NAME)], this->model_);
  this->publish_text_(this->info_texts_[static_cast<size_t>(Info::FIRMWARE)], this->firmware_);
  this->publish_text_(this->info_texts_[static_cast<size_t>(Info::SYSTEM_VERSION)], this->system_version_);
  for (size_t i = 0; i < static_cast<size_t>(Setting::COUNT); i++) {
    const Setting s = static_cast<Setting>(i);
    if (s == Setting::RANGE)
      continue;  // set by the start-up sequence, read once it has settled
    // What the frames' temperatures were computed with (their emissivity and
    // reflected temperature, for spots of another material) is read at every
    // start, entity or not.
    const bool for_frames = s == Setting::ENV_CORRECTION || s == Setting::EMISSIVITY || s == Setting::REFLECTED;
    if (this->initial_set_[i]) {
      this->write_setting(s, this->initial_[i]);
    } else if (this->entities_[i][0] != nullptr || for_frames) {
      if (setting_info(s).get != 0)
        this->queue_setting_read_(s);
      else
        this->show_setting_(s, UNKNOWN);
    }
    if (this->entities_[i][0] != nullptr && has_attributes(s)) {
      this->queue_(this->make_job_(std::string("read the limits of ") + setting_info(s).name, attribute_read(s),
                                   [s](bool ok, const std::vector<uint8_t> &r) {
                                     Attributes a{};
                                     if (ok && parse_attributes(r.data(), r.size(), &a))
                                       ESP_LOGI(TAG, "%s: the camera takes %u..%u, step %u", setting_info(s).name,
                                                a.min, a.max, a.step);
                                   }));
    }
  }
  for (uint8_t t = 1; t <= static_cast<uint8_t>(Info::LAST); t++) {
    const Info info = static_cast<Info>(t);
    text_sensor::TextSensor *ts = this->info_texts_[t];
    if (ts == nullptr || info == Info::NAME || info == Info::FIRMWARE || info == Info::SYSTEM_VERSION)
      continue;
    this->queue_(this->make_job_("read device information", info_command(info),
                                 [this, ts](bool ok, const std::vector<uint8_t> &r) {
                                   if (ok)
                                     this->publish_text_(ts, text_or_hex(r));
                                 }));
  }
  if (this->palettes_text_ != nullptr)
    this->queue_palettes_();
  if (this->heartbeat_ms_ != 0) {
    this->queue_(this->make_job_("start the heartbeat", action_command(Action::HEARTBEAT_START)));
    this->last_heartbeat_ms_ = millis();
  }
  this->queue_polls_();
  this->last_poll_ms_ = millis();
}

void ThermalMasterP1::queue_polls_() {
  for (Reading r : {Reading::DEVICE_TEMPERATURE, Reading::POWERED_TIME, Reading::DEVICE_STATUS,
                    Reading::SHUTTER_STATUS, Reading::CRG_VALUE, Reading::SENSOR_VTEMP, Reading::STREAM_SOURCE,
                    Reading::DIGITAL_VIDEO_OUTPUT, Reading::SYSTEM_DATA_FLAG, Reading::CURSOR_POSITION,
                    Reading::CAMERA_POINT}) {
    const size_t i = static_cast<size_t>(r);
    if (this->reading_sensors_[i] != nullptr || this->reading_texts_[i] != nullptr)
      this->queue_reading_(r);
  }
  if (this->camera_max_ != nullptr || this->camera_min_ != nullptr)
    this->queue_reading_(Reading::CAMERA_FRAME);
  this->publish_info_rows_();
}

void ThermalMasterP1::queue_reading_(Reading r) {
  this->queue_(this->make_job_("a reading", reading_command(r, this->point_x_, this->point_y_),
                               [this, r](bool ok, const std::vector<uint8_t> &resp) {
                                 if (ok) {
                                   this->publish_reading_(r, resp);
                                   return;
                                 }
                                 if (this->reading_sensors_[static_cast<size_t>(r)] != nullptr)
                                   this->reading_sensors_[static_cast<size_t>(r)]->publish_state(NAN);
                                 if (r == Reading::CAMERA_FRAME) {
                                   if (this->camera_max_ != nullptr)
                                     this->camera_max_->publish_state(NAN);
                                   if (this->camera_min_ != nullptr)
                                     this->camera_min_->publish_state(NAN);
                                 }
                               }));
}

void ThermalMasterP1::show_setting_(Setting s, int32_t v) {
  for (CameraSetting *e : this->entities_[static_cast<size_t>(s)])
    if (e != nullptr)
      e->show(s == Setting::RANGE && this->auto_range_ ? RANGE_AUTO : v);
}

void ThermalMasterP1::queue_setting_read_(Setting s) {
  if (setting_info(s).get == 0)
    return;
  this->queue_(this->make_job_(std::string("read ") + setting_info(s).name, setting_read(s),
                               [this, s](bool ok, const std::vector<uint8_t> &r) {
                                 const int32_t v = ok ? setting_value(s, r.data(), r.size()) : UNKNOWN;
                                 ESP_LOGD(TAG, "%s: %d", setting_info(s).name, static_cast<int>(v));
                                 this->known_[static_cast<size_t>(s)] = v;
                                 if (s == Setting::RANGE)
                                   this->note_range_(v);
                                 this->show_setting_(s, v);
                               }));
}

void ThermalMasterP1::note_range_(int32_t v) {
  if (v != 0 && v != 1)
    return;
  this->range_now_ = v;
  this->known_[static_cast<size_t>(Setting::RANGE)] = v;
  this->publish_text_(this->range_text_, v == 1 ? "High (-4 to 302 °F)" : "Low (302 to 1112 °F)");
}

// After a range change: the camera's state every second while it says the
// change is under way (the library's automatic switching does this), then the
// range read back.
void ThermalMasterP1::queue_range_settle_(int tries) {
  Job j;
  j.what = "wait for the range change";
  if (tries > 0)
    j.steps.push_back({Op::WAIT, {}, 0, 1000});
  add_command_(j.steps, reading_command(Reading::GAIN_SWITCH_STATE));
  j.done = [this, tries](bool ok, const std::vector<uint8_t> &r) {
    const bool changing = ok && !r.empty() && r[0] == 2;
    if (changing && tries < RANGE_SETTLE_TRIES) {
      this->queue_range_settle_(tries + 1);
      return;
    }
    if (changing)
      ESP_LOGW(TAG, "The camera still says the range is changing after %d s", RANGE_SETTLE_TRIES);
    Job read = this->make_job_("read the range", setting_read(Setting::RANGE),
                               [this](bool ok, const std::vector<uint8_t> &resp) {
                                 const int32_t v = ok ? setting_value(Setting::RANGE, resp.data(), resp.size()) : UNKNOWN;
                                 if (v == 0 || v == 1) {
                                   ESP_LOGI(TAG, "Range: %s", v == 1 ? "high" : "low");
                                   this->note_range_(v);
                                 }
                                 this->show_setting_(Setting::RANGE, v);
                                 this->range_changing_ = false;
                               });
    this->jobs_.push_back(std::move(read));  // not dropped when the queue is full: it ends the change
  };
  this->jobs_.push_back(std::move(j));
}

// The camera's palettes: the count, then each name.
void ThermalMasterP1::queue_palettes_() {
  this->queue_(this->make_job_("read the palette count", palette_count_command(),
                               [this](bool ok, const std::vector<uint8_t> &r) {
                                 if (!ok || r.empty())
                                   return;
                                 const uint8_t n = std::min<uint8_t>(r[0], 64);
                                 this->palettes_.clear();
                                 if (n == 0)
                                   this->publish_text_(this->palettes_text_, "None");
                                 for (uint8_t i = 0; i < n; i++) {
                                   this->queue_(this->make_job_(
                                       "read a palette name", palette_name_command(i),
                                       [this, i, n](bool ok, const std::vector<uint8_t> &name) {
                                         char b[8];
                                         std::snprintf(b, sizeof(b), "%s%u ", i ? ", " : "", i);
                                         this->palettes_ += b;
                                         this->palettes_ += ok ? text_or_hex(name) : std::string("?");
                                         if (i + 1 == n)
                                           this->publish_text_(this->palettes_text_, this->palettes_);
                                       }));
                                 }
                               }));
}

void ThermalMasterP1::write_setting(Setting s, int32_t v) {
  const size_t i = static_cast<size_t>(s);
  const SettingInfo &info = setting_info(s);
  if (s == Setting::RANGE && v == RANGE_AUTO) {
    this->initial_[i] = RANGE_AUTO;
    this->initial_set_[i] = true;
    if (!this->auto_range_) {
      ESP_LOGI(TAG, "Range: automatic");
      this->auto_range_ = true;
      this->auto_reset_ = true;
    }
    this->show_setting_(s, RANGE_AUTO);
    return;
  }
  if (s == Setting::RANGE)
    this->auto_range_ = false;
  if (s == Setting::ISOTHERMAL_LIMIT) {
    // The library's limits for the range the camera is in (whole degrees C).
    int32_t lo = pair_part(s, v, 0), hi = pair_part(s, v, 1);
    if (this->range_now_ == 1) {
      lo = std::max<int32_t>(lo, -70);
      hi = std::min<int32_t>(hi, 200);
    } else if (this->range_now_ == 0) {
      lo = std::max<int32_t>(lo, -50);
      hi = std::min<int32_t>(hi, 650);
    }
    v = pack_pair(s, lo, hi);
  }
  v = setting_clamp(s, v);
  // Kept for every later start, too.
  this->initial_[i] = v;
  this->initial_set_[i] = true;
  if (this->stage_ != Stage::STREAM) {
    ESP_LOGI(TAG, "%s = %d when the camera next starts", info.name, static_cast<int>(v));
    return;
  }
  ESP_LOGI(TAG, "Setting %s = %d", info.name, static_cast<int>(v));
  if (s == Setting::RANGE)
    this->range_changing_ = true;
  const bool queued = this->queue_(this->make_job_(
      std::string("set ") + info.name, setting_write(s, v), [this, s, v](bool ok, const std::vector<uint8_t> &) {
        if (s == Setting::RANGE) {
          if (ok)
            this->note_range_(v);
          this->queue_range_settle_(0);  // reads the range back
        } else if (setting_info(s).get != 0) {
          this->queue_setting_read_(s);  // the entity shows what the camera then reports
        } else {
          this->known_[static_cast<size_t>(s)] = ok ? v : UNKNOWN;
          this->show_setting_(s, ok ? v : UNKNOWN);
        }
      }));
  if (!queued && s == Setting::RANGE)
    this->range_changing_ = false;
}

void ThermalMasterP1::write_setting_part(Setting s, int part, int32_t v) {
  const size_t i = static_cast<size_t>(s);
  const int32_t base =
      this->known_[i] != UNKNOWN ? this->known_[i] : (this->initial_set_[i] ? this->initial_[i] : UNKNOWN);
  if (base == UNKNOWN) {
    ESP_LOGW(TAG, "%s: the camera has not reported its other value yet; not set", setting_info(s).name);
    this->show_setting_(s, UNKNOWN);
    return;
  }
  const int32_t first = part == 0 ? v : pair_part(s, base, 0);
  const int32_t second = part == 1 ? v : pair_part(s, base, 1);
  this->write_setting(s, pack_pair(s, first, second));
}

void ThermalMasterP1::press(Action a) {
  if (this->stage_ != Stage::STREAM) {
    ESP_LOGW(TAG, "%s: the camera is not streaming; not sent", action_name(a));
    return;
  }
  if (action_is_maintenance(a))
    ESP_LOGW(TAG, "Maintenance command: %s", action_name(a));
  else
    ESP_LOGI(TAG, "Command: %s", action_name(a));
  this->queue_(this->make_job_(action_name(a), action_command(a)));
}

void ThermalMasterP1::press_custom(Custom c) {
  if (this->stage_ != Stage::STREAM) {
    ESP_LOGW(TAG, "The camera is not streaming; maintenance command not sent");
    return;
  }
  auto p = [this](Param x) { return this->params_[static_cast<size_t>(x)]; };
  auto u16 = [](float v) { return static_cast<uint16_t>(std::max(0.0f, std::min(65535.0f, std::round(v)))); };
  auto i16 = [](float v) { return static_cast<int16_t>(std::max(-32768.0f, std::min(32767.0f, std::round(v)))); };
  auto u32 = [](float v) { return static_cast<uint32_t>(std::max(0.0f, std::min(16777215.0f, std::round(v)))); };
  const uint16_t x = u16(p(Param::CURSOR_X)), y = u16(p(Param::CURSOR_Y));
  switch (c) {
    case Custom::CURSOR_MOVE:
      ESP_LOGW(TAG, "Maintenance command: cursor to %u, %u", x, y);
      this->queue_(this->make_job_("move the cursor", cursor_position(x, y)));
      break;
    case Custom::DPC_MARK:
      ESP_LOGW(TAG, "Maintenance command: pixel %u, %u marked dead", x, y);
      this->queue_(this->make_job_("mark a dead pixel", mark_dead_pixel(x, y)));
      break;
    case Custom::DPC_UNMARK:
      ESP_LOGW(TAG, "Maintenance command: pixel %u, %u marked not dead", x, y);
      this->queue_(this->make_job_("unmark a dead pixel", unmark_dead_pixel(x, y)));
      break;
    case Custom::TPD1_RECALIBRATE:
      ESP_LOGW(TAG, "Maintenance command: one-point temperature recalibration, reference %d",
               i16(p(Param::TPD_REFERENCE)));
      this->queue_(this->make_job_("recalibrate temperatures", tpd_one_point(i16(p(Param::TPD_REFERENCE)))));
      break;
    case Custom::TPD2_POINT1:
    case Custom::TPD2_POINT2: {
      const uint8_t point = c == Custom::TPD2_POINT1 ? 0 : 1;
      ESP_LOGW(TAG, "Maintenance command: two-point temperature recalibration, point %u, reference %d", point + 1,
               i16(p(Param::TPD_REFERENCE)));
      this->queue_(this->make_job_("recalibrate temperatures", tpd_two_point(point, i16(p(Param::TPD_REFERENCE)))));
      break;
    }
    case Custom::VL_WRITE:
      ESP_LOGW(TAG, "Maintenance command: VL parameter 0x%06X = %u", static_cast<unsigned>(u32(p(Param::VL_REGISTER))),
               static_cast<unsigned>(u32(p(Param::VL_VALUE))));
      this->queue_(this->make_job_("write a VL parameter",
                                   vl_write(u32(p(Param::VL_REGISTER)), u32(p(Param::VL_VALUE)))));
      break;
    case Custom::VL_READ: {
      const uint32_t reg = u32(p(Param::VL_REGISTER));
      this->queue_(this->make_job_("read a VL parameter", vl_read(reg),
                                   [this, reg](bool ok, const std::vector<uint8_t> &r) {
                                     const float v = ok && !r.empty() ? r[0] : NAN;
                                     ESP_LOGI(TAG, "VL parameter 0x%06X: %.0f", static_cast<unsigned>(reg), v);
                                     if (this->vl_result_ != nullptr)
                                       this->vl_result_->publish_state(v);
                                   }));
      break;
    }
    case Custom::CRG_WRITE:
      ESP_LOGW(TAG, "Maintenance command: CRG value %u", static_cast<unsigned>(u32(p(Param::CRG_VALUE))));
      this->queue_(this->make_job_("set the CRG value", crg_write(u32(p(Param::CRG_VALUE)))));
      break;
  }
}

void ThermalMasterP1::publish_reading_(Reading r, const std::vector<uint8_t> &resp) {
  sensor::Sensor *s = this->reading_sensors_[static_cast<size_t>(r)];
  text_sensor::TextSensor *t = this->reading_texts_[static_cast<size_t>(r)];
  const size_t n = resp.size();
  auto u16 = [&resp](size_t i) { return static_cast<uint16_t>(resp[i] | (resp[i + 1] << 8)); };
  char b[48];
  switch (r) {
    case Reading::DEVICE_TEMPERATURE:
      if (s != nullptr)
        s->publish_state(n >= 2 ? u16(0) / 100.0f : NAN);
      break;
    case Reading::POWERED_TIME:
      if (s != nullptr) {
        uint32_t v = 0;
        if (n >= 4)
          std::memcpy(&v, resp.data(), 4);
        s->publish_state(n >= 4 ? static_cast<float>(v) : NAN);
      }
      break;
    case Reading::DEVICE_STATUS:
      if (n >= 1)
        this->publish_text_(t, device_status_name(resp[0]));
      break;
    case Reading::SHUTTER_STATUS:
      if (n >= 1)
        this->publish_text_(t, shutter_status_name(resp[0]));
      break;
    case Reading::CRG_VALUE:
      if (s != nullptr)
        s->publish_state(n >= 1 ? resp[0] : NAN);
      break;
    case Reading::SENSOR_VTEMP:
      if (s != nullptr)
        s->publish_state(n >= 2 ? u16(0) : NAN);
      break;
    case Reading::STREAM_SOURCE:
      if (n >= 1)
        this->publish_text_(t, stream_source_name(resp[0]));
      break;
    case Reading::DIGITAL_VIDEO_OUTPUT:
      if (n >= 3) {
        std::snprintf(b, sizeof(b), "%s, %s, %u frames/s", resp[0] ? "On" : "Off", video_format_name(resp[1]),
                      resp[2]);
        this->publish_text_(t, b);
      }
      break;
    case Reading::SYSTEM_DATA_FLAG:
      this->publish_text_(t, hex(resp));
      break;
    case Reading::CURSOR_POSITION:
      if (n >= 4) {
        std::snprintf(b, sizeof(b), "%u, %u", u16(0), u16(2));
        this->publish_text_(t, b);
      }
      break;
    case Reading::CAMERA_POINT:
      if (s != nullptr)
        s->publish_state(n >= 2 ? raw_to_celsius(u16(0)) : NAN);
      break;
    case Reading::CAMERA_FRAME: {
      FrameInfo f{};
      const bool ok = parse_frame_info(resp.data(), n, &f);
      if (ok)
        ESP_LOGD(TAG, "The camera's frame: max %.2f C at %u, %u; min %.2f C at %u, %u", f.max_c, f.max_x, f.max_y,
                 f.min_c, f.min_x, f.min_y);
      if (this->camera_max_ != nullptr)
        this->camera_max_->publish_state(ok ? f.max_c : NAN);
      if (this->camera_min_ != nullptr)
        this->camera_min_->publish_state(ok ? f.min_c : NAN);
      break;
    }
    case Reading::GAIN_SWITCH_STATE:
    case Reading::COUNT:
      break;
  }
}

void ThermalMasterP1::publish_info_rows_() {
  if (this->countdown_sensor_ != nullptr && this->info_seen_)
    this->countdown_sensor_->publish_state(this->ffc_countdown_.load());
}

// ---------------------------------------------------------------- connection

void ThermalMasterP1::on_connected() {
  ESP_LOGI(TAG, "Camera connected; starting it");
  this->status_clear_error();
  this->failures_ = 0;
  this->model_.clear();
  this->set_status_("Camera connected; starting it");
  this->goto_stage_(Stage::INIT, 0);
}

void ThermalMasterP1::on_disconnected() {
  const bool was_camera = this->stage_ != Stage::IDLE;
  this->stop_stream_();
  this->release_stream_();
  this->stage_ = Stage::IDLE;
  this->jobs_.clear();
  this->ctrl_gen_++;  // a request still in flight is abandoned
  this->ctrl_state_.store(CTRL_IDLE);
  this->range_now_ = -1;
  this->range_changing_ = false;
  this->info_seen_ = false;
  this->publish_streaming_(false);
  if (was_camera) {
    ESP_LOGI(TAG, "Camera disconnected");
    this->set_status_("Camera disconnected");
  }
  usb_host::USBClient::on_disconnected();
}

void ThermalMasterP1::goto_stage_(Stage s, uint32_t delay_ms) {
  this->stage_ = s;
  this->stage_at_ = millis() + delay_ms;
  if (s == Stage::INIT)
    this->queue_init_();
  else
    this->jobs_.clear();
  this->step_ = 0;
  this->step_sent_ = false;
  this->status_polling_ = false;
}

void ThermalMasterP1::fail_(const char *what) {
  this->failures_++;
  const uint32_t delay = std::min<uint32_t>(1000u << std::min<uint8_t>(this->failures_, 4), 15000);
  ESP_LOGW(TAG, "%s; starting the camera again in %u s", what, static_cast<unsigned>(delay / 1000));
  this->set_status_(std::string(what) + "; retrying");
  this->stop_stream_();
  this->release_stream_();
  this->ctrl_gen_++;
  this->ctrl_state_.store(CTRL_IDLE);
  this->range_now_ = -1;
  this->range_changing_ = false;
  this->info_seen_ = false;
  this->publish_streaming_(false);
  this->goto_stage_(Stage::BACKOFF, delay);
}

// One control request at a time, on this component's own transfer. An IN
// request's length is the length of `data`.
bool ThermalMasterP1::send_control_(uint8_t type, uint8_t request, uint16_t value, uint16_t index,
                                    std::vector<uint8_t> data) {
  const uint32_t gen = ++this->ctrl_gen_;
  this->ctrl_state_.store(CTRL_PENDING);
  this->ctrl_sent_ms_ = millis();
  if (data.size() > CTRL_MAX_DATA) {
    ESP_LOGE(TAG, "Control request of %u bytes is over %u", static_cast<unsigned>(data.size()),
             static_cast<unsigned>(CTRL_MAX_DATA));
    this->ctrl_state_.store(CTRL_FAILED);
    return false;
  }
  if (this->ctrl_inflight_.load()) {
    // The last request never came back (the camera stopped answering); its
    // transfer cannot be reused until it does, or the camera goes away.
    ESP_LOGW(TAG, "The last control request has not come back yet");
    this->ctrl_state_.store(CTRL_FAILED);
    return false;
  }
  if (this->ctrl_xfer_ == nullptr &&
      usb_host_transfer_alloc(usb_host::SETUP_PACKET_SIZE + CTRL_MAX_DATA, 0, &this->ctrl_xfer_) != ESP_OK) {
    ESP_LOGE(TAG, "No memory for a control transfer");
    this->ctrl_xfer_ = nullptr;
    this->ctrl_state_.store(CTRL_FAILED);
    return false;
  }
  usb_transfer_t *t = this->ctrl_xfer_;
  auto *setup = reinterpret_cast<usb_setup_packet_t *>(t->data_buffer);
  setup->bmRequestType = type;
  setup->bRequest = request;
  setup->wValue = value;
  setup->wIndex = index;
  setup->wLength = static_cast<uint16_t>(data.size());
  if (!(type & USB_BM_REQUEST_TYPE_DIR_IN) && !data.empty())
    std::memcpy(t->data_buffer + usb_host::SETUP_PACKET_SIZE, data.data(), data.size());
  t->num_bytes = static_cast<int>(usb_host::SETUP_PACKET_SIZE + data.size());
  t->device_handle = this->device_handle_;
  t->bEndpointAddress = 0;
  t->callback = ThermalMasterP1::control_done_;
  t->context = this;
  t->timeout_ms = 0;
  this->ctrl_xfer_gen_ = gen;
  this->ctrl_inflight_.store(true);
  const esp_err_t err = usb_host_transfer_submit_control(this->handle_, t);
  if (err != ESP_OK) {
    this->ctrl_inflight_.store(false);
    ESP_LOGW(TAG, "Cannot send a control request: %s", esp_err_to_name(err));
    this->ctrl_state_.store(CTRL_FAILED);
    return false;
  }
  return true;
}

// The USB task: a control request came back (or failed, or the camera went).
void ThermalMasterP1::control_done_(usb_transfer_t *t) {
  auto *self = static_cast<ThermalMasterP1 *>(t->context);
  const bool ok = t->status == USB_TRANSFER_STATUS_COMPLETED;
  if (self->ctrl_xfer_gen_ == self->ctrl_gen_.load()) {   // not an abandoned request
    size_t n = 0;
    if (ok && t->actual_num_bytes > static_cast<int>(usb_host::SETUP_PACKET_SIZE)) {
      n = std::min<size_t>(t->actual_num_bytes - usb_host::SETUP_PACKET_SIZE, sizeof(self->ctrl_resp_));
      std::memcpy(self->ctrl_resp_, t->data_buffer + usb_host::SETUP_PACKET_SIZE, n);
    }
    self->ctrl_resp_len_ = n;
    self->ctrl_state_.store(ok ? CTRL_OK : CTRL_FAILED);
  }
  self->ctrl_inflight_.store(false);
  App.wake_loop_threadsafe();
}

bool ThermalMasterP1::start_step_(const Step &s) {
  switch (s.op) {
    case Op::COMMAND:
      return this->send_control_(TYPE_VENDOR_OUT, REQ_COMMAND, 0, 0,
                                 std::vector<uint8_t>(s.cmd.data(), s.cmd.data() + s.cmd.size));
    case Op::STATUS:
      return this->send_control_(TYPE_VENDOR_IN, REQ_STATUS, 0, 0, std::vector<uint8_t>(1));
    case Op::RESPONSE:
      return this->send_control_(TYPE_VENDOR_IN, REQ_RESPONSE, 0, 0, std::vector<uint8_t>(s.bytes));
    case Op::IDLE_ALT:
      return this->send_control_(TYPE_STD_INTERFACE_OUT, REQ_SET_INTERFACE, 0, STREAM_INTERFACE, {});
    case Op::CLAIM_STREAM: {
      const esp_err_t err =
          usb_host_interface_claim(this->handle_, this->device_handle_, STREAM_INTERFACE, STREAM_ALT);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot claim interface %u alternate setting %u: %s", STREAM_INTERFACE, STREAM_ALT,
                 esp_err_to_name(err));
        return false;
      }
      this->iface1_claimed_ = true;
      return this->send_control_(TYPE_STD_INTERFACE_OUT, REQ_SET_INTERFACE, STREAM_ALT, STREAM_INTERFACE, {});
    }
    case Op::STREAM_ON:
      return this->send_control_(TYPE_VENDOR_DEVICE_OUT, REQ_STREAM_ON, 0, STREAM_INTERFACE, {});
    case Op::START_TRANSFERS:
      return this->start_transfers_();
    case Op::WAIT:
      this->wait_until_ = millis() + s.ms;
      return true;
  }
  return false;
}

void ThermalMasterP1::finish_job_(bool ok) {
  Job done = std::move(this->jobs_.front());
  this->jobs_.pop_front();
  this->step_ = 0;
  this->step_sent_ = false;
  this->status_polling_ = false;
  if (done.done)
    done.done(ok, ok ? done.resp : std::vector<uint8_t>{});
}

void ThermalMasterP1::run_job_() {
  if (this->jobs_.empty())
    return;
  Job &j = this->jobs_.front();
  const Step &s = j.steps[this->step_];
  const uint32_t now = millis();
  if (!this->step_sent_ && this->step_ == 0 && !this->status_polling_ && j.critical)
    ESP_LOGD(TAG, "Start-up: %s", j.what.c_str());
  if (!this->step_sent_) {
    if (s.op == Op::STATUS) {
      if (!this->status_polling_)
        this->status_started_ms_ = now;
      else if (now - this->ctrl_sent_ms_ < STATUS_POLL_GAP_MS)
        return;
    }
    if (!this->start_step_(s)) {
      this->fail_(("Could not " + j.what).c_str());
      return;
    }
    this->step_sent_ = true;
    return;
  }
  auto refused = [this, &j](const std::string &why) {
    if (j.critical) {
      this->fail_(("The camera refused to " + j.what + " (" + why + ")").c_str());
      return;
    }
    ESP_LOGW(TAG, "The camera refused to %s (%s)", j.what.c_str(), why.c_str());
    this->finish_job_(false);
  };
  if (s.op == Op::WAIT) {
    if (static_cast<int32_t>(now - this->wait_until_) < 0)
      return;
  } else if (s.op != Op::START_TRANSFERS) {
    const uint8_t st = this->ctrl_state_.load();
    if (st == CTRL_PENDING) {
      // The camera may hold a status read until it is done.
      const bool status = s.op == Op::STATUS;
      const uint32_t since = now - (status ? this->status_started_ms_ : this->ctrl_sent_ms_);
      if (since > (status ? s.ms + STATUS_SLACK_MS : CTRL_TIMEOUT_MS))
        this->fail_(("The camera did not answer (" + j.what + ")").c_str());
      return;
    }
    this->ctrl_state_.store(CTRL_IDLE);
    // Alternate setting 0 on a camera that was not streaming may be refused.
    if (st == CTRL_FAILED && s.op != Op::IDLE_ALT) {
      refused("a USB error");
      return;
    }
    if (st == CTRL_OK && s.op == Op::RESPONSE)
      j.resp.assign(this->ctrl_resp_, this->ctrl_resp_ + this->ctrl_resp_len_);
    if (st == CTRL_OK && s.op == Op::STATUS) {
      const uint8_t v = this->ctrl_resp_len_ > 0 ? this->ctrl_resp_[0] : 0;
      this->last_status_ = v;
      if (v == STATUS_BUSY) {
        if (now - this->status_started_ms_ < s.ms) {
          this->status_polling_ = true;  // read it again
          this->step_sent_ = false;
          return;
        }
        refused("still busy after " + std::to_string(s.ms / 1000) + " s");
        return;
      }
      this->status_polling_ = false;
      if (v != s.expect) {
        refused("status " + std::to_string(v) + ", not " + std::to_string(s.expect));
        return;
      }
    }
  }
  this->step_++;
  this->step_sent_ = false;
  if (this->step_ < j.steps.size())
    return;
  this->finish_job_(true);
}

// ---------------------------------------------------------------- streaming

bool ThermalMasterP1::start_transfers_() {
  auto make_pool = [](size_t count, size_t bytes) {
    return std::make_unique<video::FramePool>(count, bytes, 64, video::dma_allocator());
  };
  if (!this->raw_pool_)
    this->raw_pool_ = make_pool(RAW_FRAMES, FRAME_BYTES);
  if (!this->sinks_.empty() && !this->rad_pool_)
    this->rad_pool_ = make_pool(RAD_FRAMES, PIXELS * sizeof(float));
  if (!this->img_sinks_.empty() && !this->img_pool_)
    this->img_pool_ = make_pool(IMG_FRAMES, PIXELS * 2);
  if (!this->raw_pool_->ok() || (this->rad_pool_ && !this->rad_pool_->ok()) ||
      (this->img_pool_ && !this->img_pool_->ok())) {
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
    t->callback = ThermalMasterP1::transfer_done_;
    t->context = this;
    t->num_bytes = TRANSFER_BYTES;
    this->transfers_.push_back(t);
  }
  {
    std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
    this->pending_.reset();  // a frame of the last start must not reach the new one
  }
  this->assembler_.reset();
  this->assembling_.reset();
  this->had_frame_ = false;
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

void ThermalMasterP1::stop_stream_() {
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
}

void ThermalMasterP1::release_stream_() {
  // The host cannot close a device with an interface still claimed.
  if (this->iface1_claimed_ && this->device_handle_ != nullptr) {
    const esp_err_t err = usb_host_interface_release(this->handle_, this->device_handle_, STREAM_INTERFACE);
    if (err != ESP_OK)
      ESP_LOGW(TAG, "Cannot release interface %u: %s", STREAM_INTERFACE, esp_err_to_name(err));
  }
  this->iface1_claimed_ = false;
}

void ThermalMasterP1::loop() {
  // Connect / disconnect events. Not USBClient::loop(): that switches this
  // component's loop off whenever no USB event is waiting, and everything
  // below (commands, timeouts, the stream) runs from this loop.
  this->process_usb_events_();
  const uint32_t now = millis();
  if (this->device_handle_ != nullptr)
    this->camera_seen_ = true;
  if (this->usb_restarting_) {
    this->power_on_after_restart_(now);
  } else if (this->restart_usb_after_ms_ != 0 && !this->boot_restart_done_ && !this->camera_seen_ &&
             now >= this->restart_usb_after_ms_) {
    this->boot_restart_done_ = true;
    ESP_LOGW(TAG, "No camera %u s after boot; restarting the USB port",
             static_cast<unsigned>(this->restart_usb_after_ms_ / 1000));
    this->restart_usb();
  }
  switch (this->stage_) {
    case Stage::IDLE:
      break;
    case Stage::BACKOFF:
      if (static_cast<int32_t>(now - this->stage_at_) >= 0 && this->device_handle_ != nullptr)
        this->goto_stage_(Stage::INIT, 0);
      break;
    case Stage::INIT:
      if (static_cast<int32_t>(now - this->stage_at_) >= 0)
        this->run_job_();
      break;
    case Stage::STREAM: {
      this->run_job_();
      if (this->stage_ != Stage::STREAM)
        break;
      if (this->poll_ms_ != 0 && now - this->last_poll_ms_ >= this->poll_ms_) {
        this->last_poll_ms_ = now;
        this->queue_polls_();
      }
      if (this->heartbeat_ms_ != 0 && now - this->last_heartbeat_ms_ >= this->heartbeat_ms_) {
        this->last_heartbeat_ms_ = now;
        this->queue_(this->make_job_("send the heartbeat", action_command(Action::HEARTBEAT)));
      }
      // The automatic range: the votes want the other range.
      const int want = this->range_request_.exchange(-1);
      if (want >= 0 && this->auto_range_ && !this->range_changing_) {
        ESP_LOGI(TAG, "Automatic range: changing to the %s range", want == 1 ? "high" : "low");
        this->range_changing_ = true;
        Job j = this->make_job_("change the range", setting_write(Setting::RANGE, want),
                                [this, want](bool ok, const std::vector<uint8_t> &) {
                                  if (ok)
                                    this->note_range_(want);
                                  this->queue_range_settle_(0);
                                });
        this->jobs_.push_back(std::move(j));  // not dropped when the queue is full
      }
      // The camera's sun flag (metadata byte 138).
      if (this->info_seen_) {
        const bool sun = this->sun_flag_.load() == 1;
        if (!this->sun_known_ || sun != this->sun_published_) {
          if (sun)
            ESP_LOGW(TAG, "The camera reports the sun in view");
          else if (this->sun_known_)
            ESP_LOGI(TAG, "The camera no longer reports the sun in view");
          this->sun_known_ = true;
          this->sun_published_ = sun;
#ifdef USE_BINARY_SENSOR
          if (this->sun_sensor_ != nullptr)
            this->sun_sensor_->publish_state(sun);
#endif
        }
      }
      const bool had = this->had_frame_;
      const uint32_t since = now - (had ? this->last_frame_ms_.load() : this->stream_started_ms_);
      if (!had && since > FIRST_FRAME_TIMEOUT_MS) {
        this->fail_("No frames from the camera after starting it");
        break;
      }
      if (had && since > FRAME_TIMEOUT_MS) {
        this->fail_("The camera stopped sending frames");
        break;
      }
      const uint32_t pause = this->last_pause_ms_.exchange(0);
      if (pause != 0)
        ESP_LOGI(TAG, "The camera's frames paused for %.1f s, then went on", pause / 1000.0f);
      if (had) {
        this->failures_ = 0;
        this->publish_streaming_(since < 1000);
        this->set_status_(this->unreadable_now_ ? "Frames carry no temperatures" : "Streaming");
      }
      break;
    }
  }
  this->report_(now);
}

// ---------------------------------------------------------------- USB task

void ThermalMasterP1::transfer_done_(usb_transfer_t *t) {
  static_cast<ThermalMasterP1 *>(t->context)->on_transfer_(t);
}

void ThermalMasterP1::on_transfer_(usb_transfer_t *t) {
  const usb_transfer_status_t st = t->status;
  const bool gone = st == USB_TRANSFER_STATUS_CANCELED || st == USB_TRANSFER_STATUS_NO_DEVICE;
  if (st == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
    // A buffer is taken as a frame starts and kept for the whole frame.
    if (this->assembler_.pending() == 0 && !this->assembling_ && this->raw_pool_)
      this->assembling_ = this->raw_pool_->acquire();
    switch (this->assembler_.feed(t->data_buffer, static_cast<size_t>(t->actual_num_bytes),
                                  this->assembling_ ? this->assembling_.data() : nullptr)) {
      case Assembler::Result::FRAME: {
        const uint32_t ms = millis();
        {
          std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
          if (this->pending_)
            this->dropped_++;
          this->pending_ = std::move(this->assembling_);
          this->pending_ms_ = ms;
        }
        this->frames_++;
        this->skipped_total_.store(this->assembler_.camera_skipped());
        if (this->had_frame_ && ms - this->last_frame_ms_.load() >= PAUSE_LOG_MS)
          this->last_pause_ms_.store(ms - this->last_frame_ms_.load());
        this->last_frame_ms_.store(ms);
        this->had_frame_ = true;
        xTaskNotifyGive(this->task_);
        App.wake_loop_threadsafe();
        break;
      }
      case Assembler::Result::NO_BUFFER:
        this->no_buffer_++;
        break;
      case Assembler::Result::SHORT:
        this->short_++;
        break;
      case Assembler::Result::LONG:
        this->long_++;
        break;
      case Assembler::Result::BAD_MARKER:
        this->bad_marker_++;
        break;
      case Assembler::Result::PART:
        break;
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

void ThermalMasterP1::restart_usb() {
  if (this->usb_restarting_)
    return;
  const esp_err_t err = usb_host_lib_set_root_port_power(false);
  ESP_LOGI(TAG, "USB port powered off for a restart: %s", esp_err_to_name(err));
  if (err != ESP_OK)
    return;
  this->set_status_("Restarting the USB port");
  const uint32_t now = millis();
  this->usb_restarting_ = true;
  this->usb_bus_empty_ = false;
  this->usb_restart_ms_ = now;
  this->usb_retry_ms_ = now + USB_OFF_MS;
}

// The host powers the port again only once it has freed every device that was
// on it (until then usb_host_lib_set_root_port_power(true) fails with
// ESP_ERR_INVALID_STATE): wait until nothing is enumerated, leave the host a
// moment, then power it on, retrying while the host reports the port busy.
void ThermalMasterP1::power_on_after_restart_(uint32_t now) {
  if (static_cast<int32_t>(now - this->usb_retry_ms_) < 0)
    return;
  this->usb_retry_ms_ = now + USB_RETRY_MS;
  const bool timed_out = now - this->usb_restart_ms_ >= USB_RESTART_TIMEOUT_MS;
  uint8_t addrs[16];
  int count = 0;
  if (usb_host_device_addr_list_fill(sizeof(addrs), addrs, &count) != ESP_OK)
    count = 0;
  if (!timed_out) {
    if (count > 0) {
      this->usb_bus_empty_ = false;
      return;
    }
    if (!this->usb_bus_empty_) {
      this->usb_bus_empty_ = true;
      this->usb_empty_since_ms_ = now;
      return;
    }
    if (now - this->usb_empty_since_ms_ < USB_SETTLE_MS)
      return;
  }
  const esp_err_t err = usb_host_lib_set_root_port_power(true);
  if (err == ESP_ERR_INVALID_STATE && !timed_out)
    return;  // the port is not recovered yet
  this->usb_restarting_ = false;
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "USB port not powered on again after %u ms: %s", static_cast<unsigned>(now - this->usb_restart_ms_),
             esp_err_to_name(err));
    this->set_status_(std::string("The USB port did not power on again (") + esp_err_to_name(err) +
                      "); restart the device");
    return;
  }
  ESP_LOGI(TAG, "USB port powered on again after %u ms", static_cast<unsigned>(now - this->usb_restart_ms_));
  this->set_status_("USB port powered on again; waiting for the camera");
}

void ThermalMasterP1::task_entry_(void *arg) { static_cast<ThermalMasterP1 *>(arg)->task_loop_(); }

void ThermalMasterP1::task_loop_() {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    video::FrameRef raw;
    uint32_t ms;
    {
      std::lock_guard<std::mutex> lock(this->mailbox_mutex_);
      raw = std::move(this->pending_);
      ms = this->pending_ms_;
    }
    if (raw) {
      // Its sinks (a radiometric stage's filters) run here too. One tick's
      // rest after each frame, so lower-priority tasks always get the core
      // even if processing ever takes longer than a frame. (Resting as long
      // as the frame took, as before, passed on only 7 of the camera's 25
      // frames a second.)
      this->process_(raw, ms);
      vTaskDelay(1);
    }
  }
}

void ThermalMasterP1::process_(const video::FrameRef &raw, uint32_t ts) {
  // The camera's own picture.
  if (this->img_pool_ != nullptr && this->img_sinks_.any_wants()) {
    video::ImageFrame f;
    f.buffer = this->img_pool_->acquire();
    if (!f.buffer) {
      this->no_buffer_++;
    } else {
      picture_to_rgb565(raw.data(), f.buffer.data(), static_cast<size_t>(W) * 2);
      f.width = W;
      f.height = H;
      f.stride = W * 2;
      f.format = video::ImageFormat::RGB565_LE;
      f.timestamp_ms = ts;
      f.sequence = ++this->img_sequence_;
      this->pictures_++;
      this->img_sinks_.deliver(f, [](video::ImageSink *s, const video::ImageFrame &fr) { s->on_image_frame(fr); });
    }
  }
  // The metadata rows.
  const InfoRows info = parse_info_rows(raw.data());
  this->sun_flag_.store(info.hw_sun_protect);
  this->ffc_countdown_.store(info.ffc_countdown);
  this->info_seen_ = true;
  // The automatic range's votes (not while a change is under way: the
  // library's switching holds its frame thread until the change is done).
  if (this->auto_reset_.exchange(false))
    this->auto_.reset();
  const int range = this->range_now_.load();
  if (this->auto_range_ && !this->range_changing_) {
    const int want = this->auto_.feed(raw.data(), range);
    if (want >= 0)
      this->range_request_.store(want);
  }
  // The temperatures.
  if (this->rad_pool_ == nullptr || !this->sinks_.any_wants())
    return;
  const uint32_t t0 = micros();
  video::RadiometricFrame f;
  f.buffer = this->rad_pool_->acquire();
  if (!f.buffer) {
    this->no_buffer_++;
    return;
  }
  const bool low = range == 0;
  const uint32_t bad = decode(raw.data(), this->gain_.load(), this->offset_.load(), f.temps(), &f.stats, low);
  this->bad_pixels_.store(bad);
  this->unreadable_now_ = !low && bad == static_cast<uint32_t>(PIXELS);
  if (this->unreadable_now_) {
    this->unreadable_++;
    return;
  }
  this->centre_.store(f.temps()[(H / 2) * W + W / 2]);
  f.width = W;
  f.height = H;
  f.timestamp_ms = ts;
  f.sequence = ++this->sequence_;
  // The camera's temperatures are for a surface it sees as a black body
  // unless its own environment correction is on (the vendor app corrects for
  // emissivity and the like itself, and never turns the camera's on). Said
  // with the frame, so a spot can re-correct for its material.
  {
    const int32_t env = this->known_[static_cast<size_t>(Setting::ENV_CORRECTION)];
    const int32_t em = this->known_[static_cast<size_t>(Setting::EMISSIVITY)];
    const int32_t tu = this->known_[static_cast<size_t>(Setting::REFLECTED)];
    f.emissivity = env == 1 && em != UNKNOWN && em > 0 ? em / 16384.0f : 1.0f;
    f.reflected = tu != UNKNOWN ? static_cast<float>(tu) - 273.0f : NAN;
  }
  this->images_++;
  this->process_us_ += micros() - t0;
  this->sinks_.deliver(f, [](video::RadiometricSink *s, const video::RadiometricFrame &fr) {
    s->on_radiometric_frame(fr);
  });
}

// ---------------------------------------------------------------- status

void ThermalMasterP1::publish_text_(text_sensor::TextSensor *s, const std::string &v) {
  if (s != nullptr && s->get_state() != v)
    s->publish_state(v);
}

void ThermalMasterP1::set_status_(const std::string &s) {
  if (s == this->status_)
    return;
  this->status_ = s;
  if (this->status_sensor_ != nullptr)
    this->status_sensor_->publish_state(s);
}

void ThermalMasterP1::publish_streaming_(bool on) {
  if (this->streaming_known_ && this->streaming_published_ == on)
    return;
  this->streaming_known_ = true;
  this->streaming_published_ = on;
#ifdef USE_BINARY_SENSOR
  if (this->streaming_sensor_ != nullptr)
    this->streaming_sensor_->publish_state(on);
#endif
}

void ThermalMasterP1::report_(uint32_t now) {
  if (this->last_report_ms_ == 0) {
    this->last_report_ms_ = now;
    return;
  }
  const uint32_t elapsed = now - this->last_report_ms_;
  if (elapsed < 10000)
    return;
  this->last_report_ms_ = now;
  const uint32_t frames = this->frames_.exchange(0), images = this->images_.exchange(0),
                 pictures = this->pictures_.exchange(0), dropped = this->dropped_.exchange(0),
                 errors = this->errors_.exchange(0), shorts = this->short_.exchange(0),
                 longs = this->long_.exchange(0), bad_marker = this->bad_marker_.exchange(0),
                 nobuf = this->no_buffer_.exchange(0), us = this->process_us_.exchange(0),
                 unreadable = this->unreadable_.exchange(0);
  if (unreadable > 0)
    ESP_LOGW(TAG, "%u frames carried no temperatures (every value under 200 K) in the high range",
             static_cast<unsigned>(unreadable));
  if (!this->log_timing_ || this->stage_ != Stage::STREAM)
    return;
  const float s = elapsed / 1000.0f;
  ESP_LOGI(TAG,
           "Over %.1f s: %.2f frames/s from the camera, %.2f temperature frames/s passed on (%.2f ms each), "
           "%.2f pictures/s, %u dropped (processing behind), %u short, %u longer than a frame, %u bad markers, "
           "%u transfer errors, %u without a buffer; centre %.2f C, %u pixels "
           "without a reading (or under the low range); %s range; %u commands queued",
           s, frames / s, images / s, images ? us / 1000.0f / images : 0.0f, pictures / s,
           static_cast<unsigned>(dropped), static_cast<unsigned>(shorts),
           static_cast<unsigned>(longs), static_cast<unsigned>(bad_marker), static_cast<unsigned>(errors),
           static_cast<unsigned>(nobuf), this->centre_.load(), static_cast<unsigned>(this->bad_pixels_.load()),
           this->range_now_ == 1 ? "high" : (this->range_now_ == 0 ? "low" : "unknown"),
           static_cast<unsigned>(this->jobs_.size()));
}

}  // namespace thermal_master_p1
}  // namespace esphome
