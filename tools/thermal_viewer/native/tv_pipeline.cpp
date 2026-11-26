// The desktop viewer's pipeline and its C interface (loaded by
// thermal_viewer.py through ctypes).
//
// Frames go through the stages a device runs, with the same code:
//   radiometric::Processor   repair, outlier, temporal, smoothing, median, statistics
//   isp::CpuIsp              tone mapping, palette, mirroring, scaling; spots
//   overlay::MarkHold, draw  reticles and labels
// in the order, and with the glue, of radiometric.cpp, isp.cpp and overlay.cpp.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "isp/isp_core.h"
#include "isp/palette.h"
#include "overlay/overlay_core.h"
#include "radiometric/radiometric_core.h"
#include "tv_camera.h"
#include "video_core/frame.h"

using namespace esphome;

namespace tv {

bool Host::wait(unsigned ms) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
  while (!this->stopping()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= end)
      return true;
    std::this_thread::sleep_for(std::min<std::chrono::steady_clock::duration>(end - now, std::chrono::milliseconds(20)));
  }
  return false;
}

const std::vector<CameraType> &camera_types() {
  static const std::vector<CameraType> TYPES = {thermal_master_p1_type(), seek_compact_pro_type(), hti301_type(),
                                                mlx90640_type(), synthetic_type()};
  return TYPES;
}

}  // namespace tv

extern "C" {

// Mirrored in thermal_viewer.py; every field is 4 bytes.
struct tv_options {
  // radiometric
  int32_t temporal_frames;  // 1..16
  float motion_threshold;
  float edge_smoothing;
  int32_t median_filter;
  float outlier_level;
  float valid_min, valid_max;
  // isp
  float range_min, range_max;
  int32_t mirror_x, mirror_y;
  int32_t scale;    // 1..20
  int32_t upscale;  // 0 nearest, 1 bilinear, 2 bicubic
  float equalize;   // percent
  float detail;
  int32_t palette;  // video::PALETTE_NAMES
  // overlay
  int32_t overlay_enabled;
  uint32_t overlay_kinds;  // a bit per video::AnnotationKind
  int32_t fahrenheit;
  uint32_t hold_ms;
};

struct tv_frame_info {
  int32_t width, height;         // of the image
  int32_t sensor_width, sensor_height;
  uint32_t sequence;
  int32_t stats_valid;
  float min, max, mean, median, center;  // degrees C
  float fps;
  float process_ms;              // the three stages, for this frame
  float emissivity, reflected;   // what the source used; NaN when unknown
};

}  // extern "C"

namespace {

uint32_t now_ms() {
  static const auto t0 = std::chrono::steady_clock::now();
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count() + 1);
}

constexpr size_t POOL_FRAMES = 3;

struct Context : public tv::Host {
  // ---- pipeline (mutex_)
  std::mutex mutex_;
  tv_options opt_{};
  bool rad_changed_{true};
  std::unique_ptr<radiometric::Processor> processor_{new radiometric::Processor()};
  isp::CpuIsp cpu_;
  int palette_applied_{-1};
  std::array<isp::Spot, isp::MAX_SPOTS> spots_{};
  uint8_t spot_count_{0};
  float aspect_{4.0f / 3.0f};
  overlay::MarkHold hold_;
  overlay::Style style_;
  uint16_t last_w_{0}, last_h_{0};
  std::vector<overlay::Mark> marks_;
  // The last frame as the radiometric stage left it, to draw again when an
  // option changes before the next one.
  std::vector<float> temps_;
  uint16_t w_{0}, h_{0};
  radiometric::Stats stats_;
  float emissivity_{NAN}, reflected_{NAN};
  uint32_t sequence_{0};
  uint32_t last_ts_{0};
  bool have_last_ts_{false};
  std::unique_ptr<video::FramePool> pool_;
  size_t pool_size_{0};
  video::ImageFrame out_;
  bool out_new_{false};
  float process_ms_{0};
  std::deque<uint32_t> arrivals_;

  // ---- camera
  std::unique_ptr<tv::Camera> camera_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> running_{false};
  std::mutex side_mutex_;  // status_ and actions_
  std::string status_{"Not connected"};
  std::deque<std::string> actions_;

  // tv::Host, on the camera's thread
  void frame(const float *temps, uint16_t w, uint16_t h, const video::FrameStats *stats, float emissivity,
             float reflected) override;
  void status(const std::string &text) override {
    std::lock_guard<std::mutex> lock(this->side_mutex_);
    this->status_ = text;
  }
  bool stopping() const override { return this->stop_.load(); }
  bool take_action(std::string *name) override {
    std::lock_guard<std::mutex> lock(this->side_mutex_);
    if (this->actions_.empty())
      return false;
    *name = this->actions_.front();
    this->actions_.pop_front();
    return true;
  }

  void render_(bool refresh);
  void close();
};

// As radiometric.cpp's on_radiometric_frame().
void Context::frame(const float *temps, uint16_t w, uint16_t h, const video::FrameStats *stats, float emissivity,
                    float reflected) {
  if (w == 0 || h == 0)
    return;
  std::lock_guard<std::mutex> lock(this->mutex_);
  const auto t0 = std::chrono::steady_clock::now();
  if (this->rad_changed_) {
    radiometric::Options o;
    o.temporal_frames = static_cast<uint8_t>(
        std::min<int32_t>(std::max<int32_t>(this->opt_.temporal_frames, 1), radiometric::MAX_TEMPORAL));
    o.motion_threshold = this->opt_.motion_threshold;
    o.edge_smoothing = this->opt_.edge_smoothing;
    o.median_filter = this->opt_.median_filter != 0;
    o.outlier_level = this->opt_.outlier_level;
    o.valid_min = this->opt_.valid_min;
    o.valid_max = this->opt_.valid_max;
    o.median_stat = true;
    this->processor_->set_options(o);
    this->rad_changed_ = false;
  }
  const size_t n = static_cast<size_t>(w) * h;
  this->temps_.resize(n);
  const video::FrameStats none;
  if (!this->processor_->filters_active() &&
      (this->processor_->adopt(stats ? *stats : none, temps, w, h) || this->processor_->analyze(temps, w, h))) {
    std::memcpy(this->temps_.data(), temps, n * sizeof(float));  // nothing to change
  } else if (!this->processor_->process(temps, this->temps_.data(), w, h)) {
    return;
  }
  this->w_ = w;
  this->h_ = h;
  this->stats_ = this->processor_->stats();
  this->emissivity_ = emissivity;
  this->reflected_ = reflected;
  this->sequence_++;
  this->aspect_ = static_cast<float>(w) / h;
  this->render_(false);
  this->process_ms_ = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
  const uint32_t now = now_ms();
  this->arrivals_.push_back(now);
  while (!this->arrivals_.empty() && now - this->arrivals_.front() > 2000)
    this->arrivals_.pop_front();
}

// As isp.cpp's task_loop_() and process_(), then overlay.cpp's on_image_frame().
void Context::render_(bool refresh) {
  if (this->temps_.empty())
    return;
  const uint32_t ts = now_ms();
  isp::Options o;
  o.range_min = this->opt_.range_min;
  o.range_max = this->opt_.range_max;
  o.mirror_x = this->opt_.mirror_x != 0;
  o.mirror_y = this->opt_.mirror_y != 0;
  o.scale = static_cast<uint8_t>(std::min<int32_t>(std::max<int32_t>(this->opt_.scale, 1), 20));
  o.upscale = static_cast<isp::Upscale>(std::min<int32_t>(std::max<int32_t>(this->opt_.upscale, 0), 2));
  o.format = video::ImageFormat::RGB888;
  o.equalize = std::min(std::max(this->opt_.equalize, 0.0f), 100.0f) / 100.0f;
  o.detail = std::max(this->opt_.detail, 0.0f);
  if (refresh) {
    o.curve_blend = 1.0f;
  } else {
    o.curve_blend = this->have_last_ts_ ? isp::curve_blend_for(ts - this->last_ts_) : 1.0f;
    this->last_ts_ = ts;
    this->have_last_ts_ = true;
  }
  const int palette = std::min<int32_t>(std::max<int32_t>(this->opt_.palette, 0), isp::PALETTE_COUNT - 1);
  if (palette != this->palette_applied_) {
    this->cpu_.set_palette(isp::make_palette(static_cast<isp::Palette>(palette)));
    this->palette_applied_ = palette;
  }

  const int s = o.scale;
  const bool direct = s == 1 && isp::linear_tone(o);
  const float *in = this->temps_.data();
  if (!direct && !this->cpu_.tone_map(in, this->w_, this->h_, o))
    return;
  video::ImageFrame out;
  out.width = this->w_ * s;
  out.height = this->h_ * s;
  out.format = o.format;
  out.stride = out.width * video::bytes_per_pixel(o.format);
  out.timestamp_ms = ts;
  out.sequence = this->sequence_;
  const size_t bytes = out.length();
  if (this->pool_ == nullptr || this->pool_size_ < bytes) {
    this->out_ = video::ImageFrame();  // gives its buffer back before its pool goes
    this->pool_ = std::make_unique<video::FramePool>(POOL_FRAMES, bytes);
    this->pool_size_ = this->pool_->ok() ? bytes : 0;
  }
  out.buffer = this->pool_->acquire();
  if (!out.buffer)
    return;
  if (direct)
    this->cpu_.map_direct(in, this->w_, this->h_, o, out.buffer.data(), out.stride);
  else
    this->cpu_.render(o, out.buffer.data(), out.stride);

  // The radiometric stage's annotations, moved with the pixels.
  if (this->stats_.valid) {
    const struct {
      float x, y, v;
      video::AnnotationKind k;
    } marks[] = {
        {static_cast<float>(this->stats_.min_x), static_cast<float>(this->stats_.min_y), this->stats_.min,
         video::AnnotationKind::MIN},
        {static_cast<float>(this->stats_.max_x), static_cast<float>(this->stats_.max_y), this->stats_.max,
         video::AnnotationKind::MAX},
        {static_cast<float>(this->w_ / 2), static_cast<float>(this->h_ / 2), this->stats_.center,
         video::AnnotationKind::CENTER},
    };
    for (const auto &m : marks) {
      const isp::Point p = isp::map_point(o, this->w_, this->h_, m.x, m.y);
      out.annotations.add({static_cast<float>(p.x), static_cast<float>(p.y), m.v, m.k});
    }
  }
  for (uint8_t i = 0; i < this->spot_count_; i++) {
    const isp::Spot &sp = this->spots_[i];
    const isp::Point p = isp::spot_point(out.width, out.height, sp.x, sp.y);
    const float v = isp::correct_emissivity(isp::spot_value(in, this->w_, this->h_, o, sp.x, sp.y), this->emissivity_,
                                            isp::MATERIALS[sp.material].emissivity, this->reflected_);
    out.annotations.add({static_cast<float>(p.x), static_cast<float>(p.y), v, video::AnnotationKind::SPOT, sp.color});
  }

  if (this->opt_.overlay_enabled && out.annotations.count > 0) {
    if (out.width != this->last_w_ || out.height != this->last_h_) {
      this->style_ = overlay::style_for_height(out.height);
      this->last_w_ = out.width;
      this->last_h_ = out.height;
      this->hold_.reset();
    }
    const bool f = this->opt_.fahrenheit != 0;
    this->marks_.clear();
    for (const auto &sh : this->hold_.update(out.annotations, this->opt_.overlay_kinds, ts, this->opt_.hold_ms)) {
      const float v = f ? sh.value * 9.0f / 5.0f + 32.0f : sh.value;
      char text[16];
      std::snprintf(text, sizeof(text), "%.1f\xB0%c", v, f ? 'F' : 'C');
      overlay::Mark m{sh.x, sh.y, text};
      if (sh.kind == video::AnnotationKind::SPOT)
        m.color = overlay::SPOT_COLORS[sh.index % overlay::SPOT_COLOR_COUNT];
      this->marks_.push_back(m);
    }
    overlay::draw(out, this->marks_, this->style_);
  }
  this->out_ = out;
  this->out_new_ = true;
}

void Context::close() {
  this->stop_.store(true);
  if (this->thread_.joinable())
    this->thread_.join();
  this->camera_.reset();
  this->running_.store(false);
  std::lock_guard<std::mutex> lock(this->side_mutex_);
  this->actions_.clear();
}

void copy_text(const std::string &s, char *buf, int cap) {
  if (buf == nullptr || cap <= 0)
    return;
  const size_t n = std::min(s.size(), static_cast<size_t>(cap - 1));
  std::memcpy(buf, s.data(), n);
  buf[n] = 0;
}

const tv::CameraType *type_at(int i) {
  const auto &t = tv::camera_types();
  return i >= 0 && i < static_cast<int>(t.size()) ? &t[i] : nullptr;
}

}  // namespace

extern "C" {

// ---------------------------------------------------------------- lists
int tv_camera_count() { return static_cast<int>(tv::camera_types().size()); }
const char *tv_camera_key(int i) { return type_at(i) ? type_at(i)->key : ""; }
const char *tv_camera_label(int i) { return type_at(i) ? type_at(i)->label : ""; }
const char *tv_camera_arg_hint(int i) { return type_at(i) ? type_at(i)->arg_hint : ""; }
const char *tv_camera_unavailable(int i) { return type_at(i) ? type_at(i)->unavailable : ""; }
int tv_camera_scale(int i) { return type_at(i) ? type_at(i)->scale : 1; }
float tv_camera_outlier_level(int i) { return type_at(i) ? type_at(i)->outlier_level : 10.0f; }
int tv_camera_action_count(int i) { return type_at(i) ? static_cast<int>(type_at(i)->actions.size()) : 0; }
const char *tv_camera_action(int i, int k) {
  const tv::CameraType *t = type_at(i);
  return t && k >= 0 && k < static_cast<int>(t->actions.size()) ? t->actions[k] : "";
}

int tv_palette_count() { return video::PALETTE_COUNT; }
const char *tv_palette_name(int i) { return i >= 0 && i < video::PALETTE_COUNT ? video::PALETTE_NAMES[i] : ""; }
int tv_material_count() { return isp::MATERIAL_COUNT; }
const char *tv_material_label(int i) { return i >= 0 && i < isp::MATERIAL_COUNT ? isp::MATERIALS[i].label : ""; }
float tv_material_emissivity(int i) { return i >= 0 && i < isp::MATERIAL_COUNT ? isp::MATERIALS[i].emissivity : 0; }

// The components' defaults (radiometric::Options, isp::Options, the overlay's schema).
void tv_default_options(tv_options *o) {
  const radiometric::Options r;
  const isp::Options i;
  o->temporal_frames = r.temporal_frames;
  o->motion_threshold = r.motion_threshold;
  o->edge_smoothing = r.edge_smoothing;
  o->median_filter = r.median_filter;
  o->outlier_level = r.outlier_level;
  o->valid_min = r.valid_min;
  o->valid_max = r.valid_max;
  o->range_min = i.range_min;
  o->range_max = i.range_max;
  o->mirror_x = i.mirror_x;
  o->mirror_y = i.mirror_y;
  o->scale = i.scale;
  o->upscale = static_cast<int32_t>(i.upscale);
  o->equalize = i.equalize * 100.0f;
  o->detail = i.detail;
  o->palette = static_cast<int32_t>(isp::Palette::IRONBOW);
  o->overlay_enabled = 1;
  o->overlay_kinds = (1u << 0) | (1u << 1) | (1u << 2) | (1u << 4);
  o->fahrenheit = 0;
  o->hold_ms = 1000;
}

// ---------------------------------------------------------------- context
void *tv_create() {
  auto *c = new Context();
  tv_default_options(&c->opt_);
  return c;
}

void tv_destroy(void *ctx) {
  auto *c = static_cast<Context *>(ctx);
  c->close();
  delete c;
}

void tv_set_options(void *ctx, const tv_options *o) {
  auto *c = static_cast<Context *>(ctx);
  std::lock_guard<std::mutex> lock(c->mutex_);
  c->opt_ = *o;
  c->rad_changed_ = true;  // from the next frame on
  c->render_(true);        // the image options show at once
}

// Starts camera type `index`. 0, or -1 with the reason in the status.
int tv_open(void *ctx, int index, const char *arg) {
  auto *c = static_cast<Context *>(ctx);
  c->close();
  const tv::CameraType *t = type_at(index);
  if (t == nullptr || t->unavailable[0] != 0) {
    c->status(t ? std::string(t->label) + ": " + t->unavailable : "No such camera type");
    return -1;
  }
  {
    // A new source: nothing of the last one's is shown or filtered into it.
    std::lock_guard<std::mutex> lock(c->mutex_);
    c->temps_.clear();
    c->out_ = video::ImageFrame();
    c->out_new_ = false;
    c->have_last_ts_ = false;
    c->arrivals_.clear();
    c->processor_.reset(new radiometric::Processor());
    c->rad_changed_ = true;
    c->hold_.reset();
  }
  c->camera_.reset(t->make());
  c->stop_.store(false);
  c->running_.store(true);
  c->status(std::string("Opening ") + t->label);
  const std::string a = arg ? arg : "";
  c->thread_ = std::thread([c, a]() {
    c->camera_->run(*c, a);
    c->running_.store(false);
  });
  return 0;
}

void tv_close(void *ctx) {
  auto *c = static_cast<Context *>(ctx);
  c->close();
  c->status("Not connected");
}

int tv_running(void *ctx) { return static_cast<Context *>(ctx)->running_.load(); }

void tv_status(void *ctx, char *buf, int cap) {
  auto *c = static_cast<Context *>(ctx);
  std::lock_guard<std::mutex> lock(c->side_mutex_);
  copy_text(c->status_, buf, cap);
}

void tv_action(void *ctx, const char *name) {
  auto *c = static_cast<Context *>(ctx);
  std::lock_guard<std::mutex> lock(c->side_mutex_);
  c->actions_.push_back(name);
}

// ---------------------------------------------------------------- spots
// (fx, fy): fractions of the image as shown. Returns whether the list changed.
int tv_toggle_spot(void *ctx, float fx, float fy, int material) {
  auto *c = static_cast<Context *>(ctx);
  std::lock_guard<std::mutex> lock(c->mutex_);
  if (material < 0 || material >= isp::MATERIAL_COUNT)
    material = 0;
  const bool changed = isp::toggle_spot(c->spots_.data(), c->spot_count_, fx, fy, c->aspect_,
                                        static_cast<uint8_t>(material));
  if (changed)
    c->render_(true);
  return changed;
}

void tv_clear_spots(void *ctx) {
  auto *c = static_cast<Context *>(ctx);
  std::lock_guard<std::mutex> lock(c->mutex_);
  c->spot_count_ = 0;
  c->render_(true);
}

// "x,y,material,color;..." as the ISP's spots text sensor has it.
void tv_spots_text(void *ctx, char *buf, int cap) {
  auto *c = static_cast<Context *>(ctx);
  std::lock_guard<std::mutex> lock(c->mutex_);
  copy_text(isp::format_spots(c->spots_.data(), c->spot_count_), buf, cap);
}

// ---------------------------------------------------------------- frames
// Copies the newest image (RGB888, width * 3 bytes a row) into `rgb` if there
// is one since the last call and it fits. 1 when copied; `info` is filled
// either way once a frame has come.
int tv_get_frame(void *ctx, uint8_t *rgb, int cap, tv_frame_info *info) {
  auto *c = static_cast<Context *>(ctx);
  std::lock_guard<std::mutex> lock(c->mutex_);
  if (!c->out_.buffer)
    return 0;
  info->width = c->out_.width;
  info->height = c->out_.height;
  info->sensor_width = c->w_;
  info->sensor_height = c->h_;
  info->sequence = c->sequence_;
  info->stats_valid = c->stats_.valid;
  info->min = c->stats_.min;
  info->max = c->stats_.max;
  info->mean = c->stats_.mean;
  info->median = c->stats_.median;
  info->center = c->stats_.center;
  info->process_ms = c->process_ms_;
  info->emissivity = c->emissivity_;
  info->reflected = c->reflected_;
  const uint32_t now = now_ms();
  while (!c->arrivals_.empty() && now - c->arrivals_.front() > 2000)
    c->arrivals_.pop_front();
  info->fps = c->arrivals_.size() / 2.0f;
  if (!c->out_new_ || static_cast<size_t>(cap) < c->out_.length())
    return 0;
  std::memcpy(rgb, c->out_.buffer.data(), c->out_.length());
  c->out_new_ = false;
  return 1;
}

// The last frame's temperatures as the radiometric stage left them (sensor
// size, degrees C). Returns the number of pixels copied.
int tv_get_temps(void *ctx, float *out, int cap) {
  auto *c = static_cast<Context *>(ctx);
  std::lock_guard<std::mutex> lock(c->mutex_);
  const int n = static_cast<int>(c->temps_.size());
  if (n == 0 || cap < n)
    return 0;
  std::memcpy(out, c->temps_.data(), n * sizeof(float));
  return n;
}

}  // extern "C"
