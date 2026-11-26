#include "seek_core.h"

#include <algorithm>
#include <cstring>

namespace esphome {
namespace seek_compact_pro {

// ---------------------------------------------------------------- helpers

static uint16_t le16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
static int16_t le16s(const uint8_t *p) { return static_cast<int16_t>(le16(p)); }
static uint32_t le32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
static float lef(const uint8_t *p) {
  const uint32_t u = le32(p);
  float f;
  std::memcpy(&f, &u, sizeof(f));
  return f;
}

uint64_t frame_uptime_us(const uint16_t *raw) {
  const uint64_t ms = (static_cast<uint64_t>(raw[8]) << 48) | (static_cast<uint64_t>(raw[9]) << 32) |
                      (static_cast<uint64_t>(raw[10]) << 16) | raw[11];
  const uint64_t us = (static_cast<uint64_t>(raw[12]) << 48) | (static_cast<uint64_t>(raw[13]) << 32) |
                      (static_cast<uint64_t>(raw[14]) << 16) | raw[15];
  return us ? us : ms * 1000;
}

// ---------------------------------------------------------------- factory settings

bool parse_factory(const uint8_t *b, size_t len, Factory *out) {
  *out = Factory();
  if (b == nullptr || len < 0xC14)
    return false;
  if (le16(b + 0x02) != 0xF731 || le16(b + 0x4E) != FRAME_MAGIC)
    return false;
  Factory f;
  f.version = static_cast<int32_t>(le32(b + 0x60));
  if (f.version != 3)
    return false;
  f.sensor_ref = lef(b + 0x64);
  const uint16_t n = le16(b + 0x48);
  f.ref_pixels = (n == 0 || n == 0xFFFF) ? 320 : std::min<uint16_t>(n, RAW_W);
  f.range2_offset = le16s(b + 0x52);
  f.lut_shift = static_cast<int32_t>(le32(b + 0x43C)) != 1;
  f.k0 = lef(b + 0x98);
  f.k1 = lef(b + 0x9C);
  f.threshold = lef(b + 0xF4);
  bool finite = std::isfinite(f.sensor_ref) && std::isfinite(f.k0) && std::isfinite(f.k1) && std::isfinite(f.threshold);
  for (int k = 0; k < 7; k++) {
    for (int j = 0; j < 3; j++) {
      f.coef[k][j] = lef(b + 0xA0 + 12 * k + 4 * j);
      finite = finite && std::isfinite(f.coef[k][j]);
    }
  }
  if (!finite)
    return false;
  // Warm-up transient: the library's defaults by model (0xC04); a block with
  // its own amplitude and time constant (0x304, 0x308) uses those (inferred:
  // the camera measured has none).
  char model[17] = {};
  std::memcpy(model, b + 0xC04, 16);
  if (std::strcmp(model, "2003") == 0) {
    f.warm_a = 1.5f;
    f.warm_tau = 210.0f;
  } else if (std::strcmp(model, "2808") == 0) {
    f.warm_a = 2.0f;
    f.warm_tau = 40.0f;
  }
  // The temporal filter's sigma: (unsigned)(0x1008 / 65 x 3 + 0.5).
  const float fs = lef(b + 0x1008);
  if (len >= 0x100C && std::isfinite(fs) && fs > 0.0f && fs < 1000.0f)
    f.filter_sigma = static_cast<uint8_t>(std::min(fs / 65.0f * 3.0f + 0.5f, 10.0f));
  const float a = lef(b + 0x304), tau = lef(b + 0x308);
  if (std::isfinite(a) && std::isfinite(tau) && a > 0.0f && tau > 0.0f) {
    f.warm_a = a;
    f.warm_tau = tau;
  }
  f.valid = true;
  *out = f;
  return true;
}

// ---------------------------------------------------------------- ThermAdjust (format 3)

static constexpr uint32_t ADJUST_MAGIC = 0x86AD416D;
static constexpr size_t ADJUST_BYTES = 65536;
static constexpr size_t ADJUST_TABLE_BYTES = 0x2200;

// The library's BoundSearch_i16: i with a[i], a[i+1] around key (ascending or descending).
static int bound_search_i16(const int16_t *a, int n, int key) {
  const bool asc = a[0] <= a[n - 1];
  int lo = 0, hi = n;
  if (n >= 1) {
    int nhi;
    for (;;) {
      const int m = (lo + hi) / 2;
      nhi = hi;
      if (a[m] == key)
        break;
      nhi = m - 1;
      if (asc ? (a[m] <= key) : (key <= a[m])) {
        lo = m + 1;
        nhi = hi;
      }
      hi = nhi;
      if (!(lo < nhi))
        break;
    }
    hi = nhi;
  }
  const int v = (hi + lo) / 2;
  int r = (n - 2 <= v) ? n - 2 : v;
  const int v1 = v < 2 ? 1 : v;
  const int av = a[std::min(v, n - 1)];
  if (asc ? (key <= av) : (av < key))
    r = v1 - 1;
  return r;
}

bool ThermAdjust::parse_(const uint8_t *p, Table &t) {
  if (le32(p) != 0 || le32(p + 0x80) != ADJUST_MAGIC)
    return false;
  const int32_t ns = static_cast<int32_t>(le32(p + 0xE8)), nt = static_cast<int32_t>(le32(p + 0xEC));
  if (ns < 2 || ns > 32 || nt < 2 || nt > 64)
    return false;
  const uint16_t low = le16(p + 0xF0) & 0xF;
  t.fb = low > 5 ? 5 : static_cast<uint8_t>(5 - low);
  const uint16_t slot = le16(p + 0xFC);
  t.slot = slot < MAX_ADJUST_TABLES ? static_cast<uint8_t>(slot) : 0;
  t.ns = static_cast<uint8_t>(ns);
  t.nt = static_cast<uint8_t>(nt);
  for (int i = 0; i < ns; i++)
    t.axis[i] = le16s(p + 0x1C0 + 2 * i);
  if (!t.node.ensure(64 * 32) || !t.value.ensure(64 * 32) || !t.index.ensure(65536))
    return false;
  for (int k = 0; k < 64 * 32; k++) {
    t.node.data()[k] = le16s(p + 0x200 + 2 * k);
    t.value.data()[k] = le16s(p + 0x1200 + 2 * k);
  }
  return true;
}

bool ThermAdjust::load(const uint16_t *raw) {
  this->loaded_ = false;
  this->count_ = 0;
  video::LargeBuffer<uint8_t> buf;
  if (!buf.ensure(ADJUST_BYTES))
    return false;
  // The payload: rows 4 on, 324 words each, as bytes.
  uint8_t *b = buf.data();
  size_t n = 0;
  for (size_t y = 4; y < RAW_H && n < ADJUST_BYTES; y++) {
    for (size_t x = 0; x < 324 && n < ADJUST_BYTES; x++) {
      const uint16_t w = raw[y * RAW_W + x];
      b[n++] = static_cast<uint8_t>(w & 0xFF);
      if (n < ADJUST_BYTES)
        b[n++] = static_cast<uint8_t>(w >> 8);
    }
  }
  for (size_t k = 0; k < MAX_ADJUST_TABLES; k++) {
    const size_t off = k * ADJUST_TABLE_BYTES;
    if (off + ADJUST_TABLE_BYTES > n)
      break;
    if (this->parse_(b + off, this->t_[this->count_]))
      this->count_++;
  }
  // Applied in slot order.
  for (uint8_t i = 0; i < this->count_; i++)
    this->order_[i] = i;
  std::sort(this->order_, this->order_ + this->count_,
            [this](uint8_t a, uint8_t c) { return this->t_[a].slot < this->t_[c].slot; });
  this->loaded_ = true;
  return true;
}

void ThermAdjust::begin_(Table &t, int64_t s_q16) {
  const int fb = t.fb, ns = t.ns, nt = t.nt;
  const double sc = static_cast<double>(1 << fb);
  const int64_t s_fix = (s_q16 + (int64_t{1} << (15 - fb))) >> (16 - fb);
  const int a0 = t.axis[0], an = t.axis[ns - 1];
  const int key = a0 < an ? static_cast<int>(std::min<int64_t>(std::max<int64_t>(s_fix, a0), an))
                          : static_cast<int>(std::min<int64_t>(std::max<int64_t>(s_fix, an), a0));
  const int i = bound_search_i16(t.axis, ns, key);
  const float num = static_cast<float>(static_cast<double>(s_q16 - (static_cast<int64_t>(t.axis[i]) * (int64_t{1} << (16 - fb)))) *
                                       1.5258789e-05);
  const float den = static_cast<float>((t.axis[i + 1] - t.axis[i]) / sc);
  const float frac = den == 0.0f ? 0.0f : std::min(std::max(num / den, 0.0f), 1.0f);
  const int16_t *P = t.node.data(), *V = t.value.data();
  float node[64];
  int32_t nfix[64];
  for (int j = 0; j < nt; j++) {
    const double p0 = P[j * 32 + i] / sc, p1 = P[j * 32 + i + 1] / sc;
    const double v0 = V[j * 32 + i] / sc, v1 = V[j * 32 + i + 1] / sc;
    node[j] = static_cast<float>(p0 + frac * (p1 - p0));
    t.val[j] = static_cast<float>(v0 + frac * (v1 - v0));
    const double f = std::trunc(static_cast<double>(node[j]) * sc + 0.5);
    nfix[j] = static_cast<int32_t>(std::min(std::max(f, -32768.0), 32767.0));
  }
  for (int k = 0; k < nt; k++) {
    t.x0[k] = 0;
    t.slope[k] = 0;
  }
  for (int k = 0; k + 1 < nt; k++) {
    const float dt = node[k + 1] - node[k];
    if (dt != 0.0f) {
      t.x0[k] = static_cast<int64_t>(std::trunc(std::min(static_cast<double>(node[k]) * 65536.0 + 0.5, 2.1474836e9)));
      const double sl = static_cast<double>(t.val[k + 1] - t.val[k]) / dt * 65536.0 + 0.5;
      t.slope[k] = static_cast<int64_t>(std::trunc(std::min(std::max(sl, -2.1474836e9), 2.1474836e9)));
    }
  }
  uint8_t *idx = t.index.data();
  std::fill(idx, idx + 65536, static_cast<uint8_t>(nt - 2));
  int64_t pos = 0, prev = -32768;
  for (int k = 0; k + 1 < nt; k++) {
    const int64_t next = nfix[k + 1];
    if ((next - prev) + pos > 0x10000)
      break;
    if (next - prev > 0) {
      std::fill(idx + pos, idx + pos + (next - prev), static_cast<uint8_t>(k));
      pos += next - prev;
    }
    prev = next;
  }
}

void ThermAdjust::begin_frame(int64_t s_q16) {
  for (uint8_t i = 0; i < this->count_; i++)
    this->begin_(this->t_[i], s_q16);
}

float ThermAdjust::apply_(const Table &t, float v) const {
  const double q = std::trunc(std::min(std::max(static_cast<double>(v) * 65536.0, -2147483648.0), 2147483647.0));
  const int64_t tq = static_cast<int64_t>(q);
  const uint32_t tfix = static_cast<uint32_t>((tq >> (16 - t.fb)) + 0x8000) & 0xFFFF;
  const uint8_t k = t.index.data()[tfix];
  const int64_t prod = (tq - t.x0[k]) * t.slope[k];
  const int32_t lin = static_cast<int32_t>(static_cast<uint32_t>(static_cast<uint64_t>(prod >> 16)));
  const float corr = t.val[k] + static_cast<float>(lin) * 1.5258789e-05f;
  return v - corr;
}

float ThermAdjust::apply(float t) const {
  for (uint8_t i = 0; i < this->count_; i++)
    t = this->apply_(this->t_[this->order_[i]], t);
  return t;
}

// ---------------------------------------------------------------- picture filter (sbnuc, alpf)

static constexpr int SB_MOVE = 14;            // counts a median pixel must change to count as moving
static constexpr float SB_MOVE_FRACTION = 0.1f;
static constexpr int SB_EDGE = 60;            // diagonal gradient under which a pixel may learn
static constexpr float SB_HOT = 1.2f;         // x frame mean: hotter pixels do not learn
static constexpr float SB_GAIN = 0.125f;
static constexpr int32_t SB_DECAY_Q7 = 127;   // (int)((1 - 0.005) x 128 + 0.5)
static constexpr int SB_OUTLIER = 15;         // counts off the 3x3 median

static inline uint16_t med3(uint16_t a, uint16_t b, uint16_t c) {
  return std::max(std::min(a, b), std::min(std::max(a, b), c));
}

void SceneFilter::set_sigma(uint8_t sigma) {
  if (sigma == 0)
    sigma = 1;
  this->n_ = static_cast<uint8_t>(std::min<int>(6 * sigma, 64));
  this->sigma_set_ = true;
  for (int d = 0; d < this->n_; d++)
    this->table_[d] = static_cast<uint16_t>(static_cast<float>(d * d) / static_cast<float>(this->n_));
}

void SceneFilter::shutter() {
  this->skip_ = true;
  if (this->alpf_.data() != nullptr)
    std::memset(this->alpf_.data(), 0, IMG_PIXELS * sizeof(uint16_t));
}

void SceneFilter::reset() {
  this->have_prev_ = false;
  this->skip_ = false;
  this->updates_ = 0;
  this->replaced_ = 0;
  if (this->corr_.data() != nullptr)
    std::memset(this->corr_.data(), 0, IMG_PIXELS * sizeof(int16_t));
  if (this->alpf_.data() != nullptr)
    std::memset(this->alpf_.data(), 0, IMG_PIXELS * sizeof(uint16_t));
}

// The library's median_approx_3x3_u16: a horizontal 3-median, then a vertical
// 3-median that takes the already-filtered row above; a median only over
// three pixels of one range class (flag), else the centre stays.
void SceneFilter::median_(const uint16_t *x, const uint8_t *fl, uint16_t *m) {
  constexpr int w = IMG_W, h = IMG_H;
  uint16_t *H = this->tmp_.data();
  for (int r = 0; r < h; r++) {
    const uint16_t *xr = x + r * w;
    const uint8_t *fr = fl + r * w;
    uint16_t *hr = H + r * w;
    hr[0] = xr[0];
    hr[w - 1] = xr[w - 1];
    for (int c = 1; c < w - 1; c++) {
      const int s = fr[c - 1] + fr[c] + fr[c + 1];
      hr[c] = (s == 0 || s == 3) ? med3(xr[c - 1], xr[c], xr[c + 1]) : xr[c];
    }
  }
  std::memcpy(m, H, IMG_PIXELS * sizeof(uint16_t));
  for (int r = 1; r < h - 1; r++) {
    for (int c = 1; c < w - 1; c++) {
      const int i = r * w + c;
      const int s = fl[i - w] + fl[i] + fl[i + w];
      m[i] = (s == 0 || s == 3) ? med3(m[i - w], H[i], H[i + w]) : H[i];
    }
    for (int c : {0, w - 1}) {
      const int i = r * w + c;
      const int s = fl[i - w] + fl[i] + fl[i + w];
      m[i] = (s == 0 || s == 3) ? med3(x[i - w], x[i], x[i + w]) : x[i];
    }
  }
  const int corners[4][4] = {{0, 0, 1, 1}, {0, w - 1, 1, w - 2}, {h - 1, 0, h - 2, 1}, {h - 1, w - 1, h - 2, w - 2}};
  for (const auto &k : corners)
    m[k[0] * w + k[1]] = med3(x[k[0] * w + k[1]], x[k[0] * w + k[3]], x[k[2] * w + k[1]]);
}

bool SceneFilter::process(uint16_t *img, const uint16_t *raw) {
  if (!this->corr_.ensure(IMG_PIXELS) || !this->prev_.ensure(IMG_PIXELS) || !this->med_.ensure(IMG_PIXELS) ||
      !this->tmp_.ensure(IMG_PIXELS) || !this->alpf_.ensure(IMG_PIXELS) || !this->flag_.ensure(IMG_PIXELS))
    return false;
  if (!this->sigma_set_)
    this->set_sigma(4);
  constexpr int w = IMG_W, h = IMG_H;
  int16_t *corr = this->corr_.data();
  uint16_t *med = this->med_.data(), *prev = this->prev_.data();
  uint8_t *fl = this->flag_.data();
  // 1. the learned offsets
  uint64_t sum = 0;
  for (size_t i = 0; i < IMG_PIXELS; i++) {
    const int32_t v = static_cast<int32_t>(img[i]) + (corr[i] >> 7);
    img[i] = static_cast<uint16_t>(std::min<int32_t>(std::max<int32_t>(v, 0), 0xFFFF));
    sum += img[i];
    fl[i] = image_px(raw, i) >= SECOND_RANGE ? 1 : 0;
  }
  const uint32_t mean = static_cast<uint32_t>(sum / IMG_PIXELS);
  // 2. the 3x3 median
  this->median_(img, fl, med);
  if (!this->have_prev_) {
    std::memcpy(prev, med, IMG_PIXELS * sizeof(uint16_t));
    this->have_prev_ = true;
  }
  // 3. single-pixel outliers take the median
  uint32_t rep = 0;
  for (size_t i = 0; i < IMG_PIXELS; i++) {
    if (img[i] > med[i] + SB_OUTLIER || static_cast<int32_t>(img[i]) < static_cast<int32_t>(med[i]) - SB_OUTLIER) {
      img[i] = med[i];
      rep++;
    }
  }
  this->replaced_ = rep;
  // 4. which pixels may learn: interior, flat, not hot; how many moved
  const uint32_t hot = static_cast<uint32_t>(SB_HOT * static_cast<float>(mean));
  uint32_t valid = 0, moving = 0;
  double csum = 0;
  uint8_t *mask = fl;  // the range flags are not needed any more
  for (int r = 0; r < h; r++) {
    for (int c = 0; c < w; c++) {
      const int i = r * w + c;
      bool ok = false;
      if (r >= 1 && r < h - 1 && c >= 1 && c < w - 1) {
        const int g = std::abs(static_cast<int>(med[i - w - 1]) - med[i + w + 1]) +
                      std::abs(static_cast<int>(med[i - w + 1]) - med[i + w - 1]);
        ok = g < SB_EDGE && img[i] <= hot;
      }
      mask[i] = ok ? 1 : 0;
      if (ok) {
        valid++;
        const int d = std::min(std::max(static_cast<int>(med[i]) - prev[i], -32768), 32767);
        if (std::abs(d) > SB_MOVE)
          moving++;
        csum += corr[i] * 0.0078125;
      }
    }
  }
  const uint32_t need = static_cast<uint32_t>(SB_MOVE_FRACTION * static_cast<float>(valid));
  bool learn = false;
  if (this->skip_)
    this->skip_ = false;
  else
    learn = moving > need;
  // 5. learn from scene motion, otherwise decay
  if (learn) {
    this->updates_++;
    const float mc = valid ? static_cast<float>(csum) / static_cast<float>(valid) : static_cast<float>(csum);
    for (size_t i = 0; i < IMG_PIXELS; i++) {
      if (!mask[i])
        continue;
      const float v = ((static_cast<float>(corr[i]) * 0.0078125f - mc) -
                       SB_GAIN * static_cast<float>(static_cast<int32_t>(img[i]) - med[i])) * 128.0f + 0.5f;
      corr[i] = static_cast<int16_t>(std::trunc(std::min(std::max(v, -32768.0f), 32767.0f)));
    }
  } else {
    for (int r = 1; r < h - 1; r++)
      for (int c = 1; c < w - 1; c++) {
        const int i = r * w + c;
        corr[i] = static_cast<int16_t>((SB_DECAY_Q7 * corr[i]) >> 7);
      }
  }
  std::memcpy(prev, med, IMG_PIXELS * sizeof(uint16_t));
  // 6. the temporal filter (alpf): small changes are cored
  uint16_t *st = this->alpf_.data();
  for (size_t i = 0; i < IMG_PIXELS; i++) {
    const int32_t d = static_cast<int32_t>(img[i]) - st[i];
    const int32_t ad = std::abs(d);
    if (ad < this->n_)
      img[i] = static_cast<uint16_t>(st[i] + (d < 0 ? -this->table_[ad] : this->table_[ad]));
    st[i] = img[i];
  }
  return true;
}

// ---------------------------------------------------------------- pixel table (frame 4)

bool PixelTable::add_bad_column(uint16_t x) {
  if (x >= IMG_W || this->num_bad_columns_ >= MAX_BAD_COLUMNS)
    return false;
  this->bad_columns_[this->num_bad_columns_++] = x;
  return true;
}

bool PixelTable::build(const uint16_t *raw) {
  this->ready_ = false;
  this->count_ = 0;
  if (!this->alive_.ensure(IMG_PIXELS) || !this->gain_.ensure(IMG_PIXELS) || !this->order_.ensure(IMG_PIXELS))
    return false;
  uint8_t *alive = this->alive_.data();
  uint64_t sum = 0;
  uint32_t nonzero = 0;
  for (size_t i = 0; i < IMG_PIXELS; i++) {
    const uint16_t v = image_px(raw, i);
    if (v != 0) {
      sum += v;
      nonzero++;
    }
  }
  if (nonzero == 0)
    return false;
  // gain = 16384 x mean / value (the library's gain_coef_ex); 0 marks a bad pixel.
  const float mean = static_cast<float>(static_cast<double>(sum) / nonzero);
  uint16_t *g = this->gain_.data();
  for (size_t i = 0; i < IMG_PIXELS; i++) {
    const uint16_t v = image_px(raw, i);
    bool ok = v != 0;
    for (size_t k = 0; k < this->num_bad_columns_; k++)
      ok = ok && i % IMG_W != this->bad_columns_[k];
    g[i] = v ? static_cast<uint16_t>(std::trunc(std::min(static_cast<double>(mean) / v * 16384.0 + 0.5, 32767.0))) : 0;
    if (g[i] == 0)
      ok = false;
    alive[i] = ok ? 1 : 0;
  }

  // An order in which every bad pixel has a good or already filled
  // neighbour when its turn comes.
  video::LargeBuffer<uint8_t> done;
  if (!done.ensure(IMG_PIXELS))
    return false;
  std::memcpy(done.data(), alive, IMG_PIXELS);
  uint8_t *d = done.data();
  uint32_t *order = this->order_.data();
  size_t n = 0;
  for (int pass = 0; pass < 64; pass++) {
    bool left = false, added = false;
    for (size_t i = 0; i < IMG_PIXELS; i++) {
      if (d[i])
        continue;
      const size_t x = i % IMG_W, y = i / IMG_W;
      const bool has = (x > 0 && d[i - 1]) || (x + 1 < IMG_W && d[i + 1]) || (y > 0 && d[i - IMG_W]) ||
                       (y + 1 < IMG_H && d[i + IMG_W]);
      if (has) {
        order[n++] = static_cast<uint32_t>(i);
        d[i] = 1;
        added = true;
      } else {
        left = true;
      }
    }
    if (!left || !added)
      break;  // all placed, or the rest have no good pixel anywhere near
  }
  this->count_ = n;
  this->ready_ = true;
  return true;
}

void PixelTable::fill(uint16_t *img) const {
  if (!this->ready_ || this->count_ == 0)
    return;
  constexpr uint16_t MARK = 0xFFFF;
  const uint32_t *order = this->order_.data();
  for (size_t k = 0; k < this->count_; k++)
    img[order[k]] = MARK;
  for (size_t k = 0; k < this->count_; k++) {
    const size_t i = order[k], x = i % IMG_W, y = i / IMG_W;
    uint32_t sum = 0, n = 0;
    auto take = [&](size_t j) {
      if (img[j] != MARK) {
        sum += img[j];
        n++;
      }
    };
    if (x > 0)
      take(i - 1);
    if (x + 1 < IMG_W)
      take(i + 1);
    if (y > 0)
      take(i - IMG_W);
    if (y + 1 < IMG_H)
      take(i + IMG_W);
    img[i] = n ? static_cast<uint16_t>(sum / n) : 0;
  }
}

// ---------------------------------------------------------------- processor

bool Processor::set_factory(const uint8_t *block, size_t len) {
  const bool ok = parse_factory(block, len, &this->factory_);
  this->scene_.set_sigma(this->factory_.filter_sigma);
  return ok;
}

void Processor::reset() {
  this->table_.reset();
  this->adjust_.reset();
  this->lut_ready_ = false;
  this->have_shutter_ = false;
  this->have_start_ = false;
  this->have_range2_ = false;
  this->row2_q16_ = 0;
  this->s_q16_ = 0;
  this->warm_ = 0;
  this->scene_.reset();
}

// The row-2 mean (16.16), smoothed 7:1 (the library's temperature_process).
void Processor::sensor_(const uint16_t *raw) {
  const uint16_t n = this->factory_.ref_pixels ? this->factory_.ref_pixels : 320;
  int64_t sum = 0;
  for (uint16_t x = 0; x < n; x++)
    sum += raw[2 * RAW_W + x];
  const int64_t m = (sum * 65536) / n;
  this->row2_q16_ = this->row2_q16_ == 0 ? m : (this->row2_q16_ * 7 + m + 4) >> 3;
}

void Processor::through_lut_(const uint16_t *raw, uint16_t *out) const {
  const uint16_t *lut = this->lut_.data();
  for (size_t i = 0; i < IMG_PIXELS; i++) {
    const uint16_t v = image_px(raw, i) & 0x3FFF;
    out[i] = this->lut_ready_ ? lut[v] : v;
  }
}

Processor::Result Processor::feed(const uint16_t *raw, float *temps, video::FrameStats *stats) {
  const uint16_t id = frame_id(raw);
  if (id == FRAME_SHUTTER || id == FRAME_IMAGE || id == FRAME_PRE_SHUTTER || id == FRAME_START_SHUTTER)
    this->sensor_(raw);
  switch (id) {
    case FRAME_GAIN:
      return this->table_.build(raw) ? Result::TABLE : Result::NO_MEMORY;
    case FRAME_LUT: {
      if (!this->lut_.ensure(16384))
        return Result::NO_MEMORY;
      uint16_t *lut = this->lut_.data();
      const bool none = raw[IMG_ROW0 * RAW_W + IMG_COL0] == 0xFFFF;  // no table in this camera
      for (size_t i = 0; i < 16384; i++) {
        if (none) {
          lut[i] = static_cast<uint16_t>(i);
          continue;
        }
        const uint32_t v = raw[(4 + i / 324) * RAW_W + i % 324];
        if (!this->factory_.lut_shift)
          lut[i] = static_cast<uint16_t>(v);
        else
          lut[i] = i >= SECOND_RANGE ? static_cast<uint16_t>(std::min<uint32_t>(v << 1, 0xFFFF))
                                     : static_cast<uint16_t>(v >> 1);
      }
      this->lut_ready_ = true;
      return Result::TABLE;
    }
    case FRAME_ADJUST:
      return this->adjust_.load(raw) ? Result::TABLE : Result::NO_MEMORY;
    case FRAME_SHUTTER:
    case FRAME_START_SHUTTER:
      if (!this->shutter_.ensure(IMG_PIXELS) || !this->start_.ensure(IMG_PIXELS))
        return Result::NO_MEMORY;
      this->through_lut_(raw, this->shutter_.data());
      if (id == FRAME_START_SHUTTER || !this->have_start_) {
        std::memcpy(this->start_.data(), this->shutter_.data(), IMG_PIXELS * sizeof(uint16_t));
        this->have_start_ = true;
      }
      this->have_shutter_ = true;
      if (id == FRAME_SHUTTER)
        this->scene_.shutter();
      return Result::SHUTTER;
    case FRAME_RANGE2: {
      if (!this->range2_.ensure(IMG_PIXELS) || !this->image_.ensure(IMG_PIXELS))
        return Result::NO_MEMORY;
      uint16_t *tmp = this->image_.data();
      this->through_lut_(raw, tmp);
      const int32_t extra = this->factory_.version < 5 ? this->factory_.range2_offset : 0;
      int32_t *b = this->range2_.data();
      const uint16_t *st = this->start_.data();
      for (size_t i = 0; i < IMG_PIXELS; i++)
        b[i] = static_cast<int32_t>(tmp[i]) - (this->have_start_ ? st[i] : 0) + extra;
      this->have_range2_ = true;
      return Result::TABLE;
    }
    case FRAME_IMAGE:
      break;
    default:
      return Result::IGNORED;
  }
  if (!this->ready())
    return Result::NOT_READY;
  if (!this->image_.ensure(IMG_PIXELS))
    return Result::NO_MEMORY;

  // Pixels: 3000 + (table-9 value - shutter offset) x gain / 16384.
  uint16_t *img = this->image_.data();
  const uint16_t *sh = this->shutter_.data(), *g = this->table_.gain(), *lut = this->lut_.data();
  const int32_t *b = this->have_range2_ ? this->range2_.data() : nullptr;
  for (size_t i = 0; i < IMG_PIXELS; i++) {
    const uint16_t r = image_px(raw, i);
    int64_t off = sh[i];
    if (b != nullptr && r >= SECOND_RANGE)
      off += b[i];
    const int64_t v = PEDESTAL + (((static_cast<int64_t>(lut[r & 0x3FFF]) - off) * g[i]) >> 14);
    img[i] = static_cast<uint16_t>(std::min<int64_t>(std::max<int64_t>(v, 0), 0xFFFE));  // 0xFFFF marks bad pixels
  }
  this->table_.fill(img);
  if (this->scene_on_ && !this->scene_.process(img, raw))
    return Result::NO_MEMORY;

  // Temperatures.
  const Factory &f = this->factory_;
  this->s_q16_ = this->row2_q16_ - static_cast<int64_t>(std::trunc(static_cast<double>(f.sensor_ref) * 65536.0 + 0.5));
  const int16_t s = static_cast<int16_t>(static_cast<uint32_t>(this->s_q16_ + 0x8000) >> 16);
  double c[7];
  for (int k = 0; k < 7; k++)
    c[k] = (static_cast<double>(f.coef[k][0]) * s + f.coef[k][1]) * s + f.coef[k][2];
  const float B = static_cast<float>(c[0]), A = static_cast<float>(c[1]), C = static_cast<float>(c[2]),
              D = static_cast<float>(c[3]), E = static_cast<float>(c[4]), F = static_cast<float>(c[5]),
              G = static_cast<float>(c[6]);
  this->adjust_.begin_frame(this->s_q16_);
  const float wu = f.warm_a * std::exp(-(static_cast<float>(frame_uptime_us(raw)) / 1e6f) / f.warm_tau);
  this->warm_ = wu >= 0.005f ? wu : 0.0f;

  const bool ok = f.valid;
  float lo = INFINITY, hi = -INFINITY;
  double sum = 0;
  size_t lo_i = 0, hi_i = 0;
  for (size_t i = 0; i < IMG_PIXELS; i++) {
    float t = NAN;
    if (ok) {
      const float x = f.k0 + f.k1 * static_cast<float>(static_cast<int32_t>(img[i]) - PEDESTAL);
      t = f.threshold <= x ? G + x * (F + x * E) : D + x * (C + x * (A + x * B));
      t = this->adjust_.apply(t) - this->warm_;
      t = t * this->gain_ + this->offset_;
    }
    temps[i] = t;
    if (t < lo) {
      lo = t;
      lo_i = i;
    }
    if (t > hi) {
      hi = t;
      hi_i = i;
    }
    sum += t;
  }
  if (stats != nullptr) {
    stats->valid = ok;
    stats->min = lo;
    stats->max = hi;
    stats->mean = static_cast<float>(sum / IMG_PIXELS);
    stats->min_x = static_cast<uint16_t>(lo_i % IMG_W);
    stats->min_y = static_cast<uint16_t>(lo_i / IMG_W);
    stats->max_x = static_cast<uint16_t>(hi_i % IMG_W);
    stats->max_y = static_cast<uint16_t>(hi_i / IMG_W);
  }
  return Result::IMAGE;
}

}  // namespace seek_compact_pro
}  // namespace esphome
