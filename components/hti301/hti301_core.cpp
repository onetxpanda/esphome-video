#include "hti301_core.h"

#include <cmath>
#include <limits>

namespace esphome {
namespace hti301 {

static inline uint16_t rd16(const uint8_t *p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
static inline uint32_t rd32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
static inline void wr16(uint8_t *p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v);
  p[1] = static_cast<uint8_t>(v >> 8);
}
static inline void wr32(uint8_t *p, uint32_t v) {
  for (int i = 0; i < 4; i++)
    p[i] = static_cast<uint8_t>(v >> (8 * i));
}

// ---------------------------------------------------------------- descriptors

static constexpr uint8_t DESC_INTERFACE = 0x04;
static constexpr uint8_t DESC_ENDPOINT = 0x05;
static constexpr uint8_t DESC_CS_INTERFACE = 0x24;
static constexpr uint8_t CC_VIDEO = 0x0E;
static constexpr uint8_t SC_VIDEOCONTROL = 0x01;
static constexpr uint8_t SC_VIDEOSTREAMING = 0x02;
static constexpr uint8_t VC_HEADER = 0x01;
static constexpr uint8_t VS_FORMAT_UNCOMPRESSED = 0x04;
static constexpr uint8_t VS_FRAME_UNCOMPRESSED = 0x05;

UvcLayout parse_descriptors(const uint8_t *cfg, size_t len) {
  UvcLayout l;
  int intf = -1, alt = 0, cls = 0, sub = 0;
  bool in_first_format = false;  // between the first uncompressed format and the next format
  size_t i = 0;
  while (i + 2 <= len) {
    const uint8_t blen = cfg[i];
    if (blen < 2 || i + blen > len)
      break;
    const uint8_t *d = cfg + i;
    const uint8_t type = d[1];
    if (type == DESC_INTERFACE && blen >= 9) {
      intf = d[2];
      alt = d[3];
      cls = d[5];
      sub = d[6];
      if (cls == CC_VIDEO && sub == SC_VIDEOCONTROL && l.vc_interface == 0xFF)
        l.vc_interface = static_cast<uint8_t>(intf);
      if (cls == CC_VIDEO && sub == SC_VIDEOSTREAMING && l.vs_interface == 0xFF)
        l.vs_interface = static_cast<uint8_t>(intf);
    } else if (type == DESC_CS_INTERFACE && blen >= 3 && cls == CC_VIDEO) {
      const uint8_t subtype = d[2];
      if (sub == SC_VIDEOCONTROL && intf == l.vc_interface && subtype == VC_HEADER && blen >= 5) {
        l.uvc_version = rd16(d + 3);
      } else if (sub == SC_VIDEOSTREAMING && intf == l.vs_interface && alt == 0) {
        // Format descriptors (UVC 1.5 table A-6); the frames of a format
        // follow it, so any later format ends the first one's frames.
        const bool is_format = subtype == 0x04 || subtype == 0x06 || subtype == 0x0A || subtype == 0x0C ||
                               subtype == 0x10 || subtype == 0x12 || subtype == 0x13 || subtype == 0x15 ||
                               subtype == 0x16 || subtype == 0x18;
        if (subtype == VS_FORMAT_UNCOMPRESSED && blen >= 27 && l.format_index == 0) {
          l.format_index = d[3];
          in_first_format = true;
        } else if (is_format) {
          in_first_format = false;
        } else if (subtype == VS_FRAME_UNCOMPRESSED && blen >= 26 && in_first_format && l.frame_index == 0) {
          l.frame_index = d[3];
          l.width = rd16(d + 5);
          l.height = rd16(d + 7);
          l.max_frame_size = rd32(d + 17);
          l.frame_interval = rd32(d + 21);
        }
      }
    } else if (type == DESC_ENDPOINT && blen >= 7 && intf == l.vs_interface && intf >= 0 && alt > 0) {
      const uint8_t addr = d[2], attr = d[3];
      if ((attr & 0x03) == 0x01 && (addr & 0x80) && l.alt_count < UvcLayout::MAX_ALTS) {
        const uint16_t w = rd16(d + 4);
        StreamAlt &a = l.alts[l.alt_count++];
        a.alt = static_cast<uint8_t>(alt);
        a.endpoint = addr;
        a.packet_size = w & 0x7FF;
        a.transactions = static_cast<uint8_t>(1 + ((w >> 11) & 0x3));
        a.interval = d[6];
      }
    }
    i += blen;
  }
  l.ok = l.vc_interface != 0xFF && l.vs_interface != 0xFF && l.format_index != 0 && l.frame_index != 0 &&
         l.width != 0 && l.height > METADATA_ROWS && l.alt_count != 0;
  return l;
}

const StreamAlt *UvcLayout::alt_for(uint32_t payload) const {
  const StreamAlt *best = nullptr;
  for (uint8_t i = 0; i < this->alt_count; i++) {
    const StreamAlt &a = this->alts[i];
    const uint32_t cap = static_cast<uint32_t>(a.packet_size) * a.transactions;
    if (a.transactions != 1 || cap < payload)
      continue;  // one transaction per interval: what the host stack schedules
    if (best == nullptr || cap < best->packet_size)
      best = &a;
  }
  return best;
}

// ---------------------------------------------------------------- probe/commit

void encode_probe(const Probe &p, uint8_t *buf, size_t len) {
  std::memset(buf, 0, len);
  if (len < 26)
    return;
  wr16(buf + 0, p.hint);
  buf[2] = p.format_index;
  buf[3] = p.frame_index;
  wr32(buf + 4, p.frame_interval);
  wr32(buf + 18, p.max_frame_size);
  wr32(buf + 22, p.max_payload);
}

bool decode_probe(const uint8_t *buf, size_t len, Probe *out) {
  if (len < 26)
    return false;
  out->hint = rd16(buf + 0);
  out->format_index = buf[2];
  out->frame_index = buf[3];
  out->frame_interval = rd32(buf + 4);
  out->max_frame_size = rd32(buf + 18);
  out->max_payload = rd32(buf + 22);
  return true;
}

// ---------------------------------------------------------------- metadata

static float f32_at(const uint16_t *w, size_t i) {
  const uint32_t bits = static_cast<uint32_t>(w[i]) | (static_cast<uint32_t>(w[i + 1]) << 16);  // low word first
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

Metadata parse_metadata(const uint16_t *frame, uint16_t width, uint16_t image_rows) {
  // Word offsets from the start of the metadata rows (XTherm / ht301_hacklib).
  static constexpr size_t FPA_RAW = 1, MAX_X = 2, MAX_Y = 3, MAX_RAW = 4, MIN_X = 5, MIN_Y = 6, MIN_RAW = 7,
                          CENTER_RAW = 12;
  static constexpr size_t USER_AREA = 1279;  // correction, reflected, ambient, humidity, emissivity, distance
  static constexpr float ZEROC = 273.15f;
  static constexpr float FPA_OFF = 7800.0f, FPA_DIV = 36.0f;

  Metadata m;
  const size_t words = static_cast<size_t>(width) * METADATA_ROWS;
  const uint16_t *md = frame + static_cast<size_t>(width) * image_rows;
  // The calibration block follows a row-sized area: 3 rows wide on 384- and
  // 640-wide models, 1 row on the others.
  const size_t cal = (width == 384 || width == 640) ? static_cast<size_t>(width) * 3 : width;
  if (cal + 13 > words)
    return m;

  m.max_x = md[MAX_X];
  m.max_y = md[MAX_Y];
  m.max_raw = md[MAX_RAW];
  m.min_x = md[MIN_X];
  m.min_y = md[MIN_Y];
  m.min_raw = md[MIN_RAW];
  m.center_raw = md[CENTER_RAW];
  m.fpa_temp = 20.0f - (static_cast<float>(md[FPA_RAW]) - FPA_OFF) / FPA_DIV;
  m.cal_00 = md[cal];
  m.shutter_temp = md[cal + 1] / 10.0f - ZEROC;
  m.core_temp = md[cal + 2] / 10.0f - ZEROC;
  m.cal_01 = f32_at(md, cal + 3);
  m.cal_02 = f32_at(md, cal + 5);
  m.cal_03 = f32_at(md, cal + 7);
  m.cal_04 = f32_at(md, cal + 9);
  m.cal_05 = f32_at(md, cal + 11);
  m.valid = std::isfinite(m.cal_01) && m.cal_01 != 0.0f && std::isfinite(m.cal_02) && std::isfinite(m.cal_03) &&
            std::isfinite(m.cal_04) && std::isfinite(m.cal_05) && md[cal + 1] != 0;

  if (USER_AREA + 12 <= words) {
    m.has_environment = true;
    m.correction = f32_at(md, USER_AREA + 0);
    m.reflected = f32_at(md, USER_AREA + 2);
    m.ambient = f32_at(md, USER_AREA + 4);
    m.humidity = f32_at(md, USER_AREA + 6);
    m.emissivity = f32_at(md, USER_AREA + 8);
    m.distance = f32_at(md, USER_AREA + 10);
  }
  return m;
}

// ---------------------------------------------------------------- temperatures

static float pick(float override_value, bool have_camera, float camera, float lo, float hi, float fallback) {
  if (std::isfinite(override_value))
    return override_value;
  if (have_camera && std::isfinite(camera) && camera >= lo && camera <= hi)
    return camera;
  return fallback;
}

Environment resolve_environment(const Environment &o, const Metadata &m) {
  const Environment &d = DEFAULT_ENVIRONMENT;
  const bool c = m.has_environment;
  Environment e;
  e.emissivity = pick(o.emissivity, c, m.emissivity, 0.01f, 1.0f, d.emissivity);
  e.reflected = pick(o.reflected, c, m.reflected, -40.0f, 500.0f, d.reflected);
  e.ambient = pick(o.ambient, c, m.ambient, -40.0f, 100.0f, d.ambient);
  e.humidity = pick(o.humidity, c, m.humidity, 0.0f, 1.0f, d.humidity);
  e.distance = pick(o.distance, c, m.distance, 0.0f, 1000.0f, d.distance);
  e.correction = pick(o.correction, c, m.correction, -50.0f, 50.0f, d.correction);
  return e;
}

static inline double pow4(double x) {
  const double x2 = x * x;
  return x2 * x2;
}

bool TemperatureTable::prepare(const Metadata &m, const Environment &env) {
  static constexpr double ZEROC = 273.15;
  static constexpr float CAL_00_OFFSET = 390.0f, CAL_00_FPAMUL = 7.05f;
  if (!m.valid || !(env.emissivity > 0.0f) || !this->table_.ensure(TABLE_SIZE))
    return false;

  // Atmospheric transmittance over the (capped) distance.
  const double distance = std::fmin(std::fmax(env.distance, 0.0f), 20.0f);
  const double t = env.ambient;
  const double water = env.humidity * std::exp(1.5587 + 0.06939 * t - 2.7816e-4 * t * t + 6.8455e-7 * t * t * t);
  const double sqw = std::sqrt(std::fmax(water, 0.0));
  const double nsqd = -std::sqrt(distance);
  static constexpr double K_ATM = 1.9, A1 = 0.006569, A2 = 0.01262, B1 = -0.002276, B2 = -0.00667;
  const double atm = K_ATM * std::exp(nsqd * (A1 + B1 * sqw)) + (1.0 - K_ATM) * std::exp(nsqd * (A2 + B2 * sqw));

  const double denominator = env.emissivity * atm;
  if (!(std::fabs(denominator) > 1e-9))
    return false;
  this->numerator_sub_ = static_cast<float>((1.0 - env.emissivity) * atm * pow4(env.reflected + ZEROC) +
                                            (1.0 - atm) * pow4(env.ambient + ZEROC));
  this->denominator_ = static_cast<float>(denominator);

  const double ts = m.shutter_temp, tfpa = m.fpa_temp;
  const double c1 = m.cal_01, c2 = m.cal_02;
  this->cal_01_ = m.cal_01;
  this->cal_a_ = static_cast<float>(c2 / (2.0 * c1));
  this->cal_b_ = static_cast<float>(c2 * c2 / (4.0 * c1 * c1));
  this->cal_c_ = static_cast<float>(c1 * ts * ts + ts * c2);
  this->cal_d_ = static_cast<float>(m.cal_03 * tfpa * tfpa + m.cal_04 * tfpa + m.cal_05);
  // In float, as the reference does: the truncation to int depends on it
  // (390 - 20 * 7.05 is 248.99999 in float, 249.0 in double).
  const int corr = static_cast<int>(CAL_00_OFFSET - m.fpa_temp * CAL_00_FPAMUL);
  this->table_offset_ = static_cast<int>(m.cal_00) - (corr > 0 ? corr : 0);
  this->ambient_ = env.ambient;
  this->distance_ = static_cast<float>(distance);
  this->correction_ = env.correction;
  return true;
}

float TemperatureTable::temperature(uint16_t raw) const {
  static constexpr float ZEROC = 273.15f;
  const float val =
      ((static_cast<float>(raw & RAW_MASK) - this->table_offset_) * this->cal_d_ + this->cal_c_) / this->cal_01_ +
      this->cal_b_;
  float n = std::sqrt(std::fabs(val));
  if (std::isnan(n))
    n = 0.0f;
  const float base = n - this->cal_a_ + ZEROC;
  const float b2 = base * base;
  const float wtot = b2 * b2;
  const float q = (wtot - this->numerator_sub_) / this->denominator_;
  if (!(q >= 0.0f))
    return std::numeric_limits<float>::quiet_NaN();  // below what this scene can emit
  const float ttot = std::sqrt(std::sqrt(q)) - ZEROC;
  return ttot + (this->distance_ * 0.85f - 1.125f) * (ttot - this->ambient_) / 100.0f + this->correction_;
}

void TemperatureTable::build(uint16_t lo, uint16_t hi) {
  if (hi >= TABLE_SIZE)
    hi = TABLE_SIZE - 1;
  for (uint32_t r = lo; r <= hi; r++)
    this->table_[r] = this->temperature(static_cast<uint16_t>(r));
}

// The per-pixel loop is built for speed (the firmware is built for size,
// which neither schedules nor unrolls it).
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC push_options
#pragma GCC optimize("O2")
#endif
bool raw_to_temperatures(const uint16_t *raw, uint16_t width, uint16_t height, const Metadata &m,
                         const Environment &env, TemperatureTable &table, float *out, video::FrameStats *stats) {
  if (stats != nullptr)
    *stats = video::FrameStats();
  const size_t count = static_cast<size_t>(width) * height;
  if (count == 0 || !table.prepare(m, env))
    return false;
  // Start from the camera's own min/max of the frame (metadata), which saves
  // a pass over it; a pixel outside extends the table, so the result does
  // not depend on those figures being right.
  uint16_t lo, hi;
  if (m.min_raw <= m.max_raw && m.max_raw <= RAW_MASK) {
    lo = m.min_raw;
    hi = m.max_raw;
  } else {
    lo = hi = raw[0] & RAW_MASK;
  }
  table.build(lo, hi);
  const float *tab = table.values();  // build() fills it in place
  // Statistics on the way, as the radiometric stage would compute them: the
  // first pixel holding the min / max, the mean from per-row float sums.
  float mn = std::numeric_limits<float>::infinity(), mx = -mn;
  size_t mi = 0, ma = 0;
  double sum = 0;
  for (uint16_t y = 0; y < height; y++) {
    const size_t row = static_cast<size_t>(y) * width;
    const uint16_t *src = raw + row;
    float *dst = out + row;
    float row_sum = 0.0f;
#pragma GCC unroll 4
    for (uint16_t x = 0; x < width; x++) {
      const uint16_t v = src[x] & RAW_MASK;
      // Outside the range built so far (rare): extend the table.
      if (__builtin_expect(static_cast<uint16_t>(v - lo) > static_cast<uint16_t>(hi - lo), 0)) {
        if (v < lo) {
          table.build(v, lo - 1);
          lo = v;
        } else {
          table.build(hi + 1, v);
          hi = v;
        }
      }
      const float t = tab[v];
      dst[x] = t;
      row_sum += t;
      if (t < mn) {
        mn = t;
        mi = row + x;
      }
      if (t > mx) {
        mx = t;
        ma = row + x;
      }
    }
    sum += row_sum;
  }
  // A NaN or infinite pixel makes the sum non-finite.
  if (stats != nullptr && std::isfinite(sum)) {
    stats->valid = true;
    stats->min = mn;
    stats->max = mx;
    stats->mean = static_cast<float>(sum / count);
    stats->min_x = static_cast<uint16_t>(mi % width);
    stats->min_y = static_cast<uint16_t>(mi / width);
    stats->max_x = static_cast<uint16_t>(ma % width);
    stats->max_y = static_cast<uint16_t>(ma / width);
  }
  return true;
}
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC pop_options
#endif

// ---------------------------------------------------------------- processed image

static inline uint8_t clamp8(int v) { return static_cast<uint8_t>(v < 0 ? 0 : (v > 255 ? 255 : v)); }

void yuyv_to_rgb565(const uint8_t *src, uint16_t width, uint16_t height, uint8_t *dst, size_t dst_stride) {
  // Full-range BT.601, 8.8 fixed point: R = Y + 1.402 V', G = Y - 0.344 U' - 0.714 V', B = Y + 1.772 U'.
  for (uint16_t y = 0; y < height; y++) {
    const uint8_t *s = src + static_cast<size_t>(y) * width * 2;
    uint8_t *d = dst + static_cast<size_t>(y) * dst_stride;
    for (uint16_t x = 0; x + 1 < width; x += 2, s += 4) {
      const int u = s[1] - 128, v = s[3] - 128;
      const int rv = (359 * v + 128) >> 8;
      const int guv = (88 * u + 183 * v + 128) >> 8;
      const int bu = (454 * u + 128) >> 8;
      for (int k = 0; k < 2; k++) {
        const int yy = s[k * 2];
        const uint8_t r = clamp8(yy + rv), g = clamp8(yy - guv), b = clamp8(yy + bu);
        const uint16_t c = static_cast<uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        *d++ = static_cast<uint8_t>(c);
        *d++ = static_cast<uint8_t>(c >> 8);
      }
    }
  }
}

}  // namespace hti301
}  // namespace esphome
