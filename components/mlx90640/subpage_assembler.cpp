#include "subpage_assembler.h"

#include <cstring>

namespace esphome {
namespace mlx90640 {

static inline int pixel_subpage(int r, int c, bool chess) { return chess ? ((r + c) & 1) : (r & 1); }

void SubpageAssembler::set_mode(FrameMode m) {
  this->mode_ = m;
  this->seen_ = 0;
}

bool SubpageAssembler::add(const float *t, int subpage, bool chess) {
  if (t == nullptr || (subpage != 0 && subpage != 1))
    return false;
  float *a = this->frame_.data();
  if (this->mode_ == FrameMode::MERGE) {
    for (int r = 0; r < SENSOR_ROWS; r++)
      for (int c = 0; c < SENSOR_COLS; c++)
        if (pixel_subpage(r, c, chess) == subpage)
          a[r * SENSOR_COLS + c] = t[r * SENSOR_COLS + c];
    this->seen_ |= static_cast<uint8_t>(1u << subpage);
    if (this->seen_ != 0x3)
      return false;
    this->seen_ = 0;
    return true;
  }
  // INTERPOLATE: every neighbour used belongs to this sub-page (in chess mode
  // all four 4-connected neighbours, interleaved the vertical ones), so the
  // frame comes from one capture instant.
  std::memcpy(a, t, sizeof(float) * SENSOR_PIXELS);
  for (int r = 0; r < SENSOR_ROWS; r++) {
    for (int c = 0; c < SENSOR_COLS; c++) {
      if (pixel_subpage(r, c, chess) == subpage)
        continue;
      float sum = 0.0f;
      int n = 0;
      if (r > 0) {
        sum += t[(r - 1) * SENSOR_COLS + c];
        n++;
      }
      if (r < SENSOR_ROWS - 1) {
        sum += t[(r + 1) * SENSOR_COLS + c];
        n++;
      }
      if (chess) {
        if (c > 0) {
          sum += t[r * SENSOR_COLS + c - 1];
          n++;
        }
        if (c < SENSOR_COLS - 1) {
          sum += t[r * SENSOR_COLS + c + 1];
          n++;
        }
      }
      if (n > 0)
        a[r * SENSOR_COLS + c] = sum / n;
    }
  }
  return true;
}

}  // namespace mlx90640
}  // namespace esphome
