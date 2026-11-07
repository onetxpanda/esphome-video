#include "video_controls.h"

#include <cmath>

namespace esphome {
namespace video_controls {

static float clamp_to_traits(number::Number *n, float v) {
  const float lo = n->traits.get_min_value(), hi = n->traits.get_max_value();
  if (!std::isfinite(v))
    return lo;
  return v < lo ? lo : (v > hi ? hi : v);
}

void ControlNumber::setup() {
  float value = this->initial_;
  if (this->restore_) {
    this->pref_ = this->make_entity_preference<float>();
    float saved;
    if (this->pref_.load(&saved) && std::isfinite(saved))
      value = saved;
  }
  value = clamp_to_traits(this, value);
  if (this->apply_)
    this->apply_(value);
  this->publish_state(value);
}

void ControlNumber::control(float value) {
  value = clamp_to_traits(this, value);
  if (this->apply_)
    this->apply_(value);
  if (this->restore_)
    this->pref_.save(&value);
  this->publish_state(value);
}

void ControlSwitch::setup() {
  // A saved state wins (per restore_mode); otherwise the configured value.
  optional<bool> saved = this->get_initial_state();
  const bool state = saved.has_value() ? *saved : this->initial_;
  if (this->apply_)
    this->apply_(state);
  this->publish_state(state);
}

void ControlSwitch::write_state(bool state) {
  if (this->apply_)
    this->apply_(state);
  this->publish_state(state);
}

void ControlSelect::setup() {
  size_t index = this->initial_;
  if (this->restore_) {
    this->pref_ = this->make_entity_preference<size_t>();
    size_t saved;
    if (this->pref_.load(&saved) && this->has_index(saved))
      index = saved;
  }
  if (!this->has_index(index))
    index = 0;
  if (this->apply_)
    this->apply_(index);
  this->publish_state(index);
}

void ControlSelect::control(size_t index) {
  if (!this->has_index(index))
    return;
  if (this->apply_)
    this->apply_(index);
  if (this->restore_)
    this->pref_.save(&index);
  this->publish_state(index);
}

}  // namespace video_controls
}  // namespace esphome
