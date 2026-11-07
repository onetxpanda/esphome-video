#pragma once

// Home Assistant entities that set a component option at runtime. Each one
// calls a setter with the new value, restores its last value on boot, and
// with nothing saved applies the configured initial value.

#include <functional>

#include "esphome/components/number/number.h"
#include "esphome/components/select/select.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace video_controls {

class ControlNumber : public number::Number, public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void set_apply(std::function<void(float)> &&f) { this->apply_ = std::move(f); }
  void set_initial(float v) { this->initial_ = v; }
  void set_restore(bool r) { this->restore_ = r; }

 protected:
  void control(float value) override;
  std::function<void(float)> apply_;
  float initial_{0};
  bool restore_{true};
  ESPPreferenceObject pref_;
};

class ControlSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void set_apply(std::function<void(bool)> &&f) { this->apply_ = std::move(f); }
  void set_initial(bool v) { this->initial_ = v; }

 protected:
  void write_state(bool state) override;
  std::function<void(bool)> apply_;
  bool initial_{false};
};

class ControlSelect : public select::Select, public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void set_apply(std::function<void(size_t)> &&f) { this->apply_ = std::move(f); }
  void set_initial(size_t index) { this->initial_ = index; }
  void set_restore(bool r) { this->restore_ = r; }

 protected:
  void control(size_t index) override;
  std::function<void(size_t)> apply_;
  size_t initial_{0};
  bool restore_{true};
  ESPPreferenceObject pref_;
};

}  // namespace video_controls
}  // namespace esphome
