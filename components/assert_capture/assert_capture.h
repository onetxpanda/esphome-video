#pragma once

// Records why the firmware aborted, so it can be read after the reboot: the
// failed assert (file, line, function, expression), an abort() and who
// called it, or the task whose stack overflowed, with the task, core and
// whether it happened in an interrupt. The record lives in RAM that a
// software reset leaves alone; it is logged at the next boot and published
// to an optional text sensor. The original handlers still run, so the crash
// itself is unchanged.

#include <string>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace assert_capture {

class AssertCapture : public Component {
 public:
  float get_setup_priority() const override { return setup_priority::BUS; }
  void setup() override;
  void dump_config() override;
#ifdef USE_TEXT_SENSOR
  void set_text_sensor(text_sensor::TextSensor *s) { this->sensor_ = s; }
#endif

 protected:
  std::string last_;  // what the previous boot recorded; empty if nothing
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *sensor_{nullptr};
#endif
};

}  // namespace assert_capture
}  // namespace esphome
