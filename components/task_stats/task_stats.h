#pragma once

// Each FreeRTOS task's share of the CPU over the last update interval, from
// the kernel's run-time statistics, logged as "% of one core" (a task busy
// on one core all the time is 100%), with each core's idle time. The load
// sensor is the CPU in use, % of all cores.

#include <cstdint>
#include <vector>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

namespace esphome {
namespace task_stats {

class TaskStats : public PollingComponent {
 public:
  void update() override;
  void dump_config() override;
  void set_top(uint8_t n) { this->top_ = n; }
#ifdef USE_SENSOR
  void set_load_sensor(sensor::Sensor *s) { this->load_ = s; }
#endif

 protected:
  struct Seen {
    UBaseType_t number;  // the kernel's task number: stable for the task's life
    configRUN_TIME_COUNTER_TYPE time;
  };
  std::vector<Seen> prev_;
  configRUN_TIME_COUNTER_TYPE prev_total_{0};
  bool have_prev_{false};
  uint8_t top_{12};
#ifdef USE_SENSOR
  sensor::Sensor *load_{nullptr};
#endif
};

}  // namespace task_stats
}  // namespace esphome
