#include "task_stats.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace task_stats {

static const char *const TAG = "task_stats";

void TaskStats::dump_config() {
  ESP_LOGCONFIG(TAG, "Task statistics: every %.1f s, busiest %u tasks", this->get_update_interval() / 1000.0f,
                static_cast<unsigned>(this->top_));
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "CPU load", this->load_);
#endif
}

void TaskStats::update() {
  const UBaseType_t cap = uxTaskGetNumberOfTasks() + 4;
  std::vector<TaskStatus_t> st(cap);
  configRUN_TIME_COUNTER_TYPE total = 0;
  const UBaseType_t n = uxTaskGetSystemState(st.data(), cap, &total);
  if (n == 0)
    return;

  struct Row {
    const char *name;
    configRUN_TIME_COUNTER_TYPE dt;
  };
  std::vector<Row> rows;
  rows.reserve(n);
  std::vector<Seen> next;
  next.reserve(n);
  for (UBaseType_t i = 0; i < n; i++) {
    const TaskStatus_t &t = st[i];
    configRUN_TIME_COUNTER_TYPE dt = t.ulRunTimeCounter;  // a task new since the last report: all of it
    for (const Seen &p : this->prev_) {
      if (p.number == t.xTaskNumber) {
        dt = t.ulRunTimeCounter - p.time;  // unsigned: survives the counter wrapping
        break;
      }
    }
    rows.push_back(Row{t.pcTaskName, dt});
    next.push_back(Seen{t.xTaskNumber, t.ulRunTimeCounter});
  }
  const configRUN_TIME_COUNTER_TYPE span = total - this->prev_total_;
  this->prev_ = std::move(next);
  this->prev_total_ = total;
  if (!this->have_prev_ || span == 0) {
    this->have_prev_ = true;
    return;
  }

  // Idle tasks are named IDLE<core>; they are reported per core, not listed.
  float idle[configNUMBER_OF_CORES] = {};
  float idle_sum = 0.0f;
  std::vector<Row> busy;
  busy.reserve(rows.size());
  for (const Row &r : rows) {
    const float pct = 100.0f * static_cast<float>(r.dt) / static_cast<float>(span);
    if (std::strncmp(r.name, "IDLE", 4) == 0) {
      const char c = r.name[4];
      if (c >= '0' && c < '0' + configNUMBER_OF_CORES)
        idle[c - '0'] = pct;
      idle_sum += pct;
    } else {
      busy.push_back(r);
    }
  }
  std::sort(busy.begin(), busy.end(), [](const Row &a, const Row &b) { return a.dt > b.dt; });
  const float load = std::max(0.0f, 100.0f - idle_sum / configNUMBER_OF_CORES);

  char list[384];
  size_t len = 0;
  list[0] = '\0';
  for (size_t i = 0; i < busy.size() && i < this->top_; i++) {
    const float pct = 100.0f * static_cast<float>(busy[i].dt) / static_cast<float>(span);
    if (pct < 0.1f)
      break;
    const int w = std::snprintf(list + len, sizeof(list) - len, "%s%.16s %.1f", len ? ", " : "", busy[i].name, pct);
    if (w < 0 || static_cast<size_t>(w) >= sizeof(list) - len)
      break;
    len += static_cast<size_t>(w);
  }
  // The run-time clock is esp_timer: microseconds.
  ESP_LOGI(TAG, "Over %.1f s: CPU %.1f%% in use; idle: core 0 %.1f%%, core 1 %.1f%%", span / 1e6f, load, idle[0],
           configNUMBER_OF_CORES > 1 ? idle[configNUMBER_OF_CORES - 1] : 0.0f);
  ESP_LOGI(TAG, "Tasks, %% of one core: %s", list);
#ifdef USE_SENSOR
  if (this->load_ != nullptr)
    this->load_->publish_state(load);
#endif
}

}  // namespace task_stats
}  // namespace esphome
