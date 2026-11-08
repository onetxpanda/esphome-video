#include "assert_capture.h"

#include <cstdio>
#include <cstring>

#include <esp_attr.h>
#include <esp_cpu.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esphome/core/log.h"

namespace esphome {
namespace assert_capture {

static const char *const TAG = "assert_capture";
static constexpr uint32_t MAGIC = 0x41535254;  // "ASRT"

enum Kind : uint8_t { KIND_ASSERT = 1, KIND_ABORT = 2, KIND_STACK = 3 };

struct Record {
  uint32_t magic;
  uint8_t kind;
  uint8_t core;
  uint8_t in_isr;
  uint32_t line;
  uint32_t caller;  // return address of the code that asserted or aborted
  char file[64];
  char func[48];
  char expr[96];
  char task[20];
};

// Not cleared at startup: survives the software reset after a panic.
__NOINIT_ATTR static Record g_record;

// May run in an interrupt, with nothing else safe to call: plain copies only.
static void IRAM_ATTR copy_str(char *dst, size_t cap, const char *src) {
  size_t i = 0;
  if (src != nullptr) {
    // Keep the end of long paths: the file name is what identifies it.
    size_t len = 0;
    while (src[len] != '\0')
      len++;
    if (len >= cap)
      src += len - (cap - 1);
    for (; i + 1 < cap && src[i] != '\0'; i++)
      dst[i] = src[i];
  }
  dst[i] = '\0';
}

static void IRAM_ATTR record(Kind kind, const char *file, int line, const char *func, const char *expr,
                            const char *task, uint32_t caller) {
  if (g_record.magic == MAGIC && g_record.kind == KIND_ASSERT && kind == KIND_ABORT)
    return;  // the abort() an assert makes: keep the assert
  g_record.kind = kind;
  g_record.core = static_cast<uint8_t>(esp_cpu_get_core_id());
  g_record.in_isr = xPortInIsrContext() ? 1 : 0;
  g_record.line = static_cast<uint32_t>(line);
  g_record.caller = caller;
  copy_str(g_record.file, sizeof(g_record.file), file);
  copy_str(g_record.func, sizeof(g_record.func), func);
  copy_str(g_record.expr, sizeof(g_record.expr), expr);
  if (task == nullptr)
    task = pcTaskGetName(nullptr);
  copy_str(g_record.task, sizeof(g_record.task), task);
  g_record.magic = MAGIC;
}

void AssertCapture::setup() {
  const esp_reset_reason_t reason = esp_reset_reason();
  const bool crashed = reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT ||
                       reason == ESP_RST_WDT;
  if (g_record.magic == MAGIC && crashed) {
    // Precisions match the record's fields (each holds size - 1 characters).
    char buf[384];
    const Record &r = g_record;
    const char *where = r.in_isr ? " (in an interrupt)" : "";
    switch (r.kind) {
      case KIND_ASSERT:
        std::snprintf(buf, sizeof(buf), "assert failed: %.63s:%u %.47s(): %.95s; task %.19s, core %u%.18s; caller 0x%08X",
                      r.file, static_cast<unsigned>(r.line), r.func, r.expr, r.task, r.core, where,
                      static_cast<unsigned>(r.caller));
        break;
      case KIND_STACK:
        std::snprintf(buf, sizeof(buf), "stack overflow in task %.19s; core %u", r.task, r.core);
        break;
      default:
        std::snprintf(buf, sizeof(buf), "abort() called from 0x%08X; task %.19s, core %u%.18s",
                      static_cast<unsigned>(r.caller), r.task, r.core, where);
        break;
    }
    this->last_ = buf;
  }
  g_record.magic = 0;  // a record is reported once
  if (!this->last_.empty())
    ESP_LOGE(TAG, "Previous boot ended with %s", this->last_.c_str());
#ifdef USE_TEXT_SENSOR
  if (this->sensor_ != nullptr)
    this->sensor_->publish_state(this->last_.empty() ? std::string("none") : this->last_);
#endif
}

void AssertCapture::dump_config() {
  ESP_LOGCONFIG(TAG, "Assert capture:");
  if (this->last_.empty()) {
    ESP_LOGCONFIG(TAG, "  Previous boot: no assert, abort or stack overflow recorded");
  } else {
    ESP_LOGE(TAG, "  Previous boot ended with %s", this->last_.c_str());
  }
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Last crash", this->sensor_);
#endif
}

}  // namespace assert_capture
}  // namespace esphome

// Link-time wraps (-Wl,--wrap=...): record, then run the original.
extern "C" {
void __real___assert_func(const char *file, int line, const char *func, const char *expr) __attribute__((noreturn));
void __real_abort(void) __attribute__((noreturn));
void __real_vApplicationStackOverflowHook(TaskHandle_t task, char *name);

void IRAM_ATTR __attribute__((noreturn)) __wrap___assert_func(const char *file, int line, const char *func,
                                                               const char *expr) {
  esphome::assert_capture::record(esphome::assert_capture::KIND_ASSERT, file, line, func, expr, nullptr,
                                  reinterpret_cast<uint32_t>(__builtin_return_address(0)));
  __real___assert_func(file, line, func, expr);
}

void IRAM_ATTR __attribute__((noreturn)) __wrap_abort(void) {
  esphome::assert_capture::record(esphome::assert_capture::KIND_ABORT, nullptr, 0, nullptr, nullptr, nullptr,
                                  reinterpret_cast<uint32_t>(__builtin_return_address(0)));
  __real_abort();
}

void IRAM_ATTR __wrap_vApplicationStackOverflowHook(TaskHandle_t task, char *name) {
  esphome::assert_capture::record(esphome::assert_capture::KIND_STACK, nullptr, 0, nullptr, nullptr, name, 0);
  __real_vApplicationStackOverflowHook(task, name);
}
}
