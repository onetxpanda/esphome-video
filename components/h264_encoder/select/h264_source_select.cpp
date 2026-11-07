#include "h264_source_select.h"

#include "esphome/core/log.h"

namespace esphome {
namespace h264_encoder {

static const char *const TAG = "h264_encoder.select";

void H264SourceSelect::setup() {
  size_t index = this->encoder_->get_active_source();
  if (this->restore_) {
    this->pref_ = this->make_entity_preference<size_t>();
    size_t saved;
    if (this->pref_.load(&saved) && this->has_index(saved) && saved < this->encoder_->source_count())
      index = saved;
  }
  this->encoder_->set_active_source(static_cast<uint8_t>(index));
  this->publish_state(index);
}

void H264SourceSelect::dump_config() {
  LOG_SELECT("", "H.264 source select", this);
  ESP_LOGCONFIG(TAG, "  Restore value: %s", YESNO(this->restore_));
}

void H264SourceSelect::control(size_t index) {
  if (index >= this->encoder_->source_count())
    return;
  this->encoder_->set_active_source(static_cast<uint8_t>(index));
  if (this->restore_)
    this->pref_.save(&index);
  this->publish_state(index);
}

}  // namespace h264_encoder
}  // namespace esphome
