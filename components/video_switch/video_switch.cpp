#include "video_switch.h"

#include "esphome/core/log.h"

namespace esphome {
namespace video_switch {

static const char *const TAG = "video_switch";

void SwitchInput::on_image_frame(const video::ImageFrame &frame) { this->switch_->handle_frame(this->entry_, frame); }
bool SwitchInput::wants_frames() const { return this->switch_->entry_wanted(this->entry_); }

void VideoSwitch::add_source(video::ImageSource *source, uint8_t entry) {
  auto input = std::make_unique<SwitchInput>(this, entry);
  source->add_image_sink(input.get());
  this->inputs_.push_back(std::move(input));
}

void VideoSwitch::setup() {
  if (!this->sinks_.modifying_sink_is_exclusive())
    this->mark_failed(LOG_STR("A consumer that draws into frames (an overlay) must be this switch's only one"));
}

void VideoSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "Video switch:");
  for (size_t i = 0; i < this->names_.size(); i++)
    ESP_LOGCONFIG(TAG, "  %s%s", this->names_[i], i == this->active_ ? " (active)" : "");
}

void VideoSwitch::set_active(uint8_t entry) {
  if (entry >= this->names_.size() || entry == this->active_)
    return;
  ESP_LOGI(TAG, "Source: %s -> %s", this->names_[this->active_], this->names_[entry]);
  this->active_ = entry;
  this->sinks_.each([](video::ImageSink *s) { s->on_source_change(); });
}

void VideoSwitch::handle_frame(uint8_t entry, const video::ImageFrame &frame) {
  if (entry != this->active_ || this->is_failed())
    return;
  std::unique_lock<std::mutex> lock(this->deliver_mutex_, std::try_to_lock);
  if (!lock.owns_lock())
    return;
  this->sinks_.deliver(frame, [](video::ImageSink *s, const video::ImageFrame &f) { s->on_image_frame(f); });
}

void SwitchSelect::setup() {
  size_t index = this->switch_->get_active();
  if (this->restore_) {
    this->pref_ = this->make_entity_preference<size_t>();
    size_t saved;
    if (this->pref_.load(&saved) && this->has_index(saved) && saved < this->switch_->entry_count())
      index = saved;
  }
  this->switch_->set_active(static_cast<uint8_t>(index));
  this->publish_state(index);
}

void SwitchSelect::dump_config() {
  LOG_SELECT("", "Video switch select", this);
  ESP_LOGCONFIG(TAG, "  Restore value: %s", YESNO(this->restore_));
}

void SwitchSelect::control(size_t index) {
  if (index >= this->switch_->entry_count())
    return;
  this->switch_->set_active(static_cast<uint8_t>(index));
  if (this->restore_)
    this->pref_.save(&index);
  this->publish_state(index);
}

}  // namespace video_switch
}  // namespace esphome
