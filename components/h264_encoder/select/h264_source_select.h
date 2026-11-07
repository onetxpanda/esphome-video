#pragma once

#include "esphome/components/select/select.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "../h264_encoder.h"

namespace esphome {
namespace h264_encoder {

// Chooses which of the encoder's sources is encoded. Options are in the same
// order as the encoder's `source` list; the choice survives reboots unless
// restore_value is false.
class H264SourceSelect : public select::Select, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_encoder(H264Encoder *encoder) { this->encoder_ = encoder; }
  void set_restore(bool restore) { this->restore_ = restore; }

 protected:
  void control(size_t index) override;

  H264Encoder *encoder_{nullptr};
  bool restore_{true};
  ESPPreferenceObject pref_;
};

}  // namespace h264_encoder
}  // namespace esphome
