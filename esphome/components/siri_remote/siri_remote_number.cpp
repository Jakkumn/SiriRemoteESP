#include "siri_remote_number.h"

#include "esphome/core/log.h"

#include "siri_remote_hub.h"

namespace esphome {
namespace siri_remote {

static const char *const TAG = "siri_remote.number";

void SiriRemoteNumber::setup() {
  float value;
  if (this->restore_value_) {
    this->pref_ = this->make_entity_preference<float>();
    if (!this->pref_.load(&value)) {
      value = this->initial_value_;
    }
  } else {
    value = this->initial_value_;
  }
  if (std::isnan(value)) {
    value = this->traits.get_min_value();
  }
  this->publish_state(value);
}

void SiriRemoteNumber::dump_config() { LOG_NUMBER("", "Siri Remote Number", this); }

void SiriRemoteNumber::control(float value) {
  if (this->hub_ != nullptr) {
    this->hub_->apply_knob_change(this->kind_, value);
  }
  // Skip the flash write when HA re-issues the current value (e.g. on
  // automation reload). template_number doesn't guard, but we have six
  // entities sharing one NVS partition — small wear savings add up.
  const bool changed = std::isnan(this->state) || value != this->state;
  if (this->restore_value_ && changed) {
    this->pref_.save(&value);
  }
  this->publish_state(value);
}

}  // namespace siri_remote
}  // namespace esphome
