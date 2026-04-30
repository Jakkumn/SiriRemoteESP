#pragma once

#include <cmath>
#include <cstdint>

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace siri_remote {

class SiriRemoteHub;

enum class SiriRemoteKnob : uint8_t {
  SwipeYPriority,
  SwipeMinDistance,
  DoubleWindowMs,
  HoldThresholdMs,
  BatteryLowPct,
  BleSlaveLatency,
};

class SiriRemoteNumber : public number::Number, public Component {
 public:
  // DATA priority so this Number's setup() (which restores state from
  // Preferences and publish_state's it) runs before the hub's LATE setup,
  // which reads our `state` field to seed the event_state config.
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_hub(SiriRemoteHub *h) { hub_ = h; }
  void set_kind(SiriRemoteKnob k) { kind_ = k; }
  void set_initial_value(float v) { initial_value_ = v; }
  void set_restore_value(bool v) { restore_value_ = v; }

 protected:
  void control(float value) override;

  SiriRemoteHub *hub_{nullptr};
  SiriRemoteKnob kind_{};
  float initial_value_{NAN};
  bool restore_value_{true};
  ESPPreferenceObject pref_;
};

}  // namespace siri_remote
}  // namespace esphome
