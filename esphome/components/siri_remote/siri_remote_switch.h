#pragma once

#include "esphome/components/switch/switch.h"

namespace esphome {
namespace siri_remote {

class SiriRemoteHub;

class SiriRemoteRawStreamSwitch : public switch_::Switch {
 public:
  void set_hub(SiriRemoteHub *hub) { hub_ = hub; }

 protected:
  void write_state(bool state) override;

  SiriRemoteHub *hub_{nullptr};
};

}  // namespace siri_remote
}  // namespace esphome
