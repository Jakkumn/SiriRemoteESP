#include "siri_remote_switch.h"

#include "siri_remote_hub.h"

namespace esphome {
namespace siri_remote {

void SiriRemoteRawStreamSwitch::write_state(bool state) {
  if (hub_ != nullptr) {
    hub_->set_raw_stream_enabled(state);
  }
  this->publish_state(state);
}

}  // namespace siri_remote
}  // namespace esphome
