#include "siri_remote_button.h"

#include "esphome/core/log.h"

#include "siri_ble.h"

namespace esphome {
namespace siri_remote {

static const char *const TAG = "siri_remote.repair";

void SiriRemoteRepairButton::press_action() {
  ESP_LOGI(TAG, "repair pressed — wiping bond and re-entering discovery");
  siri_ble_repair();
}

}  // namespace siri_remote
}  // namespace esphome
