#pragma once

#include "esphome/components/button/button.h"

namespace esphome {
namespace siri_remote {

// Pressing this entity wipes the BLE bond and re-enters discovery.
// Destructive: the user must hold Back+VolUp on the remote within
// the 5-min window to re-pair.
//
// VERIFY-WITH-SECOND-BRIDGE (deferred 2026-04-29): in always-connected
// mode the remote auto-reconnects whenever a known peer is in range.
// On a single-bridge setup, pressing repair clears the bond and the
// SAME bridge wins the next discovery race — so the visible effect is
// a brief disconnect/reconnect rather than a full re-pair. The intended
// use case is moving the remote to a different physical bridge; that
// path wasn't tested in 4.C because only one ESP32-S3 was available.
// When a second board lands, confirm: power off bridge A → press repair
// in HA on bridge B → hold Back+VolUp on the remote → bridge B claims
// the bond cleanly.
class SiriRemoteRepairButton : public button::Button {
 protected:
  void press_action() override;
};

}  // namespace siri_remote
}  // namespace esphome
