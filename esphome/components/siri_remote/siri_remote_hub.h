#pragma once

#include <cstdint>
#include <string>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "esp_timer.h"

#include "event_state.h"
#include "report_decoder.h"
#include "siri_audio.h"
#include "siri_ble.h"

namespace esphome {
namespace siri_remote {

class SiriRemoteHub : public Component {
 public:
  void setup() override;
  void loop() override {}
  void dump_config() override;
  // siri_ble + Wi-Fi + Preferences must be ready before NimBLE host
  // bring-up; setup priority LATE puts us after the framework's
  // wifi/api/preferences setup.
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_voice_enabled(bool v) { voice_enabled_ = v; }
  void set_idle_disconnect_ms(uint32_t ms) { idle_disconnect_ms_ = ms; }
  void set_pairing_flush_suppress_ms(uint32_t ms) { pairing_flush_suppress_ms_ = ms; }
  void set_debug_touch_frames(bool v) { debug_touch_frames_ = v; }
  void set_debug_pcm_tcp(const std::string &host, uint16_t port) {
    pcm_tcp_host_ = host;
    pcm_tcp_port_ = port;
  }

  void set_voice_active_sensor(binary_sensor::BinarySensor *s) {
    voice_active_sensor_ = s;
  }
  void set_battery_sensor(sensor::Sensor *s) { battery_sensor_ = s; }
  void set_charging_text_sensor(text_sensor::TextSensor *s) {
    charging_text_sensor_ = s;
  }
  void set_raw_stream_enabled(bool enabled) { raw_stream_enabled_ = enabled; }

  // siri_ble callbacks (registered as C function pointers; user = this).
  void on_ble_notify(uint16_t attr_handle, const uint8_t *data, size_t len);
  void on_ble_connected(uint32_t idle_ms);
  void on_ble_disconnected();

  void emit_event(const event_state_event_t *evt);

  // Public so the esp_timer C trampoline can invoke it without a friend
  // declaration.
  void tick();

#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  void pcm_tcp_open();
  void pcm_tcp_close();
  void pcm_tcp_send(const int16_t *samples, size_t count);
#endif

  static uint32_t now_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
  }

 protected:
  void start_nimble_();
  void start_audio_();
  void start_tick_timer_();

  // Config (YAML).
  bool voice_enabled_{true};
  uint32_t idle_disconnect_ms_{0};
  uint32_t pairing_flush_suppress_ms_{1500};
  bool debug_touch_frames_{false};
  std::string pcm_tcp_host_;
  uint16_t pcm_tcp_port_{0};

  // Runtime state.
  event_state_t *es_{nullptr};
  Mutex es_lock_;
  esp_timer_handle_t tick_timer_{nullptr};
  uint32_t suppress_buttons_until_ms_{0};
  // Read by the tick timer task, written by the NimBLE host task — the
  // tick tolerates one stale read so a torn 32-bit access is fine, but
  // mark volatile to keep the compiler honest about the cross-task
  // visibility.
  volatile uint32_t last_activity_ms_{0};
  uint16_t prev_buttons_{0};

  binary_sensor::BinarySensor *voice_active_sensor_{nullptr};
  sensor::Sensor *battery_sensor_{nullptr};
  text_sensor::TextSensor *charging_text_sensor_{nullptr};

  // 0xFF sentinel = unknown; suppresses no-change publish_state spam when
  // the remote re-notifies the same byte.
  uint8_t last_battery_pct_{0xFF};
  uint8_t last_charging_byte_{0xFF};

  // Written by the loop task (Switch::write_state callback), read by the
  // NimBLE host task in the touch handler. Single 32-bit word, atomic on
  // Xtensa; volatile documents the cross-task intent.
  volatile bool raw_stream_enabled_{false};

#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  int pcm_socket_{-1};
#endif
};

}  // namespace siri_remote
}  // namespace esphome
