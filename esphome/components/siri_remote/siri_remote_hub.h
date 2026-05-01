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
#include "siri_remote_number.h"

namespace esphome {

namespace voice_assistant {
class VoiceAssistant;
}

namespace siri_remote {

class SiriRemoteMicrophone;

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

  void set_microphone(SiriRemoteMicrophone *m) { microphone_ = m; }

  void set_voice_assistant(voice_assistant::VoiceAssistant *va) { voice_assistant_ = va; }
  void set_auto_finish_response(bool v) { auto_finish_response_ = v; }

  // Sends api::VoiceAssistantAnnounceFinished to HA — mirrors what
  // voice_assistant::start_playback_timeout_() does when a media_player
  // is configured. Bound to voice_assistant's tts_end_trigger at setup
  // time so HA's assist_satellite UI returns to Idle even when the
  // bridge has no playback hardware. No-op if voice_assistant_ is null
  // or has no active API client.
  void signal_response_finished();

  void set_swipe_y_pri_number(SiriRemoteNumber *n) { swipe_y_pri_number_ = n; }
  void set_swipe_dist_number(SiriRemoteNumber *n) { swipe_dist_number_ = n; }
  void set_dbl_ms_number(SiriRemoteNumber *n) { dbl_ms_number_ = n; }
  void set_hold_ms_number(SiriRemoteNumber *n) { hold_ms_number_ = n; }
  void set_bat_low_number(SiriRemoteNumber *n) { bat_low_number_ = n; }
  void set_ble_lat_number(SiriRemoteNumber *n) { ble_lat_number_ = n; }

  // Dispatch from SiriRemoteNumber::control (loop task) into the right
  // event_state setter or siri_ble API. Mirrors the standalone main.c
  // mutex semantics: event_state setters take es_lock_; siri_ble owns
  // its own locking; battery threshold has no setter (read at publish).
  void apply_knob_change(SiriRemoteKnob kind, float value);

  // siri_ble callbacks (registered as C function pointers; user = this).
  void on_ble_notify(uint16_t attr_handle, const uint8_t *data, size_t len);
  void on_ble_connected(uint32_t idle_ms);
  void on_ble_disconnected();

  void emit_event(const event_state_event_t *evt);

  // Public so the esp_timer C trampoline can invoke it without a friend
  // declaration.
  void tick();

  // siri_audio callbacks (registered as C function pointers; user = this).
  // Fan out a single decode-task callback to both the optional TCP debug
  // sink and the optional ESPHome microphone consumer.
  void on_audio_session_start();
  void on_audio_session_end();
  void on_pcm(const int16_t *samples, size_t count);

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

  // Schedule entity->publish_state(value) on the ESPHome loop. Safe to
  // call from the BLE host task (which is where most call sites live).
  // No-op if the entity pointer is null — callers don't need to gate.
  template<typename TEntity, typename TValue>
  void defer_publish_(TEntity *entity, TValue value) {
    if (entity == nullptr)
      return;
    this->defer([entity, value] { entity->publish_state(value); });
  }

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

  // Decoded-PCM consumer. Populated by microphone.py if the user adds the
  // siri_remote microphone platform; null otherwise. Read from the
  // siri_audio decode task in start_audio_'s on_pcm thunk.
  SiriRemoteMicrophone *microphone_{nullptr};

  // Optional voice_assistant pointer. When set, the hub fires
  // request_start() directly on Mic press (saves the binary_sensor →
  // on_press automation hop) and — if auto_finish_response_ is true —
  // sends VoiceAssistantAnnounceFinished on TTS_END so HA's
  // assist_satellite UI cleanly returns to Idle without a media_player.
  voice_assistant::VoiceAssistant *voice_assistant_{nullptr};
  bool auto_finish_response_{true};

  // Six runtime-tunable Numbers. Pointers populated by number.py to_code
  // before our setup() runs; we read each one's `state` field after
  // restore (the Numbers' own DATA-priority setup() has already fired by
  // the time our LATE setup() runs).
  SiriRemoteNumber *swipe_y_pri_number_{nullptr};
  SiriRemoteNumber *swipe_dist_number_{nullptr};
  SiriRemoteNumber *dbl_ms_number_{nullptr};
  SiriRemoteNumber *hold_ms_number_{nullptr};
  SiriRemoteNumber *bat_low_number_{nullptr};
  SiriRemoteNumber *ble_lat_number_{nullptr};

#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  int pcm_socket_{-1};
#endif
};

}  // namespace siri_remote
}  // namespace esphome
