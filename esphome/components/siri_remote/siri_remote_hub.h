#pragma once

#include <atomic>
#include <cstdint>
#include <string>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "esp_timer.h"

#include "button_pulse.h"
#include "report_decoder.h"
#include "siri_audio.h"
#include "siri_ble.h"

namespace esphome {

namespace voice_assistant {
class VoiceAssistant;
}

namespace siri_remote {

class SiriRemoteMicrophone;

class SiriRemoteHub : public Component {
 public:
  void setup() override;
  // Publishes staged button pulses; disabled whenever the ring is empty.
  void loop() override;
  void dump_config() override;
  // siri_ble + Wi-Fi + Preferences must be ready before NimBLE host
  // bring-up; setup priority LATE puts us after the framework's
  // wifi/api/preferences setup.
  float get_setup_priority() const override { return setup_priority::LATE; }

  // Codegen passes a string literal (static storage), so keeping the pointer
  // is safe. "<release> (<commit>)", e.g. "0.1.2-rc.9 (5161f72)".
  void set_version(const char *v) { version_ = v; }
  void set_voice_enabled(bool v) { voice_enabled_ = v; }
  void set_idle_disconnect_ms(uint32_t ms) { idle_disconnect_ms_ = ms; }
  void set_pairing_flush_suppress_ms(uint32_t ms) { pairing_flush_suppress_ms_ = ms; }
  void set_debug_touch_frames(bool v) { debug_touch_frames_ = v; }
  void set_repeat_interval_ms(uint32_t ms) { repeat_interval_ms_ = ms; }
  void set_ble_slave_latency(uint16_t latency) { ble_slave_latency_ = latency; }
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

  // siri_ble callbacks (registered as C function pointers; user = this).
  void on_ble_notify(uint16_t attr_handle, const uint8_t *data, size_t len);
  void on_ble_connected(uint32_t idle_ms);
  void on_ble_disconnected();

  // Runs under pulse_lock_ on the NimBLE host or esp_timer task: stages the
  // pulse into pulse_ring_ and wakes loop(). Never allocates or publishes.
  void emit_event(const button_pulse_event_t *evt);

  // Public so the esp_timer C trampoline can invoke it without a friend
  // declaration.
  void tick();

  // siri_audio callbacks (registered as C function pointers; user = this).
  // Fan out a single decode-task callback to both the optional TCP debug
  // sink and the optional ESPHome microphone consumer.
  void on_audio_session_start();
  void on_audio_session_end();
  void on_pcm(const int16_t *samples, size_t count);

  static uint32_t now_ms() {
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
  }

 protected:
  void start_nimble_();
  void start_audio_();
  void start_tick_timer_();
  // Arms the one-shot tick for the next due repeat; caller holds pulse_lock_.
  void rearm_tick_locked_(uint32_t now);
  void publish_pulse_(siri_button_bit_t button, uint32_t repeat, uint32_t held_ms);

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
  const char *version_{"unknown"};
  bool voice_enabled_{true};
  uint32_t idle_disconnect_ms_{0};
  uint32_t pairing_flush_suppress_ms_{1500};
  bool debug_touch_frames_{false};
  uint32_t repeat_interval_ms_{100};
  uint16_t ble_slave_latency_{400};
  std::string pcm_tcp_host_;
  uint16_t pcm_tcp_port_{0};

  // Runtime state.
  button_pulse_t *pulse_{nullptr};
  Mutex pulse_lock_;
  esp_timer_handle_t tick_timer_{nullptr};

  // Pulses travel from emit_event to loop() through this single-producer /
  // single-consumer ring instead of one defer() per pulse: a defer heap-
  // allocates its std::function (the capture is past the 8-byte inline
  // buffer) and takes the scheduler lock, both while we hold pulse_lock_, ~10
  // times a second per held button. Producers are serialised by pulse_lock_;
  // the loop task is the only consumer. Indices only ever increase and are
  // masked on access. 32 slots is several seconds of every button held at a
  // 100 ms repeat — the loop drains it every iteration.
  struct StagedPulse {
    siri_button_bit_t button;
    uint32_t repeat;
    uint32_t held_ms;
  };
  static constexpr uint32_t PULSE_RING_SIZE = 32;  // power of two
  StagedPulse pulse_ring_[PULSE_RING_SIZE]{};
  std::atomic<uint32_t> ring_head_{0};  // next write; producer-owned
  std::atomic<uint32_t> ring_tail_{0};  // next read; loop-owned
  std::atomic<uint32_t> ring_dropped_{0};

  // Consecutive post-release silence frames the microphone didn't deliver.
  // 50 frames = 1 s of grace for voice_assistant to start the mic after a
  // quick Mic tap before the silence is stopped as unused. Counted on the
  // decode task; zeroed at session start on the NimBLE task, before any frame
  // of the new session exists (single word, so the cross-task write is safe).
  static constexpr uint32_t SILENCE_UNHEARD_LIMIT = 50;
  uint32_t silence_unheard_frames_{0};
  uint16_t prev_buttons_{0};

  binary_sensor::BinarySensor *voice_active_sensor_{nullptr};
  sensor::Sensor *battery_sensor_{nullptr};
  text_sensor::TextSensor *charging_text_sensor_{nullptr};

  // 0xFF sentinel = unknown; suppresses no-change publish_state spam when
  // the remote re-notifies the same byte.
  uint8_t last_battery_pct_{0xFF};
  uint8_t last_charging_byte_{0xFF};

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

  // Latched so the "HA is gone" warning prints once per outage rather than
  // once per pulse. Touched only from the loop task inside emit_event's
  // deferred lambda, so it needs no synchronisation.
  bool api_gate_logged_{false};
  // HA device name for the event payload; filled on first use, loop task only.
  std::string device_name_;
};

}  // namespace siri_remote
}  // namespace esphome
