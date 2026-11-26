#include "siri_remote_hub.h"

#include <cinttypes>
#include <string>

#include "esphome/components/api/api_connection.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/api/api_pb2.h"
#include "esphome/components/api/api_server.h"
#include "esphome/components/voice_assistant/voice_assistant.h"
#include "esphome/core/application.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "siri_remote_microphone.h"

#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
#include "siri_audio_pcm_tcp.h"
#endif

namespace esphome {
namespace siri_remote {

static const char *const TAG = "siri_remote";

static constexpr uint32_t TICK_PERIOD_MS = 50;

static void ble_notify_thunk(uint16_t attr_handle, const uint8_t *data,
                             size_t len, void *user) {
  static_cast<SiriRemoteHub *>(user)->on_ble_notify(attr_handle, data, len);
}

static void ble_connected_thunk(uint32_t idle_ms, void *user) {
  static_cast<SiriRemoteHub *>(user)->on_ble_connected(idle_ms);
}

static void ble_disconnected_thunk(void *user) {
  static_cast<SiriRemoteHub *>(user)->on_ble_disconnected();
}

static void emit_thunk(const button_pulse_event_t *evt, void *user) {
  static_cast<SiriRemoteHub *>(user)->emit_event(evt);
}

static void tick_thunk(void *user) {
  static_cast<SiriRemoteHub *>(user)->tick();
}

static void audio_session_start_thunk(void *user) {
  static_cast<SiriRemoteHub *>(user)->on_audio_session_start();
}
static void audio_session_end_thunk(void *user) {
  static_cast<SiriRemoteHub *>(user)->on_audio_session_end();
}
static void audio_pcm_thunk(const int16_t *samples, size_t count, void *user) {
  static_cast<SiriRemoteHub *>(user)->on_pcm(samples, count);
}

// Action attached to voice_assistant's tts_end_trigger so we can react
// at TTS_END time without owning a media_player. Forwards to the hub's
// signal_response_finished(); allocated once at setup and never freed.
class FinishResponseAction : public Action<std::string> {
 public:
  explicit FinishResponseAction(SiriRemoteHub *hub) : hub_(hub) {}
  void play(const std::string & /*url*/) override {
    if (this->hub_ != nullptr) {
      this->hub_->signal_response_finished();
    }
  }

 private:
  SiriRemoteHub *hub_;
};

void SiriRemoteHub::setup() {
  ESP_LOGI(TAG, "siri_remote setup: voice=%d idle_disconnect_ms=%" PRIu32,
           (int) voice_enabled_, idle_disconnect_ms_);

  // ESPHome may have already initialized NVS via Preferences.
  // nvs_flash_init is idempotent on success; we still handle the rare
  // partition-corruption path so a first boot from a stale flash recovers.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ESP_ERROR_CHECK(nvs_flash_init());
  } else if (ret != ESP_OK) {
    ESP_LOGE(TAG, "nvs_flash_init returned %s", esp_err_to_name(ret));
    this->mark_failed();
    return;
  }

  button_pulse_config_t pulse_cfg = {};
  pulse_cfg.repeat_interval_ms = repeat_interval_ms_;
  pulse_ = button_pulse_create(&pulse_cfg, emit_thunk, this);
  if (pulse_ == nullptr) {
    ESP_LOGE(TAG, "button_pulse_create failed");
    this->mark_failed();
    return;
  }

  if (voice_enabled_) {
    start_audio_();
  }

  start_tick_timer_();
  start_nimble_();

  if (voice_assistant_ != nullptr && auto_finish_response_) {
    // Bind a one-shot Action onto voice_assistant's tts_end_trigger so we
    // can fan out an AnnounceFinished message ourselves. Without this,
    // HA's assist_satellite UI sticks on "Responding" forever when no
    // media_player is configured (upstream voice_assistant only sends
    // AnnounceFinished from its playback timeout).
    auto *automation = new Automation<std::string>(
        voice_assistant_->get_tts_end_trigger());
    automation->add_actions({new FinishResponseAction(this)});
  }

  ESP_LOGI(TAG, "siri_remote setup complete");
}

void SiriRemoteHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Siri Remote Hub:");
  ESP_LOGCONFIG(TAG, "  Voice enabled: %s", YESNO(voice_enabled_));
  ESP_LOGCONFIG(TAG, "  Idle-disconnect: %" PRIu32 " ms (0 = always-connected)",
                idle_disconnect_ms_);
  ESP_LOGCONFIG(TAG, "  Pairing-flush suppress: %" PRIu32 " ms",
                pairing_flush_suppress_ms_);
  ESP_LOGCONFIG(TAG, "  Pulse repeat interval: %" PRIu32 " ms (0 = press only)",
                repeat_interval_ms_);
  ESP_LOGCONFIG(TAG, "  BLE slave latency: %u", (unsigned) ble_slave_latency_);
  ESP_LOGCONFIG(TAG, "  Debug touch frames: %s", YESNO(debug_touch_frames_));
#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  ESP_LOGCONFIG(TAG, "  Debug PCM TCP: %s:%u", pcm_tcp_host_.c_str(),
                (unsigned) pcm_tcp_port_);
#endif
}

void SiriRemoteHub::start_audio_() {
  siri_audio_config_t audio_cfg = {};
  audio_cfg.on_pcm = audio_pcm_thunk;
  audio_cfg.on_session_start = audio_session_start_thunk;
  audio_cfg.on_session_end = audio_session_end_thunk;
  audio_cfg.user = this;
  esp_err_t rc = siri_audio_start(&audio_cfg);
  if (rc != ESP_OK) {
    // No decoder + no decode task = no PCM ever, but BLE notify on the
    // audio CCCD would still happily drive siri_audio_dispatch_packet,
    // and a Mic press would still kick voice_assistant.start. Disable
    // voice and mark the component failed so the user sees it.
    ESP_LOGE(TAG, "siri_audio_start failed: %s", esp_err_to_name(rc));
    voice_enabled_ = false;
    this->mark_failed();
  }
}

void SiriRemoteHub::start_tick_timer_() {
  esp_timer_create_args_t ta = {};
  ta.callback = tick_thunk;
  ta.arg = this;
  ta.name = "siri_remote_tick";
  ESP_ERROR_CHECK(esp_timer_create(&ta, &tick_timer_));
  ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer_, TICK_PERIOD_MS * 1000));
}

void SiriRemoteHub::start_nimble_() {
  siri_ble_config_t ble_cfg = {};
  ble_cfg.on_notify = ble_notify_thunk;
  ble_cfg.on_connected = ble_connected_thunk;
  ble_cfg.on_disconnected = ble_disconnected_thunk;
  ble_cfg.user = this;
  // siri_ble owns the post-pairing blackout now — it covers every
  // notification, not just buttons, and keeps both build paths identical.
  ble_cfg.pairing_flush_suppress_ms = pairing_flush_suppress_ms_;
  ble_cfg.idle_disconnect_ms = idle_disconnect_ms_;
  ble_cfg.slave_latency = ble_slave_latency_;
  // Brings up NimBLE and starts its host task.
  ESP_ERROR_CHECK(siri_ble_start(&ble_cfg));
}

void SiriRemoteHub::tick() {
  LockGuard guard(pulse_lock_);
  button_pulse_tick(pulse_, now_ms());
}

void SiriRemoteHub::on_ble_notify(uint16_t attr_handle, const uint8_t *data,
                                  size_t len) {
  const uint32_t now = now_ms();

  switch (attr_handle) {
    case SIRI_HANDLE_BUTTON: {
      uint16_t btns = siri_decode_button_bytes(data, len);

      bool mic_was = (prev_buttons_ & SIRI_BTN_MIC) != 0;
      bool mic_now = (btns & SIRI_BTN_MIC) != 0;
      if (mic_now != mic_was) {
        if (voice_enabled_) {
          if (mic_now) {
            siri_audio_session_start();
          } else {
            siri_audio_session_end();
          }
        }
        this->defer_publish_(voice_active_sensor_, mic_now);
        // Direct trigger when the user opted in via voice_assistant_id:
        // skips the binary_sensor → on_press automation hop (~5–15 ms).
        // Still defers because voice_assistant entity APIs are loop-task only.
        if (mic_now && voice_assistant_ != nullptr) {
          this->defer([this] {
            voice_assistant_->request_start(/*continuous=*/false,
                                            /*silence_detection=*/true);
          });
        }
      }
      prev_buttons_ = btns;

      LockGuard guard(pulse_lock_);
      button_pulse_feed_buttons(pulse_, btns, now);
      return;
    }

    case SIRI_HANDLE_AUDIO:
      if (voice_enabled_) {
        siri_audio_dispatch_packet(data, len);
      }
      return;

    case SIRI_HANDLE_BATTERY:
      if (len >= 1) {
        const uint8_t lvl = data[0];
        if (lvl == last_battery_pct_) return;
        last_battery_pct_ = lvl;
        ESP_LOGI(TAG, "battery=%u%%", (unsigned) lvl);
        this->defer_publish_(battery_sensor_, static_cast<float>(lvl));
      }
      return;

    case SIRI_HANDLE_CHARGING:
      if (len >= 1) {
        const uint8_t b = data[0];
        if (b == last_charging_byte_) return;
        last_charging_byte_ = b;
        const char *s = siri_decode_charging_state(b);
        ESP_LOGI(TAG, "charging state=0x%02x => %s", b, s);
        this->defer_publish_(charging_text_sensor_, std::string(s));
      }
      return;

    case SIRI_HANDLE_TOUCH: {
      // Touch/gesture support is deliberately dropped — swipes are not a
      // feature of this firmware and nothing downstream consumes a frame.
      // We stay subscribed to the touch CCCD rather than removing its
      // SETUP_STEPS[] entry, because altering the setup chain risks the
      // always-connected wake behaviour for no real gain (~50 frames/sec,
      // and only while a finger is actually on the pad).
      if (!debug_touch_frames_) {
        return;
      }
      siri_touch_frame_t frame;
      if (siri_decode_touch_frame(data, len, &frame)) {
        ESP_LOGD(TAG,
                 "touch x=%" PRId32 " y=%" PRId32 " p=%u down=%d ctr=%" PRIu32,
                 frame.x, frame.y, (unsigned) frame.pressure,
                 (int) frame.finger_down, frame.remote_counter);
      }
      return;
    }
  }
}

void SiriRemoteHub::on_ble_connected(uint32_t idle_ms) {
  ESP_LOGI(TAG, "remote connected (idle %" PRIu32 " ms)", idle_ms);
}

void SiriRemoteHub::on_ble_disconnected() {
  ESP_LOGI(TAG, "remote disconnected");
  LockGuard guard(pulse_lock_);
  button_pulse_reset(pulse_);
}

void SiriRemoteHub::signal_response_finished() {
  if (this->voice_assistant_ == nullptr) return;
  api::APIConnection *client = this->voice_assistant_->get_api_connection();
  if (client == nullptr) return;
  api::VoiceAssistantAnnounceFinished msg;
  msg.success = true;
  if (!client->send_message(msg)) {
    ESP_LOGW(TAG, "failed to send VoiceAssistantAnnounceFinished");
  }
}

void SiriRemoteHub::on_audio_session_start() {
  // Real audio is about to flow — stop any post-release silence injection
  // so we don't mix silence with decoded frames.
  if (microphone_ != nullptr) {
    microphone_->stop_silence();
  }
#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  siri_audio_pcm_tcp_open(pcm_tcp_host_.c_str(), pcm_tcp_port_);
#endif
}

void SiriRemoteHub::on_audio_session_end() {
  // Apple stops sending audio packets at Mic-release. Without a continuous
  // signal HA's VAD stalls in "Listening" until its 9 s stt-stream-failed
  // timeout. Inject 20 ms-paced silence so VAD detects end-of-speech and
  // finalizes STT; the mic's own stop() (called by voice_assistant when it
  // transitions out of STREAMING_MICROPHONE) cancels the timer.
  if (microphone_ != nullptr) {
    microphone_->start_silence();
  }
#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  siri_audio_pcm_tcp_close();
#endif
}

void SiriRemoteHub::on_pcm(const int16_t *samples, size_t count) {
  if (microphone_ != nullptr) {
    microphone_->fire_data(samples, count);
  }
#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  siri_audio_pcm_tcp_send(samples, count);
#endif
}

// emit_event runs on the BLE host (or esp_timer) task — defer the HA event
// to the main loop. evt is invalid after we return, so we snapshot its
// fields by value into the lambda.
//
// Payload shape is the IR-style pulse: `repeat` indexes the pulse within a
// hold (0 = the press itself) and `held_ms` measures how long it has been
// down. Neither is a judgment — HA decides what counts as a click, a hold,
// or a ramp. There is no release event; the pulses simply stop.
void SiriRemoteHub::emit_event(const button_pulse_event_t *evt) {
  const char *btn = siri_button_name(evt->button);
  if (btn == nullptr) return;

  const uint32_t repeat = evt->repeat;
  const uint32_t held_ms = evt->held_ms;

  // The press is worth seeing at INFO; repeats arrive ~10x/sec and would
  // drown the log, so they sit at DEBUG.
  if (repeat == 0) {
    ESP_LOGI(TAG, "pulse button=%s press", btn);
  } else {
    ESP_LOGD(TAG, "pulse button=%s repeat=%" PRIu32 " held=%" PRIu32 "ms",
             btn, repeat, held_ms);
  }

  // siri_button_name() returns string literals (or NULL, filtered above), so
  // capturing the pointer is safe even though the deferred item outlives this
  // call. It also keeps string construction off this task: we run under the
  // pulse lock on the NimBLE / esp_timer task, and the loop task builds the
  // payload strings instead.
  this->defer([this, btn, repeat, held_ms] {
    // Gate on an actually-connected client. Without this, holding a button
    // across an HA restart makes APIServer log a dropped-event warning for
    // every pulse. Note is_connected() means "a client is attached", not
    // "it has subscribed to actions yet" — upstream warns about that short
    // window deliberately, and it is bounded, so we let it through.
    if (api::global_api_server == nullptr ||
        !api::global_api_server->is_connected()) {
      if (!this->api_gate_logged_) {
        this->api_gate_logged_ = true;
        ESP_LOGW(TAG, "Home Assistant not connected — dropping button pulses");
      }
      return;
    }
    if (this->api_gate_logged_) {
      this->api_gate_logged_ = false;
      ESP_LOGI(TAG, "Home Assistant reconnected — resuming button pulses");
    }

    // Must match what HA shows as the device name so a blueprint can filter
    // on device_attr(<device>, 'name'). get_friendly_name() is empty when
    // the user sets only `name:`, and HA falls back to `name` in exactly
    // that case — so mirror the fallback rather than emitting "".
    // Both names are fixed at compile time, so build the string once instead
    // of per pulse. Only this lambda touches it, always on the loop task.
    if (this->device_name_.empty()) {
      this->device_name_ = App.get_friendly_name().empty()
                               ? App.get_name().str()
                               : App.get_friendly_name().str();
    }
    // Static so the 26-char name (past the SSO limit) isn't heap-allocated as
    // a temporary on every pulse.
    static const std::string EVENT_NAME = "esphome.siri_remote_button";

    // ESPHome's own helper rather than a hand-built HomeassistantActionRequest:
    // the message's string field types changed in 2026.11 (StringRef ->
    // ProgmemStringRef), and upstream migrates this helper with them.
    // CustomAPIDevice is stateless, so a temporary is fine.
    //
    // Every value crosses the API as a string — HA compares event_data with
    // == and will not coerce, so automations must use `| int`.
    api::CustomAPIDevice().fire_homeassistant_event(
        EVENT_NAME,
        {
            {"device", this->device_name_},
            {"button", btn},
            {"repeat", std::to_string(repeat)},
            {"held_ms", std::to_string(held_ms)},
        });
  });
}

}  // namespace siri_remote
}  // namespace esphome
