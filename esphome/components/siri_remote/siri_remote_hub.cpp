#include "siri_remote_hub.h"

#include <cinttypes>
#include <string>

#include "esphome/components/api/api_pb2.h"
#include "esphome/components/api/api_server.h"
#include "esphome/core/helpers.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "siri_remote_microphone.h"

#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
#include "lwip/sockets.h"
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

// IDF ships ble_store_config_init in libbt.a but doesn't publish a header.
extern "C" void ble_store_config_init(void);

namespace esphome {
namespace siri_remote {

static const char *const TAG = "siri_remote";

static constexpr uint32_t TICK_PERIOD_MS = 50;

// Fallbacks for the case where YAML omits a Number entirely. Match the
// standalone build's CONFIG_EVENT_* defaults so a wrap missing all six
// Numbers still behaves like the standalone first-boot.
static constexpr uint32_t FALLBACK_DOUBLE_CLICK_MAX_MS = 300;
static constexpr uint32_t FALLBACK_HOLD_MIN_MS = 700;
static constexpr int32_t FALLBACK_SWIPE_MIN_DISTANCE = 40;
static constexpr int32_t FALLBACK_SWIPE_Y_PRIORITY = 30;
static constexpr uint16_t FALLBACK_BLE_SLAVE_LATENCY = 400;

static float knob_value(SiriRemoteNumber *n, float fallback) {
  return (n != nullptr && n->has_state()) ? n->state : fallback;
}

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

static void emit_thunk(const event_state_event_t *evt, void *user) {
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

static void nimble_host_task(void *param) {
  (void) param;
  nimble_port_run();
  nimble_port_freertos_deinit();
}

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

  event_state_config_t es_cfg = {};
  es_cfg.double_click_max_ms = static_cast<uint32_t>(
      knob_value(dbl_ms_number_, FALLBACK_DOUBLE_CLICK_MAX_MS));
  es_cfg.hold_min_ms = static_cast<uint32_t>(
      knob_value(hold_ms_number_, FALLBACK_HOLD_MIN_MS));
  es_cfg.swipe_min_distance = static_cast<int32_t>(
      knob_value(swipe_dist_number_, FALLBACK_SWIPE_MIN_DISTANCE));
  es_cfg.swipe_y_priority_threshold = static_cast<int32_t>(
      knob_value(swipe_y_pri_number_, FALLBACK_SWIPE_Y_PRIORITY));
  es_ = event_state_create(&es_cfg, emit_thunk, this);
  if (es_ == nullptr) {
    ESP_LOGE(TAG, "event_state_create failed");
    this->mark_failed();
    return;
  }

  if (voice_enabled_) {
    start_audio_();
  }

  start_tick_timer_();
  start_nimble_();

  // siri_ble_set_slave_latency caches the value internally and applies it
  // on the next CONN_UPDATE (or on the next bonded reconnect if no
  // connection is up yet). Safe to call after siri_ble_start.
  siri_ble_set_slave_latency(static_cast<uint16_t>(
      knob_value(ble_lat_number_, FALLBACK_BLE_SLAVE_LATENCY)));

  ESP_LOGI(TAG, "siri_remote setup complete");
}

void SiriRemoteHub::dump_config() {
  ESP_LOGCONFIG(TAG, "Siri Remote Hub:");
  ESP_LOGCONFIG(TAG, "  Voice enabled: %s", YESNO(voice_enabled_));
  ESP_LOGCONFIG(TAG, "  Idle-disconnect: %" PRIu32 " ms (0 = always-connected)",
                idle_disconnect_ms_);
  ESP_LOGCONFIG(TAG, "  Pairing-flush suppress: %" PRIu32 " ms",
                pairing_flush_suppress_ms_);
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
    ESP_LOGE(TAG, "siri_audio_start failed: %s", esp_err_to_name(rc));
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
  esp_log_level_set("NimBLE", ESP_LOG_WARN);

  ESP_ERROR_CHECK(nimble_port_init());
  ble_store_config_init();

  siri_ble_config_t ble_cfg = {};
  ble_cfg.on_notify = ble_notify_thunk;
  ble_cfg.on_connected = ble_connected_thunk;
  ble_cfg.on_disconnected = ble_disconnected_thunk;
  ble_cfg.user = this;
  ESP_ERROR_CHECK(siri_ble_start(&ble_cfg));

  nimble_port_freertos_init(nimble_host_task);
}

void SiriRemoteHub::tick() {
  uint32_t now = now_ms();
  {
    LockGuard guard(es_lock_);
    event_state_tick(es_, now);
  }
  if (idle_disconnect_ms_ > 0) {
    uint32_t last = last_activity_ms_;
    if (last != 0 && (now - last) > idle_disconnect_ms_) {
      last_activity_ms_ = 0;  // single-shot per idle window
      siri_ble_idle_disconnect();
    }
  }
}

void SiriRemoteHub::on_ble_notify(uint16_t attr_handle, const uint8_t *data,
                                  size_t len) {
  const uint32_t now = now_ms();
  last_activity_ms_ = now;

  switch (attr_handle) {
    case SIRI_HANDLE_BUTTON: {
      if (now < suppress_buttons_until_ms_) {
        ESP_LOGI(TAG, "suppressing buffered pairing-combo button notify");
        return;
      }
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
        if (voice_active_sensor_ != nullptr) {
          this->defer([this, mic_now] { voice_active_sensor_->publish_state(mic_now); });
        }
      }
      prev_buttons_ = btns;

      LockGuard guard(es_lock_);
      event_state_feed_buttons(es_, btns, now);
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
        if (battery_sensor_ != nullptr) {
          this->defer([this, lvl] { battery_sensor_->publish_state(lvl); });
        }
      }
      return;

    case SIRI_HANDLE_CHARGING:
      if (len >= 1) {
        const uint8_t b = data[0];
        if (b == last_charging_byte_) return;
        last_charging_byte_ = b;
        const char *s = siri_decode_charging_state(b);
        ESP_LOGI(TAG, "charging state=0x%02x => %s", b, s);
        if (charging_text_sensor_ != nullptr) {
          this->defer([this, s] { charging_text_sensor_->publish_state(s); });
        }
      }
      return;

    case SIRI_HANDLE_TOUCH: {
      siri_touch_frame_t frame;
      if (siri_decode_touch_frame(data, len, &frame)) {
        if (debug_touch_frames_ || raw_stream_enabled_) {
          ESP_LOGI(TAG,
                   "touch x=%" PRId32 " y=%" PRId32 " p=%u down=%d ctr=%" PRIu32,
                   frame.x, frame.y, (unsigned) frame.pressure,
                   (int) frame.finger_down, frame.remote_counter);
        }
        LockGuard guard(es_lock_);
        event_state_feed_touch(es_, &frame, now);
      }
      return;
    }
  }
}

void SiriRemoteHub::on_ble_connected(uint32_t idle_ms) {
  const uint32_t now = now_ms();
  ESP_LOGI(TAG, "remote connected (idle %" PRIu32 " ms)", idle_ms);
  last_activity_ms_ = now;
  if (idle_ms == 0) {
    // Fresh first-bond — squash the pairing-combo HID flush Apple delivers
    // right after the button CCCD subscribe lands.
    suppress_buttons_until_ms_ = now + pairing_flush_suppress_ms_;
  }
}

void SiriRemoteHub::on_ble_disconnected() {
  ESP_LOGI(TAG, "remote disconnected");
  last_activity_ms_ = 0;
  LockGuard guard(es_lock_);
  event_state_reset(es_, now_ms());
}

void SiriRemoteHub::apply_knob_change(SiriRemoteKnob kind, float value) {
  switch (kind) {
    case SiriRemoteKnob::SwipeYPriority: {
      LockGuard guard(es_lock_);
      event_state_set_swipe_y_priority(es_, static_cast<int32_t>(value));
      return;
    }
    case SiriRemoteKnob::SwipeMinDistance: {
      LockGuard guard(es_lock_);
      event_state_set_swipe_min_distance(es_, static_cast<int32_t>(value));
      return;
    }
    case SiriRemoteKnob::DoubleWindowMs: {
      LockGuard guard(es_lock_);
      event_state_set_double_click_max_ms(es_, static_cast<uint32_t>(value));
      return;
    }
    case SiriRemoteKnob::HoldThresholdMs: {
      LockGuard guard(es_lock_);
      event_state_set_hold_min_ms(es_, static_cast<uint32_t>(value));
      return;
    }
    case SiriRemoteKnob::BatteryLowPct:
      // No setter — bat_low_number_->state is read at publish_battery time
      // by future low-battery-derivation code. The Number persists for
      // forward use; runtime adjustment is the persistence side effect.
      return;
    case SiriRemoteKnob::BleSlaveLatency:
      siri_ble_set_slave_latency(static_cast<uint16_t>(value));
      return;
  }
}

void SiriRemoteHub::on_audio_session_start() {
  // Real audio is about to flow — stop any post-release silence injection
  // so we don't mix silence with decoded frames.
  if (microphone_ != nullptr) {
    microphone_->stop_silence();
  }
#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  pcm_tcp_open();
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
  pcm_tcp_close();
#endif
}

void SiriRemoteHub::on_pcm(const int16_t *samples, size_t count) {
  if (microphone_ != nullptr) {
    microphone_->fire_data(samples, count);
  }
#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
  pcm_tcp_send(samples, count);
#endif
}

// emit_event runs on the BLE host (or esp_timer) task — defer the HA
// service call to the main loop. evt is invalid after we return, so we
// snapshot its fields by value into the lambda.
void SiriRemoteHub::emit_event(const event_state_event_t *evt) {
  const char *type = event_state_action_name(evt->action);
  if (type == nullptr) return;

  const bool is_button_event = evt->action == EVT_CLICK
                            || evt->action == EVT_DOUBLE_CLICK
                            || evt->action == EVT_HOLD_START
                            || evt->action == EVT_HOLD_END;
  // hold_start fires at the threshold-cross with no duration yet; same for
  // double_click which carries no duration in the standalone payload.
  const bool has_duration = evt->action == EVT_CLICK || evt->action == EVT_HOLD_END;

  std::string action(type);
  std::string button;
  uint32_t duration_ms = evt->duration_ms;
  int32_t distance = evt->distance;

  if (is_button_event) {
    const char *btn = siri_button_name(evt->button);
    button.assign(btn != nullptr ? btn : "unknown");
    if (has_duration) {
      ESP_LOGI(TAG, "event=%s button=%s duration=%" PRIu32 "ms",
               type, button.c_str(), duration_ms);
    } else {
      ESP_LOGI(TAG, "event=%s button=%s", type, button.c_str());
    }
  } else {
    ESP_LOGI(TAG, "event=%s distance=%" PRId32, type, distance);
  }

  this->defer([action, button, duration_ms, distance, is_button_event, has_duration] {
    if (api::global_api_server == nullptr) return;
    // String storage must outlive the send call: every StringRef inside
    // resp points back into these locals (the FixedVector entries don't
    // copy). Captures-by-value in this lambda live until the lambda
    // returns, after which send_homeassistant_action has already
    // serialized the message.
    static const std::string SERVICE = "esphome.siri_remote_event";
    static const std::string K_ACTION = "action";
    static const std::string K_BUTTON = "button";
    static const std::string K_DURATION_MS = "duration_ms";
    static const std::string K_DISTANCE = "distance";
    const std::string duration_str = is_button_event && has_duration
                                       ? std::to_string(duration_ms)
                                       : std::string();
    const std::string distance_str = is_button_event ? std::string()
                                                     : std::to_string(distance);

    api::HomeassistantActionRequest resp;
    resp.service = StringRef(SERVICE);
    resp.is_event = true;

    const size_t n_pairs = is_button_event ? (has_duration ? 3 : 2) : 2;
    resp.data.init(n_pairs);

    auto &kv0 = resp.data.emplace_back();
    kv0.key = StringRef(K_ACTION);
    kv0.value = StringRef(action);

    if (is_button_event) {
      auto &kv1 = resp.data.emplace_back();
      kv1.key = StringRef(K_BUTTON);
      kv1.value = StringRef(button);
      if (has_duration) {
        auto &kv2 = resp.data.emplace_back();
        kv2.key = StringRef(K_DURATION_MS);
        kv2.value = StringRef(duration_str);
      }
    } else {
      auto &kv1 = resp.data.emplace_back();
      kv1.key = StringRef(K_DISTANCE);
      kv1.value = StringRef(distance_str);
    }

    api::global_api_server->send_homeassistant_action(resp);
  });
}

#ifdef SIRI_REMOTE_DEBUG_PCM_TCP
void SiriRemoteHub::pcm_tcp_open() {
  int sock = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0) {
    ESP_LOGW(TAG, "pcm_tcp: socket() rc=%d errno=%d", sock, errno);
    return;
  }
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(pcm_tcp_port_);
  if (::inet_aton(pcm_tcp_host_.c_str(), &addr.sin_addr) == 0) {
    ESP_LOGW(TAG, "pcm_tcp: inet_aton('%s') failed", pcm_tcp_host_.c_str());
    ::close(sock);
    return;
  }
  if (::connect(sock, (struct sockaddr *) &addr, sizeof(addr)) != 0) {
    ESP_LOGW(TAG, "pcm_tcp: connect %s:%u errno=%d (run `nc -l %u` on host)",
             pcm_tcp_host_.c_str(), (unsigned) pcm_tcp_port_, errno,
             (unsigned) pcm_tcp_port_);
    ::close(sock);
    return;
  }
  ESP_LOGI(TAG, "pcm_tcp: streaming to %s:%u", pcm_tcp_host_.c_str(),
           (unsigned) pcm_tcp_port_);
  pcm_socket_ = sock;
}

void SiriRemoteHub::pcm_tcp_close() {
  int sock = pcm_socket_;
  pcm_socket_ = -1;
  if (sock >= 0) {
    ::close(sock);
    ESP_LOGI(TAG, "pcm_tcp: stream closed");
  }
}

void SiriRemoteHub::pcm_tcp_send(const int16_t *samples, size_t count) {
  int sock = pcm_socket_;
  if (sock < 0)
    return;
  ssize_t n = ::send(sock, samples, count * sizeof(int16_t), 0);
  if (n < 0) {
    ESP_LOGW(TAG, "pcm_tcp: send errno=%d; closing", errno);
    pcm_socket_ = -1;
    ::close(sock);
  }
}
#endif  // SIRI_REMOTE_DEBUG_PCM_TCP

}  // namespace siri_remote
}  // namespace esphome
