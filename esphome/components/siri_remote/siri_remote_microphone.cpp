#include "siri_remote_microphone.h"

#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace siri_remote {

static const char *const TAG = "siri_remote.microphone";

// Match siri_audio's frame size so silence packets are interchangeable
// with real Opus-decoded frames at HA's STT/VAD layer.
static constexpr size_t SILENCE_FRAME_SAMPLES = 320;       // 20 ms @ 16 kHz
static constexpr uint64_t SILENCE_TICK_US = 20 * 1000;     // 20 ms

void SiriRemoteMicrophone::setup() {
  // gen-3 Opus decode produces 16 kHz S16LE mono — voice_assistant's
  // exact ingestion format, no conversion required.
  this->audio_stream_info_ = audio::AudioStreamInfo(16, 1, 16000);

  esp_timer_create_args_t ta = {};
  ta.callback = SiriRemoteMicrophone::silence_thunk;
  ta.arg = this;
  ta.name = "siri_mic_silence";
  esp_timer_create(&ta, &this->silence_timer_);
}

void SiriRemoteMicrophone::dump_config() {
  ESP_LOGCONFIG(TAG, "Siri Remote Microphone: 16 kHz S16LE mono");
}

void SiriRemoteMicrophone::fire_data(const int16_t *samples, size_t count) {
  if (this->state_ != microphone::STATE_RUNNING)
    return;
  const size_t bytes = count * sizeof(int16_t);
  std::vector<uint8_t> buf(bytes);
  std::memcpy(buf.data(), samples, bytes);
  this->data_callbacks_.call(buf);
}

void SiriRemoteMicrophone::start_silence() {
  if (this->silence_timer_ == nullptr)
    return;
  // Stop-then-start is safe whether or not the timer is currently armed.
  esp_timer_stop(this->silence_timer_);
  esp_timer_start_periodic(this->silence_timer_, SILENCE_TICK_US);
}

void SiriRemoteMicrophone::stop_silence() {
  if (this->silence_timer_ == nullptr)
    return;
  esp_timer_stop(this->silence_timer_);
}

void SiriRemoteMicrophone::silence_thunk(void *arg) {
  auto *self = static_cast<SiriRemoteMicrophone *>(arg);
  if (self->state_ != microphone::STATE_RUNNING)
    return;
  static const int16_t silence[SILENCE_FRAME_SAMPLES] = {0};
  self->fire_data(silence, SILENCE_FRAME_SAMPLES);
}

}  // namespace siri_remote
}  // namespace esphome
