#include "siri_remote_microphone.h"

#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace siri_remote {

static const char *const TAG = "siri_remote.microphone";

void SiriRemoteMicrophone::setup() {
  // gen-3 Opus decode produces 16 kHz S16LE mono — voice_assistant's
  // exact ingestion format, no conversion required.
  this->audio_stream_info_ = audio::AudioStreamInfo(16, 1, 16000);
  this->pcm_buf_.reserve(SIRI_AUDIO_FRAME_SAMPLES * sizeof(int16_t));
}

void SiriRemoteMicrophone::dump_config() {
  ESP_LOGCONFIG(TAG, "Siri Remote Microphone: 16 kHz S16LE mono");
}

void SiriRemoteMicrophone::fire_data(const int16_t *samples, size_t count) {
  if (this->state_ != microphone::STATE_RUNNING)
    return;
  const auto *bytes = reinterpret_cast<const uint8_t *>(samples);
  // assign() reuses the reserved capacity: no allocation after the first frame.
  this->pcm_buf_.assign(bytes, bytes + count * sizeof(int16_t));
  this->data_callbacks_.call(this->pcm_buf_);
}

}  // namespace siri_remote
}  // namespace esphome
