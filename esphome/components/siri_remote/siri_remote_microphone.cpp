#include "siri_remote_microphone.h"

#include <algorithm>
#include <cstring>
#include <span>

#include "esphome/core/log.h"

namespace esphome {
namespace siri_remote {

static const char *const TAG = "siri_remote.microphone";

void SiriRemoteMicrophone::setup() {
  // gen-3 Opus decode produces 16 kHz S16LE mono — voice_assistant's
  // exact ingestion format, no conversion required.
  this->audio_stream_info_ = audio::AudioStreamInfo(16, 1, 16000);
}

void SiriRemoteMicrophone::dump_config() {
  ESP_LOGCONFIG(TAG, "Siri Remote Microphone: 16 kHz S16LE mono");
}

void SiriRemoteMicrophone::fire_data(const int16_t *samples, size_t count) {
  if (this->state_ != microphone::STATE_RUNNING || !this->has_data_callbacks_())
    return;
  const size_t bytes = std::min(count * sizeof(int16_t), this->pcm_buf_.size());
  std::memcpy(this->pcm_buf_.data(), samples, bytes);
  // ESPHome 2026.11's replacement for data_callbacks_.call(vector), which is
  // removed in 2027.5.0. Applies mute by zeroing pcm_buf_ in place.
  this->publish_audio_(std::span<uint8_t>(this->pcm_buf_.data(), bytes),
                       this->audio_stream_info_);
}

}  // namespace siri_remote
}  // namespace esphome
