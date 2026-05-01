#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_timer.h"

#include "esphome/components/microphone/microphone.h"
#include "esphome/core/component.h"

namespace esphome {
namespace siri_remote {

// Wraps the gen-3 audio decode pipeline (siri_audio) as an ESPHome
// Microphone, so voice_assistant can ingest decoded PCM.
//
// Apple only streams audio packets while the Mic button is held. After
// release, voice_assistant's ring buffer would go empty and HA's VAD
// would stall waiting for input — so we inject silence frames at the
// same 50 Hz cadence (one per 20 ms) for the post-release window. That
// makes us look like a continuous-stream microphone to HA's VAD, which
// can then finalize STT naturally on detected silence.
class SiriRemoteMicrophone : public microphone::Microphone, public Component {
 public:
  // Base classes (microphone::Microphone, Component) lack virtual destructors,
  // so don't mark this override — but we still need it to free the
  // periodic-silence timer if the component is ever torn down.
  ~SiriRemoteMicrophone() {
    if (this->silence_timer_ != nullptr) {
      esp_timer_stop(this->silence_timer_);
      esp_timer_delete(this->silence_timer_);
    }
  }
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void start() override { this->state_ = microphone::STATE_RUNNING; }
  void stop() override {
    this->state_ = microphone::STATE_STOPPED;
    this->stop_silence();
  }

  // Called from the siri_audio decode task (CPU1 prio 5) via
  // SiriRemoteHub::on_pcm. Skips delivery while stopped. Allocates a
  // std::vector<uint8_t> per frame because that's the data_callback
  // contract — voice_assistant copies into its ring buffer immediately,
  // so the vector dies at the end of this call.
  void fire_data(const int16_t *samples, size_t count);

  // Called by SiriRemoteHub on Apple's audio session boundaries. start
  // begins 20 ms-paced silence injection; the next on_pcm path stops
  // it via stop_silence() to avoid mixing real audio with zeros.
  void start_silence();
  void stop_silence();

 protected:
  static void silence_thunk(void *arg);

  esp_timer_handle_t silence_timer_{nullptr};
};

}  // namespace siri_remote
}  // namespace esphome
