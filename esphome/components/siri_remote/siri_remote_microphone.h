#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "esphome/components/microphone/microphone.h"
#include "esphome/core/component.h"

#include "siri_audio.h"

namespace esphome {
namespace siri_remote {

// Wraps the gen-3 audio decode pipeline (siri_audio) as an ESPHome
// Microphone, so voice_assistant can ingest decoded PCM.
//
// Apple only streams audio packets while the Mic button is held. After
// release, voice_assistant's ring buffer would go empty and HA's VAD
// would stall waiting for input — so siri_audio's decode task injects
// silence frames at the same 50 Hz cadence (one per 20 ms) for the
// post-release window, delivered through fire_data like real audio. That
// makes us look like a continuous-stream microphone to HA's VAD, which
// can then finalize STT naturally on detected silence.
class SiriRemoteMicrophone : public microphone::Microphone, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void start() override { this->state_ = microphone::STATE_RUNNING; }
  void stop() override {
    this->state_ = microphone::STATE_STOPPED;
    this->stop_silence();
  }

  // Called from the siri_audio decode task (CPU1 prio 5) via
  // SiriRemoteHub::on_pcm, for decoded audio and post-release silence alike.
  // Skips delivery while stopped. The data_callback contract takes a
  // std::vector<uint8_t>; voice_assistant copies out of it before returning,
  // so one reused member buffer serves every frame.
  void fire_data(const int16_t *samples, size_t count);

  // Called by SiriRemoteHub on Apple's audio session boundaries. start
  // begins 20 ms-paced silence injection (generated on the decode task);
  // a new session or stop() ends it.
  void start_silence() { siri_audio_silence_start(); }
  void stop_silence() { siri_audio_silence_stop(); }

 protected:
  // Frame buffer, touched only by the decode task — the single producer of
  // both real and silent frames. Reused so the ~50 frames/s of a voice
  // session don't each malloc/free 640 B of internal RAM.
  std::vector<uint8_t> pcm_buf_;
};

}  // namespace siri_remote
}  // namespace esphome
