#pragma once

// siri_audio — Phase 5 voice capture for the Apple Siri Remote (gen-3).
//
// Receives raw 99-byte audio packets from siri_ble (BLE notify on
// SIRI_HANDLE_AUDIO = 0x0035), strips the 6-byte gen-3 header, decodes
// the Opus frame (16 kHz mono, 20 ms = 320 samples per frame), and
// emits PCM int16_t frames via a user callback.
//
// Architecture (Phase 5.A.2):
//   - opus_decode() runs on a dedicated task pinned to CPU1, fed by a
//     FreeRTOS queue. Keeps the BLE host task (NimBLE) unblocked.
//   - OpusDecoder + pseudostack live in PSRAM (heap_caps_malloc with
//     MALLOC_CAP_SPIRAM). First real consumer of the PSRAM heap pool
//     enabled in Phase 3.M.
//   - Decoder is created at start, reused across mic sessions (only
//     OPUS_RESET_STATE between sessions). Avoids 30-50 KB allocation
//     churn per Mic-press.
//
// Session lifecycle:
//   - main.c calls siri_audio_session_start() when the Mic button bit
//     transitions 0 → 1 on the button bitmap.
//   - main.c calls siri_audio_dispatch_packet() for every notify on
//     handle SIRI_HANDLE_AUDIO during an active session.
//   - main.c calls siri_audio_session_end() when the Mic bit clears.
//   - For each successfully-decoded Opus frame, the registered
//     pcm_cb fires with 320 int16_t samples.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "siri_audio_validate.h"

#ifdef __cplusplus
extern "C" {
#endif

// 16 kHz × 20 ms = 320 samples per Opus frame for Siri Remote audio.
#define SIRI_AUDIO_FRAME_SAMPLES 320
#define SIRI_AUDIO_SAMPLE_RATE 16000

// PCM frame callback. `samples` is a buffer of `count` 16-bit signed PCM
// samples (mono, 16 kHz). Buffer is reused on the next decoded frame —
// copy if the consumer needs to retain. Called from the decode task
// (priority 5 on CPU1), not the BLE host task. Consumer is responsible
// for any synchronization needed to deliver downstream.
typedef void (*siri_audio_pcm_cb_t)(const int16_t *samples, size_t count, void *user);

// Session-edge callbacks. Fire from main.c thread when invoked via
// siri_audio_session_start/end (so on the same thread as the caller).
typedef void (*siri_audio_session_cb_t)(void *user);

typedef struct {
    siri_audio_pcm_cb_t on_pcm;
    siri_audio_session_cb_t on_session_start;  // optional
    siri_audio_session_cb_t on_session_end;    // optional
    void *user;
} siri_audio_config_t;

// Allocate decoder + queue + decode task. Call once at boot, after PSRAM
// is initialized. Returns ESP_FAIL on PSRAM allocation failure (decoder
// state needs ~50 KB).
esp_err_t siri_audio_start(const siri_audio_config_t *cfg);

// Mark the start / end of a Mic-button-driven audio session. Resets
// decoder state on start; drains the queue on end. Idempotent — calling
// session_start when one is already active is a no-op.
void siri_audio_session_start(void);
void siri_audio_session_end(void);

// Forward a 99-byte BLE notify payload from handle 0x0035 to the decode
// task. Validates the gen-3 header (0xB8 marker at offset 5, plausible
// length at offset 4); on validation failure the packet is dropped with
// a log warning. Drops the oldest queued packet on overflow (audio is
// real-time; stale frames are worse than missing).
//
// Safe to call from the BLE host task (NimBLE notify dispatch).
void siri_audio_dispatch_packet(const uint8_t *data, size_t len);

// True iff a session is currently active (between start/end).
bool siri_audio_session_active(void);

#ifdef __cplusplus
}
#endif
