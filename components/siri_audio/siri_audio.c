#include "siri_audio.h"

#include <inttypes.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "opus.h"

static const char *TAG = "siri_audio";

// Packet layout constants + siri_audio_validate_packet() live in
// siri_audio.h + siri_audio_validate.c so the host test harness can link
// the validator without pulling in opus / FreeRTOS / esp-log.

// Queue depth: 0.5 s of 50 Hz audio (~2.5 KB of internal RAM; it was 100 /
// ~10 KB). The only thing that can back it up is the decode task itself
// falling behind on CPU1 — the PCM consumers copy and return rather than
// waiting on the network — and Opus decodes a 20 ms frame in a small fraction
// of 20 ms. Anything queued longer than this is latency HA's VAD/STT would
// have to wade through anyway, so drop-oldest past it is the better failure.
#define SIRI_AUDIO_QUEUE_DEPTH 25

// Post-session silence pacing; see siri_audio_silence_start().
#define SILENCE_PERIOD_MS 20
// If the decode task is ever this many periods behind on silence, resync to
// now instead of bursting the backlog out.
#define SILENCE_MAX_LAG_PERIODS 5

// Decode task pinned to CPU1 (Wi-Fi + BLE controller default to CPU0).
// Priority 5: above idle (0) and the IDF event loop (~3), well below
// NimBLE host task (~7) so we never starve BLE.
#define DECODE_TASK_PRIORITY 5
#define DECODE_TASK_CORE 1
#define DECODE_TASK_STACK                                                                          \
    8192  // bumped above default 4 KB; Opus
          // VLAs land on the pseudostack
          // (micro-opus), but the task itself
          // does logging + callbacks.

typedef struct {
    uint8_t bytes[SIRI_AUDIO_PACKET_BYTES];
    uint8_t len;  // expected SIRI_AUDIO_PACKET_BYTES; tolerated otherwise
} audio_packet_t;

static siri_audio_config_t s_cfg;
static OpusDecoder *s_decoder;
static QueueHandle_t s_queue;
static TaskHandle_t s_decode_task;
static volatile bool s_session_active;
// Set by siri_audio_session_start() (NimBLE host task), consumed by the decode
// task. The decoder is only ever touched from the decode task: resetting it
// from the caller could land while opus_decode() is mid-frame on the previous
// session's last packet and corrupt its state.
static volatile bool s_reset_pending;
// Set/cleared from any task; read by the decode task, which emits the frames.
static volatile bool s_silence;
static const int16_t SILENCE_FRAME[SIRI_AUDIO_FRAME_SAMPLES];

// --- Decoder lifecycle ----------------------------------------------------

static esp_err_t allocate_decoder(void)
{
    int size = opus_decoder_get_size(1);  // 1 channel (mono)
    if (size <= 0) {
        ESP_LOGE(TAG, "opus_decoder_get_size returned %d", size);
        return ESP_FAIL;
    }
    // Place decoder state in PSRAM. ~30-50 KB; relieves internal SRAM
    // pressure and exercises the PSRAM heap pool we lit up in Phase 3.M.
    s_decoder = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_decoder == NULL) {
        ESP_LOGE(TAG, "PSRAM allocation for OpusDecoder (%d bytes) failed", size);
        return ESP_ERR_NO_MEM;
    }
    int err = opus_decoder_init(s_decoder, SIRI_AUDIO_SAMPLE_RATE, 1);
    if (err != OPUS_OK) {
        ESP_LOGE(TAG, "opus_decoder_init failed: %s", opus_strerror(err));
        heap_caps_free(s_decoder);
        s_decoder = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OpusDecoder ready: %d bytes in PSRAM, %d Hz mono", size, SIRI_AUDIO_SAMPLE_RATE);
    return ESP_OK;
}

// --- RMS energy (per-frame diagnostic) ------------------------------------

// Compute RMS of a 320-sample int16 frame as an integer 0..32767. Used to
// confirm the decoded PCM contains real audio (RMS rises during speech,
// stays low during silence) without needing a transport.
static uint16_t frame_rms(const int16_t *samples, size_t count)
{
    uint64_t sumsq = 0;
    for (size_t i = 0; i < count; i++) {
        int32_t s = samples[i];
        sumsq += (uint64_t)(s * s);
    }
    uint64_t mean = sumsq / (count > 0 ? count : 1);
    // sqrt via Newton's method bounded for uint16 range.
    uint32_t x = 256;
    for (int i = 0; i < 8; i++) {
        if (x == 0)
            break;
        x = (x + (uint32_t)(mean / x)) / 2;
    }
    return x > UINT16_MAX ? UINT16_MAX : (uint16_t)x;
}

// --- Decode task ----------------------------------------------------------

static void decode_task(void *arg)
{
    (void)arg;
    int16_t pcm[SIRI_AUDIO_FRAME_SAMPLES];
    audio_packet_t pkt;
    uint32_t frame_idx = 0;
    uint32_t err_count = 0;
    const TickType_t silence_ticks = pdMS_TO_TICKS(SILENCE_PERIOD_MS);
    bool silence_running = false;
    TickType_t silence_next = 0;

    while (1) {
        // Block indefinitely unless silence is being generated, in which case
        // the receive timeout is the 20 ms frame clock — absolute deadlines, so
        // decode work and scheduling jitter don't stretch the cadence.
        TickType_t wait = portMAX_DELAY;
        if (s_silence) {
            TickType_t now = xTaskGetTickCount();
            // Signed: now is normally *before* silence_next.
            if (!silence_running || (int32_t)(now - silence_next) >
                                        (int32_t)(silence_ticks * SILENCE_MAX_LAG_PERIODS)) {
                silence_running = true;
                silence_next = now + silence_ticks;
            }
            TickType_t until = silence_next - now;
            wait = (int32_t)until > 0 ? until : 0;
        } else {
            silence_running = false;
        }
        if (xQueueReceive(s_queue, &pkt, wait) != pdTRUE) {
            if (s_silence && s_cfg.on_pcm != NULL) {
                silence_next += silence_ticks;
                s_cfg.on_pcm(SILENCE_FRAME, SIRI_AUDIO_FRAME_SAMPLES, s_cfg.user);
            }
            continue;
        }
        if (pkt.len == 0) {
            continue;  // wake-up from siri_audio_silence_start(); re-evaluate
        }
        if (s_reset_pending) {
            // Fresh utterance: opus_decode caches last-frame state for FEC and
            // packet-loss concealment, and carrying it over from the previous
            // session would inject garbage at the start of this one. Done here,
            // before the new session's first packet, because this task owns the
            // decoder. Cleared first so a start that races this reset re-arms it.
            s_reset_pending = false;
            opus_decoder_ctl(s_decoder, OPUS_RESET_STATE);
        }
        uint8_t opus_pkt_len = 0;
        if (!siri_audio_validate_packet(pkt.bytes, pkt.len, &opus_pkt_len)) {
            // Shouldn't happen — dispatch path already validated. Log
            // for visibility into a previously-unseen packet shape.
            ESP_LOGW(TAG, "decoder: malformed packet (len=%u)", pkt.len);
            continue;
        }
        // Opus packet INCLUDES the TOC byte at offset 5. Don't strip it —
        // opus_decode needs it to know the config (sample rate, frame
        // duration, channel layout).
        const uint8_t *frame = &pkt.bytes[SIRI_AUDIO_FRAME_OFFSET];
        int frame_n = (int)opus_pkt_len;

        int samples =
            opus_decode(s_decoder, frame, frame_n, pcm, SIRI_AUDIO_FRAME_SAMPLES, /*decode_fec*/ 0);
        if (samples < 0) {
            err_count++;
            if (err_count <= 5 || (err_count % 50) == 0) {
                ESP_LOGW(TAG, "opus_decode err=%d (%s); frame_idx=%lu err_count=%lu", samples,
                         opus_strerror(samples), (unsigned long)frame_idx,
                         (unsigned long)err_count);
            }
            continue;
        }

        // Diagnostic: log RMS energy every ~1 s (every 50 frames at 50 Hz).
        if ((frame_idx % 50) == 0) {
            ESP_LOGI(TAG, "frame=%lu samples=%d rms=%u", (unsigned long)frame_idx, samples,
                     frame_rms(pcm, samples));
        }

        if (s_cfg.on_pcm != NULL) {
            s_cfg.on_pcm(pcm, (size_t)samples, s_cfg.user);
        }
        frame_idx++;
    }
}

// --- Public API -----------------------------------------------------------

esp_err_t siri_audio_start(const siri_audio_config_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_decoder != NULL) {
        return ESP_ERR_INVALID_STATE;  // already started
    }
    s_cfg = *cfg;

    esp_err_t rc = allocate_decoder();
    if (rc != ESP_OK) {
        return rc;
    }

    s_queue = xQueueCreate(SIRI_AUDIO_QUEUE_DEPTH, sizeof(audio_packet_t));
    if (s_queue == NULL) {
        ESP_LOGE(TAG, "xQueueCreate failed");
        heap_caps_free(s_decoder);
        s_decoder = NULL;
        return ESP_ERR_NO_MEM;
    }

    BaseType_t ok =
        xTaskCreatePinnedToCore(decode_task, "siri_audio_decode", DECODE_TASK_STACK, NULL,
                                DECODE_TASK_PRIORITY, &s_decode_task, DECODE_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        vQueueDelete(s_queue);
        s_queue = NULL;
        heap_caps_free(s_decoder);
        s_decoder = NULL;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "decode task started (CPU%d, prio %d, stack %d)", DECODE_TASK_CORE,
             DECODE_TASK_PRIORITY, DECODE_TASK_STACK);
    return ESP_OK;
}

void siri_audio_session_start(void)
{
    if (s_session_active)
        return;
    // Before s_session_active, so no packet of this session can be queued
    // ahead of the reset request.
    s_reset_pending = true;
    s_silence = false;  // real audio is about to flow
    s_session_active = true;
    ESP_LOGI(TAG, "session start");
    if (s_cfg.on_session_start != NULL) {
        s_cfg.on_session_start(s_cfg.user);
    }
}

void siri_audio_session_end(void)
{
    if (!s_session_active)
        return;
    s_session_active = false;

    // Drain any queued packets — past the end of the session, decoded
    // PCM would arrive late and confuse downstream consumers.
    if (s_queue != NULL) {
        audio_packet_t throwaway;
        while (xQueueReceive(s_queue, &throwaway, 0) == pdTRUE) { /* drop */
        }
    }
    ESP_LOGI(TAG, "session end");
    if (s_cfg.on_session_end != NULL) {
        s_cfg.on_session_end(s_cfg.user);
    }
}

void siri_audio_silence_start(void)
{
    if (s_queue == NULL) {
        return;
    }
    s_silence = true;
    // The decode task may be blocked forever on an empty queue; a zero-length
    // packet wakes it to pick up the new pacing. If the queue is full the task
    // is busy and will see the flag on its next pass anyway.
    audio_packet_t wake = {.len = 0};
    (void)xQueueSendToBack(s_queue, &wake, 0);
}

void siri_audio_silence_stop(void)
{
    s_silence = false;  // the decode task drops back to an untimed wait
}

bool siri_audio_session_active(void)
{
    return s_session_active;
}

void siri_audio_dispatch_packet(const uint8_t *data, size_t len)
{
    if (!s_session_active || s_queue == NULL) {
        return;  // not in a session, or not started — ignore
    }
    // Validate before copy so a wrong-shape notify (truncated, wrong TOC,
    // bogus length) is dropped without spending memcpy + queue cycles.
    if (!siri_audio_validate_packet(data, len, NULL)) {
        return;
    }

    audio_packet_t pkt;
    pkt.len = (uint8_t)len;
    memcpy(pkt.bytes, data, len);

    // Bounded queue with drop-oldest on overflow. xQueueSendToBack returns
    // pdFALSE if the queue is full; in that case yank one item and retry.
    if (xQueueSendToBack(s_queue, &pkt, 0) != pdTRUE) {
        audio_packet_t throwaway;
        (void)xQueueReceive(s_queue, &throwaway, 0);
        (void)xQueueSendToBack(s_queue, &pkt, 0);
    }
}
