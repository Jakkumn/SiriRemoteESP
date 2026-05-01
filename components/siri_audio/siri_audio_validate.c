// Pure packet validator for the gen-3 audio frame format. Lives in its
// own translation unit so the host test harness can link it without
// pulling in opus/FreeRTOS/esp-log. Layout reference is in siri_audio.h
// and memory/project_gen3_protocol.md.

#include "siri_audio_validate.h"

#include <stddef.h>
#include <stdint.h>

bool siri_audio_validate_packet(const uint8_t *data, size_t len, uint8_t *out_opus_len)
{
    if (out_opus_len != NULL) {
        *out_opus_len = 0;
    }
    if (data == NULL) {
        return false;
    }
    // Minimum: 5-byte header + 1-byte Opus payload (TOC alone is the
    // minimum legal Opus packet for any config).
    if (len < SIRI_AUDIO_HEADER_BYTES + 1 || len > SIRI_AUDIO_PACKET_BYTES) {
        return false;
    }
    if (data[SIRI_AUDIO_FRAME_OFFSET] != SIRI_AUDIO_OPUS_TOC) {
        return false;
    }
    uint8_t opus_pkt_len = data[SIRI_AUDIO_LEN_OFFSET];
    if (opus_pkt_len < 1) {
        return false;
    }
    // Opus packet must fit entirely within the received buffer.
    if ((size_t)(SIRI_AUDIO_FRAME_OFFSET + opus_pkt_len) > len) {
        return false;
    }
    if (out_opus_len != NULL) {
        *out_opus_len = opus_pkt_len;
    }
    return true;
}
