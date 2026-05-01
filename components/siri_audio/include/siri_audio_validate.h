#pragma once

// Pure validator + packet-layout constants for the gen-3 BLE audio frame
// format. Split from siri_audio.h so the host test harness can compile
// just the validator without pulling in esp_err.h / FreeRTOS / opus.

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Gen-3 audio packet layout on BLE notify (handle 0x0035). 99 bytes total
// = 5-byte application header + Opus packet starting at offset 5. The 0xB8
// byte at offset 5 is the Opus TOC, NOT a separator — it must be passed
// to opus_decode() as part of the frame. See memory/project_gen3_protocol.md.
#define SIRI_AUDIO_PACKET_BYTES 99
#define SIRI_AUDIO_HEADER_BYTES 5
#define SIRI_AUDIO_OPUS_TOC 0xB8
#define SIRI_AUDIO_LEN_OFFSET 4
#define SIRI_AUDIO_FRAME_OFFSET 5

// Validate a raw BLE notify payload as a gen-3 audio packet. On success
// returns true and (if out_opus_len is non-NULL) writes the Opus packet
// length (TOC + payload) to *out_opus_len. On failure returns false and
// sets *out_opus_len to 0 (when non-NULL). Pure function — safe to call
// from any context.
bool siri_audio_validate_packet(const uint8_t *data, size_t len, uint8_t *out_opus_len);

#ifdef __cplusplus
}
#endif
