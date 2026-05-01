#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "siri_audio_validate.h"

// Helper: build a 99-byte well-formed gen-3 audio packet with a given Opus
// length. Header bytes are filled with 0xAA so the validator's TOC check
// at offset 5 is the only thing it will see.
static void build_packet(uint8_t *buf, uint8_t opus_len)
{
    memset(buf, 0xAA, SIRI_AUDIO_PACKET_BYTES);
    buf[SIRI_AUDIO_LEN_OFFSET] = opus_len;
    buf[SIRI_AUDIO_FRAME_OFFSET] = SIRI_AUDIO_OPUS_TOC;
}

static void test_validates_canonical_packet(void)
{
    uint8_t pkt[SIRI_AUDIO_PACKET_BYTES];
    build_packet(pkt, 80);  // typical Opus VBR length
    uint8_t out_len = 0;
    assert(siri_audio_validate_packet(pkt, sizeof(pkt), &out_len));
    assert(out_len == 80);
}

static void test_rejects_null_data(void)
{
    uint8_t out_len = 0xFF;
    assert(!siri_audio_validate_packet(NULL, SIRI_AUDIO_PACKET_BYTES, &out_len));
    // out_len must be cleared on failure so callers don't read stale state.
    assert(out_len == 0);
}

static void test_rejects_too_short(void)
{
    uint8_t pkt[SIRI_AUDIO_HEADER_BYTES] = {0};
    assert(!siri_audio_validate_packet(pkt, sizeof(pkt), NULL));
    // Single header byte is also rejected.
    assert(!siri_audio_validate_packet(pkt, 1, NULL));
}

static void test_rejects_too_long(void)
{
    uint8_t pkt[SIRI_AUDIO_PACKET_BYTES + 1];
    build_packet(pkt, 80);
    pkt[SIRI_AUDIO_PACKET_BYTES] = 0;
    assert(!siri_audio_validate_packet(pkt, sizeof(pkt), NULL));
}

static void test_rejects_wrong_toc(void)
{
    // Catches the historical 0xB8-as-separator bug: a packet whose offset-5
    // byte isn't the expected Opus TOC must be rejected before reaching
    // opus_decode().
    uint8_t pkt[SIRI_AUDIO_PACKET_BYTES];
    build_packet(pkt, 80);
    pkt[SIRI_AUDIO_FRAME_OFFSET] = 0xB7;  // off-by-one neighbor
    assert(!siri_audio_validate_packet(pkt, sizeof(pkt), NULL));
    pkt[SIRI_AUDIO_FRAME_OFFSET] = 0x00;
    assert(!siri_audio_validate_packet(pkt, sizeof(pkt), NULL));
}

static void test_rejects_zero_opus_len(void)
{
    uint8_t pkt[SIRI_AUDIO_PACKET_BYTES];
    build_packet(pkt, 0);
    assert(!siri_audio_validate_packet(pkt, sizeof(pkt), NULL));
}

static void test_rejects_oversize_opus_len(void)
{
    // opus_pkt_len that would extend past the packet buffer must be
    // rejected — feeding it to opus_decode would read past the input.
    uint8_t pkt[SIRI_AUDIO_PACKET_BYTES];
    build_packet(pkt, 95);  // 5 + 95 = 100, > 99
    assert(!siri_audio_validate_packet(pkt, sizeof(pkt), NULL));
}

static void test_accepts_max_opus_len(void)
{
    // Boundary: 5 + 94 = 99 fits exactly.
    uint8_t pkt[SIRI_AUDIO_PACKET_BYTES];
    build_packet(pkt, 94);
    uint8_t out_len = 0;
    assert(siri_audio_validate_packet(pkt, sizeof(pkt), &out_len));
    assert(out_len == 94);
}

static void test_accepts_min_opus_len(void)
{
    // Boundary: a single TOC byte is the minimum legal Opus packet.
    uint8_t pkt[SIRI_AUDIO_PACKET_BYTES];
    build_packet(pkt, 1);
    uint8_t out_len = 0;
    assert(siri_audio_validate_packet(pkt, sizeof(pkt), &out_len));
    assert(out_len == 1);
}

static void test_out_param_optional(void)
{
    uint8_t pkt[SIRI_AUDIO_PACKET_BYTES];
    build_packet(pkt, 50);
    assert(siri_audio_validate_packet(pkt, sizeof(pkt), NULL));
}

int main(void)
{
    test_validates_canonical_packet();
    test_rejects_null_data();
    test_rejects_too_short();
    test_rejects_too_long();
    test_rejects_wrong_toc();
    test_rejects_zero_opus_len();
    test_rejects_oversize_opus_len();
    test_accepts_max_opus_len();
    test_accepts_min_opus_len();
    test_out_param_optional();
    printf("test_siri_audio: ok\n");
    return 0;
}
