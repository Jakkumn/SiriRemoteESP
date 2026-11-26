#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Development-only sink that streams decoded PCM (s16le, 16 kHz, mono) to a
// TCP listener, for validating the decode path by ear:
//
//   nc -l <PORT> | ffplay -f s16le -ar 16000 -ac 1 -
//
// Shared by both build paths (CONFIG_DEBUG_AUDIO_PCM_TCP standalone, the
// `debug_pcm_tcp:` YAML key under ESPHome). Every failure is logged and the
// socket dropped; streaming resumes on the next session.
//
// open() is called from the session-start callback, which runs on the NimBLE
// host task, so it never blocks: it starts a non-blocking connect and returns.
// send() runs on the decode task and completes the connect there, dropping
// frames until the socket is writable. An unreachable host therefore costs
// audio frames on a debug channel, never BLE responsiveness.
void siri_audio_pcm_tcp_open(const char *host, uint16_t port);
void siri_audio_pcm_tcp_close(void);
void siri_audio_pcm_tcp_send(const int16_t *samples, size_t count);

#ifdef __cplusplus
}
#endif
