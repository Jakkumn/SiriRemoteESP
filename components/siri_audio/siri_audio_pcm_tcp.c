#include "siri_audio_pcm_tcp.h"

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>

#include "esp_log.h"
#include "lwip/sockets.h"

static const char *TAG = "pcm_tcp";

// Written by open/close (NimBLE host task) and by send (decode task). Single
// words, so each access is atomic on Xtensa; the residual close-vs-send race is
// the same one the per-glue copies this replaced always had, and is tolerated
// on a debug-only path.
static volatile int s_sock = -1;
static volatile bool s_connected;
static char s_host[40];
static uint16_t s_port;

static void drop(int sock)
{
    if (s_sock == sock) {
        s_sock = -1;
        s_connected = false;
    }
    close(sock);
}

void siri_audio_pcm_tcp_open(const char *host, uint16_t port)
{
    siri_audio_pcm_tcp_close();

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(port),
    };
    if (inet_aton(host, &addr.sin_addr) == 0) {
        ESP_LOGW(TAG, "inet_aton('%s') failed", host);
        return;
    }
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        ESP_LOGW(TAG, "socket() rc=%d errno=%d", sock, errno);
        return;
    }
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags < 0 || fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGW(TAG, "fcntl(O_NONBLOCK) errno=%d", errno);
        close(sock);
        return;
    }
    int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (rc != 0 && errno != EINPROGRESS) {
        ESP_LOGW(TAG, "connect %s:%u errno=%d (run `nc -l %u` on the host)", host, (unsigned)port,
                 errno, (unsigned)port);
        close(sock);
        return;
    }
    snprintf(s_host, sizeof(s_host), "%s", host);
    s_port = port;
    s_connected = false;  // confirmed (and logged) by the first send()
    s_sock = sock;
}

void siri_audio_pcm_tcp_close(void)
{
    int sock = s_sock;
    s_sock = -1;
    s_connected = false;
    if (sock >= 0) {
        close(sock);
        ESP_LOGI(TAG, "stream closed");
    }
}

void siri_audio_pcm_tcp_send(const int16_t *samples, size_t count)
{
    int sock = s_sock;
    if (sock < 0) {
        return;
    }
    if (!s_connected) {
        fd_set wfds;
        FD_ZERO(&wfds);
        FD_SET(sock, &wfds);
        struct timeval tv = {0, 0};
        if (select(sock + 1, NULL, &wfds, NULL, &tv) <= 0) {
            return;  // still connecting; drop this frame
        }
        int err = 0;
        socklen_t len = sizeof(err);
        if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &err, &len) != 0 || err != 0) {
            ESP_LOGW(TAG, "connect %s:%u failed errno=%d (run `nc -l %u` on the host)", s_host,
                     (unsigned)s_port, err, (unsigned)s_port);
            drop(sock);
            return;
        }
        // Back to blocking for the sends themselves: a partial non-blocking
        // write would leave the stream on an odd byte and garble every sample
        // after it. Blocking here only ever stalls the decode task.
        int flags = fcntl(sock, F_GETFL, 0);
        if (flags >= 0) {
            (void)fcntl(sock, F_SETFL, flags & ~O_NONBLOCK);
        }
        s_connected = true;
        ESP_LOGI(TAG, "streaming to %s:%u", s_host, (unsigned)s_port);
    }
    if (send(sock, samples, count * sizeof(int16_t), 0) < 0) {
        ESP_LOGW(TAG, "send errno=%d; closing", errno);
        drop(sock);
    }
}
