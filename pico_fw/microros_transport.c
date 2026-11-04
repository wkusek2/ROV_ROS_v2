#include "microros_transport.h"
#include <stdio.h>
#include <time.h>
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"

// potrzebne przez rcutils / uxr (brak libc POSIX na Pico)
void usleep(uint64_t us) { sleep_us(us); }

int clock_gettime(clockid_t unused, struct timespec *tp) {
    (void)unused;
    uint64_t m = time_us_64();
    tp->tv_sec = (time_t)(m / 1000000);
    tp->tv_nsec = (long)((m % 1000000) * 1000);
    return 0;
}

bool pico_usb_transport_open(struct uxrCustomTransport *t) { (void)t; return true; }
bool pico_usb_transport_close(struct uxrCustomTransport *t) { (void)t; return true; }

size_t pico_usb_transport_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err) {
    (void)t;
    if (!stdio_usb_connected()) { *err = 1; return 0; }
    // stdio_usb buforuje i wysyla blokami; bez CRLF (PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
    stdio_put_string((const char *)buf, (int)len, false, false);
    stdio_flush();
    return len;
}

size_t pico_usb_transport_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout_ms, uint8_t *err) {
    (void)t;
    uint64_t t0 = time_us_64();
    for (size_t i = 0; i < len; i++) {
        int64_t left = (int64_t)timeout_ms * 1000 - (int64_t)(time_us_64() - t0);
        if (left < 0) { *err = 1; return i; }
        int c = getchar_timeout_us((uint32_t)left);
        if (c == PICO_ERROR_TIMEOUT) { *err = 1; return i; }
        buf[i] = (uint8_t)c;
    }
    return len;
}
