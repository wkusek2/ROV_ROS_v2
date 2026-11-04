#pragma once
// Transport micro-ROS (XRCE-DDS custom transport) po USB CDC Pico (stdio_usb), z ramkowaniem.
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>

bool   pico_usb_transport_open(struct uxrCustomTransport *t);
bool   pico_usb_transport_close(struct uxrCustomTransport *t);
size_t pico_usb_transport_write(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err);
size_t pico_usb_transport_read(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout_ms, uint8_t *err);
