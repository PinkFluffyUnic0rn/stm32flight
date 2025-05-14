#ifndef UTIL_H
#define UTIL_H

int uartprintf(const char *format, ...);

void memcpyv(volatile void *dest, const volatile void *src, size_t n);

uint8_t crc8(const uint8_t *data, uint8_t len);

#endif
