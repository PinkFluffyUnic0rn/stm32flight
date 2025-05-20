#ifndef UTIL_H
#define UTIL_H

int uartprintf(const char *format, ...);

void memcpyv(volatile void *dest, const volatile void *src, size_t n);

int ndelay(int ns);

#endif
