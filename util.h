#ifndef UTIL_H
#define UTIL_H

// Print data into debug UART interface.
//
// format, ... -- arguments same as for printf.
int uartprintf(const char *format, ...);

// volatile version of memcpy.
//
// dest -- destination buffer.
// src -- source buffer.
void memcpyv(volatile void *dest, const volatile void *src, size_t n);

// microseconds delay.
//
// us -- delay in microseconds.
int udelay(int us);

// milliseconds delay.
//
// ms -- delay in microseconds.
int mdelay(int ms);

// is value in list.
//
// v -- value.
// num, ... -- list of values.
int isvalinlist(int v, int num, ...);

#endif
