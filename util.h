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

// submicroseconds delay.
//
// ns -- delay in nanoseconds, will be rounded to the closest possible
// 	value determined by MCU's clock frequency and delay timer
// 	prescaler. Maximum and minimum values also determined by
// 	MCU's clock and timer prescaler.
int ndelay(int ns);

int isvalinlist(int v, int num, ...);

#endif
