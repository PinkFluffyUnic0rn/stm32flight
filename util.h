#ifndef UTIL_H
#define UTIL_H

// Get systick value as second in float value
#define getsysticksf() ((float) HAL_GetTick() / 1000.0)

// Check if int value is power of 2
#define ispow2(v) ((v != 0) && ((v & (v - 1)) == 0))

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
