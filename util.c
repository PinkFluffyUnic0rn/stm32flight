#include "mcudef.h"

#include <periphconf.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "util.h"

// ftos helping function, that
// append character to resulting
// string and jumping to end of
// the function in case if next
// append will cause buffer overflow
#define FTOS_STRAPPEND(s, sz, v, end)	\
do {					\
	*(s)++ = v;			\
	if (--(sz) == 1)		\
		goto end;		\
} while (0);

void memcpyv(volatile void *dest, const volatile void *src, size_t n)
{
	int i;

	for (i = 0; i < n; ++i)
		((uint8_t *) dest)[i] = ((uint8_t *) src)[i];
}

int uartprintf(const char *format, ...)
{
	char buf[1024];
	va_list args;
	
	va_start(args, format);

	vsprintf(buf, format, args);

	Dev[DEV_UART].write(Dev[DEV_UART].priv, buf, strlen(buf));

	return 0;
}

int udelay(int us)
{
	int c;

	__HAL_TIM_SET_COUNTER(pconf_delayhtim, 0);

	while ((c = __HAL_TIM_GET_COUNTER(pconf_delayhtim)) < us);

	return 0;
}

int mdelay(int ms)
{
	int i;

	for (i = 0; i < ms; ++i)
		udelay(1000);

	return 0;
}

int ucounterstart()
{
	__HAL_TIM_SET_COUNTER(pconf_delayhtim, 0);

	return 0;
}

int ucounterget()
{
	return __HAL_TIM_GET_COUNTER(pconf_delayhtim);
}

int isvalinlist(int v, int num, ...)
{
	va_list args;
	int i;

	va_start(args, num);

	for (i = 0; i < num; ++i) {
		if (v == va_arg(args, int)) {
			va_end(args);
			return 1;
		}
	}

	va_end(args);

	return 0;
}

char *ftos(float f, char *s, size_t sz, int zfill, int order)
{
	float scale[] = {1, 10.0, 100.0, 1000.0, 10000.0,
		100000.0, 1000000.0, 10000000.0, 100000000.0};
	float max[] = { 2147483647, 214748364, 21474836, 2147483,
		214748, 21474, 2147, 214, 21};

	char tmp[16];
	int scaled;
	char *p;
	int i;

	// if number is negative, append '-' to
	// resulting string and make number positive
	if (f < 0.0) {
		f *= -1;
		FTOS_STRAPPEND(s, sz, '-', end)
	}

	// if number scaling will int result integer
	// overflow, clip it to maximum possible number
	if (f > max[order])
		f = max[order];

	// scale number to preserve requested
	// digits count of fraction digits,
	// round number, if more than zero
	// fraction digits requested
	scaled = f * scale[order] + ((order == 0) ? 0.0 : 0.5);

	// extract digits from scaled number
	p = tmp;
	do {
		*p++ = '0' + (scaled % 10);
		scaled /= 10;
	} while (scaled != 0);

	// in case of integer output, fill it
	// with required number of zeros
	if (order == 0 && zfill > (p - tmp) ) {
		int zeros;

		zeros = zfill - (p - tmp);

		for (i = 0; i < zeros; ++i) {
			FTOS_STRAPPEND(s, sz, '0', end)
		}
	}

	// append zeros to resulting string
	for (i = 0; i <= order - (int) (p - tmp); ++i) {
		// append zero
		FTOS_STRAPPEND(s, sz, '0', end)

		// append decimal point to resulting string,
		// if more than one zero were appended
		if (i == 0 && order != p - tmp)
			FTOS_STRAPPEND(s, sz, '.', end)
	}

	// append extracted digits to resulting string
	do {
		// append decimal point to resulting string,
		// after integer part, if fraction part exists
		if (p - tmp == order)
			FTOS_STRAPPEND(s, sz, '.', end)

		// append digit
		FTOS_STRAPPEND(s, sz, *(--p), end)
	} while (p != tmp);

end:
	// make string null terminated
	*s++ = '\0';

	// return pointer to position in output buffer
	// right after end of resulting string
	return s;
}
