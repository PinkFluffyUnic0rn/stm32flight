#include "mcudef.h"

#include <periphconf.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "util.h"

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

char *ftos(float f, char *s, size_t sz, int order, float max)
{
	float scale[] = {10.0, 100.0, 1000.0, 10000.0,
		100000.0, 1000000.0, 10000000.0, 100000000.0}; 
	char tmp[16];
	int scaled;
	char *p;
	int sgn;
	int i;

	if (f < 0.0) {
		f *= -1;
		sgn = 1;
	}
	else
		sgn = 0;

	if (f > max)
		f = max;

	scaled = f * scale[order] + 0.5;

	for (p = tmp; ; ++p) {
		*p = '0' + (scaled % 10);
		scaled /= 10;

		if (scaled == 0)
			break;
	}

	if (sgn)
		*s++ = '-';

	for (i = 0; i <= (order - (p - tmp)); ++i) {
		*s++ = '0';

		if (i == 0 && order != (p - tmp))
			*s++ = '.';
	}

	for (; p != tmp; ) {
		if (sz == 1)
			break;

		if (p - tmp == order)
			*s++ = '.';

		*s++ = *(p--);
		--sz;
	}
	
	*s++ = '\0';

	return s;
}
