#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "global.h"
#include "util.h"

// delay timer handler
extern TIM_HandleTypeDef htim10;

// debug uart handler
extern UART_HandleTypeDef huart4;

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

	HAL_UART_Transmit(&huart4, (uint8_t *) buf, strlen(buf), 100);

	return 0;
}

int udelay(int us)
{
	int c;

	__HAL_TIM_SET_COUNTER(&htim10, 0);

	while ((c = __HAL_TIM_GET_COUNTER(&htim10)) < us);

	return 0;
}

int mdelay(int ms)
{
	int i;

	for (i = 0; i < ms; ++i)
		udelay(1000);

	return 0;
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
