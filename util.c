#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "util.h"

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
