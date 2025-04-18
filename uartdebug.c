#ifndef UARTDEBUG_H
#define UARTDEBUG_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "uartdebug.h"

extern UART_HandleTypeDef huart4;

int uartprintf(const char *format, ...)
{
	char buf[1024];
	va_list args;

	va_start(args, format);

	vsprintf(buf, format, args);

	HAL_UART_Transmit(&huart4, (uint8_t *) buf, strlen(buf), 100);
	
	return 0;
}

#endif
