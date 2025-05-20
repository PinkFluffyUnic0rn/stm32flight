#ifndef UARTCONF_H
#define UARTCONF_H

#include "device.h"

#define UART_MAXDEVS 1
#define UART_CMDSIZE 64

struct uart_device {
	UART_HandleTypeDef *huart;
};

int uart_initdevice(void *is, struct cdevice *dev);

#endif
