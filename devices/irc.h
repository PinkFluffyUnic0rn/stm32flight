#ifndef IRC_H
#define IRC_H

#include "device.h"

#define IRC_MAXDEVS 1

struct irc_data {
	int power;
	int frequency;
};

struct irc_device {
	UART_HandleTypeDef *huart;
	uint32_t power;
	uint32_t frequency;
};

int irc_initdevice(void *is, struct cdevice *dev);

#endif
