#ifndef CRSF_H
#define CRSF_H

#include "device.h"

#define CRSF_MAXDEVS 1
#define CRSF_CHANNELCOUNT 10

struct crsf_device {
	UART_HandleTypeDef *huart;
};

struct crsf_data {
	float chf[CRSF_CHANNELCOUNT];
};

int crsf_initdevice(void *is, struct cdevice *dev);

#endif
