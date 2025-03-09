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

struct crsf_tele {
	float lat;
	float lon;
	float speed;
	float course;
	float alt;
	float balt;
	float vspeed;
	uint8_t sats;
	float pitch;
	float roll;
	float yaw;
};

int crsf_initdevice(void *is, struct cdevice *dev);

#endif
