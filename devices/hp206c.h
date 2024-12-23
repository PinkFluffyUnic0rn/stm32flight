#ifndef HP206C_H
#define HP206C_H

#include "device.h"

#define HP_MAXDEVS 1

enum HP_OSR {
	HP_OSR_4096 = 0,
	HP_OSR_2048 = 1,
	HP_OSR_1024 = 2,
	HP_OSR_512 = 3,
	HP_OSR_256 = 4,
	HP_OSR_128 = 5
};

struct hp_data {
	int32_t alt;
	int32_t temp;
	float altf;
	float tempf;
};

struct hp_device {
	I2C_HandleTypeDef *hi2c;
	enum HP_OSR osr;
	float alt0;
};

int hp_initdevice(void *is, struct cdevice *dev);

#endif
