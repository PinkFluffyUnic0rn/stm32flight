#ifndef DSP368_H
#define DSP368_H

#include "device.h"

#define DPS_MAXDEVS 1

enum DPS_OSR {
	DPS_OSR_1 = 0,
	DPS_OSR_2 = 1,
	DPS_OSR_4 = 2,
	DPS_OSR_8 = 3,
	DPS_OSR_16 = 4,
	DPS_OSR_32 = 5,
	DPS_OSR_64 = 6,
	DPS_OSR_128 = 7
};

enum DPS_RATE {
	DPS_RATE_1 = 0,
	DPS_RATE_2 = 1,
	DPS_RATE_4 = 2,
	DPS_RATE_8 = 3,
	DPS_RATE_16 = 4,
	DPS_RATE_32 = 5,
	DPS_RATE_64 = 6,
	DPS_RATE_128 = 7,
};

struct dps_data {
	float altf;
	float pressf;
	float tempf;
};

struct dps_device {
	I2C_HandleTypeDef *hi2c;
	enum DPS_OSR osr;
	enum DPS_RATE rate;

	int16_t c0, c1;
	int32_t c00, c10, c01, c11, c20, c21, c30;
};

int dps_initdevice(void *is, struct cdevice *dev);

#endif
