#ifndef LIS3MDL_H
#define LIS3MDL_H

#include "device.h"

#define LIS_MAXDEVS 1

enum LIS_RATE {
	LIS_RATE_0625	= 0x0,
	LIS_RATE_125	= 0x1,
	LIS_RATE_25	= 0x2,
	LIS_RATE_5	= 0x3,
	LIS_RATE_10	= 0x4,
	LIS_RATE_20	= 0x5,
	LIS_RATE_40	= 0x6,
	LIS_RATE_80	= 0x7,
};

enum LIS_SCALE {
	LIS_SCALE_4	= 0x0,
	LIS_SCALE_8	= 0x1,
	LIS_SCALE_12	= 0x2,
	LIS_SCALE_16	= 0x3
};

struct lis_data {
	int16_t x, y, z;
	float fx, fy, fz;
};

struct lis_device {
	I2C_HandleTypeDef *hi2c;
	enum LIS_RATE rate;
	enum LIS_SCALE scale;
};

int lis_initdevice(void *is, struct cdevice *dev);

#endif
