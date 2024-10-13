#ifndef QMC5883L_H
#define QMC5883L

#include "device.h"

#define QMC_MAXDEVS 1

enum QMC_RATE {
	QMC_RATE_10	= 0x0,
	QMC_RATE_50	= 0x1,
	QMC_RATE_100	= 0x2,
	QMC_RATE_200	= 0x3,
};

enum QMC_SCALE {
	QMC_SCALE_2	= 0,
	QMC_SCALE_8	= 1
};

enum QMC_OSR {
	QMC_OSR_512	= 0,
	QMC_OSR_256	= 1,
	QMC_OSR_128	= 2,
	QMC_OSR_64	= 3
};


struct qmc_data {
	int16_t x, y, z;
	float fx, fy, fz;
};

struct qmc_device {
	I2C_HandleTypeDef *hi2c;
	int devtype;
	enum QMC_RATE rate;
	enum QMC_SCALE scale;
	enum QMC_SCALE osr;
};

int qmc_initdevice(void *is, struct cdevice *dev);

#endif
