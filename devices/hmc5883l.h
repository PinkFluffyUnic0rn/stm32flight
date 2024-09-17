#ifndef HMC5883L_H
#define HMC5883L

#include "device.h"

#define HMC_MAXDEVS 1

enum HMC_RATE {
	HMC_RATE_0_75	= 0x0,
	HMC_RATE_1_5	= 0x1,
	HMC_RATE_3	= 0x2,
	HMC_RATE_7_5	= 0x3,
	HMC_RATE_15	= 0x4,
	HMC_RATE_30	= 0x5,
	HMC_RATE_75	= 0x6
};

enum HMC_SCALE {
	HMC_SCALE_088	= 0,
	HMC_SCALE_1_3	= 1,
	HMC_SCALE_1_9	= 2,
	HMC_SCALE_2_5	= 3,
	HMC_SCALE_4_0	= 4,
	HMC_SCALE_4_7	= 5,
	HMC_SCALE_5_6	= 6,
	HMC_SCALE_8_1	= 7
};

struct hmc_data {
	int16_t x, y, z;
	float fx, fy, fz;
};

struct hmc_device {
	I2C_HandleTypeDef *hi2c;
	int devtype;
	enum HMC_RATE rate;
	enum HMC_SCALE scale;
};

int hmc_initdevice(void *is, struct cdevice *dev);

#endif
