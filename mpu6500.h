#ifndef MPU6500_H
#define MPU6500_H

#include "driver.h"

#define MPU_MAXDEVS 4

enum MPU_ACCELSCALE {
	MPU_2G	= 0,
	MPU_4G	= 8,
	MPU_8G	= 0x10,
	MPU_16G	= 0x18
};

enum MPU_GYROSCALE {
	MPU_250DPS	= 0x0,
	MPU_500DPS	= 0x8,
	MPU_1000DPS	= 0x10,
	MPU_2000DPS	= 0x18
};

struct mpu_data {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	float afx, afy, afz;
	float gfx, gfy, gfz;
};

struct mpu_device {
	I2C_HandleTypeDef *hi2c;
	enum MPU_ACCELSCALE accelscale;
	enum MPU_GYROSCALE gyroscale;
};

int mpu_getdriver(struct driver *driver);

#endif
