#ifndef MPU6500_H
#define MPU6500_H

#include "device.h"

#define MPU_MAXDEVS 1

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

enum MPU_DLPF {
	MPU_260DLPF	= 0,
	MPU_184DLPF	= 1,
	MPU_94DLPF	= 2,
	MPU_44DLPF	= 3,
	MPU_21DLPF	= 4,
	MPU_10DLPF	= 5,
	MPU_5DLPF	= 6
};

enum MPU_DEVTYPE {
	MPU_DEV6050 = 0x68,
	MPU_DEV6500 = 0x70
};

struct mpu_data {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	float afx, afy, afz;
	float gfx, gfy, gfz;
};

struct mpu_device {
	I2C_HandleTypeDef *hi2c;
	int devtype;
	enum MPU_ACCELSCALE accelscale;
	enum MPU_GYROSCALE gyroscale;
	enum MPU_DLPF dlpfwidth;
};

int mpu_initdevice(void *is, struct cdevice *dev);

#endif
