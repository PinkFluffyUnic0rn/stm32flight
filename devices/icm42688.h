#ifndef ICM42688_H
#define ICM42688_H

#include "device.h"

#define ICM_MAXDEVS 1

enum ICM_GYROSCALE {
	ICM_2000DPS	= 0x0,
	ICM_1000DPS	= 0x1,
	ICM_500DPS	= 0x2,
	ICM_250DPS	= 0x3,
	ICM_125DPS	= 0x4,
	ICM_62_5DPS	= 0x5,
	ICM_31_25DPS	= 0x6,
	ICM_15_625DPS	= 0x7,
};

enum ICM_GYRORATE {
	ICM_GYRO32K	= 0x1,
	ICM_GYRO16K	= 0x2,
	ICM_GYRO8K	= 0x3,
	ICM_GYRO4K	= 0x4,
	ICM_GYRO2K	= 0x5,
	ICM_GYRO1K	= 0x6,
	ICM_GYRO200	= 0x7,
	ICM_GYRO100	= 0x8,
	ICM_GYRO50	= 0x9,
	ICM_GYRO2_5	= 0xa,
	ICM_GYRO1_25	= 0xb,
};

enum ICM_GYROORDER {
	ICM_GYROORDER1	= 0x1,
	ICM_GYROORDER2	= 0x2,
	ICM_GYROORDER3	= 0x3,
};

enum ICM_GYROLPF {
	ICM_GYROLPF2	= 0x0,
	ICM_GYROLPF4	= 0x1,
	ICM_GYROLPF5	= 0x2,
	ICM_GYROLPF8	= 0x3,
	ICM_GYROLPF10	= 0x4,
	ICM_GYROLPF16	= 0x5,
	ICM_GYROLPF20	= 0x6,
	ICM_GYROLPF40	= 0x7,
	ICM_GYROLPFLL	= 0x15,
};

enum ICM_ACCELSCALE {
	ICM_16G	= 0x0,
	ICM_8G	= 0x1,
	ICM_4G	= 0x2,
	ICM_2G	= 0x3
};

enum ICM_ACCELRATE {
	ICM_ACCEL32K	= 0x1,
	ICM_ACCEL16K	= 0x2,
	ICM_ACCEL8K	= 0x3,
	ICM_ACCEL4K	= 0x4,
	ICM_ACCEL2K	= 0x5,
	ICM_ACCEL1K	= 0x6,
	ICM_ACCEL200	= 0x7,
	ICM_ACCEL100	= 0x8,
	ICM_ACCEL50	= 0x9,
	ICM_ACCEL25	= 0xa,
	ICM_ACCEL12_5	= 0xb,
	ICM_ACCEL500	= 0xf
};

enum ICM_ACCELLPF {
	ICM_ACCELLPF2	= 0x0,
	ICM_ACCELLPF4	= 0x1,
	ICM_ACCELLPF5	= 0x2,
	ICM_ACCELLPF8	= 0x3,
	ICM_ACCELLPF10	= 0x4,
	ICM_ACCELLPF16	= 0x5,
	ICM_ACCELLPF20	= 0x6,
	ICM_ACCELLPF40	= 0x7,
	ICM_ACCELLPFLL	= 0x15
};

enum ICM_ACCELORDER {
	ICM_ACCELORDER1	= 0x1,
	ICM_ACCELORDER2	= 0x2,
	ICM_ACCELORDER3	= 0x3,
};

struct icm_data {
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	float afx, afy, afz;
	float gfx, gfy, gfz;
};

struct icm_stdata {
	float ax, ay, az;
	float gx, gy, gz;
};

struct icm_device {
	SPI_HandleTypeDef *hspi;

	GPIO_TypeDef *gpio;
	uint16_t pin;

	enum ICM_GYROSCALE gyroscale;
	enum ICM_GYRORATE gyrorate;
	enum ICM_GYROORDER gyroorder;
	enum ICM_GYROLPF gyrolpf;
	enum ICM_ACCELSCALE accelscale;
	enum ICM_ACCELRATE accelrate;
	enum ICM_ACCELLPF accellpf;
	enum ICM_ACCELORDER accelorder;
};

int icm_initdevice(void *is, struct cdevice *dev);

#endif
