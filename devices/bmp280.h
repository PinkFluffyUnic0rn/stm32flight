#ifndef BMP280_H
#define BMP280_H

#include "device.h"

#define BMP_MAXDEVS 1
#define BMP_WRITERETRIES 10

enum BMP_STANDBY {
	BMP_STANDBY_05		= 0x0,
	BMP_STANDBY_62		= 0x1,
	BMP_STANDBY_125		= 0x2,
	BMP_STANDBY_250		= 0x3,
	BMP_STANDBY_500		= 0x4,
	BMP_STANDBY_1000	= 0x5,
	BMP_STANDBY_2000	= 0x6,
	BMP_STANDBY_4000	= 0x7
};

enum BMP_IIR {
	BMP_IIR_0	= 0x0,
	BMP_IIR_2	= 0x2,
	BMP_IIR_4	= 0x4,
	BMP_IIR_8	= 0x8,
	BMP_IIR_16	= 0x10
};

enum BMP_OVERSAMPLING {
	BMP_OVERSAMPLING_0	= 0x0,
	BMP_OVERSAMPLING_1	= 0x1,
	BMP_OVERSAMPLING_2	= 0x2,
	BMP_OVERSAMPLING_4	= 0x3,
	BMP_OVERSAMPLING_8	= 0x4,
	BMP_OVERSAMPLING_16	= 0x5
};

enum BMP_MODE {
	BMP_MODE_SLEEP	= 0x0,
	BMP_MODE_FORCED	= 0x1,
	BMP_MODE_NORMAL	= 0x3,
};

struct bmp_data {
	float temp;
	float press;
	float alt;
};

struct bmp_device {
	I2C_HandleTypeDef *hi2c;

	int32_t t_fine;
	uint16_t digt1, digp1;
	int16_t digp[10];
	int16_t digt[4];
};

int bmp_initdevice(void *is, struct cdevice *dev);

#endif
