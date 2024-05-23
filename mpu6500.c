#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "mpu6500.h"

#define MPU_ADDR 0x68

enum MPU_REGISTER {
	MPU_WHOAMI		= 117,
	MPU_POWERMANAGEMENT	= 107,
	MPU_USERCONTROL		= 106,
	MPU_GYROCONF		= 27,
	MPU_ACCELCONF		= 28,
	MPU_ACCELMEASURE	= 59,
};

static struct mpu_device mpu_devs[MPU_MAXDEVS];
static size_t mpu_devcount = 0;

int mpu_write(struct mpu_device *dev, uint8_t addr, uint8_t val)
{
	HAL_I2C_Mem_Write(dev->hi2c, MPU_ADDR << 1, addr,
		1, &val, 1, 1000);

	return 0;
}

int mpu_read(struct mpu_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	HAL_I2C_Mem_Read(dev->hi2c, MPU_ADDR << 1, addr,
		1, data, size, 1000);

	return 0;
}

int mpu_getintdata(struct mpu_device *dev, struct mpu_data *data)
{
	uint8_t buf[14];

	mpu_read(dev, MPU_ACCELMEASURE, buf, 14);

	data->ax = buf[0] << 8 | buf[1];
	data->ay = buf[2] << 8 | buf[3];
	data->az = buf[4] << 8 | buf[5];

	data->gx = buf[8] << 8 | buf[9];
	data->gy = buf[10] << 8 | buf[11];
	data->gz = buf[12] << 8 | buf[13];

	return 0;
}

int mpu_getdata(void *d, size_t addr, void *dt, size_t sz)
{
	struct mpu_device *dev;
	struct mpu_data *data;
	static int16_t accamp[] = { 0x4000, 0x2000, 0x1000, 0x800 };
	static float gyroamp[] = { 131.072, 65.536, 32.768, 16.384 };

	data = (struct mpu_data *) dt;
	dev = (struct mpu_device *) d;

	if (sz < sizeof(struct mpu_data))
		return (-1);

	mpu_getintdata(dev, data);

	data->afx = (data->ax) / (float) accamp[dev->accelscale >> 3];
	data->afy = (data->ay) / (float) accamp[dev->accelscale >> 3];
	data->afz = (data->az) / (float) accamp[dev->accelscale >> 3];

	data->gfx = data->gx / gyroamp[dev->gyroscale >> 3];
	data->gfy = data->gy / gyroamp[dev->gyroscale >> 3];
	data->gfz = data->gz / gyroamp[dev->gyroscale >> 3];

	return 0;
}

int mpu_init(struct mpu_device *dev)
{
	uint8_t check;

	check = 0;	
	mpu_read(dev, MPU_WHOAMI, &check, 1);

	if (check != 0x70)
		return (-1);

	mpu_write(dev, MPU_POWERMANAGEMENT, 0x0);
	mpu_write(dev, MPU_ACCELCONF, dev->accelscale);
	mpu_write(dev, MPU_GYROCONF, dev->gyroscale);

	return 0;
}

int mpu_initdevice(void *is, struct device *dev)
{
	int r;

	memmove(mpu_devs + mpu_devcount, is, sizeof(struct mpu_device));
	
	sprintf(dev->name, "%s%d", "mpu6500", mpu_devcount);

	dev->type = DEVTYPE_CHAR;

	dev->priv = mpu_devs + mpu_devcount;

	dev->read = mpu_getdata;
	dev->write = NULL;
	dev->eraseall = NULL;
	dev->erasesector = NULL;
	dev->writesector = NULL;

	dev->writesize = 1;
	dev->sectorsize = 0;
	dev->totalsize = 0;

	r = mpu_init(mpu_devs + mpu_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}

int mpu_getdriver(struct driver *driver)
{
	driver->initdevice = mpu_initdevice;

	return 0;
}
