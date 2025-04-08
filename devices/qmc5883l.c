#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "qmc5883l.h"

#define QMC_ADDR 0x0d

enum QMC_REGISTER {
	QMC_DATA	= 0x0,
	QMC_TOUT	= 0x7,
	QMC_CONF	= 0x9,
	QMC_MODE	= 0x2,
	QMC_SETRESET	= 0xb,
	QMC_ID		= 0xd
};

static struct qmc_device qmc_devs[QMC_MAXDEVS];
static size_t qmc_devcount = 0;

int qmc_write(struct qmc_device *dev, uint8_t addr, uint8_t val)
{
	HAL_I2C_Mem_Write(dev->hi2c, QMC_ADDR << 1, addr,
		1, &val, 1, 1000);

	return 0;
}

int qmc_read(struct qmc_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	HAL_I2C_Mem_Read(dev->hi2c, QMC_ADDR << 1, addr,
		1, data, size, 1000);

	return 0;
}

int qmc_getintdata(struct qmc_device *dev, struct qmc_data *data)
{
	uint8_t buf[6];

	qmc_read(dev, QMC_DATA, buf, 6);

	data->x = buf[0] | buf[1] << 8;
	data->y = buf[2] | buf[3] << 8;
	data->z = buf[4] | buf[5] << 8;
	
	return 0;
}

int qmc_getdata(void *d, void *dt, size_t sz)
{
	struct qmc_device *dev;
	struct qmc_data *data;

	data = (struct qmc_data *) dt;
	dev = (struct qmc_device *) d;

	if (sz < sizeof(struct qmc_data))
		return (-1);

	qmc_getintdata(dev, data);

	data->fx = (data->x) / 0.732421875;
	data->fy = (data->y) / 0.732421875;
	data->fz = (data->z) / 0.732421875;

	return 0;
}

int qmc_init(struct qmc_device *dev)
{
	uint8_t check;

	check = 0;	
	qmc_read(dev, QMC_ID, &check, 1);

	if (check != 0xff)
		return (-1);

	qmc_write(dev, QMC_SETRESET, 0x01);

	qmc_write(dev, QMC_CONF,
		dev->osr << 6 | dev->scale << 4 | dev->rate << 2 | 0x1);

	return 0;
}

int qmc_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(qmc_devs + qmc_devcount, is, sizeof(struct qmc_device));
	
	sprintf(dev->name, "%s_%d", "qmc5883l", qmc_devcount);

	dev->priv = qmc_devs + qmc_devcount;
	dev->read = qmc_getdata;
	dev->write = NULL;
	dev->interrupt = NULL;

	r = qmc_init(qmc_devs + qmc_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
