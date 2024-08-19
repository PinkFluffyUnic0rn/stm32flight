#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "hmc5883l.h"

#define HMC_ADDR 0x1e

enum HMC_REGISTER {
	HMC_CONFA	= 0,
	HMC_CONFB	= 1,
	HMC_MODE	= 2,
	HMC_DATA	= 3,
	HMC_ID		= 10
};

static struct hmc_device hmc_devs[HMC_MAXDEVS];
static size_t hmc_devcount = 0;

int hmc_write(struct hmc_device *dev, uint8_t addr, uint8_t val)
{
	HAL_I2C_Mem_Write(dev->hi2c, HMC_ADDR << 1, addr,
		1, &val, 1, 1000);

	return 0;
}

int hmc_read(struct hmc_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	HAL_I2C_Mem_Read(dev->hi2c, HMC_ADDR << 1, addr,
		1, data, size, 1000);

	return 0;
}

int hmc_getintdata(struct hmc_device *dev, struct hmc_data *data)
{
	uint8_t buf[6];

	hmc_read(dev, HMC_DATA, buf, 6);

	data->x = buf[0] << 8 | buf[1];
	data->y = buf[4] << 8 | buf[5];
	data->z = buf[2] << 8 | buf[3];
	
	return 0;
}

int hmc_getdata(void *d, size_t addr, void *dt, size_t sz)
{
	struct hmc_device *dev;
	struct hmc_data *data;
	static float amp[] = { 0.73, 0.92, 1.22, 1.52,
		2.27, 2.56, 3.03 };

	data = (struct hmc_data *) dt;
	dev = (struct hmc_device *) d;

	if (sz < sizeof(struct hmc_data))
		return (-1);

	hmc_getintdata(dev, data);

	data->fx = (data->x) / amp[dev->scale];
	data->fy = (data->y) / amp[dev->scale];
	data->fz = (data->z) / amp[dev->scale];

	return 0;
}

int hmc_init(struct hmc_device *dev)
{
	char id[3];
	
	hmc_write(dev, HMC_MODE, 0x0);

	hmc_write(dev, HMC_CONFA, (0x2 << 5) | (dev->rate << 2));
	hmc_write(dev, HMC_CONFB, dev->scale << 5);

	memset(id, 0, 3);
	hmc_read(dev, HMC_ID, (uint8_t *) id, 3);

	if (strncmp("H43", id, 3) != 0)
		return (-1);

	return 0;
}

int hmc_initdevice(void *is, struct device *dev)
{
	int r;

	memmove(hmc_devs + hmc_devcount, is, sizeof(struct hmc_device));
	
	sprintf(dev->name, "%s%d", "hmc5883l", hmc_devcount);

	dev->type = DEVTYPE_CHAR;

	dev->priv = hmc_devs + hmc_devcount;

	dev->read = hmc_getdata;
	dev->write = NULL;
	dev->eraseall = NULL;
	dev->erasesector = NULL;
	dev->writesector = NULL;

	dev->writesize = 1;
	dev->sectorsize = 0;
	dev->totalsize = 0;

	r = hmc_init(hmc_devs + hmc_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}

int hmc_getdriver(struct driver *driver)
{
	driver->initdevice = hmc_initdevice;

	return 0;
}
