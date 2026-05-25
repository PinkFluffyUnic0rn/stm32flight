#include <string.h>
#include <stdio.h>

#include "util.h"

#include "mmc5983ma.h"
#include "periphconf.h"

#define MMC_ADDR 0x30

enum MMC_REGISTER {
	MMC_DATA	= 0x0,
	MMC_TOUT	= 0x7,
	MMC_STATUS	= 0x8,
	MMC_CTRL0	= 0x9,
	MMC_CTRL1	= 0xa,
	MMC_CTRL2	= 0xb,
	MMC_CTRL3	= 0xc,
	MMC_ID		= 0x2f,
};

static struct mmc_device mmc_devs[MMC_MAXDEVS];
static size_t mmc_devcount = 0;

int mmc_write(struct mmc_device *dev, uint8_t addr, uint8_t val)
{
	HAL_I2C_Mem_Write(dev->hi2c, MMC_ADDR << 1, addr,
		1, &val, 1, 100);

	return 0;
}

int mmc_read(struct mmc_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	HAL_I2C_Mem_Read(dev->hi2c, MMC_ADDR << 1, addr,
		1, data, size, 100);

	return 0;
}

int mmc_getintdata(struct mmc_device *dev, struct mmc_data *data)
{
	static volatile uint8_t buf[7];
	static int init = 0;
	int t;

	if (!init) {
		mmc_read(dev, MMC_DATA, (uint8_t *) buf, 6);

		init = 1;
	}

	t = 0;
	while (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY
			&& t < 100000) {
		udelay(10);
		t += 10;
	}

	data->y = -(buf[0] << 10 | buf[1] << 2 | buf[6] >> 6);
	data->x = buf[2] << 10 | buf[3] << 2 | buf[6] >> 4;
	data->z = -(buf[4] << 10 | buf[5] << 2 | buf[6] >> 2);

	pconf_i2cmemread(dev->hi2c, MMC_ADDR << 1, MMC_DATA,
		(uint8_t *) buf, 6);

	return 0;
}

int mmc_getdata(void *d, void *dt, size_t sz)
{
	struct mmc_device *dev;
	struct mmc_data *data;

	data = (struct mmc_data *) dt;
	dev = (struct mmc_device *) d;

	if (sz < sizeof(struct mmc_data))
		return (-1);

	mmc_getintdata(dev, data);

	data->fx = 8.0 * (data->x) / 131072.0;
	data->fy = 8.0 * (data->y) / 131072.0;
	data->fz = 8.0 * (data->z) / 131072.0;

	return 0;
}

int mmc_init(struct mmc_device *dev)
{
	uint8_t check;

	check = 0;	
	mmc_read(dev, MMC_ID, &check, 1);

//	uartprintf("id: %hu\r\n", check);

	if (check != 0x30)
		return (-1);
/*
	mmc_write(dev, MMC_CONF,
		dev->osr << 6 | dev->scale << 4 | dev->rate << 2 | 0x1);
*/
	mmc_write(dev, MMC_CTRL0, 0x1 << 5);
	mmc_write(dev, MMC_CTRL1, dev->bw);
	// mmc_write(dev, MMC_CTRL2, 0x40 | 0x8 | dev->freq);
	mmc_write(dev, MMC_CTRL2, (0x1 << 7) | (0x1 << 6)
		| (0x1 << 3) | dev->freq);

	return 0;
}

int mmc_initdevice(struct mmc_device *is, struct cdevice *dev)
{
	int r;

	memmove(mmc_devs + mmc_devcount, is, sizeof(struct mmc_device));

	sprintf(dev->name, "%s_%d", "mmc5883ma", mmc_devcount);

	dev->priv = mmc_devs + mmc_devcount;
	dev->read = mmc_getdata;
	dev->write = NULL;
	dev->interrupt = NULL;

	r = mmc_init(mmc_devs + mmc_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
