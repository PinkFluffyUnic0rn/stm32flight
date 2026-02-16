#include <string.h>
#include <stdio.h>
#include <math.h>

#include "dps368.h"

#define DPS_ADDR 0x77

enum DPS_COMMAND {
	DPS_PRS		= 0x00,
	DPS_TMP		= 0x03,
	DPS_PRSCFG	= 0x06,
	DPS_TMPCFG	= 0x07,
	DPS_MEASCFG	= 0x08,
	DPS_CFGREG	= 0x09,
	DPS_PRODID	= 0x0d,
	DPS_C0		= 0x10,
	DPS_C00		= 0x13,
	DPS_C11		= 0x1b,
};

static struct dps_device dps_devs[DPS_MAXDEVS];
static size_t dps_devcount = 0;

static int dps_write(struct dps_device *dev, uint8_t addr, uint8_t val)
{
	HAL_I2C_Mem_Write(dev->hi2c, DPS_ADDR << 1, addr,
		1, &val, 1, 1000);

	return 0;
}

static int dps_read(struct dps_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	HAL_I2C_Mem_Read(dev->hi2c, DPS_ADDR << 1, addr,
		1, data, size, 1000);

	return 0;
}

static int32_t twocompl(uint32_t v, uint8_t l)
{
	if (v & (int) 1 << (l - 1))
		return ((int32_t) v) - ((int32_t) 1 << l);

	return v;
}

int dps_getdata(void *d, void *dt, size_t sz)
{
	struct dps_device *dev;
	struct dps_data *data;
	float scf[] = { 524288.0, 1572864.0, 3670016.0, 7864320.0,
		253952.0, 516096.0, 1040384.0, 2088960.0 };
	static uint8_t buf[16];
	static int init = 0;
	float psc, tsc;

	data = (struct dps_data *) dt;
	dev = (struct dps_device *) d;

	if (!init) {
		dps_read(dev, DPS_PRS, buf, 6);

		init = 1;
	}

	psc = twocompl((buf[0] << 16) | (buf[1] << 8) | buf[2], 24)
		/ scf[dev->osr];
	tsc = twocompl((buf[3] << 16) | (buf[4] << 8) | buf[5], 24)
		/ scf[dev->osr];

	data->pressf = dev->c00
		+ psc * (dev->c10 + psc * (dev->c20 + psc * dev->c30))
		+ tsc * dev->c01
		+ tsc * psc * (dev->c11 + psc * dev->c21);

	data->tempf = (dev->c0 * 0.5 + dev->c1 * tsc);

	data->altf = (1.0f - powf(data->pressf / 101325.0f, 0.190295f))
		* 44330.0f;

	HAL_I2C_Mem_Read_DMA(dev->hi2c, DPS_ADDR << 1, DPS_PRS,
		1, buf, 6);

	return 0;
}

int dps_init(struct dps_device *dev)
{
	uint8_t data[16];
	int i;

	memset(data, 0, 16);

	dps_read(dev, DPS_PRODID, data, 1);

	if (data[0] != 0x10)
		return (-1);

	dps_write(dev, DPS_MEASCFG, 0x07);

	dps_write(dev, DPS_TMPCFG, 0x80 | dev->rate << 4 | dev->osr);

	dps_write(dev, DPS_PRSCFG, dev->rate << 4 | dev->osr);

	dps_write(dev, DPS_CFGREG, (dev->osr > 3) ? 0x0c : 0x00);

	dps_read(dev, DPS_C0, data, 3);

	dev->c0 = twocompl(data[0] << 4 | data[1] >> 4, 12);
	dev->c1 = twocompl((data[1] & 0x0f) << 8 | data[2], 12);

	dps_read(dev, DPS_C00, data, 8);
	dps_read(dev, DPS_C11, data + 8, 7);

	dev->c00 = twocompl(data[0] << 12 | data[1] << 4
		| data[2] >> 4, 20);
	dev->c10 = twocompl((data[2] & 0x0f) << 16 | data[3] << 8
		| data[4], 20);
	dev->c01 = twocompl(data[5] << 8 | data[6], 16);
	dev->c11 = twocompl(data[7] << 8 | data[8], 16);
	dev->c20 = twocompl(data[9] << 8 | data[10], 16);
	dev->c21 = twocompl(data[11] << 8 | data[12], 16);
	dev->c30 = twocompl(data[13] << 8 | data[14], 16);

	for (i = 0; i < 100; ++i) {
		struct dps_data d;

		dps_getdata(dev, &d, sizeof(struct dps_data));

		HAL_Delay(10);
	}

	return 0;
}

int dps_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(dps_devs + dps_devcount, is, sizeof(struct dps_device));

	sprintf(dev->name, "%s_%d", "dps368", dps_devcount);

	dev->priv = dps_devs + dps_devcount;
	dev->read = dps_getdata;
	dev->write = NULL;
	dev->interrupt = NULL;

	r = dps_init(dps_devs + dps_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
