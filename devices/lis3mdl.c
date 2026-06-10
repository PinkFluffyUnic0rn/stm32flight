#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "util.h"

#include "lis3mdl.h"
#include "periphconf.h"

#define LIS_ADDR 0x1c

enum LIS_REGISTER {
	LIS_OFFSETXL	= 0x05,
	LIS_OFFSETXH	= 0x06,
	LIS_OFFSETYL	= 0x07,
	LIS_OFFSETYH	= 0x08,
	LIS_OFFSETZL	= 0x09,
	LIS_OFFSETZH	= 0x0a,
	LIS_WHOAMI	= 0x0f,
	LIS_CTRLREG1	= 0x20,
	LIS_CTRLREG2	= 0x21,
	LIS_CTRLREG3	= 0x22,
	LIS_CTRLREG4	= 0x23,
	LIS_OUT		= 0x28
};

static struct lis_device lis_devs[LIS_MAXDEVS];
static size_t lis_devcount = 0;

int lis_write(struct lis_device *dev, uint8_t addr, uint8_t val)
{
	HAL_I2C_Mem_Write(dev->hi2c, LIS_ADDR << 1, addr,
		1, &val, 1, 1000);

	return 0;
}

int lis_read(struct lis_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	HAL_I2C_Mem_Read(dev->hi2c, LIS_ADDR << 1, addr,
		1, data, size, 1000);

	return 0;
}

int lis_getintdata(struct lis_device *dev, struct mag_data *data)
{
	static volatile uint8_t buf[6];
	static int init = 0;
	int t;

	if (!init) {
		lis_read(dev, LIS_OUT, (uint8_t *) buf, 6);

		init = 1;
	}

	t = 0;
	while (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY
			&& t < 100000) {
		udelay(10);
		t += 10;
	}

	data->y = -(buf[0] | buf[1] << 8);
	data->x = (buf[2] | buf[3] << 8);
	data->z = buf[4] | buf[5] << 8;

	pconf_i2cmemread(dev->hi2c, LIS_ADDR << 1, LIS_OUT,
		(uint8_t *) buf, 6);

	return 0;
}

int lis_getdata(void *d, void *dt, size_t sz)
{
	struct lis_device *dev;
	struct mag_data *data;
	static int16_t amp[] = { 6842, 3421, 2281, 1711 };

	data = (struct mag_data *) dt;
	dev = (struct lis_device *) d;

	if (sz < sizeof(struct mag_data))
		return (-1);

	lis_getintdata(dev, data);

	data->fx = data->x / (float) amp[dev->scale];
	data->fy = data->y / (float) amp[dev->scale];
	data->fz = data->z / (float) amp[dev->scale];

	return 0;
}

int lis_configure(void *d, const char *cmd, ...)
{
	struct lis_device *dev;
	va_list args;

	dev = (struct lis_device *) d;

	va_start(args, cmd);

	if (strcmp(cmd, "offset") == 0) {
		int16_t x, y, z;

		x = va_arg(args, int);
		y = va_arg(args, int);
		z = va_arg(args, int);

		lis_write(dev, LIS_OFFSETXL, x & 0xff);
		lis_write(dev, LIS_OFFSETXH, x >> 8);
		lis_write(dev, LIS_OFFSETYL, y & 0xff);
		lis_write(dev, LIS_OFFSETYH, y >> 8);
		lis_write(dev, LIS_OFFSETZL, z & 0xff);
		lis_write(dev, LIS_OFFSETZH, z >> 8);
	}

	va_end(args);

	HAL_Delay(1000);

	return 0;
}

int lis_init(struct lis_device *dev)
{
	uint8_t check;

	check = 0;
	lis_read(dev, LIS_WHOAMI, &check, 1);

	if (check != 0x3d)
		return (-1);

	lis_write(dev, LIS_CTRLREG1, 0x0 | dev->rate << 2 | 0x3 << 5);
	lis_write(dev, LIS_CTRLREG2, 0x0 | dev->scale << 5);
	lis_write(dev, LIS_CTRLREG3, 0x0);
	lis_write(dev, LIS_CTRLREG4, 0x3 << 2);

	return 0;
}

int lis_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(lis_devs + lis_devcount, is, sizeof(struct lis_device));

	sprintf(dev->name, "%s_%d", "lis3mdl", lis_devcount);

	dev->priv = lis_devs + lis_devcount;
	dev->read = lis_getdata;
	dev->configure = lis_configure;
	dev->write = NULL;
	dev->interrupt = NULL;

	r = lis_init(lis_devs + lis_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
