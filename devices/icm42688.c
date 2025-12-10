#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#include "util.h"

#include "icm42688.h"

#define DELT 21
#define DELTSQR 440
#define BITSHIFT 6

enum ICM_REGISTER {
	ICM_WHOAMI		= 117,
	ICM_BANKSELECT		= 118,
	ICM_PWRMGMT		= 78,
	ICM_GYROCONFIG0		= 79,
	ICM_ACCELCONFIG0	= 80,
	ICM_GYROCONFIG1		= 81,
	ICM_GYROACCELCONFIG0	= 82,
	ICM_ACCELCONFIG1	= 83,
	ICM_TEMPMEASURE		= 29,
	ICM_ACCELMEASURE	= 31,
};

static struct icm_device icm_devs[ICM_MAXDEVS];
static size_t icm_devcount = 0;

int icm_write(struct icm_device *dev, uint8_t addr, uint8_t val)
{
	uint8_t sbuf[2];

	sbuf[0] = addr;
	sbuf[1] = val;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(dev->hspi, sbuf, 2, 100);

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return 0;
}

int icm_read(struct icm_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	uint8_t waddr = addr | 0x80;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(dev->hspi, &waddr, 1, 100);
	HAL_SPI_Receive(dev->hspi, data, size, 100);

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return 0;
}

int icm_getintdata(struct icm_device *dev, struct icm_data *data)
{
	uint8_t buf[14];

	icm_read(dev, ICM_TEMPMEASURE, buf, 14);

	data->t = buf[0] << 8 | buf[1];

	data->ax = buf[2] << 8 | buf[3];
	data->ay = buf[4] << 8 | buf[5];
	data->az = buf[6] << 8 | buf[7];

	data->gx = buf[8] << 8 | buf[9];
	data->gy = buf[10] << 8 | buf[11];
	data->gz = buf[12] << 8 | buf[13];

	return 0;
}

int icm_getdata(void *d, void *dt, size_t sz)
{
	struct icm_device *dev;
	struct icm_data *data;
	static int16_t accamp[] = { 0x800, 0x1000, 0x2000, 0x4000 };
	static float gyroamp[] = { 16.384, 32.768, 65.536, 131.072,
		262.144, 524.288, 1040.254, 2097.152};

	data = (struct icm_data *) dt;
	dev = (struct icm_device *) d;

	if (sz < sizeof(struct icm_data))
		return (-1);

	icm_getintdata(dev, data);

	data->ft = (data->t / 132.48) + 25.0;

	data->afx = (data->ax) / (float) accamp[dev->accelscale];
	data->afy = (data->ay) / (float) accamp[dev->accelscale];
	data->afz = (data->az) / (float) accamp[dev->accelscale];

	data->gfx = data->gx / gyroamp[dev->gyroscale];
	data->gfy = data->gy / gyroamp[dev->gyroscale];
	data->gfz = data->gz / gyroamp[dev->gyroscale];

	return 0;
}

int icm_selftest(struct icm_device *dev, struct icm_stdata *stdata)
{
	struct icm_data data;
	int32_t ax, ay, az, gx, gy, gz;
	int32_t stax, stay, staz, stgx, stgy, stgz;
	uint8_t stbuf[6];
	int i;

	icm_write(dev, 0x76, 0);
	icm_write(dev, ICM_PWRMGMT, 0x0f);
	icm_write(dev, ICM_GYROCONFIG0, ICM_250DPS << 5 | ICM_GYRO1K);
	icm_write(dev, ICM_ACCELCONFIG0, ICM_4G << 5 | ICM_ACCEL1K);
	icm_write(dev, ICM_GYROACCELCONFIG0, 0x44);

	mdelay(100);

	ax = ay = az = gx = gy = gz = 0;
	for (i = 0; i < 200; ++i) {
		icm_getintdata(dev, &data);

		ax += data.ax;	ay += data.ay;	az += data.az;
		gx += data.gx;	gy += data.gy;	gz += data.gz;

		mdelay(1);
	}

	ax /= 200;	ay /= 200;	az /= 200;
	gx /= 200;	gy /= 200;	gz /= 200;

	icm_write(dev, 0x70, 0x7f);

	mdelay(100);

	stax = stay = staz = stgx = stgy = stgz = 0;
	for (i = 0; i < 200; ++i) {
		icm_getintdata(dev, &data);

		stax += data.ax;	stay += data.ay;	staz += data.az;
		stgx += data.gx;	stgy += data.gy;	stgz += data.gz;

		mdelay(1);
	}

	stax /= 200;	stay /= 200;	staz /= 200;
	stgx /= 200;	stgy /= 200;	stgz /= 200;

	icm_write(dev, 0x70, 0x0);

	mdelay(100);

	icm_write(dev, 0x76, 1);
	icm_read(dev, 0x5f, stbuf, 3);

	icm_write(dev, 0x76, 2);
	icm_read(dev, 0x3b, stbuf + 3, 3);

	stdata->ax = fabsf(stax - ax)
		/ (13.1 * powf(1.01f, stbuf[3] - 1) + 0.5f);
	stdata->ay = fabsf(stay - ay)
		/ (13.1 * powf(1.01f, stbuf[4] - 1) + 0.5f);
	stdata->az = fabsf(staz - az)
		/ (13.1 * powf(1.01f, stbuf[5] - 1) + 0.5f);

	stdata->gx = fabsf(stgx - gx)
		/ (26.2 * powf(1.01f, stbuf[0] - 1) + 0.5f);
	stdata->gy = fabsf(stgy - gy)
		/ (26.2 * powf(1.01f, stbuf[1] - 1) + 0.5f);
	stdata->gz = fabsf(stgz - gz)
		/ (26.2 * powf(1.01f, stbuf[2] - 1) + 0.5f);

	return 0;
}

int icm_init(struct icm_device *dev)
{
	uint8_t check;
	uint8_t v;

	check = 0;
	icm_read(dev, ICM_WHOAMI, &check, 1);

	if (check != 0x47)
		return (-1);

	icm_write(dev, ICM_BANKSELECT, 0);

	icm_write(dev, ICM_PWRMGMT, 0x00);

	icm_write(dev, ICM_GYROACCELCONFIG0,
		dev->accellpf << 4 | dev->gyrolpf);

	icm_read(dev, 0x4d, &v, 1);
	icm_write(dev, 0x4d, (v & ~0xc0) | 0x40);
/*
	icm_write(dev, ICM_BANKSELECT, 1);

	icm_write(dev, 0x0c, DELT);
	icm_write(dev, 0x0d, DELTSQR & 0xff);
	icm_write(dev, 0x0e, (DELTSQR >> 8) | (BITSHIFT << 4));

	icm_write(dev, ICM_BANKSELECT, 2);

	icm_write(dev, 0x03, DELT << 1);
	icm_write(dev, 0x04, DELTSQR & 0xff);
	icm_write(dev, 0x05, (DELTSQR >> 8) | (BITSHIFT << 4));

	icm_write(dev, 0x76, 0);
*/

	icm_write(dev, ICM_PWRMGMT, 0x0f);

	icm_write(dev, ICM_GYROCONFIG0,
		dev->gyroscale << 5 | dev->gyrorate);
	icm_write(dev, ICM_ACCELCONFIG0,
		dev->accelscale << 5 | dev->accelrate);

	return 0;
}

int icm_configure(void *d, const char *cmd, ...)
{
	struct icm_device *dev;
	va_list args;

	dev = (struct icm_device *) d;

	va_start(args, cmd);

	if (strcmp(cmd, "self-test") == 0) {
		struct icm_stdata *data;
		size_t sz;

		data = va_arg(args, struct icm_stdata *);
		sz = va_arg(args, size_t);

		if (sz < sizeof(struct icm_stdata)) {
			va_end(args);
			return (-1);
		}

		icm_selftest(dev, data);
	}

	va_end(args);

	return 0;
}

int icm_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(icm_devs + icm_devcount, is, sizeof(struct icm_device));

	sprintf(dev->name, "%s_%d", "icm42688", icm_devcount);

	dev->priv = icm_devs + icm_devcount;
	dev->read = icm_getdata;
	dev->configure = icm_configure;
	dev->write = NULL;
	dev->interrupt = NULL;

	r = icm_init(icm_devs + icm_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
