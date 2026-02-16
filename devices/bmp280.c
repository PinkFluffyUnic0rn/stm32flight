#include <string.h>
#include <stdio.h>

#include "util.h"

#include "bmp280.h"

#define BMP_ADDR 0x76

enum BMP_REGISTER {
	BMP_ID		= 0xd0,
	BMP_RESET	= 0xe0,
	BMP_STATUS	= 0xf3,
	BMP_CTRLMEAS	= 0xf4,
	BMP_CONFIG	= 0xf5,
	BMP_PRESS	= 0xf7,
	BMP_TEMP	= 0xfa,
	BMP_CALIB	= 0x88
};

static struct bmp_device bmp_devs[BMP_MAXDEVS];
static size_t bmp_devcount = 0;

int bmp_read(struct bmp_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	if (HAL_I2C_Mem_Read(dev->hi2c, BMP_ADDR << 1, addr,
		1, data, size, 1000) != HAL_OK)
		return (-1);

	return 0;
}

int bmp_waitwrite(struct bmp_device *dev)
{
	uint8_t s;
	int i;

	s = 1;

	for (i = 0; i < BMP_WRITERETRIES; ++i) {
		if (bmp_read(dev, BMP_STATUS, &s, 1) == 0 && !(s & 0x1))
			break;
		mdelay(100);
	}

	if (i >= BMP_WRITERETRIES)
		return (-1);

	return 0;
}

int bmp_write(struct bmp_device *dev, uint8_t addr, uint8_t val)
{
	HAL_I2C_Mem_Write(dev->hi2c, BMP_ADDR << 1, addr,
		1, &val, 1, 1000);

	return bmp_waitwrite(dev);
}

float bmp_tempf(struct bmp_device *dev, int32_t adc_T)
{
	float var1, var2, T;

	var1 = (((float) adc_T) / 16384.0
		- ((float) dev->digt1) / 1024.0) * ((float) dev->digt[2]);
	var2 = ((((float) adc_T) / 131072.0
		- ((float) dev->digt1) / 8192.0) *
		(((float) adc_T) / 131072.0 - ((float) dev->digt1) / 8192.0))
			* ((float) dev->digt[3]);

	dev->t_fine = (int32_t) (var1 + var2);
	T = (var1 + var2) / 5120.0;

	return T;
}

float bmp_pressf(struct bmp_device *dev, int32_t adc_P)
{
	float var1, var2, p;

	var1 = ((float) dev->t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((float) dev->digp[6]) / 32768.0;
	var2 = var2 + var1 * ((float) dev->digp[5]) * 2.0;
	var2 = (var2 / 4.0) + (((float) dev->digp[4]) * 65536.0);
	var1 = (((float) dev->digp[3]) * var1 * var1 / 524288.0
		+ ((float) dev->digp[2]) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((float) dev->digp1);

	if (var1 == 0.0)
		return 0;

	p = 1048576.0 - (float) adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((float) dev->digp[9]) * p * p / 2147483648.0;
	var2 = p * ((float) dev->digp[8]) / 32768.0;
	p = p + (var1 + var2 + ((float) dev->digp[7])) / 16.0;

	return p;
}

int bmp_getdata(void *d, void *dt, size_t sz)
{
	struct bmp_device *dev;
	struct bmp_data *data;
	uint8_t buf[8];
	int32_t pres, temp;

	data = (struct bmp_data *) dt;
	dev = (struct bmp_device *) d;

	if (sz < sizeof(struct bmp_data))
		return (-1);

	bmp_read(dev, BMP_PRESS, buf, 8);

	pres = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
	temp = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

	// bmp_tempf should be always first, it sets t_fine field
	data->temp = bmp_tempf(dev, temp);
	data->press = bmp_pressf(dev, pres);

	return 0;
}

int bmp_init(struct bmp_device *dev,
	enum BMP_STANDBY sb, enum BMP_IIR iir, enum BMP_OVERSAMPLING os,
	enum BMP_MODE m)
{
	uint8_t trimdata[25];

	if (bmp_write(dev, BMP_RESET, 0xb6) < 0)
		return (-1);

	if (bmp_write(dev, BMP_CONFIG, (sb << 5) | iir) < 0)
		return (-1);

	if (bmp_write(dev, BMP_CTRLMEAS, (os << 5) | (os << 2) | m) < 0)
		return (-1);

	memset(trimdata, 0, 25);
	bmp_read(dev, BMP_CALIB, trimdata, 25);

	dev->digt1	= (trimdata[1] << 8) | trimdata[0];
	dev->digt[2]	= (trimdata[3] << 8) | trimdata[2];
	dev->digt[3]	= (trimdata[5] << 8) | trimdata[4];
	dev->digp1	= (trimdata[7] << 8) | trimdata[5];
	dev->digp[2]	= (trimdata[9] << 8) | trimdata[6];
	dev->digp[3]	= (trimdata[11] << 8) | trimdata[10];
	dev->digp[4]	= (trimdata[13] << 8) | trimdata[12];
	dev->digp[5]	= (trimdata[15] << 8) | trimdata[14];
	dev->digp[6]	= (trimdata[17] << 8) | trimdata[16];
	dev->digp[7]	= (trimdata[19] << 8) | trimdata[18];
	dev->digp[8]	= (trimdata[21] << 8) | trimdata[20];
	dev->digp[9]	= (trimdata[23] << 8) | trimdata[22];

	return 0;
}

int bmp_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(bmp_devs + bmp_devcount, is, sizeof(struct bmp_device));

	sprintf(dev->name, "%s_%d", "bmp280", bmp_devcount);

	dev->priv = bmp_devs + bmp_devcount;
	dev->read = bmp_getdata;
	dev->write = NULL;
	dev->interrupt = NULL;

	r = bmp_init(bmp_devs + bmp_devcount++, BMP_STANDBY_05,
		BMP_IIR_16, BMP_OVERSAMPLING_16, BMP_MODE_NORMAL);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
