#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "util.h"

#include "mpu6500.h"

#define MPU_ADDR 0x68

enum MPU_REGISTER {
	MPU_WHOAMI		= 117,
	MPU_POWERMANAGEMENT	= 107,
	MPU_USERCONTROL		= 106,
	MPU_CONF		= 26,
	MPU_GYROCONF		= 27,
	MPU_ACCELCONF		= 28,
	MPU_ACCELCONF2		= 29,	// MPU6500 only
	MPU_ACCELMEASURE	= 59,

	X_OFFS_USR_H		= 19,
	X_OFFS_USR_L		= 20,
	Y_OFFS_USR_H		= 21,
	Y_OFFS_USR_L		= 22,
	Z_OFFS_USR_H		= 23,
	Z_OFFS_USR_L		= 24,

	MPU_PATH_SIGNAL_RESET	= 104,
	MPU_XA_OFFSET_H		= 119,
	MPU_XA_OFFSET_L		= 120,
	MPU_YA_OFFSET_H		= 122,
	MPU_YA_OFFSET_L		= 123,
	MPU_ZA_OFFSET_H		= 125,
	MPU_ZA_OFFSET_L		= 126
};

enum MPU_AXIS {
	MPU_X	= 0,
	MPU_Y	= 1,
	MPU_Z	= 2
};

static struct mpu_device mpu_devs[MPU_MAXDEVS];
static size_t mpu_devcount = 0;

int mpu_write(struct mpu_device *dev, uint8_t addr, uint8_t val)
{
	uint8_t sbuf[2];

	sbuf[0] = addr;
	sbuf[1] = val;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(dev->hspi, sbuf, 2, 100);

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return 0;
}

int mpu_read(struct mpu_device *dev, uint8_t addr,
	uint8_t *data, uint16_t size)
{
	uint8_t waddr = addr | 0x80;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(dev->hspi, &waddr, 1, 100);
	HAL_SPI_Receive(dev->hspi, data, size, 100);

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

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

int mpu_getdata(void *d, void *dt, size_t sz)
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

int mpu_selftest(struct mpu_device *dev, struct mpu_stdata *stdata)
{
	static const unsigned short sttb[256] = {
		2620,	2646,	2672,	2699,	2726,	2753,	2781,
		2808,	2837,	2865,	2894,	2923,	2952,	2981,
		3011,	3041,	3072,	3102,	3133,	3165,	3196,
		3228,	3261,	3293,	3326,	3359,	3393,	3427,
		3461,	3496,	3531,	3566,	3602,	3638,	3674,
		3711,	3748,	3786,	3823,	3862,	3900,	3939,
		3979,	4019,	4059,	4099,	4140,	4182,	4224,
		4266,	4308,	4352,	4395,	4439,	4483,	4528,
		4574,	4619,	4665,	4712,	4759,	4807,	4855,
		4903,	4953,	5002,	5052,	5103,	5154,	5205,
		5257,	5310,	5363,	5417,	5471,	5525,	5581,
		5636,	5693,	5750,	5807,	5865,	5924,	5983,
		6043,	6104,	6165,	6226,	6289,	6351,	6415,
		6479,	6544,	6609,	6675,	6742,	6810,	6878,
		6946,	7016,	7086,	7157,	7229,	7301,	7374,
		7448,	7522,	7597,	7673,	7750,	7828,	7906,
		7985,	8065,	8145,	8227,	8309,	8392,	8476,
		8561,	8647,	8733,	8820,	8909,	8998,	9088,
		9178,	9270,	9363,	9457,	9551,	9647,	9743,
		9841,	9939,	10038,	10139,	10240,	10343,	10446,
		10550,	10656,	10763,	10870,	10979,	11089,	11200,
		11312,	11425,	11539,	11654,	11771,	11889,	12008,
		12128,	12249,	12371,	12495,	12620,	12746,	12874,
		13002,	13132,	13264,	13396,	13530,	13666,	13802,	
		13940,	14080,	14221,	14363,	14506,	14652,	14798,
		14946,	15096,	15247,	15399,	15553,	15709,	15866,
		16024,	16184,	16346,	16510,	16675,	16842,	17010,
		17180,	17352,	17526,	17701,	17878,	18057,	18237,
		18420,	18604,	18790,	18978,	19167,	19359,	19553,
		19748,	19946,	20145,	20347,	20550,	20756,	20963,
		21173,	21385,	21598,	21814,	22033,	22253,	22475,
		22700,	22927,	23156,	23388,	23622,	23858,	24097,	
		24338,	24581,	24827,	25075,	25326,	25579,	25835,
		26093,	26354,	26618,	26884,	27153,	27424,	27699,
		27976,	28255,	28538,	28823,	29112,	29403,	29697,
		29994,	30294,	30597,	30903,	31212,	31524,	31839,
		32157,	32479,	32804,	33132
	};

	struct mpu_data data;
	int32_t ax, ay, az, gx, gy, gz;
	int32_t stax, stay, staz, stgx, stgy, stgz;
	int32_t ftgx, ftgy, ftgz, ftax, ftay, ftaz; 
	uint8_t stbuf[6];
	int i;

	mpu_write(dev, 0x19, 0x0);
	mpu_write(dev, 0x1a, 0x02);
	mpu_write(dev, 0x1b, 0x1);
	mpu_write(dev, 0x1c, 0x02);
	mpu_write(dev, 0x1d, 0x1);

	ax = ay = az = gx = gy = gz = 0;
	for (i = 0; i < 200; ++i) {
		mpu_getintdata(dev, &data);

		ax += data.ax;	ay += data.ay;	az += data.az;
		gx += data.gx;	gy += data.gy;	gz += data.gz;
	}

	ax /= 200;	ay /= 200;	az /= 200;
	gx /= 200;	gy /= 200;	gz /= 200;

	mpu_write(dev, MPU_GYROCONF, 0xe0);
	mpu_write(dev, MPU_ACCELCONF, 0xe0);

	mdelay(100);

	stax = stay = staz = stgx = stgy = stgz = 0;
	for (i = 0; i < 200; ++i) {
		mpu_getintdata(dev, &data);

		stax += data.ax;	stay += data.ay;	staz += data.az;
		stgx += data.gx;	stgy += data.gy;	stgz += data.gz;
	}

	stax /= 200;	stay /= 200;	staz /= 200;
	stgx /= 200;	stgy /= 200;	stgz /= 200;

	mpu_write(dev, MPU_GYROCONF, 0x0);
	mpu_write(dev, MPU_ACCELCONF, 0x0);
	mdelay(100);

	mpu_read(dev, 0, stbuf, 3);
	mpu_read(dev, 13, stbuf + 3, 3);


	ftgx = sttb[stbuf[0] - 1];
	ftgy = sttb[stbuf[1] - 1];
	ftgz = sttb[stbuf[2] - 1];

	ftax = sttb[stbuf[3] - 1];
	ftay = sttb[stbuf[4] - 1];
	ftaz = sttb[stbuf[5] - 1];

	stdata->ax = 100.0f * ((float) (stax - ax) - ftax) / ftax;
	stdata->ay = 100.0f * ((float) (stay - ay) - ftay) / ftay;
	stdata->az = 100.0f * ((float) (staz - az) - ftaz) / ftaz;

	stdata->gx = 100.0f * ((float) (stgx - gx) - ftgx) / ftgx;
	stdata->gy = 100.0f * ((float) (stgy - gy) - ftgy) / ftgy;
	stdata->gz = 100.0f * ((float) (stgz - gz) - ftgz) / ftgz;

	return 0;
}

int mpu_gyrooffset(struct mpu_device *dev, int16_t v, enum MPU_AXIS a)
{
	switch (a) {
	case MPU_X:
		mpu_write(dev, X_OFFS_USR_H, (v >> 8) & 0xff);
		mpu_write(dev, X_OFFS_USR_L, v & 0x00ff);
		break;

	case MPU_Y:
		mpu_write(dev, Y_OFFS_USR_H, (v >> 8) & 0xff);
		mpu_write(dev, Y_OFFS_USR_L, v & 0x00ff);
		break;

	case MPU_Z:
		mpu_write(dev, Z_OFFS_USR_H, (v >> 8) & 0xff);
		mpu_write(dev, Z_OFFS_USR_L, v & 0x00ff);
		break;
	}

	return 0;
}

int mpu_acceloffset(struct mpu_device *dev, int16_t v, enum MPU_AXIS a)
{
	switch (a) {
	case MPU_X:
		mpu_write(dev, MPU_XA_OFFSET_H, (v >> 8) & 0xff);
		mpu_write(dev, MPU_XA_OFFSET_L, v & 0x00fe);
		break;

	case MPU_Y:
		mpu_write(dev, MPU_YA_OFFSET_H, (v >> 8) & 0xff);
		mpu_write(dev, MPU_YA_OFFSET_L, v & 0x00fe);
		break;

	case MPU_Z:
		mpu_write(dev, MPU_ZA_OFFSET_H, (v >> 8) & 0xff);
		mpu_write(dev, MPU_ZA_OFFSET_L, v & 0x00fe);
		break;
	}

	return 0;
}

int mpu_configure(void *d, const char *cmd, ...)
{
	struct mpu_device *dev;
	va_list args;

	dev = (struct mpu_device *) d;

	va_start(args, cmd);


	if (strcmp(cmd, "self-test") == 0) {
		struct mpu_stdata *data;
		size_t sz;

		data = va_arg(args, struct mpu_stdata *);
		sz = va_arg(args, size_t);

		if (sz < sizeof(struct mpu_stdata)) {
			va_end(args);
			return (-1);
		}

		mpu_selftest(dev, data);

	}
	else if (strcmp(cmd, "offset") == 0) {
		mpu_gyrooffset(dev, va_arg(args, int), MPU_X);
		mpu_gyrooffset(dev, va_arg(args, int), MPU_Y);
		mpu_gyrooffset(dev, va_arg(args, int), MPU_Z);

		mpu_acceloffset(dev, va_arg(args, int), MPU_X);
		mpu_acceloffset(dev, va_arg(args, int), MPU_Y);
		mpu_acceloffset(dev, va_arg(args, int), MPU_Z);
	}

	va_end(args);

	return 0;
}

int mpu_init(struct mpu_device *dev)
{
	uint8_t check;

	check = 0;	
	mpu_read(dev, MPU_WHOAMI, &check, 1);

	if (check != dev->devtype)
		return (-1);

	mpu_write(dev, MPU_POWERMANAGEMENT, 0x0);
	mpu_write(dev, MPU_PATH_SIGNAL_RESET, 0x07);

	if (dev->devtype == MPU_DEV6500) {
		mpu_write(dev, MPU_USERCONTROL, 0x10);
		mpu_write(dev, MPU_ACCELCONF2, dev->dlpfwidth);
	}

	mpu_write(dev, MPU_CONF, dev->dlpfwidth);

	mpu_write(dev, MPU_ACCELCONF, dev->accelscale);

	mpu_write(dev, MPU_GYROCONF, dev->gyroscale);	

	return 0;
}

int mpu_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(mpu_devs + mpu_devcount, is, sizeof(struct mpu_device));

	sprintf(dev->name, "%s_%d", "mpu6500", mpu_devcount);

	dev->priv = mpu_devs + mpu_devcount;
	dev->read = mpu_getdata;
	dev->configure = mpu_configure;
	dev->write = NULL;
	dev->interrupt = NULL;

	r = mpu_init(mpu_devs + mpu_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
