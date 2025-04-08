#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

#include "hp206c.h"

#define HP_ADDR 0x76

enum HP_COMMAND {
	HP_RST		= 0x6,
	HP_ADCCVT	= 0x40,
	HP_PT		= 0x10,
	HP_AT		= 0x11,
	HP_P		= 0x11,
	HP_A		= 0x11,
	HP_T		= 0x11,
	HP_CAL		= 0x28,
};

static struct hp_device hp_devs[HP_MAXDEVS];
static size_t hp_devcount = 0;

int hp_getdata(void *d, void *dt, size_t sz)
{
	struct hp_device *dev;
	struct hp_data *data;
	uint8_t buf[6];
	uint8_t c;

	data = (struct hp_data *) dt;
	dev = (struct hp_device *) d;

	if (sz < sizeof(struct hp_data))
		return (-1);

	c = HP_AT;
	HAL_I2C_Master_Transmit(dev->hi2c, HP_ADDR << 1, &c, 1, 1000);
	HAL_I2C_Master_Receive(dev->hi2c, (HP_ADDR << 1) | 0x1, buf, 6,
		1000);

	data->temp = buf[0] << 16 | buf[1] << 8 | buf[2];
	data->alt = buf[3] << 16 | buf[4] << 8 | buf[5];

	data->tempf = data->temp / 100.0;
	data->altf = data->alt / 100.0;;

	c = HP_ADCCVT | dev->osr << 2;
	HAL_I2C_Master_Transmit(dev->hi2c, HP_ADDR << 1, &c, 1, 1000);

	return 0;
}

int hp_init(struct hp_device *dev)
{
	uint8_t c;
	int i;

	c = HP_RST;
	HAL_I2C_Master_Transmit(dev->hi2c, HP_ADDR << 1, &c, 1, 1000);

	c = HP_ADCCVT | dev->osr << 2;
	HAL_I2C_Master_Transmit(dev->hi2c, HP_ADDR << 1, &c, 1, 1000);

	for (i = 0; i < 10; ++i) {
		struct hp_data hd;
		
		HAL_Delay(35);

		hp_getdata(dev, &hd, sizeof(struct hp_data));
	}

	return 0;
}

int hp_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(hp_devs + hp_devcount, is, sizeof(struct hp_device));
	
	sprintf(dev->name, "%s_%d", "hp206c", hp_devcount);

	dev->priv = hp_devs + hp_devcount;
	dev->read = hp_getdata;
	dev->write = NULL;
	dev->interrupt = NULL;

	r = hp_init(hp_devs + hp_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
