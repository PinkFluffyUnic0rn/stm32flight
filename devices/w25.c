#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "util.h"

#include "w25.h"

static struct w25_device devs[W25_MAXDEVS];
size_t devcount = 0;

#define W25_WTIMEOUT 100000
#define W25_CETIMEOUT 100000
#define W25_SETIMEOUT 400
#define W25_BETIMEOUT 3000

#define min(a, b) ((a) < (b) ? (a) : (b))

static uint32_t w25_getid(struct w25_device *dev)
{
	uint8_t sbuf[4], rbuf[4];

	sbuf[0] = 0x9f;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 1, 100);
	HAL_SPI_Receive(dev->hspi, rbuf, 3, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return ((rbuf[0] << 16) | (rbuf[1] << 8) | rbuf[2]);
}

static int w25_init(struct w25_device *dev)
{
	uint8_t sbuf[4];
	uint32_t id;

	mdelay(100);

	sbuf[0] = 0x66;
	sbuf[1] = 0x99;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 2, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	mdelay(100);

	id = w25_getid(dev);

	if ((id >> 16) != 0xef)
		return (-1);

	return 0;
}

static int w25_writeenable(struct w25_device *dev)
{
	uint8_t sbuf[4];

	sbuf[0] = 0x06;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 1, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return 0;
}

static int w25_writedisable(struct w25_device *dev)
{
	uint8_t sbuf[4];

	sbuf[0] = 0x04;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 1, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return 0;
}

static int w25_waitwrite(struct w25_device *dev, int timeout)
{
	uint8_t sbuf[4], rbuf[4];
	int us;

	sbuf[0] = 0x05;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(dev->hspi, sbuf, 1, 100);

	us = 0;
	do {
		udelay(10);
		HAL_SPI_Receive(dev->hspi, rbuf, 1, 100);
		us += 10;
	} while ((rbuf[0] & 0x01) == 0x01 && us < timeout);

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return 0;
}

static int w25_blockprotect(struct w25_device *dev, uint8_t flags)
{
	uint8_t sbuf[4];

	sbuf[0] = 0x50;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 1, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	sbuf[0] = 0x01;
	sbuf[1] = (flags & 0x0f) << 2;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 2, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return 0;
}

int w25_read(void *d, size_t addr, void *data, size_t sz)
{
	struct w25_device *dev;
	uint8_t sbuf[4];

	dev = (struct w25_device *) d;

	sbuf[0] = 0x03;
	sbuf[1] = (addr >> 16) & 0xff;
	sbuf[2] = (addr >> 8) & 0xff;
	sbuf[3] = addr & 0xff;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 4, 100);
	HAL_SPI_Receive(dev->hspi, data, sz, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	return 0;
}

int w25_write(void *d, size_t addr, const void *data, size_t sz)
{
	struct w25_device *dev;
	uint8_t sbuf[4];
	
	dev = (struct w25_device *) d;

	// if (dev->writemode == W25_IOCTL_READWRITE) ??
	w25_waitwrite(dev, W25_WTIMEOUT);

	w25_blockprotect(dev, 0x00);
	w25_writeenable(dev);

	sbuf[0] = 0x02;
	sbuf[1] = (addr >> 16) & 0xff;
	sbuf[2] = (addr >> 8) & 0xff;
	sbuf[3] = addr & 0xff;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 4, 100);
	HAL_SPI_Transmit(dev->hspi, (uint8_t  *) data, sz, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	if (dev->writemode == W25_IOCTL_READWRITE) {
		w25_waitwrite(dev, W25_WTIMEOUT);
		w25_writedisable(dev);
		w25_blockprotect(dev, 0x0f);
	}

	return 0;
}

int w25_eraseall(void *d)
{
	struct w25_device *dev;
	uint8_t sbuf[4];
	
	dev = (struct w25_device *) d;

	w25_waitwrite(dev, W25_WTIMEOUT);
	w25_blockprotect(dev, 0x00);
	w25_writeenable(dev);

	sbuf[0] = 0xc7;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 1, 100);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	w25_waitwrite(dev, W25_CETIMEOUT);
	w25_writedisable(dev);
	w25_blockprotect(dev, 0x0f);

	return 0;
}

int w25_erasesector(void *d, size_t addr)
{
	struct w25_device *dev;
	uint8_t sbuf[4];
	
	dev = (struct w25_device *) d;

	w25_waitwrite(dev, W25_WTIMEOUT);
	w25_blockprotect(dev, 0x00);
	w25_writeenable(dev);

	sbuf[0] = 0x20;
	sbuf[1] = (addr >> 16) & 0xff;
	sbuf[2] = (addr >> 8) & 0xff;
	sbuf[3] = addr & 0xff;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 4, 1000);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	w25_waitwrite(dev, W25_SETIMEOUT);
	w25_writedisable(dev);
	w25_blockprotect(dev, 0x0f);

	return 0;
}

int w25_eraseblock(void *d, size_t addr)
{
	struct w25_device *dev;
	uint8_t sbuf[4];
	
	dev = (struct w25_device *) d;

	w25_waitwrite(dev, W25_WTIMEOUT);
	w25_blockprotect(dev, 0x00);
	w25_writeenable(dev);

	sbuf[0] = 0xd8;
	sbuf[1] = (addr >> 16) & 0xff;
	sbuf[2] = (addr >> 8) & 0xff;
	sbuf[3] = addr & 0xff;

	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(dev->hspi, sbuf, 4, 1000);
	HAL_GPIO_WritePin(dev->gpio, dev->pin, GPIO_PIN_SET);

	w25_waitwrite(dev, W25_BETIMEOUT);
	w25_writedisable(dev);
	w25_blockprotect(dev, 0x0f);

	return 0;
}

int w25_writesector(void *d, size_t addr, const void *data,
	size_t sz)
{
	struct w25_device *dev;
	int i;
	
	dev = (struct w25_device *) d;

	for (i = 0; i < sz; i += min(W25_PAGESIZE, sz - i))
		w25_write(dev, addr + i, data + i, W25_PAGESIZE);

	return 0;
}

int w25_ioctl(void *d, int req, ...)
{
	struct w25_device *dev;
	dev = (struct w25_device *) d;
	
	dev->writemode = req;

	return 0;
}

int w25_initdevice(void *is, struct bdevice *dev)
{
	int r;

	memmove(devs + devcount, is, sizeof(struct w25_device));
	
	sprintf(dev->name, "%s_%d", "w25q", devcount);

	dev->priv = devs + devcount;

	dev->read = w25_read;
	dev->write = w25_write;
	dev->ioctl = w25_ioctl;
	dev->eraseall = w25_eraseall;
	dev->erasesector = w25_erasesector;
	dev->eraseblock = w25_eraseblock;
	dev->writesector = w25_writesector;

	dev->writesize = W25_PAGESIZE;
	dev->sectorsize = W25_SECTORSIZE;
	dev->totalsize = W25_TOTALSIZE;

	devs[devcount].writemode = W25_IOCTL_READWRITE;
	
	r = w25_init(devs + devcount++);


	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
