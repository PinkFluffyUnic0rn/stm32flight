#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "uartdebug.h"
#include "esp8266.h"

#define ESP_TIMEOUT 1000
#define ESP_PAYLOADSZ (64 - 4)
#define ESP_FIFOSIZE 8

struct fifo {
	char cmd[ESP_FIFOSIZE][ESP_CMDSIZE];
	size_t bot;
	size_t top;
};

static struct esp_device esp_devs[ESP_MAXDEVS];
static size_t esp_devcount = 0;

static volatile struct fifo fifo;

static const uint8_t crsf_crc8tbl [] = {
	0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 
	0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d, 
	0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 
	0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f, 
	0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 
	0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9, 
	0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 
	0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b, 
	0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 
	0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0, 
	0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 
	0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2, 
	0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 
	0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44, 
	0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 
	0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16, 
	0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 
	0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92, 
	0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 
	0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0, 
	0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 
	0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36, 
	0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 
	0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64, 
	0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 
	0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f, 
	0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 
	0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d, 
	0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 
	0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab, 
	0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 
	0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9
};

static uint8_t crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc;
	int i;

	crc = 0x00;
	for (i = 0; i < len; ++i)
		crc = crsf_crc8tbl[crc ^ *data++];

	return crc;
}

void memcpyv(volatile void *dest, const volatile void *src, size_t n)
{
	int i;

	for (i = 0; i < n; ++i)
		((uint8_t *) dest)[i] = ((uint8_t *) src)[i];
}

int esp_initfifo(volatile struct fifo *f)
{
	f->bot = f->top = 0;

	return 0;
}

int esp_dequeque(volatile struct fifo *f, char *out)
{
	if (f->bot == f->top)
		return (-1);
	
	memcpyv(out, f->cmd[f->bot], ESP_CMDSIZE);

	f->bot = (f->bot + 1) % ESP_FIFOSIZE;

	return 0;
}

int esp_interrupt(void *dev, const void *p)
{
	struct esp_device *d;
	uint8_t b;
	uint8_t id;
	uint8_t crc;
	uint16_t size;
	uint8_t buf[128];

	d = dev;

	if (*((uint16_t *) p) != d->intpin)
		return 0;
	
	HAL_GPIO_WritePin(d->csgpio, d->cspin, GPIO_PIN_RESET);

	b = 0x3;
	HAL_SPI_Transmit(d->hspi, &b, 1, 1000);

	b = 0x0;
	HAL_SPI_Transmit(d->hspi, &b, 1, 1000);

	HAL_SPI_Receive(d->hspi, buf, 64, 1000);
	HAL_GPIO_WritePin(d->csgpio, d->cspin, GPIO_PIN_SET);

	id = buf[0];
	crc = buf[1];
	size = *((uint16_t *) (buf + 2));

	if (size > (64 - 4) || crc8(buf + 2, size + 2) != crc || id != 0xaa)
		return 0;

	memcpyv(fifo.cmd[fifo.top], buf + 4, size);

	fifo.cmd[fifo.top][size] = '\0';

	if ((fifo.top + 1) % ESP_FIFOSIZE != fifo.bot)
		fifo.top = (fifo.top + 1) % ESP_FIFOSIZE;

	return 0;
}

static int _esp_send(struct esp_device *dev, int timeout,
	const char *data)
{
	int pos;
	size_t l;

	l = strlen(data) + 1;

	for (pos = 0; pos < l; pos += ESP_PAYLOADSZ) {
		uint16_t size;
		uint8_t buf[64];
		uint8_t b;

		size = ((l - pos) < ESP_PAYLOADSZ)
			? (l - pos) : ESP_PAYLOADSZ;

		memcpy(buf + 4, data + pos, size);

		buf[0] = 0xaa;
		*((uint16_t *) (buf + 2)) = size;
		buf[1] = crc8(buf + 2, size + 2);

		HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_RESET);

		b = 0x2;
		HAL_SPI_Transmit(dev->hspi, &b, 1, 1000);

		b = 0x0;
		HAL_SPI_Transmit(dev->hspi, &b, 1, 1000);

		HAL_SPI_Transmit(dev->hspi, buf, 64, 1000);

		HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_SET);

		HAL_Delay(1);
	}

	return 0;
}

int esp_send(void *dev, void *dt, size_t sz)
{
	struct esp_device *d;

	d = dev;
	
	if (_esp_send(d, ESP_TIMEOUT, dt) < 0)
		return (-1);

	return 0;
}

int esp_printf(struct esp_device *dev, const char *format, ...)
{
	char buf[1024];
	va_list args;

	va_start(args, format);

	vsnprintf(buf, 1024, format, args);

	return esp_send(dev, buf, strlen(buf));
}

int esp_read(void *dev, void *dt, size_t sz)
{
	if (esp_dequeque(&fifo, dt) < 0)
		return (-1);

	return 0;
}

static int esp_init(struct esp_device *dev)
{
	HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(dev->bootgpio, dev->bootpin, GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_RESET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_SET);

	return 0;
}

int esp_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(esp_devs + esp_devcount, is, sizeof(struct esp_device));

	sprintf(dev->name, "%s_%d", "esp8266", esp_devcount);

	dev->priv = esp_devs + esp_devcount;
	dev->read = esp_read;
	dev->configure = NULL;
	dev->write = esp_send;
	dev->interrupt = esp_interrupt;
	dev->error = NULL;

	r = esp_init(esp_devs + esp_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
