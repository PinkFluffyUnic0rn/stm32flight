#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "util.h"
#include "crc.h"

#include "esp8266.h"

#define ESP_TIMEOUT 1000
#define ESP_SPITIMEOUTUS 10
#define ESP_SPIPACKSIZE 64
#define ESP_PAYLOADSZ (ESP_SPIPACKSIZE - 4)
#define ESP_FIFOSIZE 8

struct fifo {
	char cmd[ESP_FIFOSIZE][ESP_CMDSIZE];
	size_t bot;
	size_t top;
};

static struct esp_device esp_devs[ESP_MAXDEVS];
static size_t esp_devcount = 0;

static volatile struct fifo fifo;

static int esp_initfifo(volatile struct fifo *f)
{
	f->bot = f->top = 0;

	return 0;
}

static int esp_dequeque(volatile struct fifo *f, char *out)
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
	uint8_t buf[ESP_SPIPACKSIZE];

	d = dev;

	if (*((uint16_t *) p) != d->intpin)
		return 0;

	HAL_GPIO_WritePin(d->csgpio, d->cspin, GPIO_PIN_RESET);

	b = 0x3;
	HAL_SPI_Transmit(d->hspi, &b, 1, 1000);

	b = 0x0;
	HAL_SPI_Transmit(d->hspi, &b, 1, 1000);

	HAL_SPI_Receive(d->hspi, buf, ESP_SPIPACKSIZE, 1000);
	HAL_GPIO_WritePin(d->csgpio, d->cspin, GPIO_PIN_SET);

	id = buf[0];
	crc = buf[1];
	size = *((uint16_t *) (buf + 2));

	if (size > (ESP_SPIPACKSIZE - 4)
			|| crc8(buf + 2, size + 2) != crc
			|| id != 0xaa)
		return 0;

	memcpyv(fifo.cmd[fifo.top], buf + 4, size);

	fifo.cmd[fifo.top][size] = '\0';

	if ((fifo.top + 1) % ESP_FIFOSIZE != fifo.bot)
		fifo.top = (fifo.top + 1) % ESP_FIFOSIZE;

	return 0;
}

int esp_send(void *d, void *dt, size_t sz)
{
	struct esp_device *dev;
	int pos;
	size_t l;
	char *data;

	dev = d;
	data = dt;

	l = strlen(data);

	for (pos = 0; pos < l; pos += ESP_PAYLOADSZ) {
		uint8_t buf[ESP_SPIPACKSIZE];
		uint16_t size;
		int t;
		uint8_t b;

		t = 0;
		while (HAL_GPIO_ReadPin(dev->busygpio,
				dev->busypin) == GPIO_PIN_SET
				&& t / 1000 < ESP_SPITIMEOUTUS) {
			udelay(1);
			t += 100;
		}

		size = ((l - pos) < ESP_PAYLOADSZ)
			? (l - pos) : ESP_PAYLOADSZ;

		memcpy(buf + 4, data + pos, size);

		buf[0] = 0xaa;
		*((uint16_t *) (buf + 2)) = size;
		buf[1] = crc8(buf + 2, size + 2);

		HAL_GPIO_WritePin(dev->csgpio, dev->cspin,
			GPIO_PIN_RESET);

		b = 0x2;
		HAL_SPI_Transmit(dev->hspi, &b, 1, 1000);

		b = 0x0;
		HAL_SPI_Transmit(dev->hspi, &b, 1, 1000);

		HAL_SPI_Transmit(dev->hspi, buf, ESP_SPIPACKSIZE, 1000);

		HAL_GPIO_WritePin(dev->csgpio, dev->cspin,
			GPIO_PIN_SET);	
	}

	return 0;
}

int esp_read(void *dev, void *dt, size_t sz)
{
	if (esp_dequeque(&fifo, dt) < 0)
		return (-1);

	return 0;
}

static int esp_run(struct esp_device *dev)
{
	HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(dev->bootgpio, dev->bootpin, GPIO_PIN_SET);
	mdelay(250);
	HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_RESET);
	mdelay(250);
	HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_SET);
	mdelay(250);
	HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_SET);

	return 0;
}

static int esp_flash(struct esp_device *dev)
{
	HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(dev->bootgpio, dev->bootpin, GPIO_PIN_RESET);
	mdelay(250);
	HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_RESET);
	mdelay(250);
	HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_SET);
	mdelay(250);
	HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_SET);

	return 0;
}

int esp_configure(void *d, const char *cmd, ...)
{
	struct esp_device *dev;
	va_list args;

	dev = (struct esp_device *) d;

	va_start(args, cmd);

	if (strcmp(cmd, "flash") == 0)
		esp_flash(dev);
	else if (strcmp(cmd, "run") == 0)
		esp_run(dev);

	va_end(args);

	return 0;
}
static int esp_init(struct esp_device *dev)
{	
	esp_initfifo(&fifo);

	esp_run(dev);

	return 0;
}

int esp_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(esp_devs + esp_devcount, is, sizeof(struct esp_device));

	sprintf(dev->name, "%s_%d", "esp8266", esp_devcount);

	dev->priv = esp_devs + esp_devcount;
	dev->read = esp_read;
	dev->configure = esp_configure;
	dev->write = esp_send;
	dev->interrupt = esp_interrupt;
	dev->error = NULL;

	r = esp_init(esp_devs + esp_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
