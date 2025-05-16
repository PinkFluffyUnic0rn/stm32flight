#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include "util.h"

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

	l = strlen(data);

	for (pos = 0; pos < l; pos += ESP_PAYLOADSZ) {
		uint16_t size;
		uint8_t buf[64];
		uint8_t b;
		
		// need timeout
		while (HAL_GPIO_ReadPin(dev->bootgpio,
				dev->bootpin) == GPIO_PIN_SET);

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

int esp_configure(void *d, const char *cmd, ...)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	struct esp_device *dev;
	va_list args;

	dev = (struct esp_device *) d;

	va_start(args, cmd);

	if (strcmp(cmd, "flash") == 0) {
		GPIO_InitStruct.Pin = dev->bootpin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(dev->bootgpio, &GPIO_InitStruct);

		HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev->bootgpio, dev->bootpin, GPIO_PIN_RESET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_RESET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_SET);
	}
	else if (strcmp(cmd, "run") == 0) {
		GPIO_InitStruct.Pin = dev->bootpin;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(dev->bootgpio, &GPIO_InitStruct);

		HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(dev->bootgpio, dev->bootpin, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_RESET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_SET);

		GPIO_InitStruct.Pin = dev->bootpin;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(dev->bootgpio, &GPIO_InitStruct);
	}

	va_end(args);

	return 0;
}
static int esp_init(struct esp_device *dev)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	esp_initfifo(&fifo);

	GPIO_InitStruct.Pin = dev->bootpin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(dev->bootgpio, &GPIO_InitStruct);

	HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(dev->bootgpio, dev->bootpin, GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_RESET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(dev->rstgpio, dev->rstpin, GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(dev->csgpio, dev->cspin, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = dev->bootpin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(dev->bootgpio, &GPIO_InitStruct);

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
