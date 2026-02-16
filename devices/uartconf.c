#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "util.h"
#include "uartconf.h"

#define UART_TIMEOUT 1000
#define UART_FIFOSIZE 4

struct fifo {
	char cmd[UART_FIFOSIZE][UART_CMDSIZE];
	size_t bot;
	size_t top;
};

static struct uart_device uart_devs[UART_MAXDEVS];
static size_t uart_devcount = 0;

static uint8_t Rxbyte;
static volatile struct fifo fifo;

int uart_initfifo(volatile struct fifo *f)
{
	f->bot = f->top = 0;

	return 0;
}

int uart_dequeque(volatile struct fifo *f, char *out)
{
	if (f->bot == f->top)
		return (-1);

	memcpyv(out, f->cmd[f->bot], UART_CMDSIZE);

	f->bot = (f->bot + 1) % UART_FIFOSIZE;

	return 0;
}

int uart_interrupt(void *dev, const void *h)
{
	struct uart_device *d;
	static size_t Rxoffset = 0;
	volatile char *cmd;

	d = dev;

	if (((UART_HandleTypeDef *)h)->Instance != d->huart->Instance)
		return 0;

	cmd = fifo.cmd[fifo.top] + Rxoffset;

	*cmd = Rxbyte;

	HAL_UART_Transmit(d->huart, (uint8_t *) &Rxbyte, 1, 100);

	if (*cmd == '\r') {
		HAL_UART_Transmit(d->huart, (uint8_t *) "\n", 1, 100);

		*cmd = '\0';

		if ((fifo.top + 1) % UART_FIFOSIZE != fifo.bot)
			fifo.top = (fifo.top + 1) % UART_FIFOSIZE;

		Rxoffset = 0;
	}
	else if (*cmd == '\b') {
		if (Rxoffset > 0)
			Rxoffset--;
	}
	else if (Rxoffset++ == (UART_CMDSIZE - 1))
		Rxoffset = 0;

	return 0;
}

int uart_send(void *dev, void *dt, size_t sz)
{
	struct uart_device *d;

	d = dev;

	HAL_UART_Transmit(d->huart, (uint8_t *) dt, sz, UART_TIMEOUT);

	return 0;
}

int uart_read(void *dev, void *dt, size_t sz)
{
	if (uart_dequeque(&fifo, dt) < 0)
		return (-1);

	return 0;
}

int uart_initdevice(void *is, struct cdevice *dev)
{
	memmove(uart_devs + uart_devcount, is, sizeof(struct uart_device));

	sprintf(dev->name, "%s_%d", "uart", uart_devcount);

	dev->priv = uart_devs + uart_devcount;
	dev->read = uart_read;
	dev->configure = NULL;
	dev->write = uart_send;
	dev->interrupt = uart_interrupt;
	dev->error = NULL;

	uart_initfifo(&fifo);

	HAL_UART_Receive_DMA(uart_devs[uart_devcount].huart, &Rxbyte, 1);

	dev->status = DEVSTATUS_INIT;

	uart_devcount++;

	return 0;
}
