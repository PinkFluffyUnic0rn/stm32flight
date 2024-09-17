#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "crsf.h"

#define RXCIRCSIZE 16

struct packet {
	uint8_t type;
	uint8_t len;
	uint8_t pl[64];
};

static struct crsf_device crsf_devs[CRSF_MAXDEVS];
static size_t crsf_devcount = 0;

static uint8_t Rxbuf;

volatile static uint8_t Packstate;
volatile static uint8_t Packrest;
volatile static uint8_t Packw;
volatile static uint8_t Packr;
volatile static struct packet Pack[RXCIRCSIZE];

int crsf_interrupt(void *dev, const void *h) 
{
	struct crsf_device *d;
	uint8_t b;

	d = dev;
	
	if (((UART_HandleTypeDef *)h)->Instance != d->huart->Instance)
		return 0;

	b = Rxbuf;

	if (Packstate == 0) {
		if ((Packw + 1) % RXCIRCSIZE == Packr)
			return 0;

		if (b == 0xc8) {
			Packw = (Packw + 1) % RXCIRCSIZE;
			Packstate = 1;
		}
	}
	else if (Packstate == 1) {
		if (b > 64)
			Packstate = 0;

		Pack[Packw].len = Packrest = b;

		Packstate = 2;
	}
	else if (Packstate == 2) {
		Pack[Packw].type = b;

		Packstate = 3;
	}
	else {
		Pack[Packw].pl[Pack[Packw].len - Packrest] = b;
		--Packrest;
	
		if (Packrest == 0)
			Packstate = 0;
	}

	return 0;
}

int crsf_read(void *dev, void *dt, size_t sz)
{
	struct crsf_data *data;
	uint32_t merged;
	uint32_t value;
	int curb;
	int i;

	data = (struct crsf_data *) dt;

	if (Packr == Packw)
		return (-1);

	if (Pack[Packr].len < 0x18) {
		Packr = (Packr + 1) % RXCIRCSIZE;
		return (-1);
	}

	if (Pack[Packr].type != 0x16) {
		Packr = (Packr + 1) % RXCIRCSIZE;
		return (-1);
	}
	
	merged = value = 0;
	curb = 0;

	for (i = 0; i < CRSF_CHANNELCOUNT; i++) {
		while (merged < 11) {
			value |= (Pack[Packr].pl[curb++]) << merged;
			merged += 8;
		}

		data->chf[i] = ((value & 0x000007ff) - 992.0) / 819.5;

		value >>= 11;
		merged -= 11;
	}

	Packr = (Packr + 1) % RXCIRCSIZE;

	return 0;
}

int crsf_init(struct crsf_device *crsf)
{
	HAL_UART_Receive_DMA(crsf->huart, &Rxbuf, 1);

	Packstate = 0;
	Packrest = 0;
	Packw = Packr = 0;

	return 0;
}

int crsf_initdevice(void *is, struct cdevice *dev)
{
	int r; 

	memmove(crsf_devs + crsf_devcount, is, sizeof(struct crsf_device));
	
	sprintf(dev->name, "%s%d", "crsf", crsf_devcount);

	dev->priv = crsf_devs + crsf_devcount;
	dev->read = crsf_read;
	dev->write = NULL;
	dev->interrupt = crsf_interrupt;

	r = crsf_init(crsf_devs + crsf_devcount++);
	
	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return 0;
};
