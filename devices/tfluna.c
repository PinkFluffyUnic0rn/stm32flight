#include <stdint.h>
#include <string.h>
#include <stdio.h>


#include "crc.h"
#include "util.h"

#include "tfluna.h"

#define RXCIRCSIZE 8

enum TF_ITSTATE {
	TF_ITSTATE_HEAD1,
	TF_ITSTATE_HEAD2,
	TF_ITSTATE_DISTL,
	TF_ITSTATE_DISTH,
	TF_ITSTATE_AMPL,
	TF_ITSTATE_AMPH,
	TF_ITSTATE_TEMPL,
	TF_ITSTATE_TEMPH,
	TF_ITSTATE_CHECKSUM,
};

enum TF_PACKTYPE {
	TF_PACKTYPE_DIST
};

struct tf_packet {
	union {
		struct {
			uint16_t dist;
			uint16_t amp;
			uint16_t temp;
			uint8_t checksum;
		} dist;

		char dummy[8];
	};
	uint16_t sum;
	enum TF_PACKTYPE type;
};

static struct tf_device tf_devs[TF_MAXDEVS];
static size_t tf_devcount = 0;

static uint8_t Rxbuf;

volatile static enum TF_ITSTATE Packstate;
volatile static uint8_t Packw;
volatile static uint8_t Packr;
volatile static struct tf_packet Pack[RXCIRCSIZE];

int tf_interrupt(void *dev, const void *h) 
{
	struct tf_device *d;
	uint8_t b;

	d = dev;

	if (((UART_HandleTypeDef *)h)->Instance != d->huart->Instance)
		return 0;

	b = Rxbuf;

	if (Packstate == TF_ITSTATE_HEAD1) {
		if (b == 0x59) {
			Pack[Packw].type = TF_PACKTYPE_DIST;
			Pack[Packw].sum = b;
			
			Packstate = TF_ITSTATE_HEAD2;
		}
	}
	else if (Packstate == TF_ITSTATE_HEAD2) {
		if (b != 0x59) {
			Packstate = TF_ITSTATE_HEAD1;
			return 0;
		}
	
		Pack[Packw].sum += b;

		Packstate = TF_ITSTATE_DISTL;
	}
	else if (Packstate == TF_ITSTATE_DISTL) {
		Pack[Packw].dist.dist = b;
		Pack[Packw].sum += b;

		Packstate = TF_ITSTATE_DISTH;
	}
	else if (Packstate == TF_ITSTATE_DISTH) {
		Pack[Packw].dist.dist |= b << 8;
		Pack[Packw].sum += b;

		Packstate = TF_ITSTATE_AMPL;
	}
	else if (Packstate == TF_ITSTATE_AMPL) {
		Pack[Packw].dist.amp = b;
		Pack[Packw].sum += b;

		Packstate = TF_ITSTATE_AMPH;
	}
	else if (Packstate == TF_ITSTATE_AMPH) {
		Pack[Packw].dist.amp |= b << 8;
		Pack[Packw].sum += b;

		Packstate = TF_ITSTATE_TEMPL;
	}
	else if (Packstate == TF_ITSTATE_TEMPL) {
		Pack[Packw].dist.temp = b;
		Pack[Packw].sum += b;

		Packstate = TF_ITSTATE_TEMPH;
	}
	else if (Packstate == TF_ITSTATE_TEMPH) {
		Pack[Packw].dist.temp |= b << 8;
		Pack[Packw].sum += b;
		Pack[Packw].sum &= 0xff;

		Packstate = TF_ITSTATE_CHECKSUM;
	}
	else if (Packstate == TF_ITSTATE_CHECKSUM) {
		Pack[Packw].dist.checksum = b;

		if ((Packw + 1) % RXCIRCSIZE != Packr)
			Packw = (Packw + 1) % RXCIRCSIZE;

		Packstate = TF_ITSTATE_HEAD1;
	}

	return 0;
}

int tf_error(void *dev, const void *h)
{
	struct tf_device *d;
	const UART_HandleTypeDef *huart;

	d = dev;
	huart = h;

	if (huart->Instance != d->huart->Instance)
		return 0;

	if ((huart->ErrorCode
		& (HAL_UART_ERROR_FE | HAL_UART_ERROR_NE)) == 0) {
		return 0;
	}

	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);

	HAL_UART_Receive_DMA(d->huart, &Rxbuf, 1);

	Packstate = TF_ITSTATE_HEAD1;
	Packw = Packr = 0;

	return 0;
}

int tf_read(void *dev, void *dt, size_t sz)
{
	struct tf_data *data;

	data = (struct tf_data *) dt;

	if (Packr == Packw)
		return (-1);

	if (Pack[Packr].type != TF_PACKTYPE_DIST) {
		Packr = (Packr + 1) % RXCIRCSIZE;
		return (-1);
	}

	if (Pack[Packr].sum != Pack[Packr].dist.checksum) {
		Packr = (Packr + 1) % RXCIRCSIZE;
		return (-1);
	}
	
	data->dist = Pack[Packr].dist.dist * 0.01;
	data->amp = Pack[Packr].dist.amp;
	data->temp = Pack[Packr].dist.temp * 0.125 - 256.0;

	Packr = (Packr + 1) % RXCIRCSIZE;

	return 0;
}

int tf_write(void *dev, void *dt, size_t sz)
{
	return 0;
}

int tf_init(struct tf_device *tf)
{
	HAL_UART_Receive_DMA(tf->huart, &Rxbuf, 1);

	Packstate = 0;
	Packw = Packr = 0;

	return 0;
}

int tf_initdevice(struct tf_device *is, struct cdevice *dev)
{
	int r; 

	memmove(tf_devs + tf_devcount, is,
		sizeof(struct tf_device));

	sprintf(dev->name, "%s_%d", "tf", tf_devcount);

	dev->priv = tf_devs + tf_devcount;
	dev->read = tf_read;
	dev->write = tf_write;
	dev->interrupt = tf_interrupt;
	dev->error = tf_error;

	r = tf_init(tf_devs + tf_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return 0;
};
