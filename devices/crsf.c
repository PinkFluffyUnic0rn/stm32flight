#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>


#include "crc.h"
#include "util.h"

#include "crsf.h"

#define RXCIRCSIZE 16

#define endian4(v) ((v & 0xff) << 24 | (v & 0xff00) << 8 \
	| (v & 0xff0000) >> 8 | (v & 0xff000000) >> 24 )

#define endian2(v) ((v & 0xff) << 8 | (v & 0xff00) >> 8)

struct packet {
	uint8_t len;
	uint8_t type;
	uint8_t pl[64];
	uint8_t crc;
};


struct __attribute__ ((packed)) battery {
	int16_t voltage;
	int16_t current;
	int8_t cap[3];
	int8_t rem;
};

struct __attribute__ ((packed)) gps {
	int32_t lat;
	int32_t lon;
	int16_t speed;
	int16_t course;
	uint16_t alt;
	uint8_t sats;
};

struct __attribute__ ((packed)) baroaltitude {
	uint16_t balt;
	int16_t vspeed;
};

struct __attribute__ ((packed)) attitude {
	int16_t pitch;
	int16_t roll;
	int16_t yaw;
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
		--Packrest;

		Packstate = 3;
	}
	else if (Packstate == 3) {
		Pack[Packw].pl[Pack[Packw].len - Packrest - 1] = b;

		--Packrest;

		if (Packrest == 1)
			Packstate = 4;
	}
	else if (Packstate == 4) {
		Pack[Packw].crc = b;

		Packstate = 0;
	}

	return 0;
}

int crsf_error(void *dev, const void *h)
{
	struct crsf_device *d;
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

	Packstate = 0;
	Packrest = 0;
	Packw = Packr = 0;

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

	if (crc8((uint8_t *)(Pack + Packr) + 1, 0x17)
			!= Pack[Packr].crc) {
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

int crsf_write(void *dev, void *dt, size_t sz)
{
	struct crsf_device *d;
	struct crsf_tele *tele;
	struct gps *gpspack;
	struct baroaltitude *baltpack;
	struct attitude *attpack;
	struct battery *batpack;
	uint8_t buf[64];
	static uint8_t bufcommon[128];

	d = dev;
	tele = dt;
	batpack =  (struct battery *) (buf + 3);
	gpspack =  (struct gps *) (buf + 3);
	baltpack = (struct baroaltitude *) (buf + 3);
	attpack = (struct attitude *) (buf + 3);

	buf[0] = 0xc8;

	buf[1] = 15;
	buf[2] = 0x21;
	memcpy(buf + 3, tele->mode, 14);
	(buf + 3)[13] = '\0';
	buf[15] = crc8(buf + 2, 13);

	memcpy(bufcommon, buf, 17);

	buf[1] = 10;
	buf[2] = 0x08;
	batpack->voltage = endian2((int16_t)(tele->bat * 10));
	batpack->current = endian2((int16_t)(tele->curr * 10));
	batpack->cap[0] = batpack->cap[1] = batpack->cap[2] = 0; 
	batpack->rem = (int8_t)(tele->batrem);
	buf[11] = crc8(buf + 2, 9);

	memcpy(bufcommon + 17, buf, 12);

	buf[1] = 17;
	buf[2] = 0x02;
	gpspack->lat = endian4((int32_t)(tele->lat * 1e7));
	gpspack->lon = endian4((int32_t)(tele->lon * 1e7));
	gpspack->speed = endian2((int16_t)(tele->speed * 18.52));
	gpspack->course = endian2((int16_t)(tele->course * 100.0));
	gpspack->alt = endian2((uint16_t)(tele->alt + 1000.0));
	gpspack->sats = tele->sats;
	buf[18] = crc8(buf + 2, 16);

	memcpy(bufcommon + 29, buf, 19);

	buf[1] = 6;
	buf[2] = 0x09;
	baltpack->balt = endian2((uint16_t)(tele->balt * 10.0 + 1e4));
	baltpack->vspeed = endian2((int16_t)(tele->vspeed * 100.0));
	buf[7] = crc8(buf + 2, 5);

	memcpy(bufcommon + 48, buf, 8);

	buf[1] = 8;
	buf[2] = 0x1e;
	attpack->pitch = endian2((int16_t)(tele->pitch * 1e4));
	attpack->roll = endian2((int16_t)(tele->roll * 1e4));
	attpack->yaw = endian2((int16_t)(tele->yaw * 1e4));
	buf[9] = crc8(buf + 2, 7);

	memcpy(bufcommon + 56, buf, 10);

	buf[1] = 4;
	buf[2] = 0x07;
	*((int16_t *)(buf + 3)) = endian2((int16_t)(tele->vspeed * 100.0));
	buf[5] = crc8(buf + 2, 3);

	memcpy(bufcommon + 66, buf, 6);

	HAL_UART_Transmit_DMA(d->huart, (uint8_t *) bufcommon, 72);

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

	sprintf(dev->name, "%s_%d", "crsf", crsf_devcount);

	dev->priv = crsf_devs + crsf_devcount;
	dev->read = crsf_read;
	dev->write = crsf_write;
	dev->interrupt = crsf_interrupt;
	dev->error = crsf_error;

	r = crsf_init(crsf_devs + crsf_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return 0;
};
