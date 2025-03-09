#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

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

static struct crsf_device crsf_devs[CRSF_MAXDEVS];
static size_t crsf_devcount = 0;

static uint8_t Rxbuf;

volatile static uint8_t Packstate;
volatile static uint8_t Packrest;
volatile static uint8_t Packw;
volatile static uint8_t Packr;
volatile static struct packet Pack[RXCIRCSIZE];

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

uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc;
	int i;

	crc = 0x00;
	for (i = 0; i < len; ++i)
		crc = crsf_crc8tbl[crc ^ *data++];

	return crc;
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
	
	if (crsf_crc8((uint8_t *)(Pack + Packr) + 1, 0x17)
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
	uint8_t buf[64];

	d = dev;
	tele = dt;

	buf[0] = 0xc8;
	buf[1] = 17;
	buf[2] = 0x02;
	
	*((int32_t *)(buf + 3)) = endian4((uint32_t)(tele->lat * 1e7));
	*((int32_t *)(buf + 7)) = endian4((uint32_t)(tele->lon * 1e7));
	*((int16_t *)(buf + 11)) = endian2((uint16_t)(tele->speed * 10.0));
	*((int16_t *)(buf + 13)) = endian2((uint16_t)(tele->course * 100.0));
	*((uint16_t *)(buf + 15)) = endian2((uint16_t)(tele->alt + 1000.0));
	*((int8_t *)(buf + 17)) = tele->sats;

	buf[18] = crsf_crc8(buf + 2, 16);

	HAL_UART_Transmit(d->huart, (uint8_t *) buf, 19, 1000);

	buf[0] = 0xc8;
	buf[1] = 5;
	buf[2] = 0x09;
	
	*((int16_t *)(buf + 3)) = endian2((uint16_t)(tele->balt * 10.0 + 10000.0));
	*((int16_t *)(buf + 5)) = endian2((uint16_t)(tele->vspeed * 100.0));

	buf[6] = crsf_crc8(buf + 2, 4);

	HAL_UART_Transmit(d->huart, (uint8_t *) buf, 7, 1000);

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

	r = crsf_init(crsf_devs + crsf_devcount++);
	
	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return 0;
};
