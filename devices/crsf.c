#include "stm32f3xx_hal.h"
#include <stdint.h>

#include "crsf.h"

struct packet {
	uint8_t type;
	uint8_t len;
	uint8_t pl[64];
};

static uint8_t Rxbuf;

volatile static uint8_t Packstate;
volatile static uint8_t Packrest;
volatile static uint8_t Packw;
volatile static uint8_t Packr;
volatile static struct packet Pack[16];

int crsf_interrupt(struct crsf_device *dev, const void *h) 
{
	uint8_t b;
	
	if (((UART_HandleTypeDef *)h)->Instance != dev->huart->Instance)
		return 0;

	b = Rxbuf;

	if (Packstate == 0) {
		if (b == 0xc8) {
			Packw = (Packw + 1) % 16;
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
	
		if (Packrest == 0) {
			Packr = Packw;
			Packstate = 0;
		}
	}

	return 0;
}

int crsf_read(struct crsf_channels *c)
{
	uint32_t merged;
	uint32_t value;
	int curb;
	int i;

	if (Packr == Packw)
		return (-1);

	if (Pack[Packr].type != 0x16)
		return (-1);

	merged = value = 0;
	curb = 0;

	for (i = 0; i < CRSF_CHANNELCOUNT; i++) {
		while (merged < 11) {
			value |= (Pack[Packr].pl[curb++]) << merged;
			merged += 8;
		}

		c->chf[i] = ((value & 0x000007ff) - 992.0) / 819.5;

		value >>= 11;
		merged -= 11;
	}

	return 0;
}

int crsf_init(struct crsf_device *crsf)
{
	HAL_UART_Receive_DMA(crsf->huart, &Rxbuf, 1);

	Packstate = 0;
	Packrest = 0;
	Packw = Packr = 0;

	return 0;
};
