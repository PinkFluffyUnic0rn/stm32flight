#include <stdint.h>
#include <string.h>
#include <stdio.h>


#include "crc.h"
#include "util.h"

#include "msp.h"

#define RXCIRCSIZE 4

enum MSP_CHARCOLOR {
	MSP_CHARCOLOR_WHITE = 0,
	MSP_CHARCOLOR_GREEN = 1,
	MSP_CHARCOLOR_ORANGE = 2,
	MSP_CHARCOLOR_RED = 3,
};

struct packet {
	char type;
	uint8_t len;
	uint8_t cmd;
	uint8_t pl[256];
	uint8_t crc;
};

static struct msp_device msp_devs[MSP_MAXDEVS];
static size_t msp_devcount = 0;

static uint8_t Rxbuf;

volatile static uint8_t Packstate;
volatile static uint8_t Packrest;
volatile static uint8_t Packw;
volatile static uint8_t Packr;
volatile static struct packet Pack[RXCIRCSIZE];

int msp_interrupt(void *dev, const void *h) 
{
	struct msp_device *d;
	uint8_t b;

	d = dev;

	if (((UART_HandleTypeDef *)h)->Instance != d->huart->Instance)
		return 0;
	
	b = Rxbuf;
	
	if (Packstate == 0) {
		if ((Packw + 1) % RXCIRCSIZE == Packr)
			return 0;
		
		if (b == '$') {
			Packw = (Packw + 1) % RXCIRCSIZE;
			Packstate = 1;
		}
	}
	else if (Packstate == 1) {
		if (b != 'M')
			goto drop;

		Packstate = 2;
	}
	else if (Packstate == 2) {
		if (b != '<' && b != '>' && b != '!')
			goto drop;
	
		Pack[Packw].type = b;

		Packstate = 3;
	}
	else if (Packstate == 3) {
		Pack[Packw].len = Packrest = b;
		
		Packstate = 4;
	}
	else if (Packstate == 4) {
		Pack[Packw].cmd = b;
	
		Packstate = 5;
	}
	else if (Packstate == 5) {
		if (Packrest == 0) {
			Pack[Packw].crc = b;
			Packstate = 0;
			return 0;
		}

		Pack[Packw].pl[Pack[Packw].len - Packrest] = b;
	
		--Packrest;
	}

	return 0;

drop:
	Packstate = 0;

	return 0;
}

int msp_error(void *dev, const void *h)
{
	struct msp_device *d;
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

int msp_read(void *dev, void *dt, size_t sz)
{
	struct msp_data *data;
	uint8_t crc;
	int i;

	data = (struct msp_data *) dt;

	if (Packr == Packw)
		return (-1);

	memcpyv(data->pl, Pack[Packr].pl, Pack[Packr].len);
	data->cmd = Pack[Packr].cmd;
	data->len = Pack[Packr].len;
	data->type = Pack[Packr].type;

	crc = Pack[Packr].len;
	crc |= Pack[Packr].cmd;

	for (i = 0; i < Pack[Packr].len; ++i)
		crc |= Pack[Packr].pl[i];

	Packr = (Packr + 1) % RXCIRCSIZE;

	if (crc != Pack[Packr].crc)
		return (-1);

	return 0;
}


int msp_sendpacket(struct msp_device *dev,
	uint8_t cmd, uint8_t *pl, size_t len)
{
	static uint8_t buf[256];

	buf[0] = '$';
	buf[1] = 'M';
	buf[2] = '>';
	buf[3] = len;
	buf[4] = cmd;

	memcpy(buf + 5, pl, len);

	int i;
	buf[len + 5] = 0;
	for (i = 3; i < len + 5; ++i)
		buf[len + 5] ^= buf[i];

	HAL_UART_Transmit(dev->huart, (uint8_t *) buf, len + 6, 1000);
	
	return 0;
}
// 0 -- write
// 1 -- green
// 2 -- orange
// 3 -- red



static int msp_drawstring(struct msp_device *dev, int x, int y,
	int color, const char *str)
{
	uint8_t buf[256];
	
	buf[0] = 3;
	buf[1] = y;
	buf[2] = x;
	buf[3] = color;

	strncpy((char *) buf + 4, str, 252);
	buf[255] = '\0';

	msp_sendpacket(dev, 182, buf, strlen(str) + 4);

	return 0;
}

static int msp_drawmode(struct msp_device *dev, int x, int y,
	struct msp_osd *osd)
{
	size_t sz;
	const char *str;
	enum MSP_CHARCOLOR color;

	if (osd->armed) {
		str = "ARMED";
		color = MSP_CHARCOLOR_ORANGE;
	}
	else {
		str = "DISARMED";
		color = MSP_CHARCOLOR_GREEN;
	}

	msp_drawstring(dev, x, y, color, str);


	if (osd->attmode == MSP_ATTMODE_GYRO)	str = "ACCRO";
	else					str = "STABILIZED";

	msp_drawstring(dev, x, y + 1, MSP_CHARCOLOR_WHITE, str);


	if (osd->yawmode == MSP_YAWMODE_GYRO)	str = "GYROYAW";
	else					str = "MAGNETOMETER";

	msp_drawstring(dev, x, y + 2, MSP_CHARCOLOR_WHITE, str);

	if (osd->altmode == MSP_ALTMODE_ACCEL)		str = "ACCEL";
	else if (osd->altmode == MSP_ALTMODE_SPEED)	str = "SPEED";
	else						str = "ALT";

	msp_drawstring(dev, x, y + 3, MSP_CHARCOLOR_WHITE, str);

	return 0;
}

static int msp_drawalt(struct msp_device *dev, int x, int y,
	struct msp_osd *osd)
{
	char buf[16];

	snprintf(buf, 16, "%0.1f%c", osd->alt, 0x0c); // meters
	msp_drawstring(dev, x, y, MSP_CHARCOLOR_WHITE, buf);
	
	snprintf(buf, 16, "%0.1f%c", osd->vspeed, 0x9f); // mps (meters)
	msp_drawstring(dev, x, y + 1, MSP_CHARCOLOR_WHITE, buf);
		
	snprintf(buf, 16, "%c%0.1f", 0x7a, osd->temp); // temperature
	msp_drawstring(dev, x, y + 2, MSP_CHARCOLOR_WHITE, buf);

	return 0;
}

static int msp_drawpower(struct msp_device *dev, int x, int y,
	struct msp_osd *osd)
{
	char buf[16];
	uint8_t batsym;

	if (osd->batrem > 95.0)
		batsym  = 0x90; // full
	else if (osd->batrem > 82.0)
		batsym  = 0x91;
	else if (osd->batrem > 64.0)
		batsym  = 0x92;
	else if (osd->batrem > 46.0)
		batsym  = 0x93;
	else if (osd->batrem > 28.0)
		batsym  = 0x94;
	else if (osd->batrem > 10.0)
		batsym  = 0x95;
	else 
		batsym  = 0x96;

	snprintf(buf, 16, "%c%0.2f%c", batsym, osd->bat, 0x06); // volts
	msp_drawstring(dev, x, y, MSP_CHARCOLOR_WHITE, buf);


	snprintf(buf, 16, " %0.1f%c", osd->curr, 0x9a); // amps
	msp_drawstring(dev, x, y + 1, MSP_CHARCOLOR_WHITE, buf);

	return 0;
}

static int msp_drawspeed(struct msp_device *dev, int x, int y,
	struct msp_osd *osd)
{
	char buf[16];

	snprintf(buf, 16, "%0.1f%c", osd->speed, 0x9e); // kmh
	msp_drawstring(dev, x, y, MSP_CHARCOLOR_WHITE, buf);
	
	return 0;
}

static int msp_drawgps(struct msp_device *dev, int x, int y,
	struct msp_osd *osd)
{
	char buf[16];
	
	snprintf(buf, 16, "%c%c%d", 0x1e, 0x1f, osd->sats); // sat
	msp_drawstring(dev, x, y, MSP_CHARCOLOR_WHITE, buf);

	snprintf(buf, 16, " %c%0.5f", 0x89, osd->lat); // lat
	msp_drawstring(dev, x, y + 1, MSP_CHARCOLOR_WHITE, buf);
	
	snprintf(buf, 16, " %c%0.5f", 0x98, osd->lon); // lon
	msp_drawstring(dev, x, y + 2, MSP_CHARCOLOR_WHITE, buf);
	
	return 0;
}


static int msp_drawyaw(struct msp_device *dev, int x, int y,
	struct msp_osd *osd)
{
	char buf[16];
	uint8_t dirsym;
	int dirn;

	dirn = ((osd->yaw / (M_PI)) * 8.0) + 0.5;

	if (dirn > 8)	dirn = 8;
	if (dirn < -7)	dirn = -7;

	dirsym = 0x68 - dirn;

	snprintf(buf, 2, "%c", dirsym); // arrow
	msp_drawstring(dev, x, y, MSP_CHARCOLOR_WHITE, buf);

	return 0;
}


int msp_write(void *dev, void *dt, size_t sz)
{
	struct msp_device *d;
	uint8_t dpcmd;

	d = dev;

	dpcmd = 2;
	msp_sendpacket(d, 182, &dpcmd, 1);

	msp_drawmode(d, 1, 1, dt);
	
	msp_drawalt(d, 15, 1, dt);
	
	msp_drawpower(d, 38, 1, dt);
	
	msp_drawspeed(d, 5, 15, dt);
	
	msp_drawgps(d, 43, 7, dt);
	
	msp_drawyaw(d, 25, 1, dt);

	dpcmd = 4;
	msp_sendpacket(d, 182, &dpcmd, 1);

	return 0;
}

int msp_init(struct msp_device *msp)
{
	struct msp_data md;
	int t;
	
	uint8_t mspbuf[32] = {
		101, 0, 125, 0, 0, 0, 0x7a,
		0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0
	};


	Packstate = 0;
	Packrest = 0;
	Packw = Packr = 0;
	
	HAL_UART_Receive_DMA(msp->huart, &Rxbuf, 1);
	
	t = 0;

	while (msp_read(msp, &md, sizeof(struct msp_data)) <= 0
			&& md.cmd != 101 && t < 100000) {
		HAL_Delay(100);
		t += 100;
	}

	msp_write(msp, mspbuf, 23);

	return 0;
}

int msp_initdevice(void *is, struct cdevice *dev)
{
	int r; 

	memmove(msp_devs + msp_devcount, is, sizeof(struct msp_device));

	sprintf(dev->name, "%s_%d", "msp", msp_devcount);

	dev->priv = msp_devs + msp_devcount;
	dev->read = msp_read;
	dev->write = msp_write;
	dev->interrupt = msp_interrupt;
	dev->error = msp_error;
	
	dev->status = DEVSTATUS_IT;

	r = msp_init(msp_devs + msp_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return 0;
};
