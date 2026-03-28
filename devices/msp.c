#include <stdint.h>
#include <string.h>
#include <stdio.h>


#include "crc.h"
#include "util.h"

#include "msp.h"

#define MSP_RXCIRCSIZE 2
#define MSP_BUFSIZE 64
#define MSP_DRAWSTEPS 14

enum MSP_CHAR {
	MSP_CHAR_M = 0x0c,
	MSP_CHAR_MPS = 0x9f,
	MSP_CHAR_TEMP = 0x7a,
	MSP_BAT_6 = 0x90,
	MSP_BAT_5 = 0x91,
	MSP_BAT_4 = 0x92,
	MSP_BAT_3 = 0x93,
	MSP_BAT_2 = 0x94,
	MSP_BAT_1 = 0x95,
	MSP_BAT_0 = 0x96,
	MSP_VOLT = 0x06,
	MSP_AMP = 0x9a,
	MSP_KMH = 0x9e,
	MSP_SATL = 0x1e,
	MSP_SATR = 0x1f,
	MSP_LAT = 0x89,
	MSP_LON = 0x98,
	MSP_ARROW_0 = 0x68
};

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

struct msp_buffer
{
	uint8_t buf[MSP_BUFSIZE];
	size_t sz;
};

static struct msp_device msp_devs[MSP_MAXDEVS];
static size_t msp_devcount = 0;

static uint8_t Rxbuf;

volatile static uint8_t Packstate;
volatile static uint8_t Packrest;
volatile static uint8_t Packw;
volatile static uint8_t Packr;
volatile static struct packet Pack[MSP_RXCIRCSIZE];

static struct msp_buffer Sendbuffer;

static int msp_initbuffer(struct msp_buffer *buf)
{
	buf->sz = 0;

	return 0;
}

static uint8_t *msp_stretchbuffer(struct msp_buffer *buf, size_t sz)
{
	size_t nsz;
	uint8_t *p;

	p = buf->buf + buf->sz;

	nsz = buf->sz + sz;

	if (nsz >= MSP_BUFSIZE) {
		nsz = MSP_BUFSIZE;
		sz = nsz - MSP_BUFSIZE;
	}

	buf->sz = nsz;

	return p;
}

int msp_interrupt(void *dev, const void *h) 
{
	struct msp_device *d;
	uint8_t b;

	d = dev;

	if (((UART_HandleTypeDef *)h)->Instance != d->huart->Instance)
		return 0;

	b = Rxbuf;
	
	if (Packstate == 0) {
		if ((Packw + 1) % MSP_RXCIRCSIZE == Packr)
			return 0;
		
		if (b == '$') {
			Packw = (Packw + 1) % MSP_RXCIRCSIZE;
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

	Packr = (Packr + 1) % MSP_RXCIRCSIZE;

	if (crc != Pack[Packr].crc)
		return (-1);

	return 0;
}

int msp_configure(void *d, const char *cmd, ...)
{
	return 0;
}

static int msp_writepacket(struct msp_buffer *sbuf,
	uint8_t cmd, uint8_t *pl, size_t len)
{
	uint8_t *buf;
	int i;

	buf = msp_stretchbuffer(sbuf, len + 6);
	
	buf[0] = '$';
	buf[1] = 'M';
	buf[2] = '>';
	buf[3] = len;
	buf[4] = cmd;

	memcpy(buf + 5, pl, len);

	buf[len + 5] = 0;
	for (i = 3; i < len + 5; ++i)
		buf[len + 5] ^= buf[i];

	return 0;
}

static int msp_drawstring(struct msp_buffer *sbuf, int x, int y,
	int color, const char *str)
{
	uint8_t *buf;
	size_t len;
	int i;

	len = strlen(str) + 4;

	buf = msp_stretchbuffer(sbuf, len + 6);

	buf[0] = '$';
	buf[1] = 'M';
	buf[2] = '>';
	buf[3] = len;
	buf[4] = 182;
	buf[5] = 3;
	buf[6] = y;
	buf[7] = x;
	buf[8] = color;

	// string length check?
	memcpy(buf + 9, str, len - 4);

	buf[len + 5] = 0;
	for (i = 3; i < len + 5; ++i)
		buf[len + 5] ^= buf[i];

	return 0;

}

static int msp_drawmode(struct msp_buffer *sbuf, int x, int y,
	struct msp_osd *osd, int step)
{
	const char *str;
	enum MSP_CHARCOLOR color;

	if (step == 0) {
		if (osd->armed) {
			str = "ARMED";
			color = MSP_CHARCOLOR_ORANGE;
		}
		else {
			str = "DISARM";
			color = MSP_CHARCOLOR_GREEN;
		}

		msp_drawstring(sbuf, x, y, color, str);
	}
	else if (step == 1) {
		if (osd->attmode == MSP_ATTMODE_GYRO)
			str = "ACCRO";
		else
			str = "ACC";

		msp_drawstring(sbuf, x, y + 1, MSP_CHARCOLOR_WHITE, str);
	}
	else if (step == 2) {
		if (osd->yawmode == MSP_YAWMODE_GYRO)
			str = "GYRO";
		else
			str = "MAG";

		msp_drawstring(sbuf, x, y + 2, MSP_CHARCOLOR_WHITE, str);
	}
	else if (step == 3) {
		if (osd->altmode == MSP_ALTMODE_ACCEL)
			str = "ACCEL";
		else if (osd->altmode == MSP_ALTMODE_SPEED)
			str = "SPEED";
		else
			str = "ALT";

		msp_drawstring(sbuf, x, y + 3, MSP_CHARCOLOR_WHITE, str);
	}

	return 0;
}

static int msp_drawalt(struct msp_buffer *sbuf, int x, int y,
	struct msp_osd *osd, int step)
{
	char buf[16];

	if (step == 0) {
		snprintf(buf, 16, "%0.1f%c", 
			(double) osd->alt, MSP_CHAR_M);

		msp_drawstring(sbuf, x, y, MSP_CHARCOLOR_WHITE, buf);
	}
	else if (step == 1) {
		snprintf(buf, 16, "%0.1f%c", (double) osd->vspeed,
			MSP_CHAR_MPS);
		msp_drawstring(sbuf, x, y + 1,
			MSP_CHARCOLOR_WHITE, buf);
	}	
	else if (step == 2) {
		snprintf(buf, 16, "%c%0.1f", MSP_CHAR_TEMP, 
			(double) osd->temp);
		msp_drawstring(sbuf, x, y + 2,
			MSP_CHARCOLOR_WHITE, buf);
	}

	return 0;
}

static int msp_drawpower(struct msp_buffer *sbuf, int x, int y,
	struct msp_osd *osd, int state)
{
	char buf[16];
	uint8_t batsym;

	if (state == 0) {
		if (osd->batrem > 95.0)
			batsym  = MSP_BAT_6;
		else if (osd->batrem > 82.0)
			batsym  = MSP_BAT_5;
		else if (osd->batrem > 64.0)
			batsym  = MSP_BAT_4;
		else if (osd->batrem > 46.0)
			batsym  = MSP_BAT_3;
		else if (osd->batrem > 28.0)
			batsym  = MSP_BAT_2;
		else if (osd->batrem > 10.0)
			batsym  = MSP_BAT_1;
		else 
			batsym  = MSP_BAT_0;

		snprintf(buf, 16, "%c%0.2f%c", batsym,
			(double) osd->bat, MSP_VOLT);
		msp_drawstring(sbuf, x, y, MSP_CHARCOLOR_WHITE, buf);
	}
	else if (state == 2) {
		snprintf(buf, 16, " %0.1f%c",
			(double) osd->curr, MSP_AMP);
		msp_drawstring(sbuf, x, y + 1,
			MSP_CHARCOLOR_WHITE, buf);
	}

	return 0;
}

static int msp_drawspeed(struct msp_buffer *sbuf, int x, int y,
	struct msp_osd *osd)
{
	char buf[16];

	snprintf(buf, 16, "%0.1f%c", (double) osd->speed, MSP_KMH);
	msp_drawstring(sbuf, x, y, MSP_CHARCOLOR_WHITE, buf);
	
	return 0;
}

static int msp_drawgps(struct msp_buffer *sbuf, int x, int y,
	struct msp_osd *osd, int state)
{
	char buf[16];

	if (state == 0) {	
		snprintf(buf, 16, "%c%c%d", MSP_SATL,
			MSP_SATR, osd->sats);
		msp_drawstring(sbuf, x, y, MSP_CHARCOLOR_WHITE, buf);
	}
	else if (state == 1) {
		snprintf(buf, 16, " %c%0.5f", MSP_LAT,
			(double) osd->lat);
		msp_drawstring(sbuf, x, y + 1,
			MSP_CHARCOLOR_WHITE, buf);
	}
	else if (state == 2) {
		snprintf(buf, 16, " %c%0.5f", MSP_LON,
			(double) osd->lon);
		msp_drawstring(sbuf, x, y + 2,
			MSP_CHARCOLOR_WHITE, buf);
	}

	return 0;
}

static int msp_drawyaw(struct msp_buffer *sbuf, int x, int y,
	struct msp_osd *osd)
{
	char buf[16];
	uint8_t dirsym;
	int dirn;

	dirn = ((osd->yaw / (M_PI)) * 8.0) + 0.5;

	if (dirn > 8)	dirn = 8;
	if (dirn < -7)	dirn = -7;

	dirsym = MSP_ARROW_0 - dirn;

	snprintf(buf, 2, "%c", dirsym);
	msp_drawstring(sbuf, x, y, MSP_CHARCOLOR_WHITE, buf);

	return 0;
}

int msp_write(void *dev, void *dt, size_t sz)
{
	struct msp_device *d;
	uint8_t dpcmd;
	static int step = 0;
	int t;

	d = dev;

	t = 0;
	while (HAL_UART_GetState(d->huart) != HAL_UART_STATE_READY
			&& t < 100000) {
		udelay(100);
		t += 100;
	}

	msp_initbuffer(&Sendbuffer);

	if (step == 0) {
		dpcmd = 2;
		msp_writepacket(&Sendbuffer, 182, &dpcmd, 1);
	}
		
	if (step == 0)
		msp_drawmode(&Sendbuffer, 1, 1, dt, 0);
	else if (step == 1)
		msp_drawmode(&Sendbuffer, 1, 1, dt, 1);
	else if (step == 2)
		msp_drawmode(&Sendbuffer, 1, 1, dt, 2);
	else if (step == 3)
		msp_drawmode(&Sendbuffer, 1, 1, dt, 3);
	else if (step == 4)
		msp_drawalt(&Sendbuffer, 15, 1, dt, 0);
	else if (step == 5)
		msp_drawalt(&Sendbuffer, 15, 1, dt, 1);
	else if (step == 6)
		msp_drawalt(&Sendbuffer, 15, 1, dt, 2);
	else if (step == 7)
		msp_drawpower(&Sendbuffer, 38, 1, dt, 0);
	else if (step == 8)
		msp_drawpower(&Sendbuffer, 38, 1, dt, 1);
	else if (step == 9)
		msp_drawspeed(&Sendbuffer, 5, 15, dt);
	else if (step == 10)
		msp_drawgps(&Sendbuffer, 43, 7, dt, 0);
	else if (step == 11)
		msp_drawgps(&Sendbuffer, 43, 7, dt, 1);
	else if (step == 12)
		msp_drawgps(&Sendbuffer, 43, 7, dt, 2);
	else if (step == 13)
		msp_drawyaw(&Sendbuffer, 25, 1, dt);

	if (step == MSP_DRAWSTEPS - 1) {
		dpcmd = 4;
		msp_writepacket(&Sendbuffer, 182, &dpcmd, 1);
	}
	if (++step == MSP_DRAWSTEPS)
		step = 0;

	HAL_UART_Transmit_DMA(d->huart, Sendbuffer.buf,
		Sendbuffer.sz);
	
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
	
	HAL_UART_DMAStop(msp->huart);

	return 0;
}

int msp_initdevice(void *is, struct cdevice *dev)
{
	int r; 

	memmove(msp_devs + msp_devcount, is, sizeof(struct msp_device));

	sprintf(dev->name, "%s_%d", "msp", msp_devcount);

	dev->priv = msp_devs + msp_devcount;
	dev->read = msp_read;
	dev->configure = msp_configure;
	dev->write = msp_write;
	dev->interrupt = msp_interrupt;
	dev->error = msp_error;
	
	dev->status = DEVSTATUS_IT;

	r = msp_init(msp_devs + msp_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return 0;
};
