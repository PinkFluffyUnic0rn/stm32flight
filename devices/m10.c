#include "stm32f4xx_hal.h"
#include "esp8266.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "m10.h"

#define RXCIRCSIZE 4

#define NMEA_MAXMSG 84
#define NMEA_MAXFIELDCOUNT 32

#define UBX_MAXMSG 128
#define UBX_TIMEOUT 1000
#define UBX_RETRIES 5

#define UBX_UART1OUTPROT_UBX	0x10740001
#define UBX_MSGOUT_NMEAVTG	0x209100b1
#define UBX_MSGOUT_NMEAGSA	0x209100c0
#define UBX_MSGOUT_NMEAGSV	0x209100c5
#define UBX_MSGOUT_NMEAGLL	0x209100ca

#define ubx_valset1(m10, id, v) _ubx_valset1(m10, id, v, 0)
#define ubx_valset1na(m10, id, v) _ubx_valset1(m10, id, v, 1)

enum M10_MSGTYPE {
	M10_MSGTYPE_UBX,
	M10_MSGTYPE_NMEA
};

enum M10_ITSTATE {
	M10_ITSTATE_START,
	M10_ITSTATE_NMEACHAR,
	M10_ITSTATE_NMEAXORH,
	M10_ITSTATE_NMEAXORL,
	M10_ITSTATE_NMEACR,
	M10_ITSTATE_NMEALF,
	M10_ITSTATE_UBXSYNC2,
	M10_ITSTATE_UBXCLASS,
	M10_ITSTATE_UBXID,
	M10_ITSTATE_UBXLENL,
	M10_ITSTATE_UBXLENH,
	M10_ITSTATE_UBXCHAR,
	M10_ITSTATE_UBXCHECKA,
	M10_ITSTATE_UBXCHECKB
};

struct m10_msg {
	union {
		struct {
			char msg[NMEA_MAXMSG];
			uint8_t field[NMEA_MAXFIELDCOUNT];
			uint16_t xorgot;
			uint16_t xorshould;
		};

		uint8_t ubx[UBX_MAXMSG];
	};

	uint8_t cnt;
	enum M10_MSGTYPE type;
};

extern struct esp_device espdev;

static struct m10_device m10_devs[M10_MAXDEVS];
static size_t m10_devcount = 0;

static uint8_t Rxbuf;

volatile static uint8_t Msgw;
volatile static uint8_t Msgr;
volatile static struct m10_msg Msg[RXCIRCSIZE];

static uint8_t asciitohex(uint8_t c)
{
	switch(c) {
	case '0':	return 0x0;
	case '1':	return 0x1;
	case '2':	return 0x2;
	case '3':	return 0x3;
	case '4':	return 0x4;
	case '5':	return 0x5;
	case '6':	return 0x6;
	case '7':	return 0x7;
	case '8':	return 0x8;
	case '9':	return 0x9;
	case 'A':	return 0xa;
	case 'B':	return 0xb;
	case 'C':	return 0xc;
	case 'D':	return 0xd;
	case 'E':	return 0xe;
	case 'F':	return 0xf;
	default:	return 0x0;
	}
}

int m10_interrupt(void *dev, const void *h) 
{
	static uint16_t Ubxlen = 0;
	static uint8_t Pos = 0;
	static uint8_t State = 0;
	struct m10_device *d;
	uint8_t b;

	d = dev;

	if (((UART_HandleTypeDef *)h)->Instance != d->huart->Instance)
		return 0;
	
	b = Rxbuf;

	switch (State) {
	case M10_ITSTATE_START:
		if (b == 0xb5) {
			Msg[Msgw].cnt = 0;
			Msg[Msgw].type = M10_MSGTYPE_UBX;
			
			Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;

			State = M10_ITSTATE_UBXSYNC2;
		}
		else if (b == '$') {
			Msg[Msgw].type = M10_MSGTYPE_NMEA;
			Msg[Msgw].cnt = 0;
			Msg[Msgw].xorgot = 0;
			Pos = 0;

			Msg[Msgw].field[Msg[Msgw].cnt] = Pos;

			State = M10_ITSTATE_NMEACHAR;
		}

		break;

	case M10_ITSTATE_NMEACHAR:
		if (Pos >= NMEA_MAXMSG)
			goto drop;

		if (b == ',') {
			Msg[Msgw].msg[Pos++] = '\0';

			if (++(Msg[Msgw].cnt) >= NMEA_MAXFIELDCOUNT)
				goto drop;

			Msg[Msgw].field[Msg[Msgw].cnt] = Pos;
			
			Msg[Msgw].xorgot ^= b;
		}
		else if (b == '*')
			State = M10_ITSTATE_NMEAXORH;
		else  {
			Msg[Msgw].msg[Pos++] = b;
			
			Msg[Msgw].xorgot ^= b;
		}

		break;

	case M10_ITSTATE_NMEAXORH:
		Msg[Msgw].xorshould = asciitohex(b) << 4;
		State = M10_ITSTATE_NMEAXORL;
		break;

	case M10_ITSTATE_NMEAXORL:
		Msg[Msgw].xorshould |= asciitohex(b);
		State = M10_ITSTATE_NMEACR;
		break;

	case M10_ITSTATE_NMEACR:
		if (b != '\r')
			goto drop;

		State = M10_ITSTATE_NMEALF;
	
		break;

	case M10_ITSTATE_NMEALF:
		if (b != '\n')
			goto drop;

		if ((Msgw + 1) % RXCIRCSIZE != Msgr)
			Msgw = (Msgw + 1) % RXCIRCSIZE;

		State = M10_ITSTATE_START;
	
		break;

	case M10_ITSTATE_UBXSYNC2:
		if (b != 0x62)
			goto drop;
			
		Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;
	
		State = M10_ITSTATE_UBXCLASS;
		
		break;

	case M10_ITSTATE_UBXCLASS:
		if (Msg[Msgw].cnt >= NMEA_MAXFIELDCOUNT) goto drop;
		
		Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;

		State = M10_ITSTATE_UBXID;
		break;
	
	case M10_ITSTATE_UBXID:
		if (Msg[Msgw].cnt >= NMEA_MAXFIELDCOUNT) goto drop;
		
		Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;
	
		State = M10_ITSTATE_UBXLENL;
		break;

	case M10_ITSTATE_UBXLENL:
		if (Msg[Msgw].cnt >= NMEA_MAXFIELDCOUNT) goto drop;
		
		Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;

		Ubxlen = b;

		State = M10_ITSTATE_UBXLENH;

		break;
	
	case M10_ITSTATE_UBXLENH:
		if (Msg[Msgw].cnt >= NMEA_MAXFIELDCOUNT) goto drop;
		
		Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;
	
		Ubxlen |= (b << 8);
	
		State = M10_ITSTATE_UBXCHAR;
	
		break;

	case M10_ITSTATE_UBXCHAR:
		if (Msg[Msgw].cnt >= NMEA_MAXFIELDCOUNT) goto drop;
	
		Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;
		
		--Ubxlen;
		
		if (Ubxlen == 0)
			State = M10_ITSTATE_UBXCHECKA;
		
		break;

	case M10_ITSTATE_UBXCHECKA:
		if (Msg[Msgw].cnt >= NMEA_MAXFIELDCOUNT) goto drop;
		
		Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;
	
		State = M10_ITSTATE_UBXCHECKB;
		
		break;

	case M10_ITSTATE_UBXCHECKB:
		if (Msg[Msgw].cnt >= NMEA_MAXFIELDCOUNT) goto drop;
		
		Msg[Msgw].ubx[Msg[Msgw].cnt++] = b;

		if ((Msgw + 1) % RXCIRCSIZE != Msgr)
			Msgw = (Msgw + 1) % RXCIRCSIZE;

		State = 0;
	
		break;
	}

	return 0;

drop:
	State = 0;

	return 0;
}

int m10_error(void *dev, const void *h)
{
	struct m10_device *d;
	
	d = dev;

	if (((UART_HandleTypeDef *)h)->Instance != d->huart->Instance)
		return 0;

	return 0;
}

static void memcpyv(volatile void *dest, const volatile void *src, size_t n)
{
	int i;

	for (i = 0; i < n; ++i)
		((uint8_t *) dest)[i] = ((uint8_t *) src)[i];
}

static int ubx_receivemsgs(struct m10_msg *m, const uint8_t **msg,
	const uint32_t *msglen, int msgcount, int timeout)
{
	int t;

	for (t = 0; t < timeout; t += 1) {
		int i;

		if (Msgr == Msgw) {
			HAL_Delay(1);
			continue;
		}

		memcpyv(m, Msg + Msgr, sizeof(struct m10_msg));
		
		Msgr = (Msgr + 1) % RXCIRCSIZE;

		if (m->type != M10_MSGTYPE_UBX) {
			HAL_Delay(1);
			continue;
		}

		for (i = 0; i < msgcount; ++i) {
			if (memcmp(m->ubx, msg[i], msglen[i]) == 0)
				return i;
		}

		HAL_Delay(1);
	}

	return (-1);
}

static int ubx_send(struct m10_device *m10, uint8_t class, uint8_t id,
	uint16_t size, const uint8_t *data, int noack)
{
	uint8_t ack[8] = {
		0xb5, 0x62, 0x05, 0x01,
		0x02, 0x00, 0x06, 0x8a
	};
	uint8_t nack[8] = {
		0xb5, 0x62, 0x05, 0x01,
		0x02, 0x00, 0x06, 0x8a
	};
	struct m10_msg m;
	uint8_t buf[32];
	const uint8_t *msg[2];
	uint32_t msglen[2];
	int i;	
	
	buf[0] = 0xb5;
	buf[1] = 0x62;
	buf[2] = class;
	buf[3] = id;
  	buf[4] = size & 0xff;
	buf[5] = size >> 8;

	for (i = 0; i < size; ++i)
		buf[6 + i] = data[i];

	buf[6 + size] = buf[6 + size + 1] = 0;
	for (i = 2; i < size + 6; ++i) {
		buf[6 + size] += buf[i];
		buf[6 + size + 1] += buf[6 + size];
	}

	msg[0] = ack;
	msg[1] = nack;
	msglen[0] = 8;
	msglen[1] = 8;

	for (i = 0; i < UBX_RETRIES; ++i) {
		HAL_UART_Transmit(m10->huart, buf, 6 + size + 2, 1000);
	
		if (noack)
			return 0;

		if (ubx_receivemsgs(&m, msg, msglen,
				2, UBX_TIMEOUT) == 0)
			return 0;
	}

	return (-1);
}

static int _ubx_valset1(struct m10_device *m10, uint32_t id, uint8_t v,
	int noack)
{
	uint8_t buf[16];
	
	buf[0] = 0x0;
	buf[1] = 0x1;
	buf[2] = 0x0;
	buf[3] = 0x0;

	buf[4] = id & 0xff;
	buf[5] = (id >> 8) & 0xff;
	buf[6] = (id >> 16) & 0xff;
	buf[7] = (id >> 24);
	buf[8] = v;

	return ubx_send(m10, 0x06, 0x8a, 0x9, buf, noack);
}

static float m10_time(const char *p)
{
	char buf[16];
	
	if (strlen(p) < 9)
		return 0.0;
	
	memmove(buf + 0, p, 2);
	buf[2] = '\0';

	memmove(buf + 3, p + 2, 2);
	buf[5] = '\0';

	strcpy(buf + 6, p + 4);

	return strtof(buf + 0, NULL) * 3600.0
		+ strtof(buf + 3, NULL) * 60.0
		+ strtof(buf + 6, NULL);
}

static int m10_latitude(const char *p, uint8_t *lat, float *latmin)
{
	char buf[16];
	
	if (strlen(p) < 5) {
		*lat = 0;
		*latmin = 0.0;
		
		return 0;
	}
	
	memmove(buf + 0, p, 2);
	buf[2] = '\0';
	strcpy(buf + 3, p + 2);

	*lat = atoi(buf + 0);
	*latmin = strtof(buf + 3, NULL);
	
	return 0;
}

static int m10_longitude(const char *p, uint8_t *lon, float *lonmin)
{
	char buf[16];

	if (strlen(p) < 5) {
		*lon = 0;
		*lonmin = 0.0;

		return 0;
	}
	
	memmove(buf + 0, p, 3);
	buf[3] = '\0';
	strcpy(buf + 4, p + 3);

	*lon = atoi(buf + 0);
	*lonmin = strtof(buf + 4, NULL);

	return 0;
}

static int m10_date(const char *p, char *date)
{
	if (strlen(p) < 6) {
		date[0] = '\0';
		return 0;
	}

	memmove(date + 0, p, 2);
	date[2] = '.';
	memmove(date + 3, p + 2, 2);
	date[5] = '.';
	memmove(date + 6, p + 4, 2);
	date[8] = '\0';

	return 0;
}

int m10_read(void *dev, void *dt, size_t sz)
{
	struct m10_data *data;
	struct m10_msg m;

	data = (struct m10_data *) dt;

	if (Msgr == Msgw)
		return (-1);

	memcpyv(&m, Msg + Msgr, sizeof(struct m10_msg));
	
	Msgr = (Msgr + 1) % RXCIRCSIZE;

	if (m.type == M10_MSGTYPE_UBX)
		return (-1);
	
	if (m.xorgot != m.xorshould)
		return (-1);

	if (strcmp(m.msg + m.field[0], "GNGGA") == 0 ) {
		data->type = M10_TYPE_GGA;
		
		if (m.cnt < 13)
			return (-1);

		data->gga.time = m10_time(m.msg + m.field[1]);

		m10_latitude(m.msg + m.field[2], &(data->gga.lat),
			&(data->gga.latmin));
		data->gga.latdir = (m.msg + m.field[3])[0];

		m10_longitude(m.msg + m.field[4], &(data->gga.lon),
			&(data->gga.lonmin));
		data->gga.londir = (m.msg + m.field[5])[0];

		data->gga.quality = atoi(m.msg + m.field[6]);
		data->gga.sats = atoi(m.msg + m.field[7]);
		data->gga.alt = strtof(m.msg + m.field[9], NULL);

		return 0;
	}
	else if (strcmp(m.msg + m.field[0], "GNRMC") == 0 ) {
		data->type = M10_TYPE_RMC;
		
		if (m.cnt < 13)
			return (-1);

		data->rmc.time = m10_time(m.msg + m.field[1]);
	
		data->rmc.fstatus = (m.msg + m.field[2])[0] == 'A';

		m10_latitude(m.msg + m.field[3], &(data->rmc.lat),
			&(data->rmc.latmin));
		data->rmc.latdir = (m.msg + m.field[4])[0];

		m10_longitude(m.msg + m.field[5], &(data->rmc.lon),
			&(data->rmc.lonmin));
		data->rmc.londir = (m.msg + m.field[6])[0];
		
		data->rmc.speed = strtof(m.msg + m.field[7], NULL);
		data->rmc.course = strtof(m.msg + m.field[8], NULL);

		m10_date(m.msg + m.field[9], data->rmc.date);

		data->rmc.magvar = strtof(m.msg + m.field[10], NULL);
		data->rmc.magvardir = (m.msg + m.field[11])[0];

		return 0;
	}
	else 
		data->type = M10_TYPE_OTHER;

	return 0;
}

int m10_init(struct m10_device *m10)
{
	Msgw = Msgr = 0;
	
	HAL_UART_Receive_DMA(m10->huart, &Rxbuf, 1);

	if (ubx_valset1na(m10, UBX_UART1OUTPROT_UBX, 1) < 0)
		return (-1);

	if (ubx_valset1(m10, UBX_MSGOUT_NMEAVTG, 0) < 0)
		return (-1);

	if (ubx_valset1(m10, UBX_MSGOUT_NMEAGSA, 0) < 0)
		return (-1);

	if (ubx_valset1(m10, UBX_MSGOUT_NMEAGSV, 0) < 0)
		return (-1);

	if (ubx_valset1(m10, UBX_MSGOUT_NMEAGLL, 0) < 0)
		return (-1);

	if (ubx_valset1(m10, UBX_UART1OUTPROT_UBX, 0) < 0)
		return (-1);

	return 0;
}

int m10_initdevice(void *is, struct cdevice *dev)
{
	int r; 

	memmove(m10_devs + m10_devcount, is, sizeof(struct m10_device));
	
	sprintf(dev->name, "%s_%d", "m10", m10_devcount);

	dev->priv = m10_devs + m10_devcount;
	dev->read = m10_read;
	dev->write = NULL;
	dev->interrupt = m10_interrupt;

	dev->status = DEVSTATUS_IT;
	
	r = m10_init(m10_devs + m10_devcount++);
	
	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;
		
	return 0;
};
