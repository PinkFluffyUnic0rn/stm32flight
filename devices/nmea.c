#include "stm32f3xx_hal.h"
#include "esp8266.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include "nmea.h"

#define RXCIRCSIZE 4

#define NMEA_MAXMSG 84
#define NMEA_MAXFIELDCOUNT 32

#define UBX_TIMEOUT 1000

#define UBX_UART1OUTPROT_UBX 0x10740001
#define UBX_MSGOUT_NMEAVTG 0x209100b1
#define UBX_MSGOUT_NMEAGSA 0x209100c0
#define UBX_MSGOUT_NMEAGSV 0x209100c5
#define UBX_MSGOUT_NMEAGLL 0x209100ca

extern struct esp_device espdev;

static struct nmea_device nmea_devs[NMEA_MAXDEVS];
static size_t nmea_devcount = 0;

enum NMEA_MSGTYPE {
	NMEA_MSGTYPE_UBX = 0,
	NMEA_MSGTYPE_NMEA = 1
};

enum NMEA_ITSTATE {
	NMEA_ITSTATE_START = 0,
	NMEA_ITSTATE_NMEACHAR = 1,
	NMEA_ITSTATE_NMEAXORH = 2,
	NMEA_ITSTATE_NMEAXORL = 3,
	NMEA_ITSTATE_NMEACR = 4,
	NMEA_ITSTATE_NMEALF = 5,
	NMEA_ITSTATE_UBXSYNC2 = 11,
	NMEA_ITSTATE_UBXCLASS = 12,
	NMEA_ITSTATE_UBXID = 13,
	NMEA_ITSTATE_UBXLENL = 14,
	NMEA_ITSTATE_UBXLENH = 15,
	NMEA_ITSTATE_UBXCHAR = 16,
	NMEA_ITSTATE_UBXCHECKA = 17,
	NMEA_ITSTATE_UBXCHECKB = 18
};

struct nmea_msg {
	union {
		struct {
			char msg[84];
			uint8_t field[32];
			uint16_t xorgot;
			uint16_t xorshould;
		};

		uint8_t ubx[128];
	};

	uint8_t cnt;
	enum NMEA_MSGTYPE type;
};

uint8_t Rxbuf;
volatile static uint8_t Nmeaw = 0;
volatile static uint8_t Nmear = 0;
volatile static struct nmea_msg Msg[RXCIRCSIZE];

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

int nmea_interrupt(void *dev, const void *h) 
{
	static uint16_t Ubxlen = 0;
	static uint8_t Pos = 0;
	static uint8_t State = 0;
	struct nmea_device *d;
	uint8_t b;

	d = dev;

	if (((UART_HandleTypeDef *)h)->Instance != d->huart->Instance)
		return 0;
	
	b = Rxbuf;

	switch (State) {
	case NMEA_ITSTATE_START:
		if (b == 0xb5) {
			Msg[Nmeaw].cnt = 0;
			Msg[Nmeaw].type = NMEA_MSGTYPE_UBX;
			
			Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;

			State = NMEA_ITSTATE_UBXSYNC2;
		}
		else if (b == '$') {
			Msg[Nmeaw].type = NMEA_MSGTYPE_NMEA;
			Msg[Nmeaw].cnt = 0;
			Msg[Nmeaw].xorgot = 0;
			Pos = 0;

			Msg[Nmeaw].field[Msg[Nmeaw].cnt] = Pos;

			State = NMEA_ITSTATE_NMEACHAR;
		}

		break;

	case NMEA_ITSTATE_NMEACHAR:
		if (Pos >= 82)
			goto drop;

		if (b == ',') {
			Msg[Nmeaw].msg[Pos++] = '\0';

			if (++(Msg[Nmeaw].cnt) >= 32)
				goto drop;

			Msg[Nmeaw].field[Msg[Nmeaw].cnt] = Pos;
			
			Msg[Nmeaw].xorgot ^= b;
		}
		else if (b == '*')
			State = NMEA_ITSTATE_NMEAXORH;
		else  {
			Msg[Nmeaw].msg[Pos++] = b;
			
			Msg[Nmeaw].xorgot ^= b;
		}

		break;

	case NMEA_ITSTATE_NMEAXORH:
		Msg[Nmeaw].xorshould = asciitohex(b) << 4;
		State = NMEA_ITSTATE_NMEAXORL;
		break;

	case NMEA_ITSTATE_NMEAXORL:
		Msg[Nmeaw].xorshould |= asciitohex(b);
		State = NMEA_ITSTATE_NMEACR;
		break;

	case NMEA_ITSTATE_NMEACR:
		if (b != '\r')
			goto drop;

		State = NMEA_ITSTATE_NMEALF;
	
		break;

	case NMEA_ITSTATE_NMEALF:
		if (b != '\n')
			goto drop;

		if ((Nmeaw + 1) % RXCIRCSIZE != Nmear)
			Nmeaw = (Nmeaw + 1) % RXCIRCSIZE;

		State = NMEA_ITSTATE_START;
	
		break;

	case NMEA_ITSTATE_UBXSYNC2:
		if (b != 0x62)
			goto drop;
			
		Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;
	
		State = NMEA_ITSTATE_UBXCLASS;
		
		break;

	case NMEA_ITSTATE_UBXCLASS:
		Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;

		State = NMEA_ITSTATE_UBXID;
		break;
	
	case NMEA_ITSTATE_UBXID:
		Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;
	
		State = NMEA_ITSTATE_UBXLENL;
		break;

	case NMEA_ITSTATE_UBXLENL:
		Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;

		Ubxlen = b;

		State = NMEA_ITSTATE_UBXLENH;

		break;
	
	case NMEA_ITSTATE_UBXLENH:
		Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;
	
		Ubxlen |= (b << 8);
	
		State = NMEA_ITSTATE_UBXCHAR;
	
		break;

	case NMEA_ITSTATE_UBXCHAR:
		Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;
		
		--Ubxlen;
		
		if (Ubxlen == 0)
			State = NMEA_ITSTATE_UBXCHECKA;
		
		break;

	case NMEA_ITSTATE_UBXCHECKA:
		Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;
	
		State = NMEA_ITSTATE_UBXCHECKB;
		
		break;

	case NMEA_ITSTATE_UBXCHECKB:
		Msg[Nmeaw].ubx[Msg[Nmeaw].cnt++] = b;

		if ((Nmeaw + 1) % RXCIRCSIZE != Nmear)
			Nmeaw = (Nmeaw + 1) % RXCIRCSIZE;

		State = 0;
	
		break;
	}

	return 0;

drop:
	State = 0;

	return 0;
}

int nmea_error(void *dev, const void *h)
{
	struct nmea_device *d;
	
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

static int ubx_receivemsg(struct nmea_msg *m, const uint8_t *msg,
	int timeout)
{
	int t;

	for (t = 0; t < timeout; t += 1) {	
		if (Nmear == Nmeaw) {
			HAL_Delay(1);
			continue;
		}

		memcpyv(m, Msg + Nmear, sizeof(struct nmea_msg));
		
		Nmear = (Nmear + 1) % RXCIRCSIZE;

		if (m->type != NMEA_MSGTYPE_UBX) {
			HAL_Delay(1);
			continue;
		}

		if (memcmp(m->ubx, msg, 8) == 0)
			return 0;

		HAL_Delay(1);
	}

	return (-1);
}

static int ubx_send(struct nmea_device *nmea, uint8_t class, uint8_t id,
	uint16_t size, const uint8_t *data)
{
	uint8_t ack[8] = {
		0xb5, 0x62, 0x05, 0x01,
		0x02, 0x00, 0x06, 0x8a
	};
	struct nmea_msg m;
	uint8_t buf[32];
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

	HAL_UART_Transmit(nmea->huart, buf, 6 + size + 2, 1000);

	return ubx_receivemsg(&m, ack, UBX_TIMEOUT);
}

static int ubx_valset1(struct nmea_device *nmea, uint32_t id, uint8_t v)
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

	return ubx_send(nmea, 0x06, 0x8a, 0x9, buf);
}

static float nmea_time(const char *p)
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

static int nmea_latitude(const char *p, uint8_t *lat, float *latmin)
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

static int nmea_longitude(const char *p, uint8_t *lon, float *lonmin)
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

static int nmea_date(const char *p, char *date)
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

int nmea_read(void *dev, void *dt, size_t sz)
{
	struct nmea_data *data;
	struct nmea_msg m;

	data = (struct nmea_data *) dt;

	if (Nmear == Nmeaw)
		return (-1);

	memcpyv(&m, Msg + Nmear, sizeof(struct nmea_msg));
	
	Nmear = (Nmear + 1) % RXCIRCSIZE;
/*
	if (m.type == NMEA_MSGTYPE_UBX) {

  		char s[1024];
		int i;
		s[0] = '\0';
		
		for (i = 0; i < m.cnt; ++i)
			sprintf(s + strlen(s), "%hx ", m.ubx[i]);
			
		sprintf(s + strlen(s), "\r\n\r\n");

		esp_send(&espdev, s);
	
		esp_printf(&espdev, "%s\r\n", "UBX");

		return (-1);
	}
*/
	if (m.xorgot != m.xorshould)
		return (-1);	

//	esp_printf(&espdev, "%s\r\n", m.msg + m.field[0]);

	if (strcmp(m.msg + m.field[0], "GNGGA") == 0 ) {
		data->type = NMEA_TYPE_GGA;
		
		if (m.cnt < 13)
			return (-1);

		data->gga.time = nmea_time(m.msg + m.field[1]);

		nmea_latitude(m.msg + m.field[2], &(data->gga.lat),
			&(data->gga.latmin));
		data->gga.latdir = (m.msg + m.field[3])[0];

		nmea_longitude(m.msg + m.field[4], &(data->gga.lon),
			&(data->gga.lonmin));
		data->gga.londir = (m.msg + m.field[5])[0];

		data->gga.quality = atoi(m.msg + m.field[6]);
		data->gga.sats = atoi(m.msg + m.field[7]);
		data->gga.alt = strtof(m.msg + m.field[9], NULL);

		return 0;
	}
	else if (strcmp(m.msg + m.field[0], "GNRMC") == 0 ) {
		data->type = NMEA_TYPE_RMC;
		
		if (m.cnt < 13)
			return (-1);

		data->rmc.time = nmea_time(m.msg + m.field[1]);
	
		data->rmc.fstatus = (m.msg + m.field[2])[0] == 'A';

		nmea_latitude(m.msg + m.field[3], &(data->rmc.lat),
			&(data->rmc.latmin));
		data->rmc.latdir = (m.msg + m.field[4])[0];

		nmea_longitude(m.msg + m.field[5], &(data->rmc.lon),
			&(data->rmc.lonmin));
		data->rmc.londir = (m.msg + m.field[6])[0];
		
		data->rmc.speed = strtof(m.msg + m.field[7], NULL);
		data->rmc.course = strtof(m.msg + m.field[8], NULL);

		nmea_date(m.msg + m.field[9], data->rmc.date);

		data->rmc.magvar = strtof(m.msg + m.field[10], NULL);
		data->rmc.magvardir = (m.msg + m.field[11])[0];

		return 0;
	}
	else 
		data->type = NMEA_TYPE_OTHER;

	return 0;
}

int nmea_init(struct nmea_device *nmea)
{
	Nmeaw = Nmear = 0;
	
	HAL_UART_Receive_DMA(nmea->huart, &Rxbuf, 1);

	ubx_valset1(nmea, UBX_UART1OUTPROT_UBX, 0);
	
	ubx_valset1(nmea, UBX_MSGOUT_NMEAVTG, 0);
	ubx_valset1(nmea, UBX_MSGOUT_NMEAGSA, 0);
	ubx_valset1(nmea, UBX_MSGOUT_NMEAGSV, 0);
	ubx_valset1(nmea, UBX_MSGOUT_NMEAGLL, 0);

	return 0;
}

int nmea_initdevice(void *is, struct cdevice *dev)
{
	int r; 

	memmove(nmea_devs + nmea_devcount, is, sizeof(struct nmea_device));
	
	sprintf(dev->name, "%s%d", "nmea", nmea_devcount);

	dev->priv = nmea_devs + nmea_devcount;
	dev->read = nmea_read;
	dev->write = NULL;
	dev->interrupt = nmea_interrupt;

	dev->status = DEVSTATUS_IT;
	
	r = nmea_init(nmea_devs + nmea_devcount++);
	
	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;
		
	return 0;
};
