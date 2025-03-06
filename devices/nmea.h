#ifndef NMEA_H
#define NMEA_H

#include "device.h"

#define NMEA_MAXDEVS 1

struct nmea_device {
	UART_HandleTypeDef *huart;
};

enum NMEA_QUALITY
{
	NMEA_QUAL_NOFIX = 0,
	NMEA_QUAL_SINGLE = 1,
	NMEA_QUAL_PSEUDORANGE = 2,
	NMEA_QUAL_FIXEDAMB = 4,
	NMEA_QUAL_FLOATAMB = 5,
	NMEA_QUAL_DEADRECK = 6,
	NMEA_QUAL_MANUALINP = 7,
	NMEA_QUAL_SIMULATOR = 8,
	NMEA_QUAL_WAAS = 9
};

enum nmea_type
{
	NMEA_TYPE_GGA = 0,
	NMEA_TYPE_RMC = 1,
	NMEA_TYPE_OTHER = 2
};

struct nmea_data {
	enum nmea_type type;
	
	char msg[83];
	union {
		struct {
			float time;
			uint8_t lat;
			float latmin;
			char latdir;
			uint8_t lon;
			char londir;
			float lonmin;
			enum NMEA_QUALITY quality;
			uint8_t sats;
			float alt;
		} gga;
		
		struct {
			float time;
			uint8_t fstatus;
			uint8_t lat;
			float latmin;
			char latdir;
			uint8_t lon;
			char londir;
			float lonmin;
			float speed;
			float course;
			char date[10];
			float magvar;
			uint8_t magvardir;
		} rmc;
	};
};

int nmea_initdevice(void *is, struct cdevice *dev);

#endif
