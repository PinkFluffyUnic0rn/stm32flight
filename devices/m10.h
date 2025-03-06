#ifndef M10_H
#define M10_H

#include "device.h"

#define M10_MAXDEVS 1

struct m10_device {
	UART_HandleTypeDef *huart;
};

enum M10_QUALITY
{
	M10_QUAL_NOFIX = 0,
	M10_QUAL_SINGLE = 1,
	M10_QUAL_PSEUDORANGE = 2,
	M10_QUAL_FIXEDAMB = 4,
	M10_QUAL_FLOATAMB = 5,
	M10_QUAL_DEADRECK = 6,
	M10_QUAL_MANUALINP = 7,
	M10_QUAL_SIMULATOR = 8,
	M10_QUAL_WAAS = 9
};

enum M10_TYPE
{
	M10_TYPE_GGA = 0,
	M10_TYPE_RMC = 1,
	M10_TYPE_OTHER = 2
};

struct m10_data {
	enum M10_TYPE type;
	
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
			enum M10_QUALITY quality;
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

int m10_initdevice(void *is, struct cdevice *dev);

#endif
