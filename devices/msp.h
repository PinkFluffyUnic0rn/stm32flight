#ifndef MSP_H
#define MSP_H

#include "mcudef.h"

#include "device.h"

#define MSP_MAXDEVS 1

enum MSP_ATTMODE {
	MSP_ATTMODE_GYRO,
	MSP_ATTMODE_ACC
};

enum MSP_YAWMODE {
	MSP_YAWMODE_GYRO,
	MSP_YAWMODE_MAG
};

enum MSP_ALTMODE {
	MSP_ALTMODE_ACCEL,
	MSP_ALTMODE_SPEED,
	MSP_ALTMODE_POS
};

struct msp_device {
	UART_HandleTypeDef *huart;
};

struct msp_data {
	char pl[256];
	uint8_t cmd;
	uint8_t len;
	char type;
};

struct msp_osd {
	int armed;
	enum MSP_ATTMODE attmode;
	enum MSP_YAWMODE yawmode;
	enum MSP_ALTMODE altmode;
	float bat;
	float curr;
	float batrem;
	float lat;
	float lon;
	float speed;
	float course;
	float alt;
	float vspeed;
	float temp;
	uint8_t sats;
	float pitch;
	float roll;
	float yaw;
};

int msp_initdevice(void *is, struct cdevice *dev);

#endif
