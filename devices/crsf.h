/**
* @file crsf.h
* @brief CRSF eLRS device driver
*/

#ifndef CRSF_H
#define CRSF_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define CRSF_MAXDEVS 1

/**
* @brief eLRS channels count
*/
#define CRSF_CHANNELCOUNT 16

/**
* @brief device initialization and private data
*/
struct crsf_device {
	UART_HandleTypeDef *huart;	/*!< UART interface */
};

/**
* @brief output data
*/
struct crsf_data {
	float chf[CRSF_CHANNELCOUNT];	/*!< value of each
						eLRS channel */
};

/**
* @brief input (telemetry) data
*/
struct crsf_tele {
	uint8_t mode[16];	/*!< flight mode */
	float bat;		/*!< battery voltage */
	float curr;		/*!< battery current */
	float batrem;		/*!< remaining battery voltage */
	float lat;		/*!< latitude */
	float lon;		/*!< longitude */
	float speed;		/*!< speed */
	float course;		/*!< course */
	float alt;		/*!< altitude */
	float balt;		/*!< barometric altitude */
	float vspeed;		/*!< vertical speed */
	uint8_t sats;		/*!< connected satellites */
	float pitch;		/*!< pitch */
	float roll;		/*!< roll */
	float yaw;		/*!< yaw */
};

/**
* @brief initialize CRSF eLRS device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int crsf_initdevice(void *is, struct cdevice *dev);

#endif
