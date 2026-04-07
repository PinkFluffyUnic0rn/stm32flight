/**
* @file msp.h
* @brief MSP VTX device driver
*/

#ifndef MSP_H
#define MSP_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define MSP_MAXDEVS 1

/**
* @brief attitude mode
*/
enum MSP_ATTMODE {
	MSP_ATTMODE_GYRO,	/*!< gyroscope stabilized */
	MSP_ATTMODE_ACC		/*!< gyroscope + acceleromter
				stabilized */
};

/**
* @brief yaw mode
*/
enum MSP_YAWMODE {
	MSP_YAWMODE_GYRO,	/*!< gyroscope stabilized */
	MSP_YAWMODE_MAG		/*!< gyroscope + magnetometer
				stabilized */
};

/**
* @brief altitude mode
*/
enum MSP_ALTMODE {
	MSP_ALTMODE_ACCEL,	/*!< acceleration stabilized */
	MSP_ALTMODE_SPEED,	/*!< climb speed stabilized */
	MSP_ALTMODE_POS		/*!< altitude stabilized */
};

/**
* @brief device initialization and private data
*/
struct msp_device {
	UART_HandleTypeDef *huart;	/*!< UART interface */
};

/**
* @brief MSP command data
*/
struct msp_data {
	char pl[256];	/*!< payload */
	uint8_t cmd;	/*!< cmd */
	uint8_t len;	/*!< payload length */
	char type;	/*!< request or response */
};

/**
* @brief MSP osd input data
*/
struct msp_osd {
	int armed;			/*!< is UAV armed */
	enum MSP_ATTMODE attmode;	/*!< attitude mode */
	enum MSP_YAWMODE yawmode;	/*!< yaw mode */
	enum MSP_ALTMODE altmode;	/*!< altitude mode */
	float bat;			/*!< battery voltage */
	float curr;			/*!< current draw */
	float batrem;			/*!< remaining battery charge */
	float lat;			/*!< GNSS latitude */
	float lon;			/*!< GNSS longitude */
	float speed;			/*!< GNSS speed */
	float course;			/*!< GNSS course */
	float alt;			/*!< altitude */
	float vspeed;			/*!< climb speed */
	float temp;			/*!< board temperatude */
	uint8_t sats;			/*!< found GNSS satellites */
	float pitch;			/*!< pitch */
	float roll;			/*!< roll */
	float yaw;			/*!< yaw */
};

/**
* @brief initialize MSP VTX device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int msp_initdevice(void *is, struct cdevice *dev);

#endif
