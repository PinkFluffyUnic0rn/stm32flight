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
* @brief GNSS mode
*/
enum MSP_GNSSMODE {
	MSP_GNSSMODE_NONE,	/*!< no GNSS stabilization */
	MSP_GNSSMODE_SPEED,	/*!< speed stabilization */
	MSP_GNSSMODE_POS	/*!< pos stabilization */
};

/**
* @brief device initialization and private data
*/
struct msp_device {
	UART_HandleTypeDef *huart;	/*!< UART interface */
};

/**
* @brief MSP osd input data
*/
struct msp_osd {
	int armed;			/*!< is UAV armed */
	enum MSP_ATTMODE attmode;	/*!< attitude mode */
	enum MSP_YAWMODE yawmode;	/*!< yaw mode */
	enum MSP_ALTMODE altmode;	/*!< altitude mode */
	enum MSP_GNSSMODE gnssmode;	/*!< GNSS mode */
	double bat;			/*!< battery voltage */
	double curr;			/*!< current draw */
	double batrem;			/*!< remaining battery charge */
	double lat;			/*!< GNSS latitude */
	double lon;			/*!< GNSS longitude */
	double speed;			/*!< GNSS speed */
	double course;			/*!< GNSS course */
	double alt;			/*!< altitude */
	double vspeed;			/*!< climb speed */
	double temp;			/*!< board temperatude */
	uint8_t sats;			/*!< found GNSS satellites */
	double pitch;			/*!< pitch */
	double roll;			/*!< roll */
	double yaw;			/*!< yaw */
};

/**
* @brief initialize MSP VTX device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int msp_initdevice(struct msp_device *is, struct cdevice *dev);

#endif
