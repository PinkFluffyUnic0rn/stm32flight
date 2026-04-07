/**
* @file qmc5883l.h
* @brief QMC5883L device driver
*/

#ifndef QMC5883L_H
#define QMC5883L_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define QMC_MAXDEVS 1

/**
* @brief sampling rate
*/
enum QMC_RATE {
	QMC_RATE_10	= 0x0,		/*!< 10 samples/s */
	QMC_RATE_50	= 0x1,		/*!< 50 samples/s */
	QMC_RATE_100	= 0x2,		/*!< 100 samples/s */
	QMC_RATE_200	= 0x3,		/*!< 200 samples/s */
};

/**
* @brief measuremnt scale
*/
enum QMC_SCALE {
	QMC_SCALE_2	= 0,		/*!< 2 gauss */
	QMC_SCALE_8	= 1		/*!< 8 gauss */
};

/**
* @brief oversampling rate
*/
enum QMC_OSR {
	QMC_OSR_512	= 0,		/*!< 512 samples */
	QMC_OSR_256	= 1,		/*!< 256 samples */
	QMC_OSR_128	= 2,		/*!< 128 samples */
	QMC_OSR_64	= 3		/*!< 64 samples */
};

/**
* @brief output data
*/
struct qmc_data {
	int16_t x, y, z;		/*!< x, y and z raw values */
	float fx, fy, fz;		/*!< x, y and z values converted
					into gauss */
};

/**
* @brief device initialization and private data
*/
struct qmc_device {
	I2C_HandleTypeDef *hi2c;	/*!< I2C interface */
	int devtype;			/*!< device type */
	enum QMC_RATE rate;		/*!< sampling rate */
	enum QMC_SCALE scale;		/*!< measurement scale */
	enum QMC_SCALE osr;		/*!< oversampling rate */
};

/**
* @brief initialize QMC5883L device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
*/
int qmc_initdevice(void *is, struct cdevice *dev);

#endif
