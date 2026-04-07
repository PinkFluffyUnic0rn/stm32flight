/**
* @file hp206c.h
* @brief HP206C barometer device driver
*/

#ifndef HP206C_H
#define HP206C_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define HP_MAXDEVS 1

/**
* @brief oversampling rate
*/
enum HP_OSR {
	HP_OSR_4096 = 0,	/*!< 4096 samples */
	HP_OSR_2048 = 1,	/*!< 2048 samples */
	HP_OSR_1024 = 2,	/*!< 1024 samples */
	HP_OSR_512 = 3,		/*!< 512 samples */
	HP_OSR_256 = 4,		/*!< 256 samples */
	HP_OSR_128 = 5		/*!< 128 samples */
};

/**
* @brief output data
*/
struct hp_data {
	int32_t alt;		/*!< raw altitude */
	int32_t temp;		/*!< raw temperature */
	float altf;		/*!< altitude in meters  */
	float tempf;		/*!< temperature in celsius */ 
};

/**
* @brief output data
*/
struct hp_device {
	I2C_HandleTypeDef *hi2c;	/*!< I2C interface */
	enum HP_OSR osr;		/*!< oversampling rate */
	float alt0;
};

/**
* @brief initialize HP206C device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int hp_initdevice(void *is, struct cdevice *dev);

#endif
