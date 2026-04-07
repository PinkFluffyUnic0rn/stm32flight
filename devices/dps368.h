/**
* @file dps368.h
* @brief DPS368 barometer device driver
*/

#ifndef DSP368_H
#define DSP368_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define DPS_MAXDEVS 1

/**
* @brief oversampling rate
*/
enum DPS_OSR {
	DPS_OSR_1 = 0,		/*!< 1 sample */
	DPS_OSR_2 = 1,		/*!< 2 samples */
	DPS_OSR_4 = 2,		/*!< 4 samples */
	DPS_OSR_8 = 3,		/*!< 8 samples */
	DPS_OSR_16 = 4,		/*!< 16 samples */
	DPS_OSR_32 = 5,		/*!< 32 samples */
	DPS_OSR_64 = 6,		/*!< 64 samples */
	DPS_OSR_128 = 7		/*!< 128 samples */
};

/**
* @brief sampling rate
*/
enum DPS_RATE {
	DPS_RATE_1 = 0,		/*!< 1 hz */
	DPS_RATE_2 = 1,		/*!< 2 hz */
	DPS_RATE_4 = 2,		/*!< 4 hz */
	DPS_RATE_8 = 3,		/*!< 8 hz */
	DPS_RATE_16 = 4,	/*!< 16 hz */
	DPS_RATE_32 = 5,	/*!< 32 hz */
	DPS_RATE_64 = 6,	/*!< 64 hz */
	DPS_RATE_128 = 7,	/*!< 128 hz */
};

/**
* @brief output data
*/
struct dps_data {
	float altf;		/*!< altitude in meters */
	float pressf;		/*!< pressure in kPa */
	float tempf;		/*!< temperature in celsius */
};

/**
* @brief output data
*/
struct dps_device {
	I2C_HandleTypeDef *hi2c;	/*!< I2C interface */
	enum DPS_OSR osr;		/*!< oversampling rate */
	enum DPS_RATE rate;		/*!< sampling rate */

	int16_t c0, c1;			///< private: calibration values
	int32_t c00, c10, c01, c11,
		c20, c21, c30;		///< private: calibration values
};

/**
* @brief initialize DPS368 device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int dps_initdevice(void *is, struct cdevice *dev);

#endif
