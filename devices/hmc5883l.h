/**
* @file hmc5883l.h
* @brief HMC5883L magnetometer device driver
*/

#ifndef HMC5883L_H
#define HMC5883L

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define HMC_MAXDEVS 1

/**
* @brief sampling rate
*/
enum HMC_RATE {
	HMC_RATE_0_75	= 0x0,
	HMC_RATE_1_5	= 0x1,
	HMC_RATE_3	= 0x2,
	HMC_RATE_7_5	= 0x3,
	HMC_RATE_15	= 0x4,
	HMC_RATE_30	= 0x5,
	HMC_RATE_75	= 0x6
};

/**
* @brief measuremnt scale
*/
enum HMC_SCALE {
	HMC_SCALE_088	= 0,
	HMC_SCALE_1_3	= 1,
	HMC_SCALE_1_9	= 2,
	HMC_SCALE_2_5	= 3,
	HMC_SCALE_4_0	= 4,
	HMC_SCALE_4_7	= 5,
	HMC_SCALE_5_6	= 6,
	HMC_SCALE_8_1	= 7
};

/**
* @brief output data
*/
struct hmc_data {
	int16_t x, y, z;	///< x, y and z raw values
	float fx, fy, fz;	///< x, y and z values in gauss
};

/**
* @brief device initialization and private data
*/
struct hmc_device {
	I2C_HandleTypeDef *hi2c;	/*!< I2C interface */
	int devtype;			/*!< device type */
	enum HMC_RATE rate;		/*!< sampling rate */
	enum HMC_SCALE scale;		/*!< measurement scale */
};

/**
* @brief initialize QMC5883L device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int hmc_initdevice(void *is, struct cdevice *dev);

#endif
