/**
* @file mmc5983ma.h
* @brief MMC5983MA device driver
*/

#ifndef MMC5983MA_H
#define MMC5983MA_H

#include "mcudef.h"

#include "device.h"
#include "sensor.h"

/**
* @brief maximum devices of this type
*/
#define MMC_MAXDEVS 1

/**
* @brief sampling frequency
*/
enum MMC_FREQ {
	MMC_FREQ_0	= 0x0,		/*!< No continuous sampling */
	MMC_FREQ_1	= 0x1,		/*!< 1 samples/s */
	MMC_FREQ_10	= 0x2,		/*!< 10 samples/s */
	MMC_FREQ_20	= 0x3,		/*!< 20 samples/s */
	MMC_FREQ_50	= 0x4,		/*!< 50 samples/s */
	MMC_FREQ_100	= 0x5,		/*!< 100 samples/s */
	MMC_FREQ_200	= 0x6,		/*!< 200 samples/s */
	MMC_FREQ_1000	= 0x7,		/*!< 1000 samples/s */
};

/**
* @brief decimation bandwidth
*/
enum MMC_BW {
	MMC_BW_100	= 0,		/*!< 100 hz */
	MMC_BW_200	= 1,		/*!< 200 hz */
	MMC_BW_400	= 2,		/*!< 400 hz */
	MMC_BW_800	= 3		/*!< 800 hz */
};

/**
* @brief device initialization and private data
*/
struct mmc_device {
	I2C_HandleTypeDef *hi2c;	/*!< I2C interface */
	enum MMC_FREQ freq;		/*!< sampling frequency */
	enum MMC_BW bw;			/*!< decimation bandwidth */
};

/**
* @brief initialize MMC5983MA device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int mmc_initdevice(struct mmc_device *is, struct cdevice *dev);

#endif
