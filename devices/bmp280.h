/**
* @file bmp280.h
* @brief BMP280 barometer device driver
*/

#ifndef BMP280_H
#define BMP280_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define BMP_MAXDEVS 1

/**
* @brief maximum retries when writing
*/
#define BMP_WRITERETRIES 10

/**
* @brief standby time
*/
enum BMP_STANDBY {
	BMP_STANDBY_05		= 0x0,	/*!< 0.5 ms */
	BMP_STANDBY_62		= 0x1,	/*!< 62.5 ms */
	BMP_STANDBY_125		= 0x2,	/*!< 125 ms */
	BMP_STANDBY_250		= 0x3,	/*!< 250 ms */
	BMP_STANDBY_500		= 0x4,	/*!< 500 ms */
	BMP_STANDBY_1000	= 0x5,	/*!< 1000 ms */
	BMP_STANDBY_2000	= 0x6,	/*!< 2000 ms */
	BMP_STANDBY_4000	= 0x7	/*!< 4000 ms */
};

/**
* @brief IIR filter coefficient
*/
enum BMP_IIR {
	BMP_IIR_0	= 0x0,		/*!< 0 */
	BMP_IIR_2	= 0x2,		/*!< 2 */
	BMP_IIR_4	= 0x4,		/*!< 4 */
	BMP_IIR_8	= 0x8,		/*!< 8 */
	BMP_IIR_16	= 0x10		/*!< 16 */
};

/**
* @brief oversampling rate
*/
enum BMP_OVERSAMPLING {
	BMP_OVERSAMPLING_0	= 0x0,	/*!< 0 samples */
	BMP_OVERSAMPLING_1	= 0x1,	/*!< 1 samples */
	BMP_OVERSAMPLING_2	= 0x2,	/*!< 2 samples */
	BMP_OVERSAMPLING_4	= 0x3,	/*!< 4 samples */
	BMP_OVERSAMPLING_8	= 0x4,	/*!< 8 samples */
	BMP_OVERSAMPLING_16	= 0x5	/*!< 16 samples */
};

/**
* @brief power mode
*/
enum BMP_MODE {
	BMP_MODE_SLEEP	= 0x0,		/*!< sleep mode */
	BMP_MODE_FORCED	= 0x1,		/*!< forced mode */
	BMP_MODE_NORMAL	= 0x3,		/*!< normal mode */
};

/**
* @brief output data
*/
struct bmp_data {
	float temp;		/*!< temperature in celsius */
	float press;		/*!< pressure in kPa */
	float alt;		/*!< altitude in meters */
};

/**
* @brief device initialization and private data
*/
struct bmp_device {
	I2C_HandleTypeDef *hi2c;	/*!< I2C interface */

	int32_t t_fine;			///< private: calibration value
	uint16_t digt1, digp1;		///< private: calibration values
	int16_t digp[10];		///< private: calibration values
	int16_t digt[4];		///< private: calibration values
};

/**
* @brief initialize BMP280 device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int bmp_initdevice(struct bmp_device *is, struct cdevice *dev);

#endif
