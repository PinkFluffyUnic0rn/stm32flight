/**
* @file mpu6500.h
* @brief MPU-6500/6050 device driver
*/

#ifndef MPU6500_H
#define MPU6500_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define MPU_MAXDEVS 1

/**
* @brief accelerometer measurement scale
*/
enum MPU_ACCELSCALE {
	MPU_2G	= 0,		/*!< +/- 2g */
	MPU_4G	= 8,		/*!< +/- 4g */
	MPU_8G	= 0x10,		/*!< +/- 8g */
	MPU_16G	= 0x18		/*!< +/- 16g */
};

/**
* @brief gyroscope measurement scale
*/
enum MPU_GYROSCALE {
	MPU_250DPS	= 0x0,	/*!< +/- 250 dergees/sec */
	MPU_500DPS	= 0x8,	/*!< +/- 500 dergees/sec */
	MPU_1000DPS	= 0x10,	/*!< +/- 1000 dergees/sec */
	MPU_2000DPS	= 0x18	/*!< +/- 2000 dergees/sec */
};

/**
* @brief IMU lpf width
*/
enum MPU_DLPF {
	MPU_260DLPF	= 0,	/*!< 260 hz */
	MPU_184DLPF	= 1,	/*!< 184 hz */
	MPU_94DLPF	= 2,	/*!< 94 hz */
	MPU_44DLPF	= 3,	/*!< 44 hz */
	MPU_21DLPF	= 4,	/*!< 21 hz */
	MPU_10DLPF	= 5,	/*!< 10 hz */
	MPU_5DLPF	= 6	/*!< 5 hz */
};

/**
* @brief device type
*/
enum MPU_DEVTYPE {
	MPU_DEV6050 = 0x68,	/*!< MPU-6050 */
	MPU_DEV6500 = 0x70	/*!< MPU-6500 */
};

/**
* @brief output data
*/
struct mpu_data {
	int16_t ax, ay, az;	///< accelerometer x, y and z raw values
	int16_t gx, gy, gz;	///< gyroscope x, y and z raw values

	float afx, afy, afz;	///< accelerometer x, y and z values in g
	float gfx, gfy, gfz;	///< gyroscope x, y and z values in degrees/s
};

/**
* @brief self-test output data
*/
struct mpu_stdata {
	float ax, ay, az;	///< accelerometer x, y and z self-test values
	float gx, gy, gz;	///< gyroscope x, y and z self-test values
};

/**
* @brief device initialization and private data
*/
struct mpu_device {
	SPI_HandleTypeDef *hspi;	/*!< SPI interface */

	GPIO_TypeDef *gpio;		/*!< CS pin GPIO port */
	uint16_t pin;			/*!< CS pin number */

	enum MPU_DEVTYPE devtype;	/*!< device type */
	enum MPU_ACCELSCALE accelscale;	/*!< accelerometer scale */
	enum MPU_GYROSCALE gyroscale;	/*!< gyroscope scale */
	enum MPU_DLPF dlpfwidth;	/*!< LPF width */
};

/**
* @brief initialize MPU-6500/6050 device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int mpu_initdevice(struct mpu_device *is, struct cdevice *dev);

#endif
