/**
* @file icm42688.h
* @brief ICM-42688-P IMU device driver
*/

#ifndef ICM42688_H
#define ICM42688_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define ICM_MAXDEVS 1

/**
* @brief gyroscope measurement scale
*/
enum ICM_GYROSCALE {
	ICM_2000DPS	= 0x0,	/*!< 2000 degrees/s */
	ICM_1000DPS	= 0x1,	/*!< 1000 degrees/s */
	ICM_500DPS	= 0x2,	/*!< 500 degrees/s */
	ICM_250DPS	= 0x3,	/*!< 250 degrees/s */
	ICM_125DPS	= 0x4,	/*!< 125 degrees/s */
	ICM_62_5DPS	= 0x5,	/*!< 62.5 degrees/s */
	ICM_31_25DPS	= 0x6,	/*!< 31.25 degrees/s */
	ICM_15_625DPS	= 0x7,	/*!< 15.625 degrees/s */
};

/**
* @brief gyroscope sampling rate
*/
enum ICM_GYRORATE {
	ICM_GYRO32K	= 0x1,	/*!< 32 kHz */
	ICM_GYRO16K	= 0x2,	/*!< 16 kHz */
	ICM_GYRO8K	= 0x3,	/*!< 8 kHz */
	ICM_GYRO4K	= 0x4,	/*!< 4 kHz */
	ICM_GYRO2K	= 0x5,	/*!< 2 kHz */
	ICM_GYRO1K	= 0x6,	/*!< 1 kHz */
	ICM_GYRO200	= 0x7,	/*!< 200 Hz */
	ICM_GYRO100	= 0x8,	/*!< 100 Hz */
	ICM_GYRO50	= 0x9,	/*!< 50 Hz */
	ICM_GYRO2_5	= 0xa,	/*!< 25 Hz */
	ICM_GYRO1_25	= 0xb,	/*!< 12.5 Hz */
};

/**
* @brief gyroscope LPF order
*/
enum ICM_GYROORDER {
	ICM_GYROORDER1	= 0x1,	/*!< 1st order LPF */
	ICM_GYROORDER2	= 0x2,	/*!< 2nd order LPF */
	ICM_GYROORDER3	= 0x3,	/*!< 3rd order LPF */
};

/**
* @brief gyroscope LPF divider
*/
enum ICM_GYROLPF {
	ICM_GYROLPF2	= 0x0,	/*!< ODR / 2 */
	ICM_GYROLPF4	= 0x1,	/*!< max(400 Hz, ODR) / 4 */
	ICM_GYROLPF5	= 0x2,	/*!< max(400 Hz, ODR) / 5 */
	ICM_GYROLPF8	= 0x3,	/*!< max(400 Hz, ODR) / 8 */
	ICM_GYROLPF10	= 0x4,	/*!< max(400 Hz, ODR) / 10 */
	ICM_GYROLPF16	= 0x5,	/*!< max(400 Hz, ODR) / 16 */
	ICM_GYROLPF20	= 0x6,	/*!< max(400 Hz, ODR) / 20 */
	ICM_GYROLPF40	= 0x7,	/*!< max(400 Hz, ODR) / 20 */
	ICM_GYROLPFLL	= 0x15,	/*!< low latency */
};

/**
* @brief accelerometer scale
*/
enum ICM_ACCELSCALE {
	ICM_16G	= 0x0,	/*!< -/+ 16 g */
	ICM_8G	= 0x1,	/*!< -/+ 8 g */
	ICM_4G	= 0x2,	/*!< -/+ 4 g */
	ICM_2G	= 0x3	/*!< -/+ 2 g */
};

/**
* @brief accelerometer sampling rate
*/
enum ICM_ACCELRATE {
	ICM_ACCEL32K	= 0x1,	/*!< 32 kHz */
	ICM_ACCEL16K	= 0x2,	/*!< 16 kHz */
	ICM_ACCEL8K	= 0x3,	/*!< 8 kHz */
	ICM_ACCEL4K	= 0x4,	/*!< 4 kHz */
	ICM_ACCEL2K	= 0x5,	/*!< 2 kHz */
	ICM_ACCEL1K	= 0x6,	/*!< 1 kHz */
	ICM_ACCEL200	= 0x7,	/*!< 200 Hz */
	ICM_ACCEL100	= 0x8,	/*!< 100 Hz */
	ICM_ACCEL50	= 0x9,	/*!< 50 Hz */
	ICM_ACCEL25	= 0xa,	/*!< 25 Hz */
	ICM_ACCEL12_5	= 0xb,	/*!< 12.5 Hz */
	ICM_ACCEL500	= 0xf	/*!< 500 Hz */
};

/**
* @brief accelerometer LPF divider
*/
enum ICM_ACCELLPF {
	ICM_ACCELLPF2	= 0x0,	/*!< ODR / 2 */
	ICM_ACCELLPF4	= 0x1,	/*!< max(400 Hz, ODR) / 4 */
	ICM_ACCELLPF5	= 0x2,	/*!< max(400 Hz, ODR) / 5 */
	ICM_ACCELLPF8	= 0x3,	/*!< max(400 Hz, ODR) / 8 */
	ICM_ACCELLPF10	= 0x4,	/*!< max(400 Hz, ODR) / 10 */
	ICM_ACCELLPF16	= 0x5,	/*!< max(400 Hz, ODR) / 16 */
	ICM_ACCELLPF20	= 0x6,	/*!< max(400 Hz, ODR) / 20 */
	ICM_ACCELLPF40	= 0x7,	/*!< max(400 Hz, ODR) / 40 */
	ICM_ACCELLPFLL	= 0x15	/*!< low latency */
};

/**
* @brief accelerometer LPF order
*/
enum ICM_ACCELORDER {
	ICM_ACCELORDER1	= 0x1,	/*!< 1st order LPF */
	ICM_ACCELORDER2	= 0x2,	/*!< 2nd order LPF */
	ICM_ACCELORDER3	= 0x3,	/*!< 3rd order LPF */
};

/**
* @brief output data
*/
struct icm_data {
	int16_t t;		/*!< temperature raw value */
	int16_t ax, ay, az;	///< accelerometer x, y and z raw values
	int16_t gx, gy, gz;	///< gyroscope x, y and z raw values

	float ft;		/*!< temperature in celsius */
	float afx, afy, afz;	///< accelerometer x, y and z values in g
	float gfx, gfy, gfz;	///< gyroscope x, y and z values in degrees/s
};

/**
* @brief self-test output data
*/
struct icm_stdata {
	float ax, ay, az;	///< accelerometer x, y and z self-test values
	float gx, gy, gz;	///< gyroscope x, y and z self-test values
};

/**
* @brief device initialization and private data
*/
struct icm_device {
	SPI_HandleTypeDef *hspi;		/*!< SPI interface */

	GPIO_TypeDef *gpio;		/*!< CS pin GPIO port */
	uint16_t pin;			/*!< CS pin number */

	enum ICM_GYROSCALE gyroscale;	/*!< gyroscope scale */
	enum ICM_GYRORATE gyrorate;	/*!< gyroscope sampling rate */
	enum ICM_GYROORDER gyroorder;	/*!< gyroscope LPF order */
	enum ICM_GYROLPF gyrolpf;	/*!< gyroscope LPF divider */
	enum ICM_ACCELSCALE accelscale;	/*!< accelerometer scale */
	enum ICM_ACCELRATE accelrate;	/*!< accelerometer 
					sampling rate */
	enum ICM_ACCELLPF accellpf;	/*!< accelerometer
					LPF divider */
	enum ICM_ACCELORDER accelorder;	/*!< acceleromter LPF order */
};

/**
* @brief initialize ICM-42688-P device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int icm_initdevice(void *is, struct cdevice *dev);

#endif
