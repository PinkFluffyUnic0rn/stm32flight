#ifndef SENSOR_H
#define SENSOR_H

/**
* @brief magnetometer output data
*/
struct mag_data {
	int16_t x, y, z;	///< x, y and z raw values
	float fx, fy, fz;	///< x, y and z values in gauss
};

/**
* @brief IMU output data
*/
struct imu_data {
	int16_t t;		/*!< temperature raw value */
	int16_t ax, ay, az;	///< accelerometer x, y and z raw values
	int16_t gx, gy, gz;	///< gyroscope x, y and z raw values

	float ft;		/*!< temperature in celsius */
	float afx, afy, afz;	///< accelerometer x, y and z values in g
	float gfx, gfy, gfz;	///< gyroscope x, y and z values in degrees/s
};

/**
* @brief barometer output data
*/
struct baro_data {
	float altf;		/*!< altitude in meters */
	float pressf;		/*!< pressure in kPa */
	float tempf;		/*!< temperature in celsius */
};

#endif
