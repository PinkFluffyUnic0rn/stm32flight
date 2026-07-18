#ifndef SENSOR_H
#define SENSOR_H

/**
* @brief magnetometer output data
*/
struct mag_data {
	int32_t x, y, z;	///< x, y and z raw values
	double fx, fy, fz;	///< x, y and z values in gauss
};

/**
* @brief IMU output data
*/
struct imu_data {
	int16_t t;		/*!< temperature raw value */
	int16_t ax, ay, az;	///< accelerometer x, y and z raw values
	int16_t gx, gy, gz;	///< gyroscope x, y and z raw values

	double ft;		/*!< temperature in celsius */
	double afx, afy, afz;	///< accelerometer x, y and z values in g
	double gfx, gfy, gfz;	///< gyroscope x, y and z values in degrees/s
};

/**
* @brief barometer output data
*/
struct baro_data {
	double altf;		/*!< altitude in meters */
	double pressf;		/*!< pressure in kPa */
	double tempf;		/*!< temperature in celsius */
};

#endif
