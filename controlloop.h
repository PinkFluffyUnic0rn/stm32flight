#ifndef CONTROLLOOP_H
#define CONTROLLOOP_H

/**
* @brief Correction values needed for each axis and total thrust.
*/
struct corvals {
	float roll;	/*!< roll correction value */
	float pitch;	/*!< pitch correction value */
	float yaw;	/*!< ywa correction value */
	float thrust;	/*!< thrust coreection value */
};

/**
* @brief Init/set stabilization loop.
* @param init 1, if called during initilization, 0 otherwise
* @return always 0
*/
int setstabilize(int init);

/**
* @brief Calculate acceleration, speed and position
* 	estimation for each of 3 axes.
* @param seconds passed from last position update 
* @return always 0
*/
int updateposition(float dt);

/**
* @brief Update PID controllers and get needed correction values.
* @param seconds passed from last position update 
* @param needed correction values
* @return always 0
*/
int updatecorrection(float dt, struct corvals *cor);

/**
* @brief Apply thrust using passed correction values.
* @param correction values
* @return always 0
*/
int applythrust(const struct corvals *cor);

#endif
