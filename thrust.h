#ifndef THRUST_H
#define THRUST_H

#include <stdint.h>

/**
* @brief Send dshot special command to a DSHOT output.
* @param n a DSHOT output's number
* @param cmd command number
* @return always 0
*/
int dshotcmd(int n, uint16_t cmd);

/**
* @brief Set motors thrust. All values should be between 0.0 and 1.0.
* @param left-top motor thrust
* @param right-top motor thrust
* @param left-bottom motor thrust
* @param right-bottom motor thrust
* @return always 0
*/
int setthrust(float ltd, float rtd, float lbd, float rbd);

/**
* @brief Init ESC's.
* @return none
*/
void dshotinit();

#endif
