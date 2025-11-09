/**
* @file global.h
* @brief Global values accesible throught whole project.
*/

#ifndef GLOBAL_H
#define GLOBAL_H

/**
* @brief clock frequency
*/
#define OCSFREQ 128000000

/**
* @brief periodic event timer prescaler
*/
#define PRESCALER 128

/**
* @brief PWM settings
*/
#define PWM_MAXCOUNT 3200

/**
* @defgroup DSHOT300 DSHOT300 values
* @{
*/
#define DSHOT_BITLEN 427	/*!< DSHOT-300 bit length
				in microseconds */
#define DSHOT_0 160		/*!< DSHOT-300 0 bit length
				in microseconds */
#define DSHOT_1 320		/*!< DSHOT-300 0 bit length
				in microseconds */
/**
* @}
*/

/**
* @brief periodic event timer period
*/
#define TIMPERIOD 0xfff

/**
* @brief periodic event timer ticks per second
*/
#define TICKSPERSEC (OCSFREQ / PRESCALER)

/**
* @brief delay timer prescaler
*/
#define DELAYPRESCALER 128

/**
* @brief maximum length for info packet sent back to operator
*/
#define INFOLEN 512

/**
* @brief maximum length for control command
*/
#define CMDSIZE 64

#endif
