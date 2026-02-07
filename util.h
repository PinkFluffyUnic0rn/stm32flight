/**
* @file util.h
* @brief Utility functions
*/

#ifndef UTIL_H
#define UTIL_H

/**
* @brief Get systick value as seconds in float value
* @return systick value
*/
#define getsysticksf() ((float) HAL_GetTick() / 1000.0)

/**
* @brief Check if int value is power of 2
* @param v value to check
* @return 1, if v is power of 2, 0 otherwise
*/
#define ispow2(v) ((v != 0) && ((v & (v - 1)) == 0))

/**
* @brief Set debug uart handler.
* @param huart debug uart handler.
* @return always 0
*/
int uartprintfinit(UART_HandleTypeDef *huart);

/**
* @brief Print data into debug UART interface.
* @param format arguments same as for printf.
* @return always 0
*/
int uartprintf(const char *format, ...);

/**
* @brief Volatile version of memcpy.
* @param dest destination buffer.
* @param src source buffer.
* @return none
*/
void memcpyv(volatile void *dest, const volatile void *src, size_t n);

/**
* @brief Set delay timer handler.
*
* @param htim delay timer handler.
* @return always 0
*/
int delayinit(TIM_HandleTypeDef *htim);

/**
* @brief Microseconds delay.
*
* @param us delay in microseconds.
* @return always 0
*/
int udelay(int us);

/**
* @brief Milliseconds delay.
*
* @param ms delay in microseconds.
* @return always 0
*/
int mdelay(int ms);

/**
* @brief Is value in list.
* @param v value.
* @param num list of values.
* @return 1, if v is in list, 0 otherwise
*/
int isvalinlist(int v, int num, ...);

#endif
