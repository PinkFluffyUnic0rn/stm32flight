/**
* @file mcudef.h
* @brief choose HAL drivers depending on choosen MCU
*/

#ifdef STM32F4xx
#include "stm32f4xx_hal.h"
#elif STM32H7xx
#include "stm32h7xx_hal.h"
#endif
