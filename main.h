#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void error_handler(void);

#endif
