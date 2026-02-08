/**
* @file stm32periph.h
* @brief STM32 periphery handling functions
*/

#ifndef PERIPHCONF_H
#define PERIPHCONF_H

#include "stm32f4xx_hal.h"

extern ADC_HandleTypeDef pconf_hadcs[5];
extern DMA_HandleTypeDef pconf_hdmas[16];
extern I2C_HandleTypeDef pconf_hi2cs[3];
extern SPI_HandleTypeDef pconf_hspis[3];
extern TIM_HandleTypeDef pconf_htims[5];
extern UART_HandleTypeDef pconf_huarts[5];

int pconf_uartidx(USART_TypeDef* inst);

int pconf_adcidx(ADC_TypeDef *inst);

int pconf_i2cidx(I2C_TypeDef *inst);

int pconf_spiidx(SPI_TypeDef *inst);

int pconf_timidx(TIM_TypeDef *inst);

void pconf_init();

#endif
