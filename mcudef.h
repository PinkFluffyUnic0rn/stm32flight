/**
* @file mcudef.h
* @brief choose HAL drivers depending on choosen MCU
* 	and configure memory regions attributes
*/

#ifdef STM32F4xx
#include "stm32f4xx_hal.h"

#define ITCMRAM
#define DTCMBUF
#define BDMABUF

#elif STM32H7xx
#include "stm32h7xx_hal.h"

#define ITCMRAM __attribute__((section(".itcmram"), noinline, noclone, used))
#define DTCMRAM __attribute__((section(".dtcmram")))
#define BDMABUF __attribute__((section(".bdma_buffer")))
#define DMABUF __attribute__((section(".dma_buffer")))

#endif
