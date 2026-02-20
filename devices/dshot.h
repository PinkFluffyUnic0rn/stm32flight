#ifndef DSHOT_H
#define DSHOT_H

#include "mcudef.h"

#include "device.h"

#define DSHOT_MAXDEVS 1

#ifdef STM32F4xx
#define DSHOT_BITLEN 427	
#define DSHOT_0 160
#define DSHOT_1 320
#elif STM32H7xx
#define DSHOT_BITLEN 833	
#define DSHOT_0 313
#define DSHOT_1 625
#endif

struct dshot_device {
	TIM_HandleTypeDef *htim[4];
	uint32_t timch[4];
	
	int timcc[4];
	volatile uint32_t *timccr[4];
	DMA_HandleTypeDef *hdma[4];
};

struct dshot_data {
	float thrust[4];
};

int dshot_initdevice(void *is, struct cdevice *dev);

#endif
