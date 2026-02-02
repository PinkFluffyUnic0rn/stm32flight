#ifndef DSHOT_H
#define DSHOT_H

#include "stm32f4xx_hal.h"

#include "device.h"

#define DSHOT_MAXDEVS 1

#define DSHOT_BITLEN 427	
#define DSHOT_0 160
#define DSHOT_1 320

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
