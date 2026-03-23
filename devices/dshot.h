#ifndef DSHOT_H
#define DSHOT_H

#include "mcudef.h"

#include "device.h"

#define DSHOT_MAXDEVS 1

enum DSHOT_TYPE {
	DSHOT_150,
	DSHOT_300,
	DSHOT_600,
	DSHOT_1200,
};

enum DSHOT_MODE {
	DSHOT_TIMBURST,
	DSHOT_DMACCR
};

struct dshot_device {
	TIM_HandleTypeDef *htim[4];
	uint32_t timch[4];
	int timfreq;
	enum DSHOT_MODE mode;
	enum DSHOT_TYPE type;

	int timcc[4];
	volatile uint32_t *timccr[4];
	DMA_HandleTypeDef *hdma[4];
	int bitlen;
	int zerolen;
	int onelen;
};

struct dshot_data {
	float thrust[4];
};

int dshot_initdevice(void *is, struct cdevice *dev);

#endif
