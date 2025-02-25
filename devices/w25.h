#ifndef W25_H
#define W25_H

#include "device.h"

#define W25_PAGESIZE 256
#define W25_SECTORSIZE 4096
#define W25_BLOCKSIZE (4096 * 16)
#define W25_TOTALSIZE (1024 * 1024 * 16)

#define W25_MAXDEVS 1

struct w25_device {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *gpio;
	uint16_t pin;
};

int w25_initdevice(void *is, struct bdevice *dev);

#endif
