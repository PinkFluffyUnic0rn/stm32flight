#ifndef ESP_H
#define ESP_H

#include "device.h"

#define ESP_MAXDEVS 1
#define ESP_CMDSIZE 64

struct esp_device {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *csgpio;
	uint16_t cspin;

	GPIO_TypeDef *rstgpio;
	uint16_t rstpin;

	GPIO_TypeDef *bootgpio;
	uint16_t bootpin;

	uint16_t intpin;
};

int esp_initdevice(void *is, struct cdevice *dev);

#endif
