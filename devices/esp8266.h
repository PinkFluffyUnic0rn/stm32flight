/**
* @file esp8266.h
* @brief esp8266/esp8285 configuration device SPI driver
*/

#ifndef ESP_H
#define ESP_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define ESP_MAXDEVS 1

/**
* @brief maximum command size
*/
#define ESP_CMDSIZE 64

/**
* @brief device initialization and private data
*/
struct esp_device {
	SPI_HandleTypeDef *hspi;	/*!< SPI interface */
	GPIO_TypeDef *csgpio;		/*!< chip select GPIO port */
	uint16_t cspin;			/*!< chip select GPIO number */

	GPIO_TypeDef *rstgpio;		/*!< reset GPIO port */
	uint16_t rstpin;		/*!< reset GPIO number */

	GPIO_TypeDef *bootgpio;		/*!< boot GPIO port */
	uint16_t bootpin;		/*!< reset GPIO number */

	GPIO_TypeDef *busygpio;		/*!< busy state GPIO port */
	uint16_t busypin;		/*!< busy state GPIO 
						pin number */

	uint16_t intpin;		/*!< data ready interrupt
						GPIO pin number */
};

/**
* @brief initialize ESP8266/ESP8285 device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int esp_initdevice(void *is, struct cdevice *dev);

#endif
