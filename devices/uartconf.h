/**
* @file uartconf.h
* @brief UART debug device driver
*/

#ifndef UARTCONF_H
#define UARTCONF_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define UART_MAXDEVS 1

/**
* @brief maximum command size to
* transfer through debug UART
*/
#define UART_CMDSIZE 64

/**
* @brief device initialization and private data
*/
struct uart_device {
	UART_HandleTypeDef *huart;	/*!< UART interface */
	int interactive;		/*!< is UART debug device is
					in interactive mode */
};

/**
* @brief initialize UART debug device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int uart_initdevice(struct uart_device *is, struct cdevice *dev);

#endif
