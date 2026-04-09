/**
* @file w25.h
* @brief W25Q flash device driver
*/

#ifndef W25_H
#define W25_H

#include "mcudef.h"

#include "device.h"

/**
* @brief device page size for writing
*/
#define W25_PAGESIZE 256

/**
* @brief device sector size
*/
#define W25_SECTORSIZE 4096

/**
* @brief device block size
*/
#define W25_BLOCKSIZE (4096 * 16)

/**
* @brief device total size
*/
#define W25_TOTALSIZE (1024 * 1024 * 16)

/**
* @brief maximum devices of this type
*/
#define W25_MAXDEVS 1

/**
* @brief device mode
*/
enum W25_IOCTL {
	W25_IOCTL_READWRITE = 0,
	W25_IOCTL_WRITEONLY = 1
};

/**
* @brief device initialization and private data
*/
struct w25_device {
	SPI_HandleTypeDef *hspi;	/*!< SPI interface */
	GPIO_TypeDef *gpio;		/*!< CS pin GPIO port */
	uint16_t pin;			/*!< CS pin number */

	enum W25_IOCTL writemode;	/*!< private: device mode,
       						1, if write only,
						0 otherwise */
};

/**
* @brief initialize W25Q device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int w25_initdevice(struct w25_device *is, struct bdevice *dev);

#endif
