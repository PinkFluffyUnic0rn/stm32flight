/**
* @file device.h
* @brief Common interfaces for character and block devices
*/

#ifndef DEVICE_H
#define DEVICE_H

#include <stdint.h>
#include <stddef.h>

/**
* @brief maximum device name length
*/
#define DEVNAMEMAX 32

/**
* @brief check if interrupts on devices are enabled
*
* @param dev device
* @return 1 if interrupts are enabled on device, 0 otherwise
*/
#define DEVITENABLED(dev) ((dev) == DEVSTATUS_IT \
	|| (dev) == DEVSTATUS_INIT)

/**
* @brief device status
*/
enum DEVSTATUS {
	DEVSTATUS_IT = 0,	/*!< interrupts are enabled, but
				device not initialized */
	DEVSTATUS_INIT = 1,	/*!< device initialized */
	DEVSTATUS_FAILED = 2,	/*!< device failed to initialize */
	DEVSTATUS_NOINIT = 3	/*!< device not initialized */
};

/**
* @brief character device
*/
struct cdevice {
	char name[DEVNAMEMAX];			/*!< device name */

	/**
	* @brief read data from device
	* @param dev device's private data
	* @param data output buffer
	* @param sz output buffer size
	* @return -1 in case of error, 0 otherwise
	*/	
	int (*read)(void *priv, void *data, size_t sz);

	/**
	* @brief write data to device
	* @param dev device's private data
	* @param data input buffer
	* @param sz input buffer size
	* @return -1 in case of error, 0 otherwise
	*/	
	int (*write)(void *priv, void *data, size_t sz);

	/**
	* @brief process interrupt triggered by device
	* @param dev device's private data
	* @param interface that triggered interrupt
	* @return -1 in case of error, 0 otherwise
	*/
	int (*interrupt)(void *priv, const void *h);
	
	
	/**
	* @brief process error interrupt triggered by device
	* @param dev device's private data
	* @param interface that triggered error interrupt
	* @return -1 in case of error, 0 otherwise
	*/
	int (*error)(void *priv, const void *h);

	/**
	* @brief confugure device
	* @param dev device's private data
	* @param cmd configuration command followed by it's arguments
	* @return -1 in case of error, 0 otherwise
	*/
	int (*configure)(void *dev, const char *cmd, ...);

	volatile enum DEVSTATUS status;		/*!< device status */

	void *priv;				/*!< device's private
						data */
};

/**
* @brief block device
*/
struct bdevice {
	char name[DEVNAMEMAX];	/*!< device name */

	/**
	* @brief read data from device
	* @param dev device's private data
	* @param addr address on device to read data from
	* @param data output buffer
	* @param sz output buffer size
	* @return -1 in case of error, 0 otherwise
	*/	
	int (*read)(void *priv, size_t addr, void *data, size_t sz);

	/**
	* @brief write data to device
	* @param dev device's private data
	* @param addr address on device to write data
	* @param data input buffer
	* @param sz input buffer size
	* @return -1 in case of error, 0 otherwise
	*/	
	int (*write)(void *priv, size_t addr, const void *data,
		size_t sz);

	/**
	* @brief process interrupt triggered by device
	* @param dev device's private data
	* @param interface that triggered interrupt
	* @return -1 in case of error, 0 otherwise
	*/
	int (*interrupt)(void *priv, const void *h);

	/**
	* @brief confugure device
	* @param dev device's private data
	* @param req configuration command followed by it's arguments
	* @return -1 in case of error, 0 otherwise
	*/
	int (*ioctl)(void *priv, int req, ...);

	/**
	* @brief erase all data on device
	* @param dev device's private data
	* @return -1 in case of error, 0 otherwise
	*/
	int (*eraseall)(void *priv);

	/**
	* @brief erase sector on device
	* @param dev device's private data
	* @param addr starting address of sector
	* @return -1 in case of error, 0 otherwise
	*/
	int (*erasesector)(void *priv, size_t addr);

	/**
	* @brief erase block on device
	* @param dev device's private data
	* @param addr starting address of block
	* @return -1 in case of error, 0 otherwise
	*/
	int (*eraseblock)(void *priv, size_t addr);


	/**
	* @brief write whole sector on device
	* @param dev device's private data
	* @param addr starting address of device's sector to write data
	* @param data input buffer
	* @param sz input buffer size
	* @return -1 in case of error, 0 otherwise
	*/	
	int (*writesector)(void *priv, size_t addr, const void *data,
		size_t sz);

	int status;		/*!< device status */

	size_t writesize;	/*!< write block size */
	size_t sectorsize;	/*!< sector size */
	size_t totalsize;	/*!< total size */

	void *priv;		/*!< device's private */
};

#endif
