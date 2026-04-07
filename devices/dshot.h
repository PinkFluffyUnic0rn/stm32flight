/**
* @file dshot.h
* @brief dshot PWM device driver
*/

#ifndef DSHOT_H
#define DSHOT_H

#include "mcudef.h"

#include "device.h"

/**
* @brief maximum devices of this type
*/
#define DSHOT_MAXDEVS 1

/**
* @brief DShot bitrate
*/
enum DSHOT_TYPE {
	DSHOT_150,	/*!< DShot-150 */
	DSHOT_300,	/*!< DShot-300 */
	DSHOT_600,	/*!< DShot-600 */
	DSHOT_1200,	/*!< DShot-1200 */
};

/**
* @brief DShot DMA mode
*/
enum DSHOT_MODE {
	DSHOT_TIMBURST, /*!< TIM PWM BURST mode */
	DSHOT_DMACCR	/*!< separate DMA stream for each PWM channel */
};

/**
* @brief device initialization and private data
*/
struct dshot_device {
	TIM_HandleTypeDef *htim[4];	/*!< timer used for
					each PWM channel */
	uint32_t timch[4];		/*!< timer channel number
					of each PWM channel */
	int timfreq;			/*!< timer freequency */
	enum DSHOT_MODE mode;		/*!< DMA mode */
	enum DSHOT_TYPE type;		/*!< DShot bitrate */

	int timcc[4];			/*!< private: DMA trigger event
						correspondign to each
						PWM channel */
	volatile uint32_t *timccr[4];	/*!< private: capture-compare
						register corresponding
						to each PWM channel*/
	DMA_HandleTypeDef *hdma[4];	/*!< private: DMA stream
						corresponding to each
						PWM channel*/
	int bitlen;			/*!< private: length of bit
					in timer clock cycles */
	int zerolen;			/*!< private: length of zero
					in timer clock cycles */;
	int onelen;;			/*!< private: length of one
					in timer clock cycles */
};

/**
* @brief input data
*/
struct dshot_data {
	float thrust[4];	/*!< value of each PWM channel */
};

/**
* @brief initialize DShot PWM device.
*
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int dshot_initdevice(void *is, struct cdevice *dev);

#endif
