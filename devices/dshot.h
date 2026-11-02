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
	DSHOT_150BD,	/*!< bidirectional DShot-150 */
	DSHOT_300BD,	/*!< bidirectional DShot-300 */
	DSHOT_600BD,	/*!< bidirectional DShot-600 */
	DSHOT_1200BD	/*!< bidirectional DShot-1200 */
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
	int polescount;			/*!< motors poles count */

	int timcc[4];			/*!< private: DMA trigger event
						correspondign to each
						PWM channel */
	volatile uint32_t *timccr[4];	/*!< private: capture-compare
						register corresponding
						to each PWM channel*/
	DMA_HandleTypeDef *hdma[4];	/*!< private: DMA stream
						corresponding to each
						PWM channel*/
	int isrx[4];
	int bitlen;			/*!< private: length of bit
					in timer clock cycles */
	int zerolen;			/*!< private: length of zero
					in timer clock cycles */;
	int onelen;			/*!< private: length of one
					in timer clock cycles */
	int tele;			/*!< private: is telemetry 
					enabled */
};

/**
* @brief input data
*/
struct dshot_data {
	float thrust[4];	/*!< value of each PWM channel */
};

/**
* @brief output data
*/
struct dshot_tele {
	float rps[4];	/*!< RPM of each motor */
	int isvalid[4];	/*!< is rpm value valid */
};

/**
* @brief initialize DShot PWM device.
* @param is device initialization and private
	data structure with set non-private fields
* @param dev block device context to initialize
* @return -1 in case of error, 0 otherwise
*/
int dshot_initdevice(struct dshot_device *is, struct cdevice *dev);

#endif
