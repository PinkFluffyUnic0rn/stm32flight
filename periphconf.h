/**
* @file periphconf.h
* @brief STM32 periphery handling functions
*/

#ifndef PERIPHCONF_H
#define PERIPHCONF_H

#include "mcudef.h"

#include "device.h"
#include "icm42688.h"
#include "dps368.h"
#include "esp8266.h"
#include "qmc5883l.h"
#include "crsf.h"
#include "w25.h"
#include "m10.h"
#include "uartconf.h"
#include "irc.h"
#include "dshot.h"

/**
* @defgroup DEVIDS
* @brief Character devices IDs
* @{
*/
enum DEV_ID {
	DEV_IMU		= 0,	/*!< IMU device number */
	DEV_BARO	= 1,	/*!< barometer device number */
	DEV_MAG		= 2,	/*!< magnetometer device number */
	DEV_CRSF	= 3,	/*!< eLRS device number */
	DEV_GNSS	= 4,	/*!< GNSS device number */
	DEV_RF		= 5,	/*!< ESP device number */
	DEV_UART	= 6,	/*!< UART debug device number */
	DEV_IRC		= 7,	/*!< video TX device number */
	DEV_DSHOT	= 8,	/*!< DShot-300 device number */
	DEV_COUNT	= 9	/*!< character devices count */
};
/**
* @}
*/


/**
* @defgroup PERIPHHANDLERS
* @brief STM32 peripheral handlers
* @{
*/
extern ADC_HandleTypeDef pconf_hadcs[5];	/*!< adc handles */
extern DMA_HandleTypeDef pconf_hdmas[16];	/*!< dma handles */
extern I2C_HandleTypeDef pconf_hi2cs[3];	/*!< i2c handles */
extern SPI_HandleTypeDef pconf_hspis[3];	/*!< spi handles */
extern TIM_HandleTypeDef pconf_htims[5];	/*!< timer handles */
extern UART_HandleTypeDef pconf_huarts[5];	/*!< uart handles */
/**
* @}
*/

/**
* @defgroup BOARDDEVICES
* @brief board devices
* @{
*/
extern GPIO_TypeDef *armgpio;	/*!< arming indication led port */
extern int armpin;		/*!< arming indication led pin */
extern GPIO_TypeDef *debuggpio;	/*!< arming indication led port */
extern int debugpin;		/*!< arming indication led pin */
extern GPIO_TypeDef *errorgpio;	/*!< arming indication led port */
extern int errorpin;		/*!< arming indication led pin */

extern TIM_HandleTypeDef *pconf_delayhtim;	/*!< delay timer
						   handler pointer */
extern TIM_HandleTypeDef *pconf_schedhtim;	/*!< scheduler timer
						   handler pointer */

extern struct cdevice Dev[DEV_COUNT];	/*!< board character devices */
extern struct bdevice Flashdev;		/*!< board block devices */

extern ADC_HandleTypeDef *pconf_batteryhadc;	/*!< battery voltage
						measuring adc handle */
extern ADC_HandleTypeDef *pconf_currenthadc;	/*!< ESC current
						measuring adc handle */
/**
* @}
*/

/**
* @brief initialize STM32 peripherals and board devices
*/
void pconf_init();

#endif
