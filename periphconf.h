/**
* @file stm32periph.h
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

extern ADC_HandleTypeDef pconf_hadcs[5];
extern DMA_HandleTypeDef pconf_hdmas[16];
extern I2C_HandleTypeDef pconf_hi2cs[3];
extern SPI_HandleTypeDef pconf_hspis[3];
extern TIM_HandleTypeDef pconf_htims[5];
extern UART_HandleTypeDef pconf_huarts[5];

extern GPIO_TypeDef *armgpio;
extern int armpin;
extern GPIO_TypeDef *debuggpio;
extern int debugpin;
extern GPIO_TypeDef *errorgpio;
extern int errorpin;

extern TIM_HandleTypeDef *pconf_delayhtim;
extern TIM_HandleTypeDef *pconf_schedhtim;

extern struct cdevice Dev[DEV_COUNT];
extern struct bdevice Flashdev;

extern ADC_HandleTypeDef *pconf_batteryhadc;
extern ADC_HandleTypeDef *pconf_currenthadc;

void pconf_init();

#endif
