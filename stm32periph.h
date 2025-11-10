/**
* @file stm32periph.h
* @brief STM32 periphery handling functions
*/

#include "stm32f4xx_hal.h"

/**
* @defgroup PERIPHCONTEXT
* @brief STM32 periphery contexts
* @{
*/
extern ADC_HandleTypeDef hadc1;			/*!< adc 1 */
extern ADC_HandleTypeDef hadc2;			/*!< adc 1 */
extern DMA_HandleTypeDef hdma_adc1;		/*!< adc 1 DMA */
extern I2C_HandleTypeDef hi2c1;			/*!< I2C */
extern DMA_HandleTypeDef hdma_i2c1_rx;		/*!< I2C RX DMA */
extern DMA_HandleTypeDef hdma_i2c1_tx;		/*!< I2C TX DMA */
extern SPI_HandleTypeDef hspi1;			/*!< SPI 1 */
extern SPI_HandleTypeDef hspi2;			/*!< SPI 2 */
extern TIM_HandleTypeDef htim1;			/*!< TIM 1 */
extern TIM_HandleTypeDef htim8;			/*!< TIM 8 */
extern TIM_HandleTypeDef htim10;		/*!< TIM 10 */
extern UART_HandleTypeDef huart1;		/*!< UART 1 */
extern UART_HandleTypeDef huart2;		/*!< UART 2 */
extern UART_HandleTypeDef huart3;		/*!< UART 3 */
extern UART_HandleTypeDef huart4;		/*!< UART 4 */
extern UART_HandleTypeDef huart5;		/*!< UART 5 */
extern DMA_HandleTypeDef hdma_usart1_rx;	/*!< UART 1 RX DMA */
extern DMA_HandleTypeDef hdma_usart2_rx;	/*!< UART 2 RX DMA */
extern DMA_HandleTypeDef hdma_usart2_tx;	/*!< UART 2 TX DMA */
extern DMA_HandleTypeDef hdma_usart3_rx;	/*!< UART 3 RX DMA */
extern DMA_HandleTypeDef hdma_uart4_rx;		/*!< UART 4 RX DMA */

extern DMA_HandleTypeDef hdma_tim1_ch1; /*!< TIM 1 CHANNEL 1 DMA */
extern DMA_HandleTypeDef hdma_tim1_ch3; /*!< TIM 1 CHANNEL 3 DMA */
extern DMA_HandleTypeDef hdma_tim1_ch2; /*!< TIM 1 CHANNEL 2 DMA */
extern DMA_HandleTypeDef hdma_tim1_ch4_trig_com; /*!< TIM 1 CHANNEL 4 DMA */
/**
* @}
*/

/**
* @brief STM32 initilization
* @return none
*/
void systemclock_config();

/**
* @brief STM32 periphery devices initilization
* @return none
*/
void periph_init();

/**
* @brief MCU hardware errors handler. Disarm
	immediately in case of any of such error.
* @return none
*/
void error_handler(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void assert_failed(uint8_t *file, uint32_t line);
