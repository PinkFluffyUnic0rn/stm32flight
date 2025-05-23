#include "stm32f4xx_hal.h"

// STM32 perithery contexts
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim10;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;

// STM32 clocks and periphery devices initilization
void systemclock_config();
void gpio_init();
void i2c_init();
void spi1_init();
void spi2_init();
void dma_init();
void tim1_init();
void tim8_init();
void tim10_init();
void adc1_init(void);
void usart1_init();
void usart2_init();
void usart3_init();
void uart4_init();

// MCU hardware errors handler. Disarm immediately
// in case of any of such error.
void error_handler(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

void assert_failed(uint8_t *file, uint32_t line);
