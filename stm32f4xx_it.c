#include "main.h"
#include "stm32f4xx_it.h"

extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void NMI_Handler(void)
{
	while (1) { }
}

void HardFault_Handler(void)
{
	while (1) { }
}

void MemManage_Handler(void)
{
	while (1) { }
}

void BusFault_Handler(void)
{
	while (1) { }
}

void UsageFault_Handler(void)
{
	while (1) { }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
	HAL_IncTick();
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim2);
}

void DMA2_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_adc1);
}

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart2);
}

