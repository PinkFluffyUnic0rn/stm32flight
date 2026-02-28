#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#include "settings.h"

#include "periphconf.h"
#include "util.h"

#define OCSFREQ 250000000

struct pconf_pin {
	GPIO_TypeDef *inst;
	uint16_t idx;
};

struct pconf_i2c {
	I2C_TypeDef *inst;
	
	enum PCONF_I2CUSAGE {
		PCONF_I2CUSAGE_DEVS,
	} usage;

	struct pconf_pin sda;
	struct pconf_pin scl;
	DMA_Stream_TypeDef *rxdma;
	DMA_Stream_TypeDef *txdma;
};

struct pconf_spi {
	SPI_TypeDef *inst;
	
	enum PCONF_SPIUSAGE {
		PCONF_SPIUSAGE_DEVS,
		PCONF_SPIUSAGE_WIFI
	} usage;

	struct pconf_pin miso;
	struct pconf_pin mosi;
	struct pconf_pin sck;
};

struct pconf_exti {
	struct pconf_pin pin;
};

struct pconf_tim {
	TIM_TypeDef *inst;
	
	enum PCONF_TIMUSAGE {
		PCONF_TIMUSAGE_PWM,
		PCONF_TIMUSAGE_SCHED,
		PCONF_TIMUSAGE_DELAY
	} usage;

	struct {
		int chan;
		struct pconf_pin pin;
		DMA_Stream_TypeDef *dma;
	} pwm[4];
	int chcnt;
};

struct pconf_adc {
	ADC_TypeDef *inst;

	enum PCONF_ADCUSAGE {
		PCONF_ADCUSAGE_MEASURE,
	} usage;

	struct pconf_pin pin;
	DMA_Stream_TypeDef *dma;
};

struct pconf_uart {
	USART_TypeDef *inst;

	enum PCONF_UARTUSAGE {
		PCONF_UARTUSAGE_CRSF,
		PCONF_UARTUSAGE_GNSS,
		PCONF_UARTUSAGE_DEBUG,
		PCONF_UARTUSAGE_IRC
	} usage;
	
	struct pconf_pin rx;
	struct pconf_pin tx;
	DMA_Stream_TypeDef *rxdma;
	DMA_Stream_TypeDef *txdma;
};

struct pconf_iface {
	enum PCONF_IFACETYPE {
		PCONF_IFACETYPE_I2C,
		PCONF_IFACETYPE_SPI,
		PCONF_IFACETYPE_UART
	} type;

	union {
		USART_TypeDef *huart;
		I2C_TypeDef *hi2c;
		struct {
			SPI_TypeDef *hspi;
			struct pconf_pin cs;
		};
	};
};

struct pconf_debug {
	struct pconf_iface iface;	
};

struct pconf_imu {
	enum PCONF_IMUTYPE {
		PCONF_IMUTYPE_ICM42688P,
		PCONF_IMUTYPE_MPU6500
	} type;

	struct pconf_iface iface;
};

struct pconf_bar {
	enum PCONF_BARTYPE {
		PCONF_BARTYPE_HP206C,
		PCONF_BARTYPE_BMP280,
		PCONF_BARTYPE_DPS368
	} type;

	struct pconf_iface iface;	
};

struct pconf_mag {
	enum PCONF_MAGTYPE {
		PCONF_MAGTYPE_QMC5883L,
		PCONF_MAGTYPE_HMC5883L
	} type;

	struct pconf_iface iface;	
};

struct pconf_flash {
	enum PCONF_FLASHTYPE {
		PCONF_FLASHTYPE_W25Q
	} type;

	struct pconf_iface iface;	
};

struct pconf_crsf {
	struct pconf_iface iface;	
};

struct pconf_gnss {
	struct pconf_iface iface;	
};

struct pconf_wireless {
	struct pconf_iface iface;
	struct pconf_pin interrupt;
	struct pconf_pin busy;
	struct pconf_pin boot;
	struct pconf_pin reset;
};

struct pconf_vtx {
	struct pconf_iface iface;	
};

struct pconf_pwm {
	struct {
		TIM_TypeDef *inst;
		int chan;
	} pwm[4];
};

struct pconf_battery {
	ADC_TypeDef *adc;
	struct pconf_pin pin;
};

struct pconf_current {
	ADC_TypeDef *adc;
	struct pconf_pin pin;
};

static void (*error_handler)(void);

#include "periphdef.h"

ADC_HandleTypeDef pconf_hadcs[5];
DMA_HandleTypeDef pconf_hdmas[16];
I2C_HandleTypeDef pconf_hi2cs[3];
SPI_HandleTypeDef pconf_hspis[3];
TIM_HandleTypeDef pconf_htims[5];
UART_HandleTypeDef pconf_huarts[5];

GPIO_TypeDef *armgpio;
int armpin;
GPIO_TypeDef *debuggpio;
int debugpin;

TIM_HandleTypeDef *pconf_delayhtim;
TIM_HandleTypeDef *pconf_schedhtim;

struct cdevice Dev[DEV_COUNT];
struct bdevice Flashdev;

ADC_HandleTypeDef *pconf_batteryhadc;
ADC_HandleTypeDef *pconf_currenthadc;

static int pconf_dma_stream_irqn(const DMA_Stream_TypeDef *dma)
{
	if (dma == DMA1_Stream0)	return DMA1_Stream0_IRQn;
	else if (dma == DMA1_Stream1)	return DMA1_Stream1_IRQn;
	else if (dma == DMA1_Stream2)	return DMA1_Stream2_IRQn;
	else if (dma == DMA1_Stream3)	return DMA1_Stream3_IRQn;
	else if (dma == DMA1_Stream4)	return DMA1_Stream4_IRQn;
	else if (dma == DMA1_Stream5)	return DMA1_Stream5_IRQn;
	else if (dma == DMA1_Stream6)	return DMA1_Stream6_IRQn;
	else if (dma == DMA1_Stream7)	return DMA1_Stream7_IRQn;
	else if (dma == DMA2_Stream0)	return DMA2_Stream0_IRQn;
	else if (dma == DMA2_Stream1)	return DMA2_Stream1_IRQn;
	else if (dma == DMA2_Stream2)	return DMA2_Stream2_IRQn;
	else if (dma == DMA2_Stream3)	return DMA2_Stream3_IRQn;
	else if (dma == DMA2_Stream4)	return DMA2_Stream4_IRQn;
	else if (dma == DMA2_Stream5)	return DMA2_Stream5_IRQn;
	else if (dma == DMA2_Stream6)	return DMA2_Stream6_IRQn;
	else if (dma == DMA2_Stream7)	return DMA2_Stream7_IRQn;
	
	return (-1);
}

static int pconf_exti_irqn(uint16_t pinidx)
{
	switch (pinidx) {
	case GPIO_PIN_0:	return EXTI0_IRQn;
	case GPIO_PIN_1:	return EXTI1_IRQn;
	case GPIO_PIN_2:	return EXTI2_IRQn;
	case GPIO_PIN_3:	return EXTI3_IRQn;
	case GPIO_PIN_4:	return EXTI4_IRQn;
	case GPIO_PIN_5:
	case GPIO_PIN_6:
	case GPIO_PIN_7:
	case GPIO_PIN_8:
	case GPIO_PIN_9:
				return EXTI9_5_IRQn;
	case GPIO_PIN_10:
	case GPIO_PIN_11:
	case GPIO_PIN_12:
	case GPIO_PIN_13:
	case GPIO_PIN_14:
	case GPIO_PIN_15:	return EXTI15_10_IRQn;
	default:		return (-1);
	}
}

static int pconf_uart_irqn(const USART_TypeDef *uart)
{
	if (uart == USART1)		return USART1_IRQn;
	else if (uart == USART2)	return USART2_IRQn;
	else if (uart == USART3)	return USART3_IRQn;
	else if (uart == UART4)		return UART4_IRQn;
	else if (uart == UART5)		return UART5_IRQn;
	else if (uart == USART6)	return USART6_IRQn;
	else if (uart == UART7)		return UART7_IRQn;
	else if (uart == UART8)		return UART8_IRQn;
	else if (uart == UART9)		return UART9_IRQn;
	else if (uart == USART10)	return USART10_IRQn;

	return (-1);
}

static int pconf_i2cev_irqn(const I2C_TypeDef *hi2c)
{
	if (hi2c == I2C1)		return I2C1_EV_IRQn;
	else if (hi2c == I2C2)		return I2C2_EV_IRQn;
	else if (hi2c == I2C3)		return I2C3_EV_IRQn;
	else if (hi2c == I2C4)		return I2C4_EV_IRQn;
	else if (hi2c == I2C5)		return I2C5_EV_IRQn;

	return (-1);
}

static int pconf_i2cer_irqn(const I2C_TypeDef *hi2c)
{
	if (hi2c == I2C1)		return I2C1_ER_IRQn;
	else if (hi2c == I2C2)		return I2C2_ER_IRQn;
	else if (hi2c == I2C3)		return I2C3_ER_IRQn;
	else if (hi2c == I2C4)		return I2C4_ER_IRQn;
	else if (hi2c == I2C5)		return I2C5_ER_IRQn;

	return (-1);
}

static int pconf_gpio_enable_clock(GPIO_TypeDef *inst)
{
	if (inst == GPIOA)		__HAL_RCC_GPIOA_CLK_ENABLE();
	else if (inst == GPIOB)		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if (inst == GPIOC)		__HAL_RCC_GPIOC_CLK_ENABLE();
	else if (inst == GPIOD)		__HAL_RCC_GPIOD_CLK_ENABLE();
	else if (inst == GPIOE)		__HAL_RCC_GPIOE_CLK_ENABLE();
	else if (inst == GPIOF)		__HAL_RCC_GPIOF_CLK_ENABLE();

	return 0;
}

static int pconf_gpio_disable_clock(GPIO_TypeDef *inst)
{
	if (inst == GPIOA)		__HAL_RCC_GPIOA_CLK_DISABLE();
	else if (inst == GPIOB)		__HAL_RCC_GPIOB_CLK_DISABLE();
	else if (inst == GPIOC)		__HAL_RCC_GPIOC_CLK_DISABLE();
	else if (inst == GPIOD)		__HAL_RCC_GPIOD_CLK_DISABLE();
	else if (inst == GPIOE)		__HAL_RCC_GPIOE_CLK_DISABLE();
	else if (inst == GPIOF)		__HAL_RCC_GPIOF_CLK_DISABLE();

	return 0;
}

static int pconf_uart_enable_clock(USART_TypeDef *inst)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
	
	if (inst == USART1)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	else if (inst == USART2)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	else if (inst == USART3)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
	else if (inst == UART4)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
	else if (inst == UART5)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
	else if (inst == USART6)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
	else if (inst == UART7)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
	else if (inst == UART8)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART8;
	else if (inst == UART9)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART9;
	else if (inst == USART10)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART10;

	if (inst == USART2 || inst == USART3 || inst == UART4 
			|| inst == UART5 || inst == UART7
			|| inst == UART8) {
		PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
	}
	else
		PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		error_handler();


	if (inst == USART1)		__HAL_RCC_USART1_CLK_ENABLE();
	else if (inst == USART2)	__HAL_RCC_USART2_CLK_ENABLE();
	else if (inst == USART3)	__HAL_RCC_USART3_CLK_ENABLE();
	else if (inst == UART4)		__HAL_RCC_UART4_CLK_ENABLE();
	else if (inst == UART5)		__HAL_RCC_UART5_CLK_ENABLE();
	else if (inst == USART6)	__HAL_RCC_USART6_CLK_ENABLE();
	else if (inst == UART7)		__HAL_RCC_UART7_CLK_ENABLE();
	else if (inst == UART8)		__HAL_RCC_UART8_CLK_ENABLE();
	else if (inst == UART9)		__HAL_RCC_UART9_CLK_ENABLE();
	else if (inst == USART10)	__HAL_RCC_USART10_CLK_ENABLE();

	return 0;
}

static int pconf_uart_disable_clock(USART_TypeDef *inst)
{
	if (inst == USART1)		__HAL_RCC_USART1_CLK_DISABLE();
	else if (inst == USART2)	__HAL_RCC_USART2_CLK_DISABLE();
	else if (inst == USART3)	__HAL_RCC_USART3_CLK_DISABLE();
	else if (inst == UART4)		__HAL_RCC_UART4_CLK_DISABLE();
	else if (inst == UART5)		__HAL_RCC_UART5_CLK_DISABLE();
	else if (inst == USART6)	__HAL_RCC_USART6_CLK_DISABLE();
	else if (inst == UART7)		__HAL_RCC_UART7_CLK_DISABLE();
	else if (inst == UART8)		__HAL_RCC_UART8_CLK_DISABLE();
	else if (inst == UART9)		__HAL_RCC_UART9_CLK_DISABLE();
	else if (inst == USART10)	__HAL_RCC_USART10_CLK_DISABLE();

	return 0;
}

static int pconf_adc_enable_clock(ADC_TypeDef *inst)
{
	if (inst == ADC1)		__HAL_RCC_ADC12_CLK_ENABLE();
	else if (inst == ADC2)		__HAL_RCC_ADC12_CLK_ENABLE();
	else if (inst == ADC3)		__HAL_RCC_ADC3_CLK_ENABLE();

	return 0;
}

static int pconf_adc_disable_clock(ADC_TypeDef *inst)
{
	if (inst == ADC1)		__HAL_RCC_ADC12_CLK_DISABLE();
	else if (inst == ADC2)		__HAL_RCC_ADC12_CLK_DISABLE();
	else if (inst == ADC3)		__HAL_RCC_ADC3_CLK_DISABLE();

	return 0;
}

static int pconf_i2c_enable_clock(I2C_TypeDef *inst)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	if (inst == I2C1)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
	else if (inst == I2C2)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	else if (inst == I2C3)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
	else if (inst == I2C4)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C4;
	else if (inst == I2C5)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2C5;

	PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C1235CLKSOURCE_D2PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		error_handler();

	if (inst == I2C1)		__HAL_RCC_I2C1_CLK_ENABLE();
	else if (inst == I2C2)		__HAL_RCC_I2C2_CLK_ENABLE();
	else if (inst == I2C3)		__HAL_RCC_I2C3_CLK_ENABLE();
	else if (inst == I2C4)		__HAL_RCC_I2C4_CLK_ENABLE();
	else if (inst == I2C5)		__HAL_RCC_I2C5_CLK_ENABLE();

	return 0;
}

static int pconf_i2c_disable_clock(I2C_TypeDef *inst)
{
	if (inst == I2C1)		__HAL_RCC_I2C1_CLK_DISABLE();
	else if (inst == I2C2)		__HAL_RCC_I2C2_CLK_DISABLE();
	else if (inst == I2C3)		__HAL_RCC_I2C3_CLK_DISABLE();
	else if (inst == I2C4)		__HAL_RCC_I2C4_CLK_DISABLE();
	else if (inst == I2C5)		__HAL_RCC_I2C5_CLK_DISABLE();

	return 0;
}

static int pconf_spi_enable_clock(SPI_TypeDef *inst)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	if (inst == SPI1)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI1;
	else if (inst == SPI2)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI2;
	else if (inst == SPI3)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3;
	else if (inst == SPI4)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI4;
	else if (inst == SPI5)
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI5;

	if (inst == SPI1 || inst == SPI2 || inst == SPI3)
		PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_CLKP;
	else if (inst == SPI4 || inst == SPI5)
		PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK1;
	else
		PeriphClkInitStruct.Spi6ClockSelection = RCC_SPI6CLKSOURCE_D3PCLK1;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		error_handler();

	if (inst == SPI1)		__HAL_RCC_SPI1_CLK_ENABLE();
	else if (inst == SPI2)		__HAL_RCC_SPI2_CLK_ENABLE();
	else if (inst == SPI3)		__HAL_RCC_SPI3_CLK_ENABLE();
	else if (inst == SPI4)		__HAL_RCC_SPI4_CLK_ENABLE();
	else if (inst == SPI5)		__HAL_RCC_SPI5_CLK_ENABLE();
	else if (inst == SPI6)		__HAL_RCC_SPI6_CLK_ENABLE();

	return 0;
}

static int pconf_spi_disable_clock(SPI_TypeDef *inst)
{
	if (inst == SPI1)		__HAL_RCC_SPI1_CLK_DISABLE();
	else if (inst == SPI2)		__HAL_RCC_SPI2_CLK_DISABLE();
	else if (inst == SPI3)		__HAL_RCC_SPI3_CLK_DISABLE();
	else if (inst == SPI4)		__HAL_RCC_SPI4_CLK_DISABLE();
	else if (inst == SPI5)		__HAL_RCC_SPI5_CLK_DISABLE();
	else if (inst == SPI6)		__HAL_RCC_SPI6_CLK_DISABLE();

	return 0;
}

static int pconf_tim_enable_clock(TIM_TypeDef *inst)
{
	if (inst == TIM1)		__HAL_RCC_TIM1_CLK_ENABLE();
	else if (inst == TIM2)		__HAL_RCC_TIM2_CLK_ENABLE();
	else if (inst == TIM3)		__HAL_RCC_TIM3_CLK_ENABLE();
	else if (inst == TIM4)		__HAL_RCC_TIM4_CLK_ENABLE();
	else if (inst == TIM5)		__HAL_RCC_TIM5_CLK_ENABLE();
	else if (inst == TIM5)		__HAL_RCC_TIM6_CLK_ENABLE();
	else if (inst == TIM6)		__HAL_RCC_TIM6_CLK_ENABLE();
	else if (inst == TIM7)		__HAL_RCC_TIM7_CLK_ENABLE();
	else if (inst == TIM8)		__HAL_RCC_TIM8_CLK_ENABLE();
	else if (inst == TIM12)		__HAL_RCC_TIM12_CLK_ENABLE();
	else if (inst == TIM13)		__HAL_RCC_TIM13_CLK_ENABLE();
	else if (inst == TIM14)		__HAL_RCC_TIM14_CLK_ENABLE();
	else if (inst == TIM15)		__HAL_RCC_TIM15_CLK_ENABLE();
	else if (inst == TIM16)		__HAL_RCC_TIM16_CLK_ENABLE();
	else if (inst == TIM17)		__HAL_RCC_TIM17_CLK_ENABLE();
	else if (inst == TIM23)		__HAL_RCC_TIM23_CLK_ENABLE();
	else if (inst == TIM24)		__HAL_RCC_TIM24_CLK_ENABLE();

	return 0;
}

static int pconf_tim_disable_clock(TIM_TypeDef *inst)
{
	if (inst == TIM1)		__HAL_RCC_TIM1_CLK_DISABLE();
	else if (inst == TIM2)		__HAL_RCC_TIM2_CLK_DISABLE();
	else if (inst == TIM3)		__HAL_RCC_TIM3_CLK_DISABLE();
	else if (inst == TIM4)		__HAL_RCC_TIM4_CLK_DISABLE();
	else if (inst == TIM5)		__HAL_RCC_TIM5_CLK_DISABLE();
	else if (inst == TIM6)		__HAL_RCC_TIM6_CLK_DISABLE();
	else if (inst == TIM7)		__HAL_RCC_TIM7_CLK_DISABLE();
	else if (inst == TIM8)		__HAL_RCC_TIM8_CLK_DISABLE();
	else if (inst == TIM12)		__HAL_RCC_TIM12_CLK_DISABLE();
	else if (inst == TIM13)		__HAL_RCC_TIM13_CLK_DISABLE();
	else if (inst == TIM14)		__HAL_RCC_TIM14_CLK_DISABLE();
	else if (inst == TIM15)		__HAL_RCC_TIM15_CLK_DISABLE();
	else if (inst == TIM16)		__HAL_RCC_TIM16_CLK_DISABLE();
	else if (inst == TIM17)		__HAL_RCC_TIM17_CLK_DISABLE();
	else if (inst == TIM23)		__HAL_RCC_TIM23_CLK_DISABLE();
	else if (inst == TIM24)		__HAL_RCC_TIM24_CLK_DISABLE();

	return 0;
}

static int pconf_uart_pinalternate(const struct pconf_pin *pin,
	USART_TypeDef *inst)
{
	if (inst == USART1) {
		if (pin->inst == GPIOB) {
			if (pin->idx == 14 || pin->idx == 15)
				return GPIO_AF4_USART1;
			else
				return GPIO_AF7_USART1;
		}
		else
			return GPIO_AF7_USART1;
	}
	else if (inst == USART2)	return GPIO_AF7_USART2;
	else if (inst == USART3)	return GPIO_AF7_USART3;
	else if (inst == UART4) {
		if (pin->inst == GPIOA) {
			if (pin->idx == 11 || pin->idx == 12)
				return GPIO_AF6_UART4;
			else
				return GPIO_AF8_UART4;
		}
		else
			return GPIO_AF8_UART4;
	}
	else if (inst == UART5) {
		if (pin->inst == GPIOB) {
			if (pin->idx == 5 || pin->idx == 6
					|| pin->idx == 12
					|| pin->idx == 13) {
				return GPIO_AF14_UART5;
			}
			else
				return GPIO_AF8_UART5;
		}
		else
			return GPIO_AF8_UART5;
	}
	else if (inst == USART6)	return GPIO_AF7_USART6;
	else if (inst == UART7) {
		if (pin->inst == GPIOA || pin->inst == GPIOB)
			return GPIO_AF11_UART7;
		else
			return GPIO_AF7_UART7;
	}
	else if (inst == UART8)		return GPIO_AF8_UART8;
	else if (inst == UART9)		return GPIO_AF11_UART9;
	else if (inst == USART10)	return GPIO_AF11_USART10;

	return (-1);
}

static int pconf_i2c_pinalternate(const struct pconf_pin *pin,
	I2C_TypeDef *inst)
{
	if (inst == I2C1)		return GPIO_AF4_I2C1;
	else if (inst == I2C2)		return GPIO_AF4_I2C2;
	else if (inst == I2C3)		return GPIO_AF4_I2C3;
	else if (inst == I2C4) {
		if (pin->inst == GPIOB)
			return GPIO_AF6_I2C4;
		else if (pin->inst == GPIOD && pin->inst == GPIOF)
			return GPIO_AF4_I2C4;

	}
	else if (inst == I2C5) {
		if (pin->inst == GPIOA)
			return GPIO_AF6_I2C5;
		else if (pin->inst == GPIOC)
			return GPIO_AF4_I2C5;
		else if (pin->inst == GPIOC) {
			if (pin->idx == 9)	return GPIO_AF6_I2C5;
			else			return GPIO_AF6_I2C5;
		}
		else if (pin->inst == GPIOF)
			return GPIO_AF6_I2C5;
	}

	return (-1);
}

static int pconf_spi_pinalternate(const struct pconf_pin *pin,
	SPI_TypeDef *inst)
{
	if (inst == SPI1)		return GPIO_AF5_SPI1;
	else if (inst == SPI2) {
		if (pin->inst == GPIOB && pin->idx == 4)
			return GPIO_AF7_SPI2;
		else
			return GPIO_AF5_SPI2;
	}
	else if (inst == SPI3) {
		if (pin->inst == GPIOC || pin->inst == GPIOA)
			return GPIO_AF6_SPI3;
		else if (pin->inst == GPIOB) {
			if (pin->idx == 2 || pin->idx == 5)
				return GPIO_AF7_SPI3;
			else
				return GPIO_AF6_SPI3;
		}
		else if (pin->inst == GPIOD)
			return GPIO_AF5_SPI3;	
	}
	else if (inst == SPI4)
		return GPIO_AF5_SPI4;
	else if (inst == SPI5)
		return GPIO_AF5_SPI5;
	else if (inst == SPI6) {
		if (pin->inst == GPIOA) {
			if (pin->idx == 0)
				return GPIO_AF5_SPI6;
			if (pin->idx == 15)
				return GPIO_AF7_SPI6;
			else
				return GPIO_AF8_SPI6;
		}
		else if (pin->inst == GPIOB)
			return GPIO_AF8_SPI6;
		else if (pin->inst == GPIOC)
			return GPIO_AF5_SPI6;
	}

	return (-1);
}

static int pconf_tim_pinalternate(const struct pconf_pin *pin,
	TIM_TypeDef *inst)
{
	if (inst == TIM1)		return GPIO_AF1_TIM1;
	else if (inst == TIM2)		return GPIO_AF1_TIM2;
	else if (inst == TIM3)		return GPIO_AF2_TIM3;
	else if (inst == TIM4)		return GPIO_AF2_TIM4;
	else if (inst == TIM5)		return GPIO_AF2_TIM5;
	else if (inst == TIM8)		return GPIO_AF3_TIM8;
	else if (inst == TIM12)		return GPIO_AF2_TIM12;
	else if (inst == TIM13)		return GPIO_AF9_TIM13;
	else if (inst == TIM14)		return GPIO_AF9_TIM14;
	else if (inst == TIM15) {
		if (pin->inst == GPIOC)
			return GPIO_AF2_TIM15;
		else
			return GPIO_AF4_TIM15;
	}
	else if (inst == TIM16)		return GPIO_AF1_TIM16;
	else if (inst == TIM17)		return GPIO_AF1_TIM17;
	else if (inst == TIM23)		return GPIO_AF13_TIM23;
	else if (inst == TIM24)		return GPIO_AF14_TIM24;

	return (-1);
}

static int pconf_dmastream_uartrx_channel(DMA_Stream_TypeDef *inst,
	USART_TypeDef *uinst)
{
	if (uinst == USART1)		return DMA_REQUEST_USART1_RX;
	else if (uinst == USART2)	return DMA_REQUEST_USART2_RX;
	else if (uinst == USART3)	return DMA_REQUEST_USART3_RX;
	else if (uinst == UART4)	return DMA_REQUEST_UART4_RX;
	else if (uinst == UART5)	return DMA_REQUEST_UART5_RX;

	return (-1);
}

static int pconf_dmastream_uarttx_channel(DMA_Stream_TypeDef *inst,
	USART_TypeDef *uinst)
{
	if (uinst == USART1)		return DMA_REQUEST_USART1_TX;
	else if (uinst == USART2)	return DMA_REQUEST_USART2_TX;
	else if (uinst == USART3)	return DMA_REQUEST_USART3_TX;
	else if (uinst == UART4)	return DMA_REQUEST_UART4_TX;
	else if (uinst == UART5)	return DMA_REQUEST_UART5_TX;

	return (-1);
}

static int pconf_dmastream_adc_channel(DMA_Stream_TypeDef *inst,
	ADC_TypeDef *adcinst)
{
	if (adcinst == ADC1)		return DMA_REQUEST_ADC1;
	else if (adcinst == ADC2)	return DMA_REQUEST_ADC2;
	else if (adcinst == ADC3)	return DMA_REQUEST_ADC3;

	return (-1);
}

static int pconf_dmastream_i2crx_channel(DMA_Stream_TypeDef *inst,
	I2C_TypeDef *i2cinst)
{
	if (i2cinst == I2C1)		return DMA_REQUEST_I2C1_RX;
	else if (i2cinst == I2C2)	return DMA_REQUEST_I2C2_RX;
	else if (i2cinst == I2C3)	return DMA_REQUEST_I2C3_RX;
	
	return (-1);
}

static int pconf_dmastream_i2ctx_channel(DMA_Stream_TypeDef *inst,
	I2C_TypeDef *i2cinst)
{
	if (i2cinst == I2C1)		return DMA_REQUEST_I2C1_TX;
	else if (i2cinst == I2C2)	return DMA_REQUEST_I2C2_TX;
	else if (i2cinst == I2C3)	return DMA_REQUEST_I2C3_TX;

	return (-1);
}

static int pconf_dmastream_pwm_channel(DMA_Stream_TypeDef *inst,
	TIM_TypeDef *timinst, int ch)
{
	if (timinst == TIM1 && ch == 1)		return DMA_REQUEST_TIM1_CH1;
	else if (timinst == TIM1 && ch == 2)	return DMA_REQUEST_TIM1_CH2;
	else if (timinst == TIM1 && ch == 3)	return DMA_REQUEST_TIM1_CH3;
	else if (timinst == TIM1 && ch == 4)	return DMA_REQUEST_TIM1_CH4;
	else if (timinst == TIM2 && ch == 1)	return DMA_REQUEST_TIM2_CH1;
	else if (timinst == TIM2 && ch == 2)	return DMA_REQUEST_TIM2_CH2;
	else if (timinst == TIM2 && ch == 3)	return DMA_REQUEST_TIM2_CH3;
	else if (timinst == TIM2 && ch == 4)	return DMA_REQUEST_TIM2_CH4;
	else if (timinst == TIM3 && ch == 1)	return DMA_REQUEST_TIM3_CH1;
	else if (timinst == TIM3 && ch == 2)	return DMA_REQUEST_TIM3_CH2;
	else if (timinst == TIM3 && ch == 3)	return DMA_REQUEST_TIM3_CH3;
	else if (timinst == TIM3 && ch == 4)	return DMA_REQUEST_TIM3_CH4;
	else if (timinst == TIM4 && ch == 1)	return DMA_REQUEST_TIM4_CH1;
	else if (timinst == TIM4 && ch == 2)	return DMA_REQUEST_TIM4_CH2;
	else if (timinst == TIM4 && ch == 3)	return DMA_REQUEST_TIM4_CH3;
	else if (timinst == TIM5 && ch == 1)	return DMA_REQUEST_TIM5_CH1;
	else if (timinst == TIM5 && ch == 2)	return DMA_REQUEST_TIM5_CH2;
	else if (timinst == TIM5 && ch == 3)	return DMA_REQUEST_TIM5_CH3;
	else if (timinst == TIM5 && ch == 4)	return DMA_REQUEST_TIM5_CH4;
	else if (timinst == TIM8 && ch == 1)	return DMA_REQUEST_TIM8_CH1;
	else if (timinst == TIM8 && ch == 2)	return DMA_REQUEST_TIM8_CH2;
	else if (timinst == TIM8 && ch == 3)	return DMA_REQUEST_TIM8_CH3;
	else if (timinst == TIM8 && ch == 4)	return DMA_REQUEST_TIM8_CH4;
	else if (timinst == TIM15 && ch == 1)	return DMA_REQUEST_TIM15_CH1;
	else if (timinst == TIM23 && ch == 1)	return DMA_REQUEST_TIM23_CH1;
	else if (timinst == TIM23 && ch == 2)	return DMA_REQUEST_TIM23_CH2;
	else if (timinst == TIM23 && ch == 3)	return DMA_REQUEST_TIM23_CH3;
	else if (timinst == TIM23 && ch == 4)	return DMA_REQUEST_TIM23_CH4;
	else if (timinst == TIM24 && ch == 1)	return DMA_REQUEST_TIM24_CH1;
	else if (timinst == TIM24 && ch == 2)	return DMA_REQUEST_TIM24_CH2;
	else if (timinst == TIM24 && ch == 3)	return DMA_REQUEST_TIM24_CH3;
	else if (timinst == TIM24 && ch == 4)	return DMA_REQUEST_TIM24_CH4;
	
	return (-1);
}

int pconf_timpwm_chan(int ch)
{
	switch (ch) {
	case 1:		return TIM_CHANNEL_1;
	case 2:		return TIM_CHANNEL_2;
	case 3:		return TIM_CHANNEL_3;
	case 4:		return TIM_CHANNEL_4;
	default:	return (-1);
	}

}

const int *pconf_timpwm_dmaids(TIM_TypeDef *inst, int ch,
	DMA_Stream_TypeDef *dmainst)
{
	static int dmaids[4];

	dmaids[0] = -1;

	if (ch == 1) {
		dmaids[0] = TIM_DMA_ID_CC1;
		dmaids[1] = -1;
	}
	else if (ch == 2) {
		dmaids[0] = TIM_DMA_ID_CC2;
		dmaids[1] = -1;
	}
	else if (ch == 3) {
		dmaids[0] = TIM_DMA_ID_CC3;
		dmaids[1] = -1;
	}
	else if (ch == 4) {
		dmaids[0] = TIM_DMA_ID_CC4;
		dmaids[1] = -1;
	}

	return dmaids;
}

static int pconf_uartidx(USART_TypeDef* inst)
{
	int i;

	for (i = 0; i < PCONF_UARTSCOUNT; ++i) {
		if (uarts[i].inst == inst)
			return i;
	}

	return (-1);
}

static int pconf_adcidx(ADC_TypeDef *inst)
{
	int i;

	for (i = 0; i < PCONF_ADCSCOUNT; ++i) {
		if (adcs[i].inst == inst)
			return i;
	}

	return (-1);
}

static int pconf_i2cidx(I2C_TypeDef *inst)
{
	int i;

	for (i = 0; i < PCONF_I2CSCOUNT; ++i) {
		if (i2cs[i].inst == inst)
			return i;
	}

	return (-1);
}

static int pconf_spiidx(SPI_TypeDef *inst)
{
	int i;

	for (i = 0; i < PCONF_SPISCOUNT; ++i) {
		if (spis[i].inst == inst)
			return i;
	}

	return (-1);
}

static int pconf_timidx(TIM_TypeDef *inst)
{
	int i;

	for (i = 0; i < PCONF_TIMSCOUNT; ++i) {
		if (tims[i].inst == inst)
			return i;
	}

	return (-1);
}

static int pconf_dmaidx(DMA_Stream_TypeDef *inst)
{
	int i;
	
	if (inst == NULL)
		return (-1);

	for (i = 0; i < PCONF_DMASCOUNT; ++i) {
		if (dmas[i] == inst)
			return i;
	}

	return (-1);
}

void pconf_mspinit_uart(UART_HandleTypeDef* huart)
{
	const struct pconf_uart *uart;
	int idx;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if ((idx = pconf_uartidx(huart->Instance)) < 0)
		return;

	uart = uarts + idx;

	pconf_uart_enable_clock(huart->Instance);

	pconf_gpio_enable_clock(uart->rx.inst);
	pconf_gpio_enable_clock(uart->tx.inst);

	if (uart->usage == PCONF_UARTUSAGE_IRC) {
		GPIO_InitStruct.Pin = uart->tx.idx;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = pconf_uart_pinalternate(
			&(uart->tx), uart->inst);
		HAL_GPIO_Init(uart->tx.inst, &GPIO_InitStruct);
	
		return;
	}

	GPIO_InitStruct.Pin = uart->rx.idx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = pconf_uart_pinalternate(&(uart->rx),
		uart->inst);
	HAL_GPIO_Init(uart->rx.inst, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = uart->tx.idx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = pconf_uart_pinalternate(&(uart->tx),
		uart->inst);
	HAL_GPIO_Init(uart->tx.inst, &GPIO_InitStruct);

	if ((idx = pconf_dmaidx(uart->rxdma)) >= 0) {
		pconf_hdmas[idx].Instance = dmas[idx];
		pconf_hdmas[idx].Init.Request
			= pconf_dmastream_uartrx_channel(
				uart->rxdma, uart->inst);
		pconf_hdmas[idx].Init.Direction = DMA_PERIPH_TO_MEMORY;
		pconf_hdmas[idx].Init.PeriphInc = DMA_PINC_DISABLE;
		pconf_hdmas[idx].Init.MemInc = DMA_MINC_ENABLE;
		pconf_hdmas[idx].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		pconf_hdmas[idx].Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		pconf_hdmas[idx].Init.Mode = DMA_CIRCULAR;
		pconf_hdmas[idx].Init.Priority = DMA_PRIORITY_LOW;
		pconf_hdmas[idx].Init.FIFOMode = DMA_FIFOMODE_DISABLE;

		if (HAL_DMA_Init(pconf_hdmas + idx) != HAL_OK)
			error_handler();

		__HAL_LINKDMA(huart, hdmarx, pconf_hdmas[idx]);

		HAL_NVIC_SetPriority(pconf_uart_irqn(uart->inst), 0, 0);
		HAL_NVIC_EnableIRQ(pconf_uart_irqn(uart->inst));
	}
	
	if ((idx = pconf_dmaidx(uart->txdma)) >= 0) {
		pconf_hdmas[idx].Instance = dmas[idx];
		pconf_hdmas[idx].Init.Request
			= pconf_dmastream_uarttx_channel(
				uart->txdma, uart->inst);
		pconf_hdmas[idx].Init.Direction = DMA_MEMORY_TO_PERIPH;
		pconf_hdmas[idx].Init.PeriphInc = DMA_PINC_DISABLE;
		pconf_hdmas[idx].Init.MemInc = DMA_MINC_ENABLE;
		pconf_hdmas[idx].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		pconf_hdmas[idx].Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		pconf_hdmas[idx].Init.Mode = DMA_NORMAL;
		pconf_hdmas[idx].Init.Priority = DMA_PRIORITY_LOW;
		pconf_hdmas[idx].Init.FIFOMode = DMA_FIFOMODE_DISABLE;

		if (HAL_DMA_Init(pconf_hdmas + idx) != HAL_OK)
			error_handler();

		__HAL_LINKDMA(huart, hdmatx, pconf_hdmas[idx]);
	
		HAL_NVIC_SetPriority(pconf_uart_irqn(uart->inst), 0, 0);
		HAL_NVIC_EnableIRQ(pconf_uart_irqn(uart->inst));
	}
}

void pconf_mspdeinit_uart(UART_HandleTypeDef* huart)
{
	const struct pconf_uart *uart;
	int idx;

	if ((idx = pconf_uartidx(huart->Instance)) < 0)
		return;

	uart = uarts + idx;

	pconf_uart_disable_clock(huart->Instance);

	pconf_gpio_disable_clock(uart->rx.inst);
	pconf_gpio_disable_clock(uart->tx.inst);

	HAL_GPIO_DeInit(uart->rx.inst, uart->rx.idx);
	HAL_GPIO_DeInit(uart->tx.inst, uart->tx.idx);

	if (uart->rxdma != NULL)
		HAL_DMA_DeInit(huart->hdmarx);

	if (uart->txdma != NULL)
		HAL_DMA_DeInit(huart->hdmatx);
}

void pconf_mspinit_adc(ADC_HandleTypeDef* hadc)
{
	const struct pconf_adc *adc;
	int idx;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if ((idx = pconf_adcidx(hadc->Instance)) < 0)
		return;

	adc = adcs + idx;

	pconf_adc_enable_clock(hadc->Instance);
	pconf_gpio_enable_clock(adc->pin.inst);

	GPIO_InitStruct.Pin = adc->pin.idx;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(adc->pin.inst, &GPIO_InitStruct);
	
	if ((idx = pconf_dmaidx(adc->dma)) >= 0) {
		pconf_hdmas[idx].Instance = dmas[idx];
		pconf_hdmas[idx].Init.Request
			= pconf_dmastream_adc_channel(
				adc->dma, adc->inst);
		pconf_hdmas[idx].Init.Direction = DMA_PERIPH_TO_MEMORY;
		pconf_hdmas[idx].Init.PeriphInc = DMA_PINC_DISABLE;
		pconf_hdmas[idx].Init.MemInc = DMA_MINC_ENABLE;
		pconf_hdmas[idx].Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		pconf_hdmas[idx].Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		pconf_hdmas[idx].Init.Mode = DMA_NORMAL;
		pconf_hdmas[idx].Init.Priority = DMA_PRIORITY_LOW;
		pconf_hdmas[idx].Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	
		if (HAL_DMA_Init(pconf_hdmas + idx) != HAL_OK)
			error_handler();

		__HAL_LINKDMA(hadc, DMA_Handle, pconf_hdmas[idx]);
	}
}

void pconf_mspdeinit_adc(ADC_HandleTypeDef* hadc)
{
	const struct pconf_adc *adc;
	int idx;

	if ((idx = pconf_adcidx(hadc->Instance)) < 0)
		return;

	adc = adcs + idx;

	pconf_adc_disable_clock(hadc->Instance);

	HAL_GPIO_DeInit(adc->pin.inst, adc->pin.idx);

	HAL_DMA_DeInit(hadc->DMA_Handle);
}

void pconf_mspinit_i2c(I2C_HandleTypeDef* hi2c)
{
	const struct pconf_i2c *i2c;
	int idx;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if ((idx = pconf_i2cidx(hi2c->Instance)) < 0)
		return;

	i2c = i2cs + idx;

	pconf_gpio_enable_clock(i2c->sda.inst);
	pconf_gpio_enable_clock(i2c->scl.inst);

	GPIO_InitStruct.Pin = i2c->sda.idx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = pconf_i2c_pinalternate(&(i2c->sda),
		i2c->inst);
	HAL_GPIO_Init(i2c->sda.inst, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = i2c->scl.idx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = pconf_i2c_pinalternate(&(i2c->scl),
		i2c->inst);
	HAL_GPIO_Init(i2c->scl.inst, &GPIO_InitStruct);
	
	pconf_i2c_enable_clock(hi2c->Instance);

	if ((idx = pconf_dmaidx(i2c->rxdma)) >= 0) {
		pconf_hdmas[idx].Instance = dmas[idx];
		pconf_hdmas[idx].Init.Request
			= pconf_dmastream_i2crx_channel(
				i2c->rxdma, i2c->inst);
		pconf_hdmas[idx].Init.Direction = DMA_PERIPH_TO_MEMORY;
		pconf_hdmas[idx].Init.PeriphInc = DMA_PINC_DISABLE;
		pconf_hdmas[idx].Init.MemInc = DMA_MINC_ENABLE;
		pconf_hdmas[idx].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		pconf_hdmas[idx].Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		pconf_hdmas[idx].Init.Mode = DMA_NORMAL;
		pconf_hdmas[idx].Init.Priority = DMA_PRIORITY_LOW;
		pconf_hdmas[idx].Init.FIFOMode = DMA_FIFOMODE_DISABLE;

		if (HAL_DMA_Init(pconf_hdmas + idx) != HAL_OK)
			error_handler();

		__HAL_LINKDMA(hi2c, hdmarx, pconf_hdmas[idx]);
	}
	
	if ((idx = pconf_dmaidx(i2c->txdma)) >= 0) {
		pconf_hdmas[idx].Instance = dmas[idx];
		pconf_hdmas[idx].Init.Request
			= pconf_dmastream_i2ctx_channel(
				i2c->txdma, i2c->inst);
		pconf_hdmas[idx].Init.Direction = DMA_MEMORY_TO_PERIPH;
		pconf_hdmas[idx].Init.PeriphInc = DMA_PINC_DISABLE;
		pconf_hdmas[idx].Init.MemInc = DMA_MINC_ENABLE;
		pconf_hdmas[idx].Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		pconf_hdmas[idx].Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		pconf_hdmas[idx].Init.Mode = DMA_NORMAL;
		pconf_hdmas[idx].Init.Priority = DMA_PRIORITY_LOW;
		pconf_hdmas[idx].Init.FIFOMode = DMA_FIFOMODE_DISABLE;

		if (HAL_DMA_Init(pconf_hdmas + idx) != HAL_OK)
			error_handler();

		__HAL_LINKDMA(hi2c, hdmatx, pconf_hdmas[idx]);
	}
	if ((idx = pconf_dmaidx(i2c->rxdma)) >= 0
			|| (idx = pconf_dmaidx(i2c->txdma)) >= 0) {
		HAL_NVIC_SetPriority(pconf_i2cev_irqn(i2c->inst), 0, 0);
		HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

		HAL_NVIC_SetPriority(pconf_i2cer_irqn(i2c->inst), 0, 0);
		HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
	}
}

void pconf_mspdeinit_i2c(I2C_HandleTypeDef* hi2c)
{
	const struct pconf_i2c *i2c;
	int idx;

	if ((idx = pconf_i2cidx(hi2c->Instance)) < 0)
		return;

	i2c = i2cs + idx;

	pconf_i2c_disable_clock(hi2c->Instance);

	pconf_gpio_disable_clock(i2c->sda.inst);
	pconf_gpio_disable_clock(i2c->scl.inst);

	HAL_GPIO_DeInit(i2c->sda.inst, i2c->sda.idx);
	HAL_GPIO_DeInit(i2c->scl.inst, i2c->scl.idx);

	if (i2c->rxdma != NULL)
		HAL_DMA_DeInit(hi2c->hdmarx);

	if (i2c->txdma != NULL)
		HAL_DMA_DeInit(hi2c->hdmatx);
}

void pconf_mspinit_spi(SPI_HandleTypeDef* hspi)
{
	const struct pconf_spi *spi;
	int idx;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if ((idx = pconf_spiidx(hspi->Instance)) < 0)
		return;

	spi = spis + idx;

	pconf_spi_enable_clock(hspi->Instance);

	pconf_gpio_enable_clock(spi->miso.inst);
	pconf_gpio_enable_clock(spi->mosi.inst);
	pconf_gpio_enable_clock(spi->sck.inst);

	GPIO_InitStruct.Pin = spi->miso.idx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = pconf_spi_pinalternate(&(spi->miso), spi->inst);
	HAL_GPIO_Init(spi->miso.inst, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = spi->mosi.idx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = pconf_spi_pinalternate(&(spi->mosi), spi->inst);
	HAL_GPIO_Init(spi->mosi.inst, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = spi->sck.idx;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = pconf_spi_pinalternate(&(spi->sck), spi->inst);
	HAL_GPIO_Init(spi->sck.inst, &GPIO_InitStruct);
}

void pconf_mspdeinit_spi(SPI_HandleTypeDef* hspi)
{
	const struct pconf_spi *spi;
	int idx;

	if ((idx = pconf_spiidx(hspi->Instance)) < 0)
		return;

	spi = spis + idx;

	pconf_spi_disable_clock(hspi->Instance);

	pconf_gpio_disable_clock(spi->miso.inst);
	pconf_gpio_disable_clock(spi->mosi.inst);
	pconf_gpio_disable_clock(spi->sck.inst);
}

void pconf_mspinit_tim(TIM_HandleTypeDef *htim)
{
	pconf_tim_enable_clock(htim->Instance);
}

void pconf_mspinit_timpwm(TIM_HandleTypeDef *htim)
{
	const struct pconf_tim *tim;
	int idx;
	int i;

	if ((idx = pconf_timidx(htim->Instance)) < 0)
		return;

	tim = tims + idx;
	
	pconf_tim_enable_clock(htim->Instance);
	
	for (i = 0; i < tim->chcnt; ++i) {
		const int *dmaidx;
		int chanidx;

		if ((idx = pconf_dmaidx(tim->pwm[i].dma)) < 0)
			continue;

		chanidx = tim->pwm[i].chan;

		pconf_hdmas[idx].Instance = dmas[idx];
		pconf_hdmas[idx].Init.Request
			= pconf_dmastream_pwm_channel(
				tim->pwm[i].dma, tim->inst, chanidx);
		pconf_hdmas[idx].Init.Direction = DMA_MEMORY_TO_PERIPH;
		pconf_hdmas[idx].Init.PeriphInc = DMA_PINC_DISABLE;
		pconf_hdmas[idx].Init.MemInc = DMA_MINC_ENABLE;
		pconf_hdmas[idx].Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		pconf_hdmas[idx].Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		pconf_hdmas[idx].Init.Mode = DMA_NORMAL;
		pconf_hdmas[idx].Init.Priority = DMA_PRIORITY_HIGH;
		pconf_hdmas[idx].Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		pconf_hdmas[idx].Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		pconf_hdmas[idx].Init.MemBurst = DMA_MBURST_SINGLE;
		pconf_hdmas[idx].Init.PeriphBurst = DMA_PBURST_SINGLE;

		if (HAL_DMA_Init(pconf_hdmas + idx) != HAL_OK)
			error_handler();

		dmaidx = pconf_timpwm_dmaids(tim->inst,
			chanidx, dmas[idx]);

		for ( ; *dmaidx != -1; ++dmaidx)
			__HAL_LINKDMA(htim, hdma[*dmaidx], pconf_hdmas[idx]);
	}
}

void pconf_mspdeinit_timpwm(TIM_HandleTypeDef *htim)
{
	const struct pconf_tim *tim;
	int idx;
	int i;

	if ((idx = pconf_timidx(htim->Instance)) < 0)
		return;

	tim = tims + idx;
	
	pconf_tim_disable_clock(htim->Instance);

	for (i = 0; i < tim->chcnt; ++i) {
		const int *dmaidx;
		int chanidx;

		chanidx = tim->pwm[i].chan;

		dmaidx = pconf_timpwm_dmaids(tim->inst, chanidx, dmas[i]);

		for ( ; *dmaidx != -1; ++dmaidx)
			HAL_DMA_DeInit(htim->hdma[*dmaidx]);
	}
}

void tim_mspdeinit(TIM_HandleTypeDef *htim)
{
	pconf_tim_disable_clock(htim->Instance);
}
 
void pconf_init_clock(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

//	HAL_PWREx_ConfigSupply(PWR_EXTERNAL_SOURCE_SUPPLY);
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = HSE_VALUE / 5000000;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		error_handler();

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			      |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			      |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
		error_handler();

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_CKPER;
	PeriphClkInitStruct.PLL2.PLL2M = 2;
	PeriphClkInitStruct.PLL2.PLL2N = 19;
	PeriphClkInitStruct.PLL2.PLL2P = 2;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 1639;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
		error_handler();
}

static void pconf_init_mpu(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct = {0};

	HAL_MPU_Disable();

	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);

	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

static void pconf_init_gpio(void)
{
	int i;

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	for (i = 0; i < PCONF_OUTPINSCOUNT; ++i) {
		HAL_GPIO_WritePin(outpins[i].inst, outpins[i].idx,
			GPIO_PIN_RESET);
	}
	
	for (i = 0; i < PCONF_INPINSCOUNT; ++i) {
		HAL_GPIO_WritePin(inpins[i].inst, inpins[i].idx,
			GPIO_PIN_RESET);
	}

	for (i = 0; i < PCONF_OUTPINSCOUNT; ++i) {
		GPIO_InitStruct.Pin = outpins[i].idx;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		
		HAL_GPIO_Init(outpins[i].inst, &GPIO_InitStruct);
	}

	for (i = 0; i < PCONF_INPINSCOUNT; ++i) {
		GPIO_InitStruct.Pin = inpins[i].idx;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	
		HAL_GPIO_Init(inpins[i].inst, &GPIO_InitStruct);
	}

	for (i = 0; i < PCONF_EXTISCOUNT; ++i) {
		GPIO_InitStruct.Pin = extis[i].pin.idx;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(extis[i].pin.inst, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(pconf_exti_irqn(extis[i].pin.idx),
			0, 0);
		HAL_NVIC_EnableIRQ(pconf_exti_irqn(extis[i].pin.idx));
	}

}

static void pconf_init_dma(void)
{
	int i;
	
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	for (i = 0; i < PCONF_DMASCOUNT; ++i) {
		int irqn;
		int prep;

		if (dmas[i] == NULL)
			continue;

		irqn = pconf_dma_stream_irqn(dmas[i]);
		
		switch (irqn) {
		case DMA1_Stream0_IRQn:		prep = 1;
		case DMA1_Stream1_IRQn:		prep = 1;
		case DMA1_Stream2_IRQn:		prep = 1;
		case DMA1_Stream5_IRQn:		prep = 1;
		case DMA1_Stream6_IRQn:		prep = 1;
		case DMA1_Stream7_IRQn:		prep = 1;
		case DMA2_Stream0_IRQn:		prep = 1;
		case DMA2_Stream1_IRQn:		prep = 0;
		case DMA2_Stream2_IRQn:		prep = 0;
		case DMA2_Stream3_IRQn:		prep = 1;
		case DMA2_Stream4_IRQn:		prep = 0;
		case DMA2_Stream5_IRQn:		prep = 1;
		case DMA2_Stream6_IRQn:		prep = 0;
		}

		HAL_NVIC_SetPriority(irqn, prep, 0);
		HAL_NVIC_EnableIRQ(pconf_dma_stream_irqn(dmas[i]));
	}
}

static void pconf_init_timpwm(int idx)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	const struct pconf_tim *tim;
	int i;

	tim = tims + idx;

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	pconf_htims[idx].Instance = tims->inst;
	pconf_htims[idx].Init.Prescaler = 0;
	pconf_htims[idx].Init.CounterMode = TIM_COUNTERMODE_UP;
	pconf_htims[idx].Init.Period = 0;
	pconf_htims[idx].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pconf_htims[idx].Init.RepetitionCounter = 0;
	pconf_htims[idx].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(pconf_htims + idx) != HAL_OK)
		error_handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(pconf_htims + idx,
			&sMasterConfig) != HAL_OK)
		error_handler();

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	for (i = 0; i < tims[idx].chcnt; ++i) {
		if (HAL_TIM_PWM_ConfigChannel(pconf_htims + idx,
				&sConfigOC,
				pconf_timpwm_chan(tims[idx].pwm[i].chan))
					!= HAL_OK)
			error_handler();
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(pconf_htims + idx,
			&sBreakDeadTimeConfig) != HAL_OK)
		error_handler();

	for (i = 0; i < tim->chcnt; ++i) {
		pconf_gpio_enable_clock(tim->pwm[i].pin.inst);

		GPIO_InitStruct.Pin = tim->pwm[i].pin.idx;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = pconf_tim_pinalternate(
			&(tim->pwm[i].pin), tim->inst);
		HAL_GPIO_Init(tim->pwm[i].pin.inst, &GPIO_InitStruct);
	}
}

static void pconf_init_tim_sched(int idx)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	pconf_htims[idx].Instance = tims[idx].inst;
	pconf_htims[idx].Init.Prescaler = (OCSFREQ / TICKSPERSEC) - 1;
	pconf_htims[idx].Init.CounterMode = TIM_COUNTERMODE_UP;
	pconf_htims[idx].Init.Period = 0xffff;
	pconf_htims[idx].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pconf_htims[idx].Init.RepetitionCounter = 0;
	pconf_htims[idx].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(pconf_htims + idx) != HAL_OK)
		error_handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	if (HAL_TIM_ConfigClockSource(pconf_htims + idx,
			&sClockSourceConfig) != HAL_OK)
		error_handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(pconf_htims + idx,
			&sMasterConfig) != HAL_OK)
		error_handler();

	HAL_TIM_Base_Start_IT(pconf_htims + idx);
}

static void pconf_init_tim_delay(int idx)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	pconf_htims[idx].Instance = tims[idx].inst;
	pconf_htims[idx].Init.Prescaler = (OCSFREQ / TICKSPERSEC) - 1;
	pconf_htims[idx].Init.CounterMode = TIM_COUNTERMODE_UP;
	pconf_htims[idx].Init.Period = 0xffff;
	pconf_htims[idx].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	pconf_htims[idx].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(pconf_htims + idx) != HAL_OK)
		error_handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(pconf_htims + idx,
			&sClockSourceConfig) != HAL_OK)
		error_handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(pconf_htims + idx,
			&sMasterConfig) != HAL_OK)
		error_handler();

	HAL_TIM_Base_Start_IT(pconf_htims + idx);
}

static void pconf_init_tim()
{
	int i;

	for (i = 0; i < PCONF_TIMSCOUNT; ++i) {
		if (tims[i].usage == PCONF_TIMUSAGE_PWM)
			pconf_init_timpwm(i);
		else if (tims[i].usage == PCONF_TIMUSAGE_SCHED)
			pconf_init_tim_sched(i);
		else if (tims[i].usage == PCONF_TIMUSAGE_DELAY)
			pconf_init_tim_delay(i);
	}
}

static void pconf_init_i2c()
{
	int i;

	for (i = 0; i < PCONF_I2CSCOUNT; ++i) {
		pconf_hi2cs[i].Instance = i2cs[i].inst;
		pconf_hi2cs[i].Init.Timing = 0x00C042E4;
		pconf_hi2cs[i].Init.OwnAddress1 = 0;
		pconf_hi2cs[i].Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		pconf_hi2cs[i].Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
		pconf_hi2cs[i].Init.OwnAddress2 = 0;
		pconf_hi2cs[i].Init.OwnAddress2Masks = I2C_OA2_NOMASK;
		pconf_hi2cs[i].Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
		pconf_hi2cs[i].Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

		if (HAL_I2C_Init(pconf_hi2cs + i) != HAL_OK)
			error_handler();
	
		if (HAL_I2CEx_ConfigAnalogFilter(pconf_hi2cs + i,
				I2C_ANALOGFILTER_ENABLE) != HAL_OK)
			error_handler();

		if (HAL_I2CEx_ConfigDigitalFilter(pconf_hi2cs + i,
				0) != HAL_OK)
			error_handler();
	}
}

static void pconf_init_spi_dev(int i)
{
	pconf_hspis[i].Instance = spis[i].inst;
	pconf_hspis[i].Init.Mode = SPI_MODE_MASTER;
	pconf_hspis[i].Init.Direction = SPI_DIRECTION_2LINES;
	pconf_hspis[i].Init.DataSize = SPI_DATASIZE_8BIT;
	pconf_hspis[i].Init.CLKPolarity = SPI_POLARITY_LOW;
	pconf_hspis[i].Init.CLKPhase = SPI_PHASE_1EDGE;
	pconf_hspis[i].Init.NSS = SPI_NSS_SOFT;
	pconf_hspis[i].Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	pconf_hspis[i].Init.FirstBit = SPI_FIRSTBIT_MSB;
	pconf_hspis[i].Init.TIMode = SPI_TIMODE_DISABLE;
	pconf_hspis[i].Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	pconf_hspis[i].Init.CRCPolynomial = 10;
	pconf_hspis[i].Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	pconf_hspis[i].Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	pconf_hspis[i].Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	pconf_hspis[i].Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	pconf_hspis[i].Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	pconf_hspis[i].Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	pconf_hspis[i].Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	pconf_hspis[i].Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	pconf_hspis[i].Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	pconf_hspis[i].Init.IOSwap = SPI_IO_SWAP_DISABLE;

	if (HAL_SPI_Init(pconf_hspis + i) != HAL_OK)
		error_handler();
}

static void pconf_init_spi_wifi(int i)
{
	pconf_hspis[i].Instance = spis[i].inst;
	pconf_hspis[i].Init.Mode = SPI_MODE_MASTER;
	pconf_hspis[i].Init.Direction = SPI_DIRECTION_2LINES;
	pconf_hspis[i].Init.DataSize = SPI_DATASIZE_8BIT;
	pconf_hspis[i].Init.CLKPolarity = SPI_POLARITY_LOW;
	pconf_hspis[i].Init.CLKPhase = SPI_PHASE_1EDGE;
	pconf_hspis[i].Init.NSS = SPI_NSS_SOFT;
	pconf_hspis[i].Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
	pconf_hspis[i].Init.FirstBit = SPI_FIRSTBIT_MSB;
	pconf_hspis[i].Init.TIMode = SPI_TIMODE_DISABLE;
	pconf_hspis[i].Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	pconf_hspis[i].Init.CRCPolynomial = 10;
	pconf_hspis[i].Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	pconf_hspis[i].Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	pconf_hspis[i].Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	pconf_hspis[i].Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	pconf_hspis[i].Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	pconf_hspis[i].Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	pconf_hspis[i].Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	pconf_hspis[i].Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	pconf_hspis[i].Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	pconf_hspis[i].Init.IOSwap = SPI_IO_SWAP_DISABLE;

	if (HAL_SPI_Init(pconf_hspis + i) != HAL_OK)
		error_handler();
}

static void pconf_init_spi(void)
{
	int i;

	for (i = 0; i < PCONF_SPISCOUNT; ++i) {
		if (spis[i].usage == PCONF_SPIUSAGE_DEVS)
			pconf_init_spi_dev(i);
		else if (spis[i].usage == PCONF_SPIUSAGE_WIFI)
			pconf_init_spi_wifi(i);
	}
}

static void pconf_init_adc(void)
{
	int i;

	for (i = 0; i < PCONF_ADCSCOUNT; ++i) {
		ADC_ChannelConfTypeDef sConfig = {0};
	
		pconf_hadcs[i].Instance = ADC1;
		pconf_hadcs[i].Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
		pconf_hadcs[i].Init.Resolution = ADC_RESOLUTION_16B;
		pconf_hadcs[i].Init.ScanConvMode = ADC_SCAN_DISABLE;
		pconf_hadcs[i].Init.EOCSelection = ADC_EOC_SINGLE_CONV;
		pconf_hadcs[i].Init.LowPowerAutoWait = DISABLE;
		pconf_hadcs[i].Init.ContinuousConvMode = DISABLE;
		pconf_hadcs[i].Init.NbrOfConversion = 1;
		pconf_hadcs[i].Init.DiscontinuousConvMode = DISABLE;
		pconf_hadcs[i].Init.ExternalTrigConv = ADC_SOFTWARE_START;
		pconf_hadcs[i].Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
		pconf_hadcs[i].Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
		pconf_hadcs[i].Init.Overrun = ADC_OVR_DATA_PRESERVED;
		pconf_hadcs[i].Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
		pconf_hadcs[i].Init.OversamplingMode = DISABLE;
		pconf_hadcs[i].Init.Oversampling.Ratio = 1;
		
		if (HAL_ADC_Init(pconf_hadcs + i) != HAL_OK)
			error_handler();

		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;
		sConfig.OffsetSignedSaturation = DISABLE;

		if (HAL_ADC_ConfigChannel(pconf_hadcs + i, &sConfig) != HAL_OK)
			error_handler();
	}
}

static void pconf_init_uart_crsf(int i)
{
	pconf_huarts[i].Instance = uarts[i].inst;
	pconf_huarts[i].Init.BaudRate = 921600;
	pconf_huarts[i].Init.WordLength = UART_WORDLENGTH_8B;
	pconf_huarts[i].Init.StopBits = UART_STOPBITS_1;
	pconf_huarts[i].Init.Parity = UART_PARITY_NONE;
	pconf_huarts[i].Init.Mode = UART_MODE_TX_RX;
	pconf_huarts[i].Init.HwFlowCtl = UART_HWCONTROL_NONE;
	pconf_huarts[i].Init.OverSampling = UART_OVERSAMPLING_16;
	pconf_huarts[i].Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	pconf_huarts[i].Init.ClockPrescaler = UART_PRESCALER_DIV1;
	pconf_huarts[i].AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(pconf_huarts + i) != HAL_OK)
		error_handler();

	if (HAL_UART_Init(pconf_huarts + i) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_SetTxFifoThreshold(pconf_huarts + i, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_SetRxFifoThreshold(pconf_huarts + i, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_DisableFifoMode(pconf_huarts + i) != HAL_OK)
		error_handler();
}

static void pconf_init_uart_gnss(int i)
{
	pconf_huarts[i].Instance = uarts[i].inst;
	pconf_huarts[i].Init.BaudRate = 115200;
	pconf_huarts[i].Init.WordLength = UART_WORDLENGTH_8B;
	pconf_huarts[i].Init.StopBits = UART_STOPBITS_1;
	pconf_huarts[i].Init.Parity = UART_PARITY_NONE;
	pconf_huarts[i].Init.Mode = UART_MODE_TX_RX;
	pconf_huarts[i].Init.HwFlowCtl = UART_HWCONTROL_NONE;
	pconf_huarts[i].Init.OverSampling = UART_OVERSAMPLING_16;
	pconf_huarts[i].Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	pconf_huarts[i].Init.ClockPrescaler = UART_PRESCALER_DIV1;
	pconf_huarts[i].AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(pconf_huarts + i) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_SetTxFifoThreshold(pconf_huarts + i, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_SetRxFifoThreshold(pconf_huarts + i, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_DisableFifoMode(pconf_huarts + i) != HAL_OK)
		error_handler();
}

static void pconf_init_uart_debug(int i)
{
	pconf_huarts[i].Instance = uarts[i].inst;
	pconf_huarts[i].Init.BaudRate = 921600;
	pconf_huarts[i].Init.WordLength = UART_WORDLENGTH_8B;
	pconf_huarts[i].Init.StopBits = UART_STOPBITS_1;
	pconf_huarts[i].Init.Parity = UART_PARITY_NONE;
	pconf_huarts[i].Init.Mode = UART_MODE_TX_RX;
	pconf_huarts[i].Init.HwFlowCtl = UART_HWCONTROL_NONE;
	pconf_huarts[i].Init.OverSampling = UART_OVERSAMPLING_16;
	pconf_huarts[i].Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	pconf_huarts[i].Init.ClockPrescaler = UART_PRESCALER_DIV1;
	pconf_huarts[i].AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(pconf_huarts + i) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_SetTxFifoThreshold(pconf_huarts + i, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_SetRxFifoThreshold(pconf_huarts + i, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
		error_handler();

	if (HAL_UARTEx_DisableFifoMode(pconf_huarts + i) != HAL_OK)
		error_handler();
}

static void pconf_init_uart_irc(int i)
{
	pconf_huarts[i].Instance = uarts[i].inst;
	pconf_huarts[i].Init.BaudRate = 9600;
	pconf_huarts[i].Init.WordLength = UART_WORDLENGTH_8B;
	pconf_huarts[i].Init.StopBits = UART_STOPBITS_1;
	pconf_huarts[i].Init.Parity = UART_PARITY_NONE;
	pconf_huarts[i].Init.Mode = UART_MODE_TX_RX;
	pconf_huarts[i].Init.HwFlowCtl = UART_HWCONTROL_NONE;
	pconf_huarts[i].Init.OverSampling = UART_OVERSAMPLING_16;
	pconf_huarts[i].Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	pconf_huarts[i].Init.ClockPrescaler = UART_PRESCALER_DIV1;
	pconf_huarts[i].AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_HalfDuplex_Init(pconf_huarts + i) != HAL_OK)
		error_handler();
}

static void pconf_init_uart()
{
	int i;

	for (i = 0; i < PCONF_UARTSCOUNT; ++i) {
		if (uarts[i].usage == PCONF_UARTUSAGE_CRSF)
			pconf_init_uart_crsf(i);
		else if (uarts[i].usage == PCONF_UARTUSAGE_GNSS)
			pconf_init_uart_gnss(i);
		else if (uarts[i].usage == PCONF_UARTUSAGE_DEBUG)
			pconf_init_uart_debug(i);
		else if (uarts[i].usage == PCONF_UARTUSAGE_IRC)
			pconf_init_uart_irc(i);
	
	}
}

static void uartdev_init()
{
	struct uart_device d;

	if (debugconf.iface.type != PCONF_IFACETYPE_UART)
		goto error;

	d.huart = pconf_huarts + pconf_uartidx(debugconf.iface.huart);

	if (uart_initdevice(&d, Dev + DEV_UART) < 0)
		goto error;

	uartprintf("%s initilized\r\n", Dev[DEV_UART].name);

	return;

error:
	uartprintf("failed to initilize UART device\r\n");
}

static int icm_init()
{
	struct icm_device d;

	if (imuconf.iface.type != PCONF_IFACETYPE_SPI)
		return (-1);

	d.hspi = pconf_hspis + pconf_spiidx(imuconf.iface.hspi);
	d.gpio = imuconf.iface.cs.inst;
	d.pin = imuconf.iface.cs.idx;

	d.gyroscale = ICM_1000DPS;
	d.gyrorate = ICM_GYRO8K;
	d.gyroorder = ICM_GYROORDER3;
	d.gyrolpf = ICM_GYROLPFLL;
	d.accelscale = ICM_4G;
	d.accelrate = ICM_ACCEL8K;
	d.accellpf = ICM_ACCELLPFLL;
	d.accelorder = ICM_ACCELORDER3;

	return icm_initdevice(&d, Dev + DEV_IMU);
}

static void imu_init()
{
	if (imuconf.type == PCONF_IMUTYPE_ICM42688P) {
		if (icm_init() < 0)
			goto error;
	}
		
	uartprintf("%s initialized\r\n", Dev[DEV_IMU].name);

	return;

error:
	uartprintf("failed to initialize IMU\r\n");
}

static int dps_init()
{
	struct dps_device d;

	if (barconf.iface.type != PCONF_IFACETYPE_I2C)
		return (-1);

	d.hi2c = pconf_hi2cs + pconf_i2cidx(barconf.iface.hi2c);
	d.rate = DPS_RATE_32;
	d.osr = DPS_OSR_16;

	return dps_initdevice(&d, Dev + DEV_BARO);
}

static void baro_init()
{
	if (barconf.type == PCONF_BARTYPE_DPS368) {
		if (dps_init() < 0)
			goto error;
	}
		
	uartprintf("%s initialized\r\n", Dev[DEV_BARO].name);

	return;

error:
	uartprintf("failed to initialize barometer\r\n");
}

static int qmc_init()
{
	struct qmc_device d;

	if (magconf.iface.type != PCONF_IFACETYPE_I2C)
		return (-1);

	d.hi2c = pconf_hi2cs + pconf_i2cidx(magconf.iface.hi2c);
	d.scale = QMC_SCALE_8;
	d.rate = QMC_RATE_100;
	d.osr = QMC_OSR_512;

	return qmc_initdevice(&d, Dev + DEV_MAG);
}

static void mag_init()
{
	if (magconf.type == PCONF_MAGTYPE_QMC5883L) {
		if (qmc_init() < 0)
			goto error;
	}
		
	uartprintf("%s initialized\r\n", Dev[DEV_MAG].name);

	return;

error:
	uartprintf("failed to initialize magnetometer\r\n");
}

static int w25dev_init()
{
	struct w25_device d;

	if (flashconf.iface.type != PCONF_IFACETYPE_SPI)
		return (-1);

	d.hspi = pconf_hspis + pconf_spiidx(flashconf.iface.hspi);
	d.gpio = flashconf.iface.cs.inst;
	d.pin = flashconf.iface.cs.idx;

	return w25_initdevice(&d, &Flashdev);
}

static void flash_init()
{
	if (flashconf.type == PCONF_FLASHTYPE_W25Q) {
		if (w25dev_init() < 0)
			goto error;
	}

	uartprintf("%s initialized\r\n", Flashdev.name);

	return;

error:
	uartprintf("failed to initialize flash device\r\n");
}

static void crsfdev_init()
{
	struct crsf_device d;

	if (crsfconf.iface.type != PCONF_IFACETYPE_UART)
		goto error;

	d.huart = pconf_huarts + pconf_uartidx(crsfconf.iface.huart);

	if (crsf_initdevice(&d, Dev + DEV_CRSF) < 0)
		goto error;		
		
	uartprintf("%s initialized\r\n", Dev[DEV_CRSF].name);

	return;

error:
	uartprintf("failed to initialize CRSF device\r\n");
}

static void m10dev_init()
{
	struct m10_device d;

	if (gnssconf.iface.type != PCONF_IFACETYPE_UART)
		goto error;

	d.huart = pconf_huarts + pconf_uartidx(gnssconf.iface.huart);

	if (m10_initdevice(&d, Dev + DEV_GNSS) < 0)
		goto error;

	uartprintf("%s initialized\r\n", Dev[DEV_GNSS].name);

	return;

error:
	uartprintf("failed to initialize GNSS device\r\n");
}

static void espdev_init()
{
	struct esp_device d;

	if (rfconf.iface.type != PCONF_IFACETYPE_SPI)
		goto error;

	d.hspi = pconf_hspis + pconf_spiidx(rfconf.iface.hspi);
	d.csgpio = rfconf.iface.cs.inst;
	d.cspin = rfconf.iface.cs.idx;
	d.rstgpio = rfconf.reset.inst;
	d.rstpin = rfconf.reset.idx;
	d.bootgpio = rfconf.boot.inst;
	d.bootpin = rfconf.boot.idx;
	d.busygpio = rfconf.busy.inst;
	d.busypin = rfconf.busy.idx;
	d.intpin = rfconf.interrupt.idx;

	if (esp_initdevice(&d, Dev + DEV_RF) < 0)
		goto error;

	uartprintf("%s initialized\r\n", Dev[DEV_RF].name);

	return;

error:
	uartprintf("failed to initialize ESP8266\r\n");
}

static void irc_init()
{
	struct irc_device d;

	if (vtxconf.iface.type != PCONF_IFACETYPE_UART)
		goto error;

	d.huart = pconf_huarts + pconf_uartidx(vtxconf.iface.huart);
	d.power = St.irc.power;
	d.frequency = St.irc.freq;

	if (irc_initdevice(&d, Dev + DEV_IRC) < 0)
		goto error;

	uartprintf("%s initilized\r\n", Dev[DEV_IRC].name);
	
	return;

error:
	uartprintf("failed to initilize IRC device\r\n");
}

static void dshot_init()
{
	struct dshot_device d;

	d.htim[0] = pconf_htims + pconf_timidx(pwmconf.pwm[0].inst);
	d.htim[1] = pconf_htims + pconf_timidx(pwmconf.pwm[1].inst);
	d.htim[2] = pconf_htims + pconf_timidx(pwmconf.pwm[2].inst);
	d.htim[3] = pconf_htims + pconf_timidx(pwmconf.pwm[3].inst);

	d.timch[0] = pconf_timpwm_chan(pwmconf.pwm[0].chan);
	d.timch[1] = pconf_timpwm_chan(pwmconf.pwm[1].chan);
	d.timch[2] = pconf_timpwm_chan(pwmconf.pwm[2].chan);
	d.timch[3] = pconf_timpwm_chan(pwmconf.pwm[3].chan);

	if (dshot_initdevice(&d, Dev + DEV_DSHOT) < 0)
		goto error;

	uartprintf("%s initilized\r\n", Dev[DEV_DSHOT].name);
	
	return;

error:
	uartprintf("failed to initilize DShot300\r\n");
}

void pconf_init(void (*errhandler)(void))
{
	int i;

	pconf_init_mpu();

	HAL_Init();

	error_handler = errhandler;

	pconf_init_clock();

	HAL_Delay(1000);

	pconf_init_gpio();
	pconf_init_dma();
	pconf_init_tim();
	pconf_init_uart();
	pconf_init_i2c();
	pconf_init_spi();
	pconf_init_adc();

	for (i = 0; i < DEV_COUNT; ++i)
		Dev[i].status = DEVSTATUS_NOINIT;

	armgpio = armpinconf.inst;
	armpin = armpinconf.idx;

	debuggpio = debugpinconf.inst;
	debugpin = debugpinconf.idx;

	pconf_delayhtim = pconf_htims + pconf_timidx(delayconf);
	pconf_schedhtim = pconf_htims + pconf_timidx(schedconf);
		
	pconf_batteryhadc = pconf_hadcs + pconf_adcidx(batconf.adc);
	pconf_currenthadc = pconf_hadcs + pconf_adcidx(curconf.adc);
	
	uartdev_init();
	imu_init();
	baro_init();
	mag_init();
	flash_init();
 	crsfdev_init();
	m10dev_init();
	espdev_init();
	irc_init();
	dshot_init();
}

void assert_failed(uint8_t *file, uint32_t line)
{

}

__attribute__((weak)) void _close(void) {}
__attribute__((weak)) void _lseek(void) {}
__attribute__((weak)) void _read(void) {}
__attribute__((weak)) void _write(void) {}
__attribute__((weak)) void _fstat(void) {}
__attribute__((weak)) void _isatty(void) {}
__attribute__((weak)) void _getpid(void) {}
__attribute__((weak)) void _kill(void) {}

void HAL_MspInit(void)
{
	__HAL_RCC_SYSCFG_CLK_ENABLE();
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	pconf_mspinit_uart(huart);
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	pconf_mspdeinit_uart(huart);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
	pconf_mspinit_adc(hadc);
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
	pconf_mspdeinit_adc(hadc);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	pconf_mspinit_i2c(hi2c);
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
	pconf_mspdeinit_i2c(hi2c);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	pconf_mspinit_spi(hspi);
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
	pconf_mspdeinit_spi(hspi);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	pconf_mspinit_tim(htim);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	pconf_mspinit_timpwm(htim);
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim)
{
	pconf_mspdeinit_timpwm(htim);
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

#ifdef PCONF_UART2_IDX_IRQ
void USART2_IRQHandler(void)
{
	HAL_UART_IRQHandler(pconf_huarts + PCONF_UART2_IDX_IRQ);
}
#endif

#ifdef PCONF_UART3_IDX_IRQ
void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(pconf_huarts + PCONF_UART3_IDX_IRQ);
}
#endif

#ifdef PCONF_UART4_IDX_IRQ
void UART4_IRQHandler(void)
{
	HAL_UART_IRQHandler(pconf_huarts + PCONF_UART4_IDX_IRQ);
}
#endif

#ifdef PCONF_I2C1_IDX_IRQ
void I2C1_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(pconf_hi2cs + PCONF_I2C1_IDX_IRQ);
}

void I2C1_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(pconf_hi2cs + PCONF_I2C1_IDX_IRQ);
}
#endif

#ifdef PCONF_DMA1_STREAM0_IRQ
void DMA1_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 0);
}
#endif

#ifdef PCONF_DMA1_STREAM1_IRQ
void DMA1_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 1);
}
#endif

#ifdef PCONF_DMA1_STREAM2_IRQ
void DMA1_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 2);
}
#endif

#ifdef PCONF_DMA1_STREAM3_IRQ
void DMA1_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 3);
}
#endif

#ifdef PCONF_DMA1_STREAM4_IRQ
void DMA1_Stream4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 4);
}
#endif

#ifdef PCONF_DMA1_STREAM5_IRQ
void DMA1_Stream5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 5);
}
#endif

#ifdef PCONF_DMA1_STREAM6_IRQ
void DMA1_Stream6_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 6);
}
#endif

#ifdef PCONF_DMA1_STREAM7_IRQ
void DMA1_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 7);
}
#endif

#ifdef PCONF_DMA2_STREAM0_IRQ
void DMA2_Stream0_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 8);
}
#endif

#ifdef PCONF_DMA2_STREAM1_IRQ
void DMA2_Stream1_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 9);
}
#endif

#ifdef PCONF_DMA2_STREAM2_IRQ
void DMA2_Stream2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 10);
}
#endif

#ifdef PCONF_DMA2_STREAM3_IRQ
void DMA2_Stream3_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 11);
}
#endif

#ifdef PCONF_DMA2_STREAM4_IRQ
void DMA2_Stream4_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 12);
}
#endif

#ifdef PCONF_DMA2_STREAM5_IRQ
void DMA2_Stream5_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 13);
}
#endif

#ifdef PCONF_DMA2_STREAM6_IRQ
void DMA2_Stream6_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 14);
}
#endif

#ifdef PCONF_DMA2_STREAM7_IRQ
void DMA2_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(pconf_hdmas + 15);
}
#endif

#ifdef PCONF_EXTI1_PIN_IRQ
void EXTI1_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(PCONF_EXTI1_PIN_IRQ);
}
#endif

#ifdef PCONF_EXTI9_5_PIN_IRQ
void EXTI9_5_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(PCONF_EXTI9_5_PIN_IRQ);

	__HAL_GPIO_EXTI_CLEAR_IT(PCONF_EXTI9_5_PIN_IRQ);
}
#endif
