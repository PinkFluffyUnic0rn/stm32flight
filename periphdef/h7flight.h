#ifndef HAL_MODULE_ENABLED
#define HSE_VALUE 20000000
#define ERROR_GPIO GPIOB
#define ERROR_PIN GPIO_PIN_13
#else

#define PCONF_DMASCOUNT 	16
#define PCONF_OUTPINSCOUNT	10
#define PCONF_INPINSCOUNT	1
#define PCONF_I2CSCOUNT		1
#define PCONF_SPISCOUNT		2
#define PCONF_EXTISCOUNT	1
#define PCONF_TIMSCOUNT		3
#define PCONF_ADCSCOUNT		2
#define PCONF_UARTSCOUNT	4

#define PCONF_DMA1_STREAM0_IRQ
#define PCONF_DMA1_STREAM1_IRQ
#define PCONF_DMA1_STREAM2_IRQ
#define PCONF_DMA1_STREAM5_IRQ
#define PCONF_DMA1_STREAM6_IRQ
#define PCONF_DMA1_STREAM7_IRQ
#define PCONF_DMA2_STREAM0_IRQ
#define PCONF_DMA2_STREAM1_IRQ
#define PCONF_DMA2_STREAM2_IRQ
#define PCONF_DMA2_STREAM3_IRQ
#define PCONF_DMA2_STREAM4_IRQ
#define PCONF_DMA2_STREAM5_IRQ
#define PCONF_DMA2_STREAM6_IRQ

#define PCONF_UART2_IDX_IRQ 0
#define PCONF_UART3_IDX_IRQ 1
#define PCONF_UART4_IDX_IRQ 2
#define PCONF_I2C1_IDX_IRQ 0

#define PCONF_EXTI9_5_PIN_IRQ GPIO_PIN_5

DMA_Stream_TypeDef *const dmas[] = {
	DMA1_Stream0,
	DMA1_Stream1,
	DMA1_Stream2,
	NULL,
	NULL,
	DMA1_Stream5,
	DMA1_Stream6,
	DMA1_Stream7,
	DMA2_Stream0,
	DMA2_Stream1,
	DMA2_Stream2,
	DMA2_Stream3,
	DMA2_Stream4,
	DMA2_Stream5,
	DMA2_Stream6,
	NULL
};

const struct pconf_pin outpins[] = {
	{
		.inst = GPIOE,
		.idx = GPIO_PIN_1
	},
	{
		.inst = GPIOE,
		.idx = GPIO_PIN_0
	},
	{
		.inst = GPIOB,
		.idx = GPIO_PIN_6
	},
	{
		.inst = GPIOC,
		.idx = GPIO_PIN_11
	},
	{
		.inst = GPIOC,
		.idx = GPIO_PIN_10
	},
	{
		.inst = GPIOA,
		.idx = GPIO_PIN_12
	},
	{
		.inst = GPIOB,
		.idx = GPIO_PIN_13
	},
	{
		.inst = GPIOB,
		.idx = GPIO_PIN_12
	},
	{
		.inst = GPIOE,
		.idx = GPIO_PIN_3
	},
	{
		.inst = GPIOE,
		.idx = GPIO_PIN_4
	}
};

const struct pconf_pin inpins[] = {
	{
		.inst = GPIOB,
		.idx = GPIO_PIN_7
	}
};

const struct pconf_exti extis[] = {
	{
		.pin = {
			.inst = GPIOB,
			.idx = GPIO_PIN_5
		}
	}
};

const struct pconf_i2c i2cs[] = {
	{
		.inst = I2C1,
		.sda = {
			.inst = GPIOB,
			.idx = GPIO_PIN_9
		},
		.scl = {
			.inst = GPIOB,
			.idx = GPIO_PIN_8
		},
		.rxdma = DMA1_Stream0,
		.txdma = DMA1_Stream7
	}
};

const struct pconf_spi spis[] = {
	{
		.inst = SPI1,
		.usage = PCONF_SPIUSAGE_DEVS,
		.miso = {
			.inst = GPIOB,
			.idx = GPIO_PIN_4
		},
		.mosi = {
			.inst = GPIOD,
			.idx = GPIO_PIN_7
		},
		.sck = {
			.inst = GPIOB,
			.idx = GPIO_PIN_3
		}
	},
	{
		.inst = SPI2,
		.usage = PCONF_SPIUSAGE_WIFI,
		.miso = {
			.inst = GPIOC,
			.idx = GPIO_PIN_2
		},
		.mosi = {
			.inst = GPIOC,
			.idx = GPIO_PIN_3
		},
		.sck = {
			.inst = GPIOD,
			.idx = GPIO_PIN_3
		}
	},
};

const struct pconf_tim tims[] = {
	{
		.inst = TIM1,
		.usage = PCONF_TIMUSAGE_PWM,
		.pwm = {
			{
				.chan = 1,
				.pin = {
					.inst = GPIOA,
					.idx = GPIO_PIN_8
				},
				.dma = DMA2_Stream1
			},
			{
				.chan = 2,
				.pin = {
					.inst = GPIOA,
					.idx = GPIO_PIN_9
				},
				.dma = DMA2_Stream2
			},
			{
				.chan = 3,
				.pin = {
					.inst = GPIOA,
					.idx = GPIO_PIN_10
				},
				.dma = DMA2_Stream6
			},
			{
				.chan = 4,
				.pin = {
					.inst = GPIOA,
					.idx = GPIO_PIN_11
				},
				.dma = DMA2_Stream4
			}
		},
		.chcnt = 4
	},
	{
		.inst = TIM8,
		.usage = PCONF_TIMUSAGE_SCHED

	},
	{
		.inst = TIM3,
		.usage = PCONF_TIMUSAGE_DELAY
	}
};

const struct pconf_adc adcs[] = {
	{
		.inst = ADC1,
		.usage = PCONF_ADCUSAGE_MEASURE,
		.pin =  {
			.inst = GPIOC,
			.idx = GPIO_PIN_4
		},
		.dma = DMA2_Stream0
	},
	{
		.inst = ADC2,
		.pin = {
			.inst = GPIOC,
			.idx = GPIO_PIN_5
		},
		.dma = DMA2_Stream3
	}
};

const struct pconf_uart uarts[] = {
	{
		.inst = USART2,
		.usage = PCONF_UARTUSAGE_CRSF,
		.rx = {
			.inst = GPIOD,
			.idx = GPIO_PIN_6
		},
		.tx = {
			.inst = GPIOD,
			.idx = GPIO_PIN_5

		},
		.rxdma = DMA1_Stream5,
		.txdma = DMA1_Stream6
	},
	{
		.inst = USART3,
		.usage = PCONF_UARTUSAGE_GNSS,
		.rx = {
			.inst = GPIOB,
			.idx = GPIO_PIN_11
		},
		.tx = {
			.inst = GPIOB,
			.idx = GPIO_PIN_10
		},
		.rxdma = DMA1_Stream1,
		.txdma = NULL
	},
	{
		.inst = UART4,
		.usage = PCONF_UARTUSAGE_DEBUG,
		.rx = {
			.inst = GPIOD,
			.idx = GPIO_PIN_0
		},
		.tx = {
			.inst = GPIOD,
			.idx = GPIO_PIN_1
		},
		.rxdma = DMA1_Stream2,
		.txdma = NULL
	},
	{
		.inst = UART5,
		.usage = PCONF_UARTUSAGE_IRC,
		.rx = {
			.inst = GPIOD,
			.idx = GPIO_PIN_2
		},
		.tx = {
			.inst = GPIOC,
			.idx = GPIO_PIN_12
		},
		.rxdma = NULL,
		.txdma = NULL
	}
};

const struct pconf_pin armpinconf = {
	.inst = GPIOB,
	.idx = GPIO_PIN_12
};

const struct pconf_pin debugpinconf = {
	.inst = GPIOE,
	.idx = GPIO_PIN_4
};

const struct pconf_pin errorpinconf = {
	.inst = GPIOB,
	.idx = GPIO_PIN_13
};

TIM_TypeDef *delayconf = TIM3;
TIM_TypeDef *schedconf = TIM8;

const struct pconf_debug debugconf = {
	.iface = {
		.type = PCONF_IFACETYPE_UART,
		.huart = UART4
	}
};

const struct pconf_imu imuconf = {
	.type = PCONF_IMUTYPE_ICM42688P,
	.iface = {
		.type = PCONF_IFACETYPE_SPI,
		.cs = {
			GPIOC,
			GPIO_PIN_11
		},
		.hspi = SPI1
	}
};

const struct pconf_bar barconf = {
	.type = PCONF_BARTYPE_DPS368,
	.iface = {
		.type = PCONF_IFACETYPE_I2C,
		.hi2c = I2C1
	}
};

const struct pconf_mag magconf = {
	.type = PCONF_MAGTYPE_QMC5883L,
	.iface = {
		.type = PCONF_IFACETYPE_I2C,
		.hi2c = I2C1
	}
};

const struct pconf_flash flashconf = {
	.type = PCONF_FLASHTYPE_W25Q,
	.iface = {
		.type = PCONF_IFACETYPE_SPI,
		.cs = {
			GPIOC,
			GPIO_PIN_10
		},
		.hspi = SPI1
	}
};

const struct pconf_crsf crsfconf = {
	.iface = {
		.type = PCONF_IFACETYPE_UART,
		.huart = USART2
	}
};

const struct pconf_gnss gnssconf = {
	.iface = {
		.type = PCONF_IFACETYPE_UART,
		.huart = USART3
	}
};

const struct pconf_wireless rfconf = {
	.iface = {
		.type = PCONF_IFACETYPE_SPI,
		.cs = {
			GPIOE,
			GPIO_PIN_0
		},
		.hspi = SPI2
	},
	.interrupt = {
		.inst = GPIOB,
		.idx = GPIO_PIN_5
	},
	.busy = {
		.inst = GPIOB,
		.idx = GPIO_PIN_7
	},
	.boot = {
		.inst = GPIOB,
		.idx = GPIO_PIN_6
	},
	.reset = {
		.inst = GPIOE,
		.idx = GPIO_PIN_1
	}
};

const struct pconf_vtx vtxconf = {
	.iface = {
		.type = PCONF_IFACETYPE_UART,
		.huart = UART5
	}
};

const struct pconf_pwm pwmconf = {
	.pwm = {
		{
			.inst = TIM1,
			.chan = 1
		},
		{
			.inst = TIM1,
			.chan = 2
		},
		{
			.inst = TIM1,
			.chan = 3
		},
		{
			.inst = TIM1,
			.chan = 4
		}
	}
};

const struct pconf_battery batconf = {
	.adc = ADC1,
	.pin = {
		.inst = GPIOC,
		.idx = GPIO_PIN_4
	}
};

const struct pconf_current curconf = {
	.adc = ADC1,
	.pin = {
		.inst = GPIOC,
		.idx = GPIO_PIN_5
	}
};

#endif
