#include "mcudef.h"

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
	DMA_Stream_TypeDef *txdma;
};

struct pconf_exti {
	struct pconf_pin pin;
};

struct pconf_tim {
	TIM_TypeDef *inst;

	enum PCONF_TIMUSAGE {
		PCONF_TIMUSAGE_PWM,
		PCONF_TIMUSAGE_PWMBURST,
		PCONF_TIMUSAGE_SCHED,
		PCONF_TIMUSAGE_DELAY
	} usage;

	struct {
		int chan;
		struct pconf_pin pin;
		DMA_Stream_TypeDef *dma;
	} pwm[4];
	DMA_Stream_TypeDef *updma;
	int chcnt;
};

struct pconf_adc {
	ADC_TypeDef *inst;

	enum PCONF_ADCUSAGE {
		PCONF_ADCUSAGE_MEASURE,
	} usage;

	struct pconf_pin pin;
	uint32_t chan;
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
	enum PCONF_TIMDMATYPE {
		PCONF_TIMDMATYPE_BURST,
		PCONF_TIMDMATYPE_CCR,
	} dmatype;

	enum PCONF_PROTO {
		PCONF_PROTO_DSHOT300,
		PCONF_PROTO_DSHOT600
	} proto;

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
