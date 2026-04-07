/**
* @file periphstruct.h
* @brief common definition for STM32 peripherals and board devices
*/

#include "mcudef.h"

/**
* @brief I/O pin
*/
struct pconf_pin {
	GPIO_TypeDef *inst;	/*!< GPIO port */
	uint16_t idx;		/*!< pin number */
};

/**
* @brief I2C interface
*/
struct pconf_i2c {
	I2C_TypeDef *inst;		/*!< I2C HAL instance */

	/**
	* @brief interface usage
	*/
	enum PCONF_I2CUSAGE {
		PCONF_I2CUSAGE_DEVS,	/*!< used for board devices */
	} usage;

	struct pconf_pin sda;		/*!< SDA pin */
	struct pconf_pin scl;		/*!< clock pin */
	DMA_Stream_TypeDef *rxdma;	/*!< DMA stream instance
						used for receiving */
	DMA_Stream_TypeDef *txdma;	/*!< DMA stream instance used
						for transmiting */
};

/**
* @brief spi interface
*/
struct pconf_spi {
	SPI_TypeDef *inst;		/*!< SPI HAL instance */

	/**
	* @brief interface usage
	*/
	enum PCONF_SPIUSAGE {
		PCONF_SPIUSAGE_DEVS,	/*!< used for board devices */
		PCONF_SPIUSAGE_WIFI	/*!< used for wifi chip */
	} usage;

	struct pconf_pin miso;		/*!< MISO pin */
	struct pconf_pin mosi;		/*!< MOSI pin */
	struct pconf_pin sck;		/*!< clock pin */
	DMA_Stream_TypeDef *txdma;	/*!< DMA stream instance used 
						to transmiting */
};

/**
* @brief external interrupt interface
*/
struct pconf_exti {
	struct pconf_pin pin;		/*!< pin used for interrupt */
};

/**
* @brief timer interface
*/
struct pconf_tim {
	TIM_TypeDef *inst;		/*!< timer HAL instance */

	/**
	* @brief timer usage
	*/
	enum PCONF_TIMUSAGE {
		PCONF_TIMUSAGE_PWM,		/*!< used for PWM
							generation */
		PCONF_TIMUSAGE_PWMBURST,	/*!< used for PWM
							generation in
							tim burst 
							mode */
		PCONF_TIMUSAGE_SCHED,		/*!< used for
							scheduling */
		PCONF_TIMUSAGE_DELAY		/*!< used for delays */
	} usage;

	/**
	* @brief PWM channels
	*/
	struct {
		int chan;			/*!< timer's channel
							number */
		struct pconf_pin pin;		/*!< pin used for
							PWM output */
		DMA_Stream_TypeDef *dma;	/*!< DMA stream instance
							used for PWM
							output */
	} pwm[4];
	DMA_Stream_TypeDef *updma;		/*!< DMA stream instance
							used for PWM
							output in PWM
							burst mode */
	int chcnt;				/*!< PWM channels
							count */
};

/**
* @brief ADC interface
*/
struct pconf_adc {
	ADC_TypeDef *inst;		/*!< SPI HAL instance */

	/**
	* @brief ADC usage
	*/
	enum PCONF_ADCUSAGE {
		PCONF_ADCUSAGE_MEASURE,	/*!< used for battery
						or current sensor */
	} usage;

	struct pconf_pin pin;		/*!< pin used for ADC input */
	uint32_t chan;			/*!< ADC channel's number */
	DMA_Stream_TypeDef *dma;	/*!< DMA stream instance used
						for ADC sampling */
};

/**
* @brief UART interface
*/
struct pconf_uart {
	USART_TypeDef *inst;		/*!< UART HAL instance */

	/**
	* @brief interface usage
	*/
	enum PCONF_UARTUSAGE {
		PCONF_UARTUSAGE_CRSF,	/*!< used for CRSF eLRS
						device */
		PCONF_UARTUSAGE_GNSS,	/*!< used for GNSS device */
		PCONF_UARTUSAGE_DEBUG,	/*!< used as debug channel */
		PCONF_UARTUSAGE_IRC,	/*!< used for IRC VTX device */
		PCONF_UARTUSAGE_MSP	/*!< used for MSP VTX device */
	} usage;

	struct pconf_pin rx;		/*!< RX pin */
	struct pconf_pin tx;		/*!< TX pin */
	DMA_Stream_TypeDef *rxdma;	/*!< DMA stream instance
						used for receiving */
	DMA_Stream_TypeDef *txdma;	/*!< DMA stream instance
						used for transmiting */
};

/**
* @brief board device interface
*/
struct pconf_iface {
	/**
	* @brief interface type
	*/
	enum PCONF_IFACETYPE {
		PCONF_IFACETYPE_I2C,	/*!< I2C interface */
		PCONF_IFACETYPE_SPI,	/*!< SPI interface */
		PCONF_IFACETYPE_UART	/*!< UART interface */
	} type;

	/**
	* @brief interface
	*/
	union {
		USART_TypeDef *huart;		/*!< I2C interface */
		I2C_TypeDef *hi2c;		/*!< I2C interface */

		/**
		* @brief SPI interface with chip select pin
		*/
		struct {
			SPI_TypeDef *hspi;	/*!< SPI interface */
			struct pconf_pin cs;	/*!< chip select pin */
		};
	};
};

/**
* @brief debug connection device
*/
struct pconf_debug {
	struct pconf_iface iface;	/*!< used interface */
};

/**
* @brief IMU device
*/
struct pconf_imu {
	/**
	* @brief IMU device type
	*/
	enum PCONF_IMUTYPE {
		PCONF_IMUTYPE_ICM42688P,	/*!< ICM-42688-P */
		PCONF_IMUTYPE_MPU6500		/*!< MPU-6500/6050 */
	} type;

	struct pconf_iface iface;		/*!< used interface */
};

/**
* @brief barometer device
*/
struct pconf_bar {
	/**
	* @brief barometer device type
	*/
	enum PCONF_BARTYPE {
		PCONF_BARTYPE_HP206C,		/*!< HP206C */
		PCONF_BARTYPE_BMP280,		/*!< BMP280 */
		PCONF_BARTYPE_DPS368		/*!< DPS368 */
	} type;

	struct pconf_iface iface;;		/*!< used interface */
};

/**
* @brief magmetometer device
*/
struct pconf_mag {
	/**
	* @brief magnetometer device type
	*/
	enum PCONF_MAGTYPE {
		PCONF_MAGTYPE_QMC5883L,	/*!< QMC5883L */
		PCONF_MAGTYPE_HMC5883L	/*!< HMC5883L */
	} type;

	struct pconf_iface iface;	/*!< used interface */
};

/**
* @brief flash device
*/
struct pconf_flash {
	/**
	* @brief flash device type
	*/
	enum PCONF_FLASHTYPE {
		PCONF_FLASHTYPE_W25Q	/*!< HMC5883L */
	} type;

	struct pconf_iface iface;	/*!< used interface */
};

/**
* @brief CRSF device
*/
struct pconf_crsf {
	struct pconf_iface iface;	/*!< used interface */
};

/**
* @brief GNSS device
*/
struct pconf_gnss {
	struct pconf_iface iface;	/*!< used interface */
};

/**
* @brief Wireless configuration device
*/
struct pconf_wireless {
	struct pconf_iface iface;	/*!< used interface */
	struct pconf_pin interrupt;	/*!< pin used to interrupt
						when new config
						data arrived */
	struct pconf_pin busy;		/*!< pin used to indicate
						device's busy state */
	struct pconf_pin boot;		/*!< device's boot pin */
	struct pconf_pin reset;		/*!< device's reset pin */
};

/**
* @brief VTX device
*/
struct pconf_vtx {
	/**
	* @brief VTX device type
	*/
	enum PCONF_VTXTYPE {
		PCONF_VTXTYPE_IRC,	/*!< VTX that uses IRC
						tramp protocol */
		PCONF_VTXTYPE_MSP	/*!< VTX that uses MSP
						protocol */
	} type;

	struct pconf_iface iface;	/*!< used interface */
};

/**
* @brief PWM throttle signal device
*/
struct pconf_pwm {
	/**
	* used DMA mode
	*/
	enum PCONF_TIMDMATYPE {
		PCONF_TIMDMATYPE_BURST,	/*!< DMA in tim burst mode */
		PCONF_TIMDMATYPE_CCR,	/*!< separate DMA stream for
						each PWM channel */
	} dmatype;
	
	/**
	* PWM protocol
	*/
	enum PCONF_PROTO {
		PCONF_PROTO_DSHOT300,	/*!< DShot-300 */
		PCONF_PROTO_DSHOT600	/*!< DShot-600 */
	} proto;

	/**
	* PWM channel
	*/
	struct {
		TIM_TypeDef *inst;	/*!< timer used for this
						PWM channel */
		int chan;		/*!< timer's channel used for
						this PWM channel */
	} pwm[4];
};

/**
* @brief battery sensor
*/
struct pconf_battery {
	ADC_TypeDef *adc;	/*!< ADC used for battery sensor */
	struct pconf_pin pin;;	/*!< pin used for battery sensor */
};

/**
* @brief current sensor
*/
struct pconf_current {
	ADC_TypeDef *adc;	/*!< ADC used for current sensor */
	struct pconf_pin pin;	/*!< ADC used for current sensor */
};
