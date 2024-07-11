#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "main.h"
#include "driver.h"
#include "uartdebug.h"
#include "mpu6500.h"
#include "bmp280.h"
#include "esp8266.h"
#include "dsp.h"

// device numbers
#define MPU_DEV 0
#define BMP_DEV 1

// timer settings
#define PRESCALER 16

#define OCSFREQ 16000000
#define TIMPERIOD 0xffff
#define TICKSPERSEC (OCSFREQ / PRESCALER)

// PWM settings
#define PWM_MAXCOUNT 20000

// DSP settings
#define LPF_COEF 0.05
#define TSTEP 0.01
#define PTSTEP 0.1
#define RTSTEP 0.1
#define YTSTEP (0.1 * M_PI)
#define PSTEP 0.1
#define ISTEP 0.001
#define DSTEP 0.001
#define WSTEP 0.01
#define CSTEP 0.001

// Wi-Fi settings:
// 	define SSID
// 	define PASSWORD
// 	define SERIP
// 	define SERVPORT
#include "wifidefs.h"

#define led0(s) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12,	\
	(s) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define led1(s) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 	\
	(s) ? GPIO_PIN_SET : GPIO_PIN_RESET)

void systemclock_config();
static void gpio_init();
static void i2c_init();
static void spi1_init();
static void dma_init();
static void tim1_init();
static void tim2_init();
static void adc1_init(void);
static void usart1_init();
static void usart2_init();
static void esc_init();
static void mpu_init();
static void bmp_init();
static void espdev_init();

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

// sensor initial values
float gx0, gy0, gz0;
float alt0, press0;

// IC drivers
struct driver drivers[4];
struct device dev[4];
struct esp_device espdev;

// DSP contexts
struct dsp_lpf presslpf;
struct dsp_compl pitchcompl;
struct dsp_compl rollcompl;
struct dsp_pidval pitchpv;
struct dsp_pidval rollpv;
struct dsp_pidval pitchspv;
struct dsp_pidval rollspv;
struct dsp_pidval yawspv;

// control values
float thrust = 0.0;
float rolltarget = 0.0, pitchtarget = 0.0, yawtarget = 0.0;
float lbw = 0.0, rbw = 0.0, ltw = 0.0, rtw = 0.0;

// PID settings and accelerometer correction
float ax0 = 0.000, ay0 = 0.0, az0 = 0.0;
int speedpid = 0;
float cf = 0.98;
float p = 0.0,		i = 0.0000,	d = 0.0;
float sp = 0.0,		si = 0.0000,	sd = 0.0;
float ysp = 0.0,	ysi = 0.0000,	ysd = 0.0;

int loopscount = 0;

uint32_t getadcv(ADC_HandleTypeDef *hadc)
{
	uint32_t v;

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1);

	v = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);
	
	return v;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	esp_interrupt(&espdev, huart);
}

int mpu_printuart(const struct mpu_data *md)
{
	uartprintf("%-15sx = %hd; y = %hd; z = %hd\n\r",
		"acceleration: ", md->ax, md->ay, md->az);

	uartprintf("%-15sx = %hd; y = %hd; z = %hd\n\r\n\r",
		"rotation: ", md->gx, md->gy, md->gz);

	return 0;
}

int mpu_printuartf(const struct mpu_data *md)
{
	uartprintf("%-15sx = %-.3f; y = %-.3f; z = %-.3f\n\r",
		"acceleration: ", (double) md->afx, (double) md->afy,
		(double) md->afz);

	uartprintf("%-15sx = %-.3f; y = %-.3f; z = %-.3f\n\r\n\r",
		"rotation: ", (double) md->gfx, (double) md->gfy,
		(double) md->gfz);

	return 0;
}

float groundpressure(float alt)
{
	float press;
	float c;
	int i;

	press = 0.0;
	c = 0.0;

	for (i = 0; i < 100; ++i) {
		struct bmp_data bd;
		float t, tt;

		dev[BMP_DEV].read(dev[BMP_DEV].priv, 0, &bd,
			sizeof(struct bmp_data));
	
		tt = bd.press / 100.0
			/ powf(1.0 - (alt / 44330.0), 5.255) - c;
		
		t = press + tt;
	
		c = (t - press) - tt;

		press = t;

		HAL_Delay(10);
	}

	press /= 100.0;

	return press;
}

float getalt(float press0, float press)
{
	return (44330.0 * (1.0 - powf(press / 100.0 / press0,
		1.0 / 5.225)));
}

int averageposition(float *ax, float *ay, float *az, float *gx,
	float *gy, float *gz)
{
	struct mpu_data md;
	float axt, ayt, azt, gxt, gyt, gzt;
	int i;

	axt = ayt = azt = gxt = gyt = gzt = 0;
	for (i = 0; i < 250; ++i) {
		dev[MPU_DEV].read(dev[MPU_DEV].priv, 0, &md,
			sizeof(struct mpu_data));

		gxt += md.gfx;
		gyt += md.gfy;
		gzt += md.gfz;

		axt += md.afx;
		ayt += md.afy;
		azt += md.afz;

		HAL_Delay(10);
	}

	*gx = gxt / 250.0;
	*gy = gyt / 250.0;
	*gz = gzt / 250.0;

	*ax = axt / 250.0;
	*ay = ayt / 250.0;
	*az = azt / 250.0;

	return 0;
}

int initstabilize(float alt)
{
	float ax, ay, az;

	alt0 = alt;

	press0 = groundpressure(alt);

	averageposition(&ax, &ay, &az, &gx0, &gy0, &gz0);

	dsp_initcompl(&pitchcompl, cf);
	dsp_initcompl(&rollcompl, cf);

	dsp_initpidval(&pitchpv, p, i, d, 0.0);
	dsp_initpidval(&rollpv, p, i, d, 0.0);

	dsp_initpidval(&pitchspv, sp, si, sd, 0.0);
	dsp_initpidval(&rollspv, sp, si, sd, 0.0);
	dsp_initpidval(&yawspv, ysp, ysi, ysd, 0.0);

	dsp_initlpf(&presslpf, LPF_COEF);

	return 0;
}

int stabilize(float dt)
{
//	struct bmp_data bd;
	struct mpu_data md;
	float lbd, rbd, ltd, rtd;
	float roll, pitch;
	float rollcor, pitchcor, yawcor;
	float gy, gx, gz;

	dt = (dt < 0.000001) ? 0.000001 : dt;
/*
	dev[BMP_DEV].read(dev[BMP_DEV].priv, 0, &bd,
		sizeof(struct bmp_data));

	dsp_updatelpf(&presslpf, bd.press);
*/
	dev[MPU_DEV].read(dev[MPU_DEV].priv, 0, &md,
		sizeof(struct mpu_data));

	gy = deg2rad((md.gfy - gy0));
	gx = deg2rad((md.gfx - gx0));
	gz = deg2rad((md.gfz - gz0));

	roll = dsp_updatecompl(&rollcompl, gy * dt,
		atan2f(-md.afx,
		sqrt(md.afy * md.afy + md.afz * md.afz))) - ax0;

	pitch = dsp_updatecompl(&pitchcompl, gx * dt,
		atan2f(md.afy,
		sqrt(md.afx * md.afx + md.afz * md.afz))) - ay0;

	if (speedpid) {
		rollcor = dsp_pid(&rollspv, rolltarget, gy, dt);
		pitchcor = dsp_pid(&pitchspv, pitchtarget, gx, dt);
	}
	else {
		rollcor = dsp_pid(&rollpv, rolltarget, roll, dt);
		pitchcor = dsp_pid(&pitchpv, pitchtarget, pitch, dt);

		rollcor = dsp_pid(&rollspv, rollcor, gy, dt);
		pitchcor = dsp_pid(&pitchspv, pitchcor, gx, dt);
	}
		
	yawcor = dsp_pid(&yawspv, yawtarget, gz, dt);
	
	lbd = trimuf(lbw * (thrust + 0.5 * rollcor
		- 0.5 * pitchcor + 0.5 * yawcor));
	rbd = trimuf(rbw * (thrust - 0.5 * rollcor
		- 0.5 * pitchcor - 0.5 * yawcor));
	ltd = trimuf(ltw * (thrust + 0.5 * rollcor
		+ 0.5 * pitchcor - 0.5 * yawcor));
	rtd = trimuf(rtw * (thrust - 0.5 * rollcor
		+ 0.5 * pitchcor + 0.5 * yawcor));


	TIM1->CCR1 = (uint16_t) ((ltd * 0.05 + 0.049) * (float) PWM_MAXCOUNT);
	TIM1->CCR2 = (uint16_t) ((rtd * 0.05 + 0.049) * (float) PWM_MAXCOUNT);
	TIM1->CCR3 = (uint16_t) ((rbd * 0.05 + 0.049) * (float) PWM_MAXCOUNT);
	TIM1->CCR4 = (uint16_t) ((lbd * 0.05 + 0.049) * (float) PWM_MAXCOUNT);

	return 0;
}

int parsecommand(char **toks, int maxtoks, char *data)
{
	int i;

	i = 0;

	toks[i++] = strtok((char *) data, " ");

	while (i < maxtoks && (toks[i++] = strtok(NULL, " ")) != NULL);

	return (i - 1);
}

int changef(char cmd, float *f, float exact, float step)
{
	if (cmd == 's')		*f = exact;
	else if (cmd == 'i')	*f += step;
	else if (cmd == 'd')	*f -= step;
	else			return (-1);

	return 0;
}

int controlcmd(char *cmd)
{
	char *toks[12];
	int tc;
	float exact;

	if (cmd[0] == '\0')
		return 0;

	tc = parsecommand(toks, 12, cmd);

	if (strcmp(toks[0], "md") == 0) {
		struct mpu_data md;
		char s[ESP_CMDSIZE];

		dev[MPU_DEV].read(dev[MPU_DEV].priv, 0, &md,
			sizeof(struct mpu_data));

		sprintf(s, "%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r",
			"accel: ", (double) md.afx, (double) md.afy,
			(double) md.afz);
		sprintf(s + strlen(s),
			"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r",
			"gyro: ", (double) deg2rad(md.gfx - gx0),
			(double) deg2rad(md.gfy - gy0),
			(double) deg2rad(md.gfz - gz0));
	
		sprintf(s + strlen(s),
			"roll: %0.3f; pitch: %0.3f\n\r",
			(double) (dsp_getcompl(&rollcompl) - ax0),
			(double) (dsp_getcompl(&pitchcompl) - ay0));

		sprintf(s + strlen(s),
			"battery: %0.3f\n\r",
			getadcv(&hadc1) / (double) 0xfff * (double) 6.6);

		esp_send(&espdev, s);
	
		return 0;
	}
	else if (strcmp(toks[0], "bd") == 0) {
		char s[ESP_CMDSIZE];
		float press;

		press = dsp_getlpf(&presslpf);

		sprintf(s, "bar: %f; alt: %f\r\n",
			(double) press, (double) getalt(press0, press));
		
		esp_send(&espdev, s);
		
		return 0;
	}
	else if (strcmp(toks[0], "vd") == 0) {
		char s[ESP_CMDSIZE];
		
		sprintf(s, "t: %.3f; r: %.3f; p: %.3f; y: %.3f\r\n",
			(double) thrust,
			(double) rolltarget,
			(double) pitchtarget,
			(double) yawtarget);

		sprintf(s + strlen(s),
			"lb: %.1f; rb: %.1f; lt: %.1f; rt: %.1f\r\n",
			(double) lbw, (double) rbw,
			(double) ltw, (double) rtw);
	
		sprintf(s + strlen(s), "cf: %.5f\r\n", (double) cf);
		
		sprintf(s + strlen(s), "loops count: %d\r\n",
			loopscount);

		esp_send(&espdev, s);
		
		return 0;
	}
	else if (strcmp(toks[0], "pd") == 0) {
		char s[ESP_CMDSIZE];
		
		sprintf(s, "pos PID: %.3f,%.3f,%.3f\r\n",
			(double) p, (double) i, (double) d);

		sprintf(s + strlen(s),
			"speed PID: %.3f,%.3f,%.3f\r\n",
			(double) sp, (double) si, (double) sd);

		sprintf(s + strlen(s),
			"yaw PID: %.3f,%.3f,%.3f\r\n",
			(double) ysp, (double) ysi, (double) ysd);

		sprintf(s + strlen(s), "%s mode\r\n",
			speedpid ? "single" : "dual");

		esp_send(&espdev, s);
		
		return 0;
	}

	else if (strcmp(toks[0], "r") == 0) {
		lbw = rbw = ltw = rtw = 0.0;
		
		return 0;
	}
	else if (strcmp(toks[0], "e") == 0) {
		lbw = rbw = ltw = rtw = 1.0;
		
		return 0;
	}
	else if (strcmp(toks[0], "dl") == 0) {
		speedpid = 0;
		
		return 0;
	}
	else if (strcmp(toks[0], "sl") == 0) {
		speedpid = 1;
		return 0;
	}

	if (tc < 2) {
		esp_send(&espdev, "Unknown command\r\n");
		return (-1);
	}

	if (strcmp(toks[0], "c") == 0) {
		initstabilize(atof(toks[1]));

		return 0;
	}

	exact = 0.0;
	if (strcmp(toks[1], "s") == 0) {
		if (tc < 3) {
			esp_send(&espdev, "Unknown command\r\n");
			return (-1);
		}
	
		exact = atof(toks[2]);
	}

	if (strcmp(toks[0], "t") == 0) {
		if (changef(toks[1][0], &thrust, exact, TSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "pt") == 0) {
		if (changef(toks[1][0], &pitchtarget, exact, PTSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "rt") == 0) {
		if (changef(toks[1][0], &rolltarget, exact, RTSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "yt") == 0) {
		if (changef(toks[1][0], &yawtarget, exact, YTSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "xc") == 0) {
		if (changef(toks[1][0], &ax0, exact, CSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "yc") == 0) {
		if (changef(toks[1][0], &ay0, exact, CSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "zc") == 0) {
		if (changef(toks[1][0], &az0, exact, CSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "p") == 0) {
		if (changef(toks[1][0], &p, exact, PSTEP) >= 0) {
			dsp_setpid(&pitchpv, p, i, d);
			dsp_setpid(&rollpv, p, i, d);
			
			return 0;
		}
	}
	else if (strcmp(toks[0], "i") == 0) {
		if (changef(toks[1][0], &i, exact, ISTEP) >= 0) {
			dsp_setpid(&pitchpv, p, i, d);
			dsp_setpid(&rollpv, p, i, d);
			
			return 0;
		}
	}
	else if (strcmp(toks[0], "d") == 0) {
		if (changef(toks[1][0], &d, exact, DSTEP) >= 0) {
			dsp_setpid(&pitchpv, p, i, d);
			dsp_setpid(&rollpv, p, i, d);
		
			return 0;
		}
	}
	else if (strcmp(toks[0], "sp") == 0) {
		if (changef(toks[1][0], &sp, exact, PSTEP) >= 0) {
			dsp_setpid(&pitchspv, sp, si, sd);
			dsp_setpid(&rollspv, sp, si, sd);

			return 0;
		}
	}
	else if (strcmp(toks[0], "si") == 0) {
		if (changef(toks[1][0], &si, exact, ISTEP) >= 0) {
			dsp_setpid(&pitchspv, sp, si, sd);
			dsp_setpid(&rollspv, sp, si, sd);
			
			return 0;
		}
	}
	else if (strcmp(toks[0], "sd") == 0) {
		if (changef(toks[1][0], &sd, exact, DSTEP) >= 0) {
			dsp_setpid(&pitchspv, sp, si, sd);
			dsp_setpid(&rollspv, sp, si, sd);
			
			return 0;
		}
	}
	else if (strcmp(toks[0], "yp") == 0) {
		if (changef(toks[1][0], &ysp, exact, PSTEP) >= 0) {
			dsp_setpid(&yawspv, ysp, ysi, ysd);

			return 0;
		}
	}
	else if (strcmp(toks[0], "yi") == 0) {
		if (changef(toks[1][0], &ysi, exact, ISTEP) >= 0) {
			dsp_setpid(&yawspv, ysp, ysi, ysd);

			return 0;
		}
	}
	else if (strcmp(toks[0], "yd") == 0) {
		if (changef(toks[1][0], &ysd, exact, DSTEP) >= 0) {
			dsp_setpid(&yawspv, ysp, ysi, ysd);
			
			return 0;
		}
	}
	else if (strcmp(toks[0], "lbt") == 0) {
		if (changef(toks[1][0], &lbw, exact, WSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "rbt") == 0) {
		if (changef(toks[1][0], &rbw, exact, WSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "ltt") == 0) {
		if (changef(toks[1][0], &ltw, exact, WSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "rtt") == 0) {
		if (changef(toks[1][0], &rtw, exact, WSTEP) >= 0)
			return 0;
	}
	else if (strcmp(toks[0], "cf") == 0) {
		if (changef(toks[1][0], &cf, exact, WSTEP) >= 0)
			return 0;
	
		dsp_initcompl(&pitchcompl, cf);
		dsp_initcompl(&rollcompl, cf);
	}	

	esp_send(&espdev, "Unknown command\r\n");

	return (-1);
}

int main(void)
{
	int ms;
	int loops;
	int mss;

	HAL_Init();

	systemclock_config();
	
	HAL_Delay(1000);

	gpio_init();
	tim1_init();
	tim2_init();
	dma_init();
	i2c_init();
	spi1_init();
	adc1_init();
	usart1_init();
	usart2_init();
	esc_init();
	bmp_init();
	mpu_init();
	espdev_init();

	if (espdev.status == ESP_INIT)
		led0(1);
	else if (espdev.status == ESP_CONNECTED)
		led1(1);

	initstabilize(0.0);

	loops = 0;
	mss = ms = 0;
	while (1) {
		char cmd[ESP_CMDSIZE];
		int c;
		
		__HAL_TIM_SET_COUNTER(&htim2, 0);

		if (esp_poll(&espdev, cmd) >= 0)
			controlcmd(cmd);

		if (ms > TICKSPERSEC/500) {
			double dt;

			dt = (float) ms / (float) TICKSPERSEC;
			
			ms = 0;

			stabilize(dt);
			++loops;
		}
	
		if (mss > TICKSPERSEC/1) {
			loopscount = loops;

			mss = loops = 0;
		}

		while ((c = __HAL_TIM_GET_COUNTER(&htim2)) < 100);
		
		ms += c;
		mss += c;
	}

	return 0;
}

void systemclock_config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		error_handler();

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
		| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
		| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
			FLASH_LATENCY_0) != HAL_OK)
		error_handler();
}

static void gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4 | GPIO_PIN_12 | GPIO_PIN_15,
		GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void dma_init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

static void i2c_init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
		error_handler();
}

static void spi1_init(void)
{
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;

	if (HAL_SPI_Init(&hspi1) != HAL_OK)
		error_handler();
}

static void tim1_init(void)
{
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = PRESCALER - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = PWM_MAXCOUNT;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
		error_handler();

	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
		error_handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1,
			&sMasterConfig) != HAL_OK)
		error_handler();

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1,
			&sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		error_handler();
	if (HAL_TIM_PWM_ConfigChannel(&htim1,
			&sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		error_handler();
	if (HAL_TIM_PWM_ConfigChannel(&htim1,
			&sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		error_handler();
	if (HAL_TIM_PWM_ConfigChannel(&htim1,
			&sConfigOC, TIM_CHANNEL_4) != HAL_OK)
		error_handler();

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1,
			&sBreakDeadTimeConfig) != HAL_OK)
		error_handler();

	HAL_TIM_MspPostInit(&htim1);
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

static void tim2_init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = PRESCALER - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = TIMPERIOD - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
		error_handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
		error_handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
		error_handler();
	
	HAL_TIM_Base_Start_IT(&htim2);
}

static void adc1_init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
		error_handler();

	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		error_handler();
}

static void usart1_init()
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart1) != HAL_OK)
		error_handler();
}

static void usart2_init()
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 921600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart2) != HAL_OK)
		error_handler();
}

static void esc_init()
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4
		= (uint16_t) (0.05 * (float) PWM_MAXCOUNT);
	HAL_Delay(2000);
}

static void mpu_init()
{
	struct mpu_device d;

	mpu_getdriver(drivers + MPU_DEV);

	d.hi2c = &hi2c1;
	d.devtype = MPU_DEV6050;
	d.accelscale = MPU_4G;
	d.gyroscale = MPU_1000DPS;
	d.dlpfwidth = MPU_44DLPF;

	if (drivers[0].initdevice(&d, dev + MPU_DEV) == 0)
		uartprintf("MPU-6500 initilized\r\n");
	else
		uartprintf("failed to initilize MPU-6500\r\n");
}

static void bmp_init()
{
	struct bmp_device d;

	bmp_getdriver(drivers + BMP_DEV);

	d.hi2c = &hi2c1;

	if (drivers[1].initdevice(&d, dev + BMP_DEV) == 0)
		uartprintf("BMP280 initilized\r\n");
	else
		uartprintf("failed to initilize BMP280\r\n");

}

static void espdev_init()
{
	char ip[32];
	
	espdev.huart = &huart1;

	if (esp_init(&espdev, SSID, PASSWORD) < 0) {
		uartprintf("failed to initilize ESP8266\r\n");
		return;
	}

	uartprintf("ESP8266 initilized\r\n");

	uartprintf("getting ip...\r\n");

	if (esp_getip(&espdev, ip) < 0)
		uartprintf("failed to get ip\r\n");
	else
		uartprintf("got ip %s\r\n", ip);

	uartprintf("connecting to %s:%d...\r\n", SERVIP, SERVPORT);
	
	if (esp_connect(&espdev, SERVIP, SERVPORT) < 0) {
		uartprintf("failed to connect to %s %s\r\n",
			SERVIP, SERVPORT);
		return;
	}

	uartprintf("connected\r\n");

}

void error_handler(void)
{
	__disable_irq();
	while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
