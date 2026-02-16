#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#include "util.h"
#include "dsp.h"

#include "dshot.h"

static struct dshot_device dshot_devs[DSHOT_MAXDEVS];
static size_t dshot_devcount = 0;

/**
* @brief Get DMA used for timer's PWM channel.
* @param htim timer handler
* @param PWM channel id
* @return pointer to DMA handler, if correct
* 	channel ID passed, NULL otherwise
*/
static DMA_HandleTypeDef *dshot_timdma(TIM_HandleTypeDef *htim,
	int timch)
{
	if (timch == TIM_CHANNEL_1)
		return htim->hdma[TIM_DMA_ID_CC1];
	else if (timch == TIM_CHANNEL_2)
		return htim->hdma[TIM_DMA_ID_CC2];
	else if (timch == TIM_CHANNEL_3)
		return htim->hdma[TIM_DMA_ID_CC3];
	else if (timch == TIM_CHANNEL_4)
		return htim->hdma[TIM_DMA_ID_CC4];
		
	return NULL;
}

/**
* @brief Get timer's PWM channel DMA ID.
* @param timch PWM channel id
* @return timer's PWM channel DMA ID, if
* 	correct channel ID passed, -1 otherwise
*/
static int dshot_timdmacc(int timch)
{
	if (timch == TIM_CHANNEL_1)
		return TIM_DMA_CC1;
	else if (timch == TIM_CHANNEL_2)
		return TIM_DMA_CC2;
	else if (timch == TIM_CHANNEL_3)
		return TIM_DMA_CC3;
	else if (timch == TIM_CHANNEL_4)
		return TIM_DMA_CC4;
		
	return (-1);
}

/**
* @brief Get pointer to timer's PWM channel counter register.
* @param htim timer handler
* @param PWM channel id
* @return pointer to timer's PWM channel counter register, if correct
* 	channel ID passed, NULL otherwise
*/
static volatile uint32_t *dshot_timccr(TIM_HandleTypeDef *htim,
	int timch)
{
	if (timch == TIM_CHANNEL_1)
		return &(htim->Instance->CCR1);
	else if (timch == TIM_CHANNEL_2)
		return &(htim->Instance->CCR2);
	else if (timch == TIM_CHANNEL_3)
		return &(htim->Instance->CCR3);
	else if (timch == TIM_CHANNEL_4)
		return &(htim->Instance->CCR4);
		
	return NULL;
}

/**
* @brief DMA callback for dshot processing.
* @param hdma DMA device that triggered this callback
* @return none
*/
static void dshotdmacb(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim;;

	htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;

	// disable DMA after bit was sent
	if (hdma == htim->hdma[TIM_DMA_ID_CC1])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC2])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC3])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
	else if(hdma == htim->hdma[TIM_DMA_ID_CC4])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
}

/**
* @brief Fill buffer, that contains PWM duty cycle values of a
	a DSHOT packet using 16 bit value.
* @param buf buffer for PWM duty cycle values that is used by DMA
* @param val 16 bit value from 0 to 2047
* @param tele 1 if telemetry bit needed, 0 otherwise
*/
static void dshotsetbuf(uint16_t *buf, uint16_t val, int tele)
{
	uint16_t pack;
	unsigned int csum;
	int i;

	// bit after thrust value is telemetry request,
	// so just shift because no telemetry is used
	pack = (val << 1) | tele;

	// calculate dshot checksum
	csum = (pack ^ (pack >> 4) ^ (pack >> 8) ^ (pack >> 12)) & 0xf;

	// append checksum to the packet
	pack = (pack << 4) | csum;

	// construct PWM duty cycle buffer for the packet
	for(i = 0; i < 16; i++) {
		buf[i] = (pack & 0x8000) ? DSHOT_1 : DSHOT_0;
		pack <<= 1;
	}

	// two bits used to maintain space between packets
	buf[16] = 0;
	buf[17] = 0;
}

/**
* @brief Fill buffer, that contains PWM duty cycle values of a
	a DSHOT packet with thrust value.
* @param buf buffer for PWM duty cycle values that is used by DMA
* @param v value from 0.0 to 1.0
* @return always 0
*/
static inline int dshotsetthrust(uint16_t *buf, float v)
{
	// convert motor thrust value from [0.0;1.0]
	// range to int value in [48; 2047] range
	// disable telemetry request bit
	dshotsetbuf(buf, (uint16_t) (trimuf(v) * 1999.0 + 0.5) + 48, 0);

	return 0;
}

int dshot_configure(void *d, const char *cmd, ...)
{
	struct dshot_device *dev;
	static uint16_t dsbuf[18];
	va_list args;
	uint16_t dcmd;
	int n;
	int i;

	dev = (struct dshot_device *) d;

	va_start(args, cmd);

	n = va_arg(args, int);

	// set DShot command code using word passed to configure command
	if (strcmp(cmd, "reverse") == 0)		dcmd = 21;
	else if (strcmp(cmd, "direct") == 0)		dcmd = 20;
	else if (strcmp(cmd, "save") == 0)		dcmd = 12;
	else {
		va_end(args);
		return 0;
	}
	// construst a PWM duty cycle buffer for dshot ESC
	dshotsetbuf(dsbuf, dcmd, 1);

	// wait for previous DSHOT packet to finish
	udelay(100);

	// send packet with command 10 times. Specification
	// requires 6 for most command, 10 is to be sure.
	for (i = 0; i < 10; ++i) {
		// instruct DMA to use the buffer
		// for stream corresponding to DSHOT output
		HAL_DMA_Start_IT(dev->hdma[n], (uint32_t) dsbuf,
			(uint32_t) (dev->timccr[n]), 18);

		// Enable DMA for stream corresponding to DSHOT output
		__HAL_TIM_ENABLE_DMA(dev->htim[n], (dev->timcc[n]));

		// wait for previous command packet to finish
		udelay(100);
	}
	
	va_end(args);

	if (dcmd == 12)
		mdelay(35);

	return 0;
}

int dshot_setthrust(void *d, void *dt, size_t sz)
{
	struct dshot_device *dev;
	struct dshot_data *data;
	static uint16_t dsbuf[4][18];

	dev = d;
	data = dt;

	/*
	      ltd    rtd
		\    /
	   p     \  /
	   |
	   v     /  \
		/    \
	      lbd    rbd

		 <- r
	*/

	// construst a PWM duty cycle
	// buffers for dshot ESCs
	dshotsetthrust(dsbuf[0], data->thrust[0]);
	dshotsetthrust(dsbuf[1], data->thrust[1]);
	dshotsetthrust(dsbuf[2], data->thrust[2]);
	dshotsetthrust(dsbuf[3], data->thrust[3]);

	// instruct DMA to use the buffers
	// for streams corresponding to DSHOT ESCs
	HAL_DMA_Start_IT(dev->hdma[0], (uint32_t) dsbuf[0],
		(uint32_t) dev->timccr[0], 18);
	HAL_DMA_Start_IT(dev->hdma[1], (uint32_t) dsbuf[1],
		(uint32_t) dev->timccr[1], 18);
	HAL_DMA_Start_IT(dev->hdma[2], (uint32_t) dsbuf[2],
		(uint32_t) dev->timccr[2], 18);
	HAL_DMA_Start_IT(dev->hdma[3], (uint32_t) dsbuf[3],
		(uint32_t) dev->timccr[3], 18);

	// Enable DMA for streams corresponding to DSHOT ESCs
	__HAL_TIM_ENABLE_DMA(dev->htim[0], dev->timcc[0]);
	__HAL_TIM_ENABLE_DMA(dev->htim[1], dev->timcc[1]);
	__HAL_TIM_ENABLE_DMA(dev->htim[2], dev->timcc[2]);
	__HAL_TIM_ENABLE_DMA(dev->htim[3], dev->timcc[3]);

	return 0;
}

int dshot_init(struct dshot_device *dev)
{
	struct dshot_data dd;
	
	// set timer's reload value for each PWM channel
	__HAL_TIM_SET_AUTORELOAD(dev->htim[0], DSHOT_BITLEN);
	__HAL_TIM_SET_AUTORELOAD(dev->htim[1], DSHOT_BITLEN);
	__HAL_TIM_SET_AUTORELOAD(dev->htim[2], DSHOT_BITLEN);
	__HAL_TIM_SET_AUTORELOAD(dev->htim[3], DSHOT_BITLEN);

	// get timer's channel IDs of each PWM channel
	dev->timcc[0] = dshot_timdmacc(dev->timch[0]);
	dev->timcc[1] = dshot_timdmacc(dev->timch[1]);
	dev->timcc[2] = dshot_timdmacc(dev->timch[2]);
	dev->timcc[3] = dshot_timdmacc(dev->timch[3]);

	// get pointers to timer's channels counter
	// registers corresponding to each PWM channel
	dev->timccr[0] = dshot_timccr(dev->htim[0], dev->timch[0]);
	dev->timccr[1] = dshot_timccr(dev->htim[1], dev->timch[1]);
	dev->timccr[2] = dshot_timccr(dev->htim[2], dev->timch[2]);
	dev->timccr[3] = dshot_timccr(dev->htim[3], dev->timch[3]);

	// get pointers to DMAs corresponding to each PWM channel
	dev->hdma[0] = dshot_timdma(dev->htim[0], dev->timch[0]);
	dev->hdma[1] = dshot_timdma(dev->htim[1], dev->timch[1]);
	dev->hdma[2] = dshot_timdma(dev->htim[2], dev->timch[2]);
	dev->hdma[3] = dshot_timdma(dev->htim[3], dev->timch[3]);

	// set dshot DMA callback for TIM channels
	// corresponding to DSHOT escs
	dev->hdma[0]->XferCpltCallback = dshotdmacb;
	dev->hdma[1]->XferCpltCallback = dshotdmacb;
	dev->hdma[2]->XferCpltCallback = dshotdmacb;
	dev->hdma[3]->XferCpltCallback = dshotdmacb;

	// start PWM on TIM channels corresponding to DSHOT ESCs
  	HAL_TIM_PWM_Start(dev->htim[0], dev->timch[0]);
  	HAL_TIM_PWM_Start(dev->htim[1], dev->timch[1]);
	HAL_TIM_PWM_Start(dev->htim[2], dev->timch[2]);
	HAL_TIM_PWM_Start(dev->htim[3], dev->timch[3]);

	// set zero initial thrust for every motor
	dd.thrust[0] = dd.thrust[1] = dd.thrust[2] = dd.thrust[3] = 0.0;
	dshot_setthrust(dev, &dd, sizeof(struct dshot_data));

	return 0;
}

int dshot_initdevice(void *is, struct cdevice *dev)
{
	int r;

	memmove(dshot_devs + dshot_devcount, is,
		sizeof(struct dshot_device));

	sprintf(dev->name, "%s_%d", "dshot", dshot_devcount);

	dev->priv = dshot_devs + dshot_devcount;
	dev->read = NULL;
	dev->configure = dshot_configure;
	dev->write = dshot_setthrust;
	dev->interrupt = NULL;
	
	r = dshot_init(dshot_devs + dshot_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
