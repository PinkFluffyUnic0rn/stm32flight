#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#include "mcudef.h"
#include "util.h"
#include "dsp.h"

#include "dshot.h"

#define DSHOT150_BITLEN (6.67 * 1e-6)
#define DSHOT300_BITLEN (3.33 * 1e-6)
#define DSHOT600_BITLEN (1.67 * 1e-6)
#define DSHOT1200_BITLEN (0.83 * 1e-6)

static struct dshot_device dshot_devs[DSHOT_MAXDEVS];
static size_t dshot_devcount = 0;

DMABUF static uint16_t nrzibuf[4][20];

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
* @brief Get timer's PWM channel flag ID.
* @param timch PWM channel id
* @return timer's PWM channel flag ID, if
* 	correct channel ID passed, -1 otherwise
*/
static int dshot_timflagcc(int timch)
{
	if (timch == TIM_CHANNEL_1)
		return TIM_FLAG_CC1;
	else if (timch == TIM_CHANNEL_2)
		return TIM_FLAG_CC2;
	else if (timch == TIM_CHANNEL_3)
		return TIM_FLAG_CC3;
	else if (timch == TIM_CHANNEL_4)
		return TIM_FLAG_CC4;
		
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

ITCMRAM int dshot_inputmode(struct dshot_device *dev, int ch)
{
	TIM_IC_InitTypeDef sConfigIC = {0};
	
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;

	HAL_DMA_Abort(dev->hdma[ch]);

	__HAL_TIM_DISABLE_DMA(dev->htim[ch], dev->timcc[ch]);

	HAL_TIM_PWM_Stop(dev->htim[ch], dev->timch[ch]);
	dev->htim[ch]->Instance->CCER |= (1U << (4U * (dev->timch[ch] >> 2)));

	HAL_TIM_IC_ConfigChannel(dev->htim[ch],
		&sConfigIC, dev->timch[ch]);

	dev->hdma[ch]->Init.Direction = DMA_PERIPH_TO_MEMORY;
	dev->hdma[ch]->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	
	HAL_DMA_Init(dev->hdma[ch]);

	HAL_TIM_IC_Start(dev->htim[ch], dev->timch[ch]);

	__HAL_TIM_CLEAR_FLAG(dev->htim[ch],
		dshot_timflagcc(dev->timch[ch]));

	__HAL_TIM_SET_AUTORELOAD(dev->htim[ch], 0xffff);

	__HAL_TIM_SET_COUNTER(dev->htim[ch], 0);

	return 0;
}

/**
* @brief DMA callback for dshot tim burst processing.
* @param hdma DMA device that triggered this callback
* @return none
*/
ITCMRAM static void dshotdmacbburst(DMA_HandleTypeDef *hdma)
{
	TIM_HandleTypeDef *htim;;

	htim = (TIM_HandleTypeDef *)((DMA_HandleTypeDef *)hdma)->Parent;
		
	if (hdma == htim->hdma[TIM_DMA_ID_UPDATE])
		__HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
}

/**
* @brief DMA callback for dshot processing.
* @param hdma DMA device that triggered this callback
* @return none
*/
ITCMRAM void dshotdmacb(DMA_HandleTypeDef *hdma)
{
	struct dshot_device *dev;
	int ch;

	dev = dshot_devs;

	if (hdma == dev->hdma[0])		ch = 0;
	else if (hdma == dev->hdma[1])		ch = 1;
	else if (hdma == dev->hdma[2])		ch = 2;
	else if (hdma == dev->hdma[3])		ch = 3;
	else					return;

	if (dev->tele == 0) {
		__HAL_TIM_DISABLE_DMA(dev->htim[ch], (dev->timcc[ch]));
		return;
	}

	dshot_inputmode(dev, ch);

	HAL_DMA_Start_IT(dev->hdma[ch], (uint32_t) dev->timccr[ch],
		(uint32_t) nrzibuf[ch], 20);

	dev->isrx[ch] = 1;

	__HAL_TIM_ENABLE_DMA(dev->htim[ch], dev->timcc[ch]);
}

/**
* @brief Wait for previous DMA transfer to complete
* @param dev DShot device that use DMA
* @return -1, if wait time exceeded, 0 otherwise
*/
ITCMRAM static int dshot_updatedmawait(struct dshot_device *dev)
{
	int t;

	t = 0;
	while (HAL_DMA_GetState(
			dev->htim[0]->hdma[TIM_DMA_ID_UPDATE])
			!= HAL_DMA_STATE_READY && t < 100) {
		udelay(1);
		t += 1;
	}

	if (t >= 100)
		return (-1);

	return 0;
}

ITCMRAM int dshot_outputmode(struct dshot_device *dev)
{
	TIM_OC_InitTypeDef sConfigOC = {0};
	int i;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = dev->tele
		? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		
	for (i = 0; i < 4; ++i) {
		HAL_DMA_Abort(dev->hdma[i]);

		__HAL_TIM_DISABLE_DMA(dev->htim[i], dev->timcc[i]);

		HAL_TIM_IC_Stop(dev->htim[i], dev->timch[i]);

		HAL_TIM_PWM_ConfigChannel(dev->htim[i],
			&sConfigOC, dev->timch[i]);

		dev->hdma[i]->Init.Direction = DMA_MEMORY_TO_PERIPH;
		dev->hdma[i]->Init.FIFOMode = DMA_FIFOMODE_ENABLE;
		
		HAL_DMA_Init(dev->hdma[i]);

		HAL_TIM_PWM_Start(dev->htim[i], dev->timch[i]);

		__HAL_TIM_CLEAR_FLAG(dev->htim[i], dshot_timflagcc(dev->timch[i]));

		__HAL_TIM_SET_AUTORELOAD(dev->htim[i], dev->bitlen - 1);
		__HAL_TIM_SET_COUNTER(dev->htim[i], 0);
	}

	return 0;
}

/**
* @brief Fill buffer, that contains PWM duty cycle values of a
	a DSHOT packet using 16 bit value.
* @param dev DShot device
* @param buf buffer for PWM duty cycle values that is used by DMA
* @param val 16 bit value from 0 to 2047
* @param tele 1 if telemetry bit needed, 0 otherwise
* @param col channel number if tim burst mode is used, -1 otherwise
* @return none
*/
ITCMRAM static void dshotsetbuf(struct dshot_device *dev,
	uint16_t *buf, uint16_t val, int tele, int col)
{
	uint16_t pack;
	unsigned int csum;
	int i;

	// bit after thrust value is telemetry request,
	// so just shift because no telemetry is used
	pack = (val << 1) | tele;

	// calculate dshot checksum
	csum = (pack ^ (pack >> 4) ^ (pack >> 8) ^ (pack >> 12));
	if (dev->tele)
		csum = ~csum;
	csum &= 0xf;

	// append checksum to the packet
	pack = (pack << 4) | csum;

	// construct PWM duty cycle buffer for the packet
	if (col < 0) {
		for (i = 0; i < 16; i++) {
			buf[i] = (pack & 0x8000)
				? dev->onelen : dev->zerolen;
			pack <<= 1;
		}

		// two bits used to maintain space between packets
		buf[16] = 0;
		buf[17] = 0;
	}
	else {
		for (i = 0; i < 16; i++) {
			buf[i * 4 + col] = (pack & 0x8000)
				? dev->onelen : dev->zerolen;
			pack <<= 1;
		}

		// two bits used to maintain space between packets
		buf[16 * 4 + col] = 0;
		buf[17 * 4 + col] = 0;
	}
}

/**
* @brief Fill buffer, that contains PWM duty cycle values of a
	a DSHOT packet with thrust value.
* @param dev DShot device
* @param buf buffer for PWM duty cycle values that is used by DMA
* @param v value from 0.0 to 1.0
* @param col channel number if tim burst mode is used, -1 otherwise
* @return always 0
*/
ITCMRAM static void dshotsetthrust(struct dshot_device *dev,
	uint16_t *buf, float v, int col)
{
	// convert motor thrust value from [0.0;1.0]
	// range to int value in [48; 2047] range
	// disable telemetry request bit
	dshotsetbuf(dev, buf,
		(uint16_t) (trimuff(v) * 1999.0f + 0.5f) + 48, 0, col);
}

int dshot_configure(void *d, const char *cmd, ...)
{
	struct dshot_device *dev;
	DMABUF static uint16_t dsbuf[18];
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
	dshotsetbuf(dev, dsbuf, dcmd, 1, -1);

	// wait for previous DSHOT packet to finish
	udelay(100);

	// send packet with command 10 times. Specification
	// requires 6 for most command, 10 is to be sure.
	for (i = 0; i < 10; ++i) {
		if (dev->mode == DSHOT_TIMBURST) {
			// wait for previous DMA transfer to complete
			dshot_updatedmawait(dev);

			// instruct DMA to the common buffers
			// for all streams corresponding to DSHOT ESCs
			TIM1->DCR = (0 << 8) | (0x0d + n);
			HAL_DMA_Start_IT(
				dev->htim[0]->hdma[TIM_DMA_ID_UPDATE],
				(uint32_t) dsbuf,
				(uint32_t) &TIM1->DMAR, 18);

			// Enable DMA for common stream
			// corresponding to DSHOT ESCs
			__HAL_TIM_ENABLE_DMA(dev->htim[0],
				TIM_DMA_UPDATE);
		}
		else {
			// instruct DMA to use the buffer
			// for stream corresponding to DSHOT output
			HAL_DMA_Start_IT(dev->hdma[n], (uint32_t) dsbuf,
				(uint32_t) (dev->timccr[n]), 18);

			// Enable DMA for stream 
			// corresponding to DSHOT output
			__HAL_TIM_ENABLE_DMA(dev->htim[n], 
				(dev->timcc[n]));
		}

		// wait for previous command packet to finish
		udelay(100);
	}
	
	va_end(args);

	if (dcmd == 12)
		mdelay(35);

	return 0;
}

ITCMRAM int dshot_setthrust(void *d, void *dt, size_t sz)
{
	struct dshot_device *dev;
	struct dshot_data *data;
	
	dev = d;
	data = dt;

	if (dev->tele)
		dshot_outputmode(dev);

	if (dev->mode == DSHOT_TIMBURST) {
		DMABUF static uint16_t mwbuf[18 * 4];

		// construst a PWM duty cycle
		// buffers for dshot ESCs
		dshotsetthrust(dev, mwbuf, data->thrust[0], 0);
		dshotsetthrust(dev, mwbuf, data->thrust[1], 1);
		dshotsetthrust(dev, mwbuf, data->thrust[2], 2);
		dshotsetthrust(dev, mwbuf, data->thrust[3], 3);

		// wait for previous DMA transfer to complete
		dshot_updatedmawait(dev);

		// instruct DMA to the common buffers
		// for all streams corresponding to DSHOT ESCs
		TIM1->DCR = (3 << 8) | (0x0d);
		HAL_DMA_Start_IT(dev->htim[0]->hdma[TIM_DMA_ID_UPDATE],
			(uint32_t) mwbuf, (uint32_t) &TIM1->DMAR,
			18 * 4);

		// Enable DMA for common stream
		// corresponding to DSHOT ESCs
		__HAL_TIM_ENABLE_DMA(dev->htim[0], TIM_DMA_UPDATE);
	}
	else {
		DMABUF static uint16_t dsbuf[4][18];

		// construst a PWM duty cycle
		// buffers for dshot ESCs	
		dshotsetthrust(dev, dsbuf[0], data->thrust[0], -1);
		dshotsetthrust(dev, dsbuf[1], data->thrust[1], -1);
		dshotsetthrust(dev, dsbuf[2], data->thrust[2], -1);
		dshotsetthrust(dev, dsbuf[3], data->thrust[3], -1);

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

		dev->isrx[0] = 0;
		dev->isrx[1] = 0;
		dev->isrx[2] = 0;
		dev->isrx[3] = 0;

		// Enable DMA for streams corresponding to DSHOT ESCs
		__HAL_TIM_ENABLE_DMA(dev->htim[0], dev->timcc[0]);
		__HAL_TIM_ENABLE_DMA(dev->htim[1], dev->timcc[1]);
		__HAL_TIM_ENABLE_DMA(dev->htim[2], dev->timcc[2]);
		__HAL_TIM_ENABLE_DMA(dev->htim[3], dev->timcc[3]);
	}

	return 0;
}

ITCMRAM int dshot_getrps(void *d, void *dt, size_t sz)
{
	static const uint8_t gcr[32] = { 
	    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
	    0xff,    9,   10,   11, 0xff,   13,   14,   15, 
	    0xff, 0xff,    2,    3, 0xff,    5,    6,    7, 
	    0xff,    0,    8,    1, 0xff,    4,    12, 0xff 
	};
	struct dshot_device *dev;
	struct dshot_tele *data;
	int i, j;
	int bitlen;

	dev = d;
	data = dt;

	if (dev->tele == 0)
		return 0;

	bitlen = dev->bitlen * 4 / 5;

	for (i = 0; i < 4; ++i) {
		uint16_t t, cs, ccs;
		uint32_t val;
		uint32_t n1, n2, n3, n4;
		uint32_t per;
		int b;

//		memcpy(data->dsbuf[i], nrzibuf[i], sizeof(uint16_t) * 20);

		data->isvalid[i] = 1;

		b = 0;
		val = 0;
		for (j = 1; j < 20 && nrzibuf[i][j] >= nrzibuf[i][j - 1]; ++j) {
			uint32_t diff;
			int l;

			diff = nrzibuf[i][j] - nrzibuf[i][j - 1];

			l = (diff + bitlen / 2) / bitlen;

			if (l == 0 || (b + l) > 20)
				goto reject;

			val <<= l;
			val |= 1U;

			b += l;
		}

		val <<= 20 - b;

		if ((n1 = gcr[val & 0x1f]) == 0xff)
			goto reject;

		if ((n2 = gcr[(val >> 5) & 0x1f]) == 0xff)
			goto reject;

		if ((n3 = gcr[(val >> 10) & 0x1f]) == 0xff)
			goto reject;

		if ((n4 = gcr[(val >> 15) & 0x1f]) == 0xff)
			goto reject;

		val = n1 | (n2 << 4) | (n3 << 8) | (n4 << 12);

		t = (val >> 4) & 0x0fff;
		cs  = (~val) & 0x000f;
		ccs = (t ^ (t >> 4) ^ (t >> 8) ^ (t >> 12)) & 0xf;

		if (cs != ccs)
			goto reject;

		if (t == 0xfff) {
			data->rps[i] = 0.0f;
			continue;
		}

		per = (t & 0x01ff) << (t >> 9);
	
		if (per == 0)
			goto reject;
	
		data->rps[i] = 1.0e6f / (float) (per * dev->polescount);
		
		continue;

reject:
		data->rps[i] = 0.0f;
		data->isvalid[i] = 0;
	}

	return 0;
}

int dshot_init(struct dshot_device *dev)
{
	struct dshot_data dd;

	// determite period, 0 and 1 duty cycles in used timer's ticks
	if (dev->type == DSHOT_150 || dev->type == DSHOT_150BD) {
		dev->bitlen = DSHOT150_BITLEN * dev->timfreq + 0.5;
		dev->onelen = 0.7 * DSHOT150_BITLEN * dev->timfreq + 0.5;
		dev->zerolen = 0.35 * DSHOT150_BITLEN * dev->timfreq + 0.5;
	}
	else if (dev->type == DSHOT_300 || dev->type == DSHOT_300BD) {
		dev->bitlen = DSHOT300_BITLEN * dev->timfreq + 0.5;
		dev->onelen = 0.7 * DSHOT300_BITLEN * dev->timfreq + 0.5;
		dev->zerolen = 0.35 * DSHOT300_BITLEN * dev->timfreq + 0.5;
	}
	else if (dev->type == DSHOT_600 || dev->type == DSHOT_600BD) {
		dev->bitlen = DSHOT600_BITLEN * dev->timfreq + 0.5;
		dev->onelen = 0.7 * DSHOT600_BITLEN * dev->timfreq + 0.5;
		dev->zerolen = 0.35 * DSHOT600_BITLEN * dev->timfreq + 0.5;
	}
	else {
		dev->bitlen = DSHOT1200_BITLEN * dev->timfreq + 0.5;
		dev->onelen = 0.7 * DSHOT1200_BITLEN * dev->timfreq + 0.5;
		dev->zerolen = 0.35 * DSHOT1200_BITLEN * dev->timfreq + 0.5;
	}

	dev->tele = (dev->type == DSHOT_150BD
		|| dev->type == DSHOT_300BD 
		|| dev->type == DSHOT_600BD
		|| dev->type == DSHOT_1200BD)
			? 1 : 0;

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

	// set dshot DMA callback for each channel, that
	// should stop DMA after packet is sent
	if (dev->mode == DSHOT_TIMBURST) {
		dev->htim[0]->hdma[TIM_DMA_ID_UPDATE]->XferCpltCallback
			= dshotdmacbburst;
	}
	else {
		dev->hdma[0] = dshot_timdma(dev->htim[0], dev->timch[0]);
		dev->hdma[1] = dshot_timdma(dev->htim[1], dev->timch[1]);
		dev->hdma[2] = dshot_timdma(dev->htim[2], dev->timch[2]);
		dev->hdma[3] = dshot_timdma(dev->htim[3], dev->timch[3]);

		dev->hdma[0]->XferCpltCallback = dshotdmacb;
		dev->hdma[1]->XferCpltCallback = dshotdmacb;
		dev->hdma[2]->XferCpltCallback = dshotdmacb;
		dev->hdma[3]->XferCpltCallback = dshotdmacb;
	}

	// start PWM on TIM channels corresponding to DSHOT ESCs
  	HAL_TIM_PWM_Start(dev->htim[0], dev->timch[0]);
  	HAL_TIM_PWM_Start(dev->htim[1], dev->timch[1]);
	HAL_TIM_PWM_Start(dev->htim[2], dev->timch[2]);
	HAL_TIM_PWM_Start(dev->htim[3], dev->timch[3]);	

	dev->isrx[0] = 0;
	dev->isrx[1] = 0;
	dev->isrx[2] = 0;
	dev->isrx[3] = 0;

	if (dev->tele == 0)
		dshot_outputmode(dev);

	// set zero initial thrust for every motor
	dd.thrust[0] = dd.thrust[1] = dd.thrust[2] = dd.thrust[3] = 0.0;
	dshot_setthrust(dev, &dd, sizeof(struct dshot_data));

	return 0;
}

int dshot_initdevice(struct dshot_device *is, struct cdevice *dev)
{
	int r;

	memmove(dshot_devs + dshot_devcount, is,
		sizeof(struct dshot_device));

	sprintf(dev->name, "%s_%d", "dshot", dshot_devcount);

	dev->priv = dshot_devs + dshot_devcount;
	dev->read = dshot_getrps;
	dev->configure = dshot_configure;
	dev->write = dshot_setthrust;
	dev->interrupt = NULL;
	
	r = dshot_init(dshot_devs + dshot_devcount++);

	dev->status = (r == 0) ? DEVSTATUS_INIT : DEVSTATUS_FAILED;

	return r;
}
