#include <math.h>

#include "dsp.h"
#include "global.h"
#include "stm32periph.h"
#include "runvals.h"
#include "util.h"

#include "thrust.h"

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

int dshotcmd(int n, uint16_t cmd)
{
	static uint16_t dsbuf[18];
	DMA_HandleTypeDef *hdma;
	int timcc;
	volatile uint32_t *ccr;
	int i;

	// select DMA device, TIM channel and CCR
	// register corresponding to the DSHOT output
	switch (n) {
	case 0:
		hdma = htim1.hdma[TIM_DMA_ID_CC1];
		timcc = TIM_DMA_CC1;
		ccr = &(htim1.Instance->CCR1);
		break;

	case 1:
		hdma = htim1.hdma[TIM_DMA_ID_CC2];
		timcc = TIM_DMA_CC2;
		ccr = &(htim1.Instance->CCR2);
		break;

	case 2:
		hdma = htim1.hdma[TIM_DMA_ID_CC3];
		timcc = TIM_DMA_CC3;
		ccr = &(htim1.Instance->CCR3);
		break;

	case 3:
		hdma = htim1.hdma[TIM_DMA_ID_CC4];
		timcc = TIM_DMA_CC4;
		ccr = &(htim1.Instance->CCR4);
		break;

	default:
		return 0;
	}

	// construst a PWM duty cycle buffer for dshot ESC
	dshotsetbuf(dsbuf, cmd, 1);

	// wait for previous DSHOT packet to finish
	udelay(100);

	// send packet with command 10 times. Specification
	// requires 6 for most command, 10 is to be sure.
	for (i = 0; i < 10; ++i) {
		// instruct DMA to use the buffer
		// for stream corresponding to DSHOT output
		HAL_DMA_Start_IT(hdma, (uint32_t) dsbuf,
			(uint32_t) ccr, 18);

		// Enable DMA for stream corresponding to DSHOT output
		__HAL_TIM_ENABLE_DMA(&htim1, timcc);

		// wait for previous command packet to finish
		udelay(100);
	}

	return 0;
}

int setthrust(float ltd, float rtd, float lbd, float rbd)
{
	static uint16_t dsbuf[4][18];
	float avgthrust;

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

	if (isnan(ltd) || isnan(rtd) || isnan(rbd)
		|| isnan(lbd) || !elrs)
		ltd = rtd = rbd = lbd = 0.0;

	avgthrust = (ltd + rtd + rbd + lbd) / 4.0;
	avgthrust = avgthrust < 0.0 ? 0.0 : avgthrust;

	dsp_updatelpf(lpf + LPF_AVGTHR, avgthrust);

	// put motors thrust values into log
	log_write(LOG_LT, ltd);
	log_write(LOG_LB, lbd);
	log_write(LOG_RB, rbd);
	log_write(LOG_RT, rtd);

	// construst a PWM duty cycle
	// buffers for dshot ESCs
	dshotsetthrust(dsbuf[st.mtr.lt], ltd);
	dshotsetthrust(dsbuf[st.mtr.lb], lbd);
	dshotsetthrust(dsbuf[st.mtr.rt], rtd);
	dshotsetthrust(dsbuf[st.mtr.rb], rbd);

	// instruct DMA to use the buffers
	// for streams corresponding to DSHOT ESCs
	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC1],
		(uint32_t) dsbuf[0],
		(uint32_t) &(htim1.Instance->CCR1), 18);
	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC2],
		(uint32_t) dsbuf[1],
		(uint32_t) &(htim1.Instance->CCR2), 18);
	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC3],
		(uint32_t) dsbuf[2],
		(uint32_t) &(htim1.Instance->CCR3), 18);
	HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_CC4],
		(uint32_t) dsbuf[3],
		(uint32_t) &(htim1.Instance->CCR4), 18);

	// Enable DMA for streams corresponding to DSHOT ESCs
	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC1);
	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC2);
	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
	__HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC4);

	return 0;
}

void dshotinit()
{
	__HAL_TIM_SET_AUTORELOAD(&htim1, DSHOT_BITLEN);

	// set dshot DMA callback for TIM channels
	// corresponding to DSHOT escs
	htim1.hdma[TIM_DMA_ID_CC1]->XferCpltCallback = dshotdmacb;
	htim1.hdma[TIM_DMA_ID_CC2]->XferCpltCallback = dshotdmacb;
	htim1.hdma[TIM_DMA_ID_CC3]->XferCpltCallback = dshotdmacb;
	htim1.hdma[TIM_DMA_ID_CC4]->XferCpltCallback = dshotdmacb;

	// start PWM on TIM channels corresponding to DSHOT ESCs
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	// set zero initial thrust for every motor
	setthrust(0.0, 0.0, 0.0, 0.0);
}
