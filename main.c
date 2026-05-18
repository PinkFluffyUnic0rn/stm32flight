/**
* @file main.c
* @brief Main file with entry point
*/

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>

#include "mcudef.h"

#include "periphconf.h"
#include "settings.h"
#include "dsp.h"
#include "log.h"
#include "controlloop.h"
#include "crc.h"
#include "runvals.h"
#include "timev.h"
#include "command.h"
#include "config.h"
#include "util.h"

#include "device.h"
#include "icm42688.h"
#include "dps368.h"
#include "esp8266.h"
#include "qmc5883l.h"
#include "crsf.h"
#include "w25.h"
#include "m10.h"
#include "uartconf.h"
#include "irc.h"
#include "msp.h"
#include "dshot.h"

/**
* @brief External interrupt callback. It calls interrupt handlers
	from drivers for devices that use external interrupts.
* @param pin number of pin triggered the callback
* @return none
*/
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	if (DEVITENABLED(Dev[DEV_RF].status)) {
		Dev[DEV_RF].interrupt(Dev[DEV_RF].priv, &pin);
	}
}

/**
* @brief UART receive callback. It calls interrupt handlers
	from drivers for devices working through UART.
* @param huart context for UART triggered that callback
* @return none
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (DEVITENABLED(Dev[DEV_GNSS].status))
		Dev[DEV_GNSS].interrupt(Dev[DEV_GNSS].priv, huart);

	if (DEVITENABLED(Dev[DEV_CRSF].status))
		Dev[DEV_CRSF].interrupt(Dev[DEV_CRSF].priv, huart);

	if (DEVITENABLED(Dev[DEV_VTX].status))
		Dev[DEV_VTX].interrupt(Dev[DEV_VTX].priv, huart);

	if (DEVITENABLED(Dev[DEV_UART].status))
		Dev[DEV_UART].interrupt(Dev[DEV_UART].priv, huart);
}

/**
* @brief SPI transmit callback. It calls interrupt handlers
	from drivers for devices working through SPI.
* @param hspi context for SPI triggered that callback
* @return none
*/
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (DEVITENABLED(Flashdev.status))
		Dev[DEV_IMU].interrupt(Flashdev.priv, hspi);
}

/**
* @brief UART error callback. Calls UART error handlers
	from drivers for devices working through UART.
* @param huart context for UART triggered that callback.
* @return none
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (DEVITENABLED(Dev[DEV_CRSF].status))
		Dev[DEV_CRSF].error(Dev[DEV_CRSF].priv, huart);

	if (DEVITENABLED(Dev[DEV_VTX].status))
		Dev[DEV_VTX].error(Dev[DEV_VTX].priv, huart);
}

/**
* @brief Non-maskable interrupt handler.
* @return none
*/
void NMI_Handler(void)
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;

	HAL_GPIO_WritePin(ERROR_GPIO, ERROR_PIN, GPIO_PIN_SET);

	while (1) {}
}

/**
* @brief Hard-fault interrupt handler.
* @return none
*/
void HardFault_Handler(void)
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;

	HAL_GPIO_WritePin(ERROR_GPIO, ERROR_PIN, GPIO_PIN_SET);
	
	while (1) {}
}

/**
* @brief Memory management error interrupt handler.
* @return none
*/
void MemManage_Handler(void)
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;
	
	HAL_GPIO_WritePin(ERROR_GPIO, ERROR_PIN, GPIO_PIN_SET);

	while (1) {}
}

/**
* @brief Bus fault error interrupt handler.
* @return none
*/
void BusFault_Handler(void)
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;
	
	HAL_GPIO_WritePin(ERROR_GPIO, ERROR_PIN, GPIO_PIN_SET);
	
	while (1) {}
}

/**
* @brief Usage fault error interrupt handler.
* @return none
*/
void UsageFault_Handler(void)
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;
	
	HAL_GPIO_WritePin(ERROR_GPIO, ERROR_PIN, GPIO_PIN_SET);

	while (1) {}
}

/**
* @brief HAL error handler.
* @return none
*/
void error_handler(void)
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;

	HAL_GPIO_WritePin(ERROR_GPIO, ERROR_PIN, GPIO_PIN_SET);

	__disable_irq();
	while (1) {}
}

/**
* @brief Get ADC sampled value.
* @param hadc ADC handle
* @param buf write buffer for DMA
* @return samlped value
*/
float adcvalue(ADC_HandleTypeDef *hadc, volatile uint16_t *buf)
{
	uint32_t v;
	int t;

	t = 0;
	while ((HAL_ADC_GetState(hadc) & HAL_ADC_STATE_REG_BUSY) != 0
			&& t < 100000) {
		udelay(10);
		t += 10;
	}

	v = *buf;

	HAL_ADC_Start_DMA(hadc, (uint32_t *) buf, 1);

	return v;
}

/**
* @brief Get battery voltage from ADC.
* @return battery voltage
*/
float batteryvoltage()
{
	ADC_HandleTypeDef *hadc;
	static volatile uint16_t buf = 0;

	hadc = pconf_batteryhadc;

	return (adcvalue(hadc, &buf)
		/ (float) 0xfff * BAT_SCALE * St.adj.batsc);
}

/**
* @brief Get ESC current from ADC.
* @return battery current in amperes
*/
float esccurrent()
{
	ADC_HandleTypeDef *hadc;
	static volatile uint16_t buf = 0;
	
	hadc = pconf_currenthadc;

	return (adcvalue(hadc, &buf)
		/ (float) 0xfff * St.adj.current.scale
		+ St.adj.current.offset);

}

int imuupdate(int ms)
{
	// get accelerometer and gyroscope readings
	Dev[DEV_IMU].read(Dev[DEV_IMU].priv, &Imudata,
		sizeof(struct icm_data));

	// apply accelerometer offsets
	Imudata.afx += (Imudata.ft - 25.0) * St.adj.acctsc.x;
	Imudata.afy += (Imudata.ft - 25.0) * St.adj.acctsc.y;
	Imudata.afz += (Imudata.ft - 25.0) * St.adj.acctsc.z;
	Imudata.afx -= St.adj.acc0.x;
	Imudata.afy -= St.adj.acc0.y;
	Imudata.afz -= St.adj.acc0.z;

	// apply gyroscope offsets
	Imudata.gfx -= St.adj.gyro0.x;
	Imudata.gfy -= St.adj.gyro0.y;
	Imudata.gfz -= St.adj.gyro0.z;

	// write accelerometer and gyroscope values into log
	writelog(LOG_ACC_X, Imudata.afx);
	writelog(LOG_ACC_Y, Imudata.afy);
	writelog(LOG_ACC_Z, Imudata.afz);
	writelog(LOG_GYRO_X, Imudata.gfx);
	writelog(LOG_GYRO_Y, Imudata.gfy);
	writelog(LOG_GYRO_Z, Imudata.gfz);

	// apply accelerometer offsets
	dsp_updatelpf(Lpf + LPF_ACCX, Imudata.afx);
	dsp_updatelpf(Lpf + LPF_ACCY, Imudata.afy);
	dsp_updatelpf(Lpf + LPF_ACCZ, Imudata.afz);

	// convert gyroscope values into radians
	dsp_updatelpf(Lpf + LPF_GYROX, Imudata.gfx);
	dsp_updatelpf(Lpf + LPF_GYROY, Imudata.gfy);
	dsp_updatelpf(Lpf + LPF_GYROZ, Imudata.gfz);

	// update accelerometer temperature
	dsp_updatelpf(Lpf + LPF_IMUTEMP, Imudata.ft);

	return 0;
}

/**
* @brief Postion estimation and stabilization loop. Callback for
* 	TEV_PID periodic event. It is the place where is almost all work
* 	happening.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int stabilize(int ms)
{
	struct corvals cor;
	float dt;
	
	// debug pin switching
	HAL_GPIO_WritePin(debuggpio, debugpin,
		!(HAL_GPIO_ReadPin(debuggpio, debugpin)));

	// emergency disarm happened
	if (Emergencydisarm) {
		setthrust(Dev + DEV_DSHOT, 0.0, 0.0, 0.0, 0.0);
		En = 0.0;
	}

	// get time passed from last invocation of this calback function
	dt = ms / (float) TICKSPERSEC;

	// divide-by-zero protection
	dt = (dt < 0.000001) ? 0.000001 : dt;

	// update battery voltage
	writelog(LOG_BAT, dsp_getlpf(Lpf + LPF_BAT));

	// toggle arming indication led
	HAL_GPIO_WritePin(armgpio, armpin,
		(En > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// update acceleration, speed, position,
	// rotation speed, and rotation for all 3 axes
	updateposition(dt);

	// get correction values for roll, pitch, yaw and thrust
	updatecorrection(dt, &cor);

	// apply thrust to motors
	applythrust(&cor);

	// update loops counter
	++Loops;

	return 0;
}

/**
* @brief Check if ERLS connection is alive, disarm if not. Callback
	for TEV_CHECK periodic event.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int checkconnection(int ms)
{
	// decrease ERLS timeout counter
	if (Elrstimeout != 0)
		--Elrstimeout;

	// if timeout conter reached 0 and no ERLS
	// packet came, disarm immediately
	if (Elrstimeout <= 0) {
		Emergencydisarm = 1;
		setthrust(Dev + DEV_DSHOT, 0.0, 0.0, 0.0, 0.0);
		En = 0.0;
	}

	// since it runs every 1 second update loop counter here too
	Loopscount = Loops;
	Loops = 0;

	return 0;
}

/**
* @brief Get readings from barometer. Callback for
	TEV_BARO periodic event.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int baroupdate(int ms)
{
	// if barometer isn't initilized, return
	if (Dev[DEV_BARO].status != DEVSTATUS_INIT)
		return 0;

	// read barometer values
	Dev[DEV_BARO].read(Dev[DEV_BARO].priv, &Barodata,
		sizeof(struct dps_data));

	// write barometer temperature and altitude values into log
	writelog(LOG_BAR_TEMP, Barodata.tempf);
	writelog(LOG_BAR_ALT, Barodata.altf);

	// update altitude low-pass filter and temperature reading
	dsp_updatelpf(Lpf + LPF_ALT, Barodata.altf);
	dsp_updatelpf(Lpf + LPF_BARTEMP, Barodata.tempf);

	return 0;
}

/**
* @brief Get readings from magnetomer. Callback for TEV_MAG
	periodic event.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int magupdate(int ms)
{
	// if magnetometer isn't initilized, return
	if (Dev[DEV_MAG].status != DEVSTATUS_INIT)
		return 0;
/*
	// read magnetometer values
	Dev[DEV_MAG].read(Dev[DEV_MAG].priv, &Qmcdata,
		sizeof(struct qmc_data));
*/
	Dev[DEV_MAG].read(Dev[DEV_MAG].priv, &Qmcdata,
		sizeof(struct lis_data));

	// apply offsets to magnetometer values
	Qmcdata.fx = St.adj.magsc.x * (Qmcdata.fx + St.adj.mag0.x);
	Qmcdata.fy = St.adj.magsc.y * (Qmcdata.fy + St.adj.mag0.y);
	Qmcdata.fz = St.adj.magsc.z * (Qmcdata.fz + St.adj.mag0.z);

	// apply scaling to magnetometer values
	Qmcdata.fx += St.adj.magthrsc.x * dsp_getlpf(Lpf + LPF_AVGTHR);
	Qmcdata.fy += St.adj.magthrsc.y * dsp_getlpf(Lpf + LPF_AVGTHR);
	Qmcdata.fz += St.adj.magthrsc.z * dsp_getlpf(Lpf + LPF_AVGTHR);

	// write magnetometer values into log
	writelog(LOG_MAG_X, Qmcdata.fx);
	writelog(LOG_MAG_Y, Qmcdata.fy);
	writelog(LOG_MAG_Z, Qmcdata.fz);

	// update magnetometer values lpf
	dsp_updatelpf(Lpf + LPF_MAGX, Qmcdata.fx);
	dsp_updatelpf(Lpf + LPF_MAGY, Qmcdata.fy);
	dsp_updatelpf(Lpf + LPF_MAGZ, Qmcdata.fz);

	return 0;
}

/**
* @brief Update log frame. If buffer isn't full, just move
	buffer pointer, otherwise save buffer content into flash and
	set buffer pointer to 0.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int logupdate(int ms)
{
	updatelog();

	return 0;
}

/**
* @brief Send telemetry through ELRS.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int telesend(int ms)
{
	const char *attmodestr[] = {"ac", "st", "sp", "ps"};
	const char *yawmodestr[] = {"sp", "cs"};
	const char *altmodestr[] = {"tr", "cl", "al"};
	const char *am;

	Tele.bat = dsp_getlpf(Lpf + LPF_BAT);
	Tele.curr = dsp_getlpf(Lpf + LPF_CUR);

	// Determine battery series cells count using it voltage
	// and calculate it's remaining charge in percents
	if (Tele.bat < 6.0)
		Tele.batrem = (Tele.bat - 3.5) / 0.7;
	else if (Tele.bat < 9.0)
		Tele.batrem = (Tele.bat - 7.0) / 1.4;
	else if (Tele.bat < 13.5)
		Tele.batrem = (Tele.bat - 10.5) / 2.1;
	else if (Tele.bat < 18.0)
		Tele.batrem = (Tele.bat - 14) / 2.8;
	else
		Tele.batrem = (Tele.bat - 21) / 3.5;

	Tele.batrem = trimf(100.0 * Tele.batrem, 0.0, 100.0);

	Tele.lat = Gnss.declat;
	Tele.lon = Gnss.declon;
	Tele.speed = dsp_getlpf(Lpf + LPF_SPEED);
//	Tele.speed = Gnss.speed;
	Tele.course = Gnss.course;
	Tele.alt = Gnss.altitude;
	Tele.sats = Gnss.satellites;
	Tele.balt = dsp_getcompl(Cmpl + CMPL_ALT) - Alt0;
	Tele.vspeed = dsp_getcompl(Cmpl + CMPL_CLIMBRATE);
	Tele.roll = dsp_getlpf(Lpf + LPF_ROLL);
	Tele.pitch = dsp_getlpf(Lpf + LPF_PITCH);
	Tele.yaw = dsp_getlpf(Lpf + LPF_YAW);

	// fill flight mode string with arm state and
	// combination of stabilization modes codes
	if (Altmode == ALTMODE_ACCEL)
		am = altmodestr[0];
	else if (Altmode == ALTMODE_SPEED)
		am = altmodestr[1];
	else if (Altmode == ALTMODE_POS)
		am = altmodestr[2];
	else
		am = "---";

	Tele.mode[0] = En > 0.5 ? 'a' : 'n';
	Tele.mode[1] = '|';
	
	if (Gnssmode == GNSSMODE_SPEED)
		memcpy(Tele.mode + 2, attmodestr[2], 2);
	else if (Gnssmode == GNSSMODE_POS)
		memcpy(Tele.mode + 2, attmodestr[3], 2);
	else
		memcpy(Tele.mode + 2, attmodestr[Speedpid ? 0 : 1], 2);
	
	Tele.mode[4] = '|';
	memcpy(Tele.mode + 5, yawmodestr[Yawspeedpid ? 0 : 1], 2);
	Tele.mode[7] = '|';
	memcpy(Tele.mode + 8, am, 2);

	// in case of emergency disarming
	// set flight mode to "stopped"
	if (Emergencydisarm)
		strcpy((char *) Tele.mode, "stopped");

	// perform telemetry write step	on CRSF eLRS
	// reciever to send it back to remote
	Dev[DEV_CRSF].write(Dev[DEV_CRSF].priv, &Tele,
		sizeof(struct crsf_tele));

	Osd.armed = (En > 0.5) ? 1 : 0;
	Osd.attmode = Speedpid ? MSP_ATTMODE_GYRO : MSP_ATTMODE_ACC;
	Osd.yawmode = Yawspeedpid ? MSP_YAWMODE_GYRO : MSP_YAWMODE_MAG;

	if (Altmode == ALTMODE_ACCEL)
		Osd.altmode = MSP_ALTMODE_ACCEL;
	else if (Altmode == ALTMODE_SPEED)
		Osd.altmode = MSP_ALTMODE_SPEED;
	else if (Altmode == ALTMODE_POS)
		Osd.altmode = MSP_ALTMODE_POS;

	if (Gnssmode == GNSSMODE_NONE)
		Osd.gnssmode = MSP_GNSSMODE_NONE;
	else if (Gnssmode == GNSSMODE_SPEED)
		Osd.gnssmode = MSP_GNSSMODE_SPEED;
	else if (Gnssmode == GNSSMODE_POS)
		Osd.gnssmode = MSP_GNSSMODE_POS;

	Osd.bat = Tele.bat;
	Osd.curr = Tele.curr;
	Osd.batrem = Tele.batrem;
	Osd.lat = Tele.lat;
	Osd.lon = Tele.lon;
	Osd.speed = Tele.speed;
	Osd.course = Tele.course;
	Osd.alt = Tele.balt;
	Osd.sats = Tele.sats;
	Osd.vspeed = Tele.vspeed;
	Osd.temp = dsp_getlpf(Lpf + LPF_BARTEMP);
	Osd.roll = Tele.roll;
	Osd.pitch = Tele.pitch;
	Osd.yaw = Tele.yaw;

	// perform OSD draw step on VTX device
	Dev[DEV_VTX].write(Dev[DEV_VTX].priv, &Osd,
		sizeof(struct msp_osd));

	return 0;
}

/**
* @brief Update battery voltage and current.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int powercheck(int ms)
{
	dsp_updatelpf(Lpf + LPF_BAT, batteryvoltage());
	dsp_updatelpf(Lpf + LPF_CUR, esccurrent());

	return 0;
}

/**
* @brief Move to next point in autopilot track.
* @return always 0
*/
int autopilotstep()
{
	Autopilottimer = 0;
	++Curpoint;

	return 0;
}

/**
* @brief Update control values when moving in autopilot mode.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int autopilotupdate(int ms)
{
	struct trackpoint *nextpoint;
	float dt;

	dt = ms / (float) TICKSPERSEC;
	
	// divide-by-zero protection
	dt = (dt < 0.000001) ? 0.000001 : dt;

	if (!Autopilot || Pointscount == 0 || Curpoint < 0
			|| Curpoint >= Pointscount) {
		return 0;
	}

	nextpoint = Points + Curpoint + 1;
	if (nextpoint->type == AUTOPILOT_START)
		Autopilottimer = 0.0;
	else if (nextpoint->type == AUTOPILOT_TAKEOFF) {
		Thrust = Autopilottimer * nextpoint->takeoff.alt
			/ nextpoint->takeoff.t;
		
		Autopilottimer += dt;

		if (Autopilottimer > nextpoint->takeoff.t)
			autopilotstep();
	}
	else if (nextpoint->type == AUTOPILOT_HOVER) {
		Thrust = nextpoint->hover.alt;
		
		Yawtarget = atan2f(nextpoint->hover.x,
			nextpoint->hover.y);
		
		Autopilottimer += dt;

		if (Autopilottimer > nextpoint->hover.t)
			autopilotstep();
	}
	else if (nextpoint->type == AUTOPILOT_LANDING) {
		if (dsp_getcompl(Cmpl + CMPL_ALT) - Alt0 > 5.0)
			Thrust -= dt;
		else if (dsp_getcompl(Cmpl + CMPL_ALT) - Alt0 > 5.0)
			Thrust -= dt * 0.5;
		else if (dsp_getcompl(Cmpl + CMPL_ALT) - Alt0 > 2.0)
			Thrust -= dt * 0.25;
		else
			Thrust -= dt * 0.125;
		
		if (dsp_getcompl(Cmpl + CMPL_ALT) - Alt0 <= 0.1)
			autopilotstep();
	}
	else if (nextpoint->type == AUTOPILOT_STOP) {
		Thrust = -1.0;
	}
	else if (nextpoint->type == AUTOPILOT_FORWARD) {
		autopilotstep();
	}

	return 0;
}

/**
* @brief Set control values using CRSF packet.
* @param cd -- CRSF packet
* @param ms -- microsecond passed from last CRSF packet
* @return always 0
*/
int crsfcmd(const struct crsf_data *cd, int ms)
{
	static float slottimeout = ELRS_PUSHTIMEOUT;
	float dt;
	int slot;

	// write first 8 channels values into log	
	writelog(LOG_CRSFCH0, cd->chf[ERLS_CH_ROLL]);
	writelog(LOG_CRSFCH1, cd->chf[ERLS_CH_PITCH]);
	writelog(LOG_CRSFCH2, cd->chf[ERLS_CH_THRUST]);
	writelog(LOG_CRSFCH3, cd->chf[ERLS_CH_YAW]);
	writelog(LOG_CRSFCH4, cd->chf[ERLS_CH_YAWMODE]);
	writelog(LOG_CRSFCH5, cd->chf[ERLS_CH_ATTMODE]);
	writelog(LOG_CRSFCH6, cd->chf[ERLS_CH_THRMODE]);
	writelog(LOG_CRSFCH7, cd->chf[ERLS_CH_ONOFF]);

	// ONOFF channel on remote is used to turn on/off
	// erls control. If this channel has low state, all remote
	// commands will be ignored, but packet still continue to
	// comming
	Elrs = (cd->chf[ERLS_CH_ONOFF] > 0.5) ? 1 : 0;

	// update ERLS timeout as we got packet
	Elrstimeout = ELRS_TIMEOUT;

	if (!Elrs) {
		En = 0.0;
		setthrust(Dev + DEV_DSHOT, 0.0, 0.0, 0.0, 0.0);

		return 0;
	}

	// get time passed from last ERLS packet
	dt = ms / (float) TICKSPERSEC;

	// SETSLOT channel is used to select settings slot, it is
	// a 6 position switch on remote used for testing
	if (cd->chf[ERLS_CH_SETSLOT] < -0.8)		slot = 0;
	else if (cd->chf[ERLS_CH_SETSLOT] < -0.4)	slot = 1;
	else if (cd->chf[ERLS_CH_SETSLOT] < 0)		slot = 2;
	else if (cd->chf[ERLS_CH_SETSLOT] < 0.4)	slot = 3;
	else if (cd->chf[ERLS_CH_SETSLOT] < 0.8)	slot = 4;
	else						slot = 5;

	// slot changed differ from current, wait some time to
	// ensure that it's not a some error, if enough time passed,
	// load settings from choosen slot
	if (slot != Curslot) {
		slottimeout -= dt;

		if (slottimeout <= 0.0) {
			readsettings(slot);
			setstabilize(0);

			Curslot = slot;
		}
	} else
		slottimeout = ELRS_PUSHTIMEOUT;

	// AUTOPILOT channel is used to turn on/off autopilot
	// if it is greater than 0, autopilot is on, otherwise
	// it is turned off
	Autopilot = 0;
/*
	if (cd->chf[ERLS_CH_AUTOPILOT] > 0.0) {
		if (Autopilot == 0)
			Curpoint = 0;

		Autopilot = 1;
	}
	else
		Autopilot = 0;
*/		

	// set magnetometer stabilization mode, if YAWMODE channel has
	// value more than 0, set gyroscope only stabilization mode 
	// otherwise
	if (Autopilot) {
	//	Yawtarget = 0.0;
		Yawspeedpid = 0;
	}
	else if (cd->chf[ERLS_CH_YAWMODE] > 0.0
			&& Dev[DEV_MAG].status == DEVSTATUS_INIT) {
		Yawspeedpid = 0;

		// in magnetometer stabilization mode absolute yaw
		// value is stabilized, so target should be integrated
		Yawtarget = circf(Yawtarget
			+ cd->chf[ERLS_CH_YAW] * dt * M_PI
				* St.ctrl.yawrate);
	}
	else {
		Yawspeedpid = 1;
		Yawtarget = cd->chf[ERLS_CH_YAW]
			* M_PI * St.ctrl.yawrate;
	}

	// set altitude hold mode, if THRMODE channel has value more
	// than 25, set climbrate stabilization mode if THRMODE channel
	// has value between -25 and 25, set vertical acceleration mode
	// if THRMODE channel value is less than -25
	if (Autopilot) {
		Altmode = ALTMODE_POS;
	}
	else if (cd->chf[ERLS_CH_THRMODE] > 0.25
			&& Dev[DEV_BARO].status == DEVSTATUS_INIT) {
		Altmode = ALTMODE_POS;
		Thrust = (cd->chf[ERLS_CH_THRUST] + 1.0)
			/ 2.0 * St.ctrl.altmax;
	}
	else if (cd->chf[ERLS_CH_THRMODE] > -0.25
			&& Dev[DEV_BARO].status == DEVSTATUS_INIT) {
		Altmode = ALTMODE_SPEED;
		Thrust = cd->chf[ERLS_CH_THRUST] * St.ctrl.climbratemax;
	}
	else {
		Altmode = ALTMODE_ACCEL;

		// Set throttle mode usging HOVER channel,
		// if it is less that 0.0, use throttle setpoint
		// range starting slightly less than 0,
		// otherwise use throttle setpoint range with
		// 0 value in middle
		if (cd->chf[ERLS_CH_HOVER] < 0.0) {
			Hovermode = 0;
			Thrust = (cd->chf[ERLS_CH_THRUST] + 0.5)
				/ 1.5 * St.ctrl.accelmax;
		}
		else {
			Hovermode = 1;
			Thrust = (cd->chf[ERLS_CH_THRUST])
				/ 2.0 * St.ctrl.accelmax;
		}
	}

	// if ALTCALIB channel is active (it's no-fix button on remote
	// used for testing), set reference altitude from current
	// altitude
	if (cd->chf[ERLS_CH_ALTCALIB] > 0.0) {
		Alt0 = dsp_getcompl(Cmpl + CMPL_ALT);
		Lat0 = Gnss.declat;
		Lon0 = Gnss.declon;
		Goffset = 1.0 - dsp_getlpf(Lpf + LPF_VAAVG);
	}

	// set acceleromter stabilization mode, if ATTMODE channel has
	// value more than 25, set gyroscope only stabilization mode, if
	// ATTMODE channel has value between -25 and 25, disarm if
	// channel ATTMODE value is less than -25
/*
	if (autopilot) {
		en = 1;
		speedpid = 0;

		Rolltarget = cd->chf[ERLS_CH_ROLL]
			* (M_PI * St.rollmax);
	}
	else 
	*/	
	if (cd->chf[ERLS_CH_ATTMODE] > 0.25) {
		int changed;

		changed = (En == 0 || Speedpid == 1);

		En = 1;
		Speedpid = 0;
		
		if (changed)	
			setstabilize(0);

		// set pitch/roll targets based on
		// channels 1-2 values (it's a joystick on most remotes)
		Rolltarget = cd->chf[ERLS_CH_ROLL]
			* (M_PI * St.ctrl.rollmax);
		Pitchtarget = -cd->chf[ERLS_CH_PITCH]
			* (M_PI * St.ctrl.pitchmax);
	}
	else if (cd->chf[ERLS_CH_ATTMODE] < -0.25) {
		En = 0;
		Speedpid = 0;
	}
	else {
		int changed;

		changed = (En == 0 || Speedpid == 0);

		En = 1;
		Speedpid = 1;
		
		if (changed)	
			setstabilize(0);

		// set pitch/roll targets based on
		// channels 1-2 values (it's a joystick on most remotes)
		Rolltarget = cd->chf[ERLS_CH_ROLL]
			* (M_PI * St.ctrl.rollrate);
		Pitchtarget = -cd->chf[ERLS_CH_PITCH]
			* (M_PI * St.ctrl.pitchrate);
	}

	if (cd->chf[ERLS_CH_GNSSMODE] > 0.25
			&& Dev[DEV_GNSS].status == DEVSTATUS_INIT
			&& M10_HASFIX(Gnss.quality)) {
		Gnssmode = GNSSMODE_POS;

		Rolltarget += dt * cd->chf[ERLS_CH_ROLL];
		Pitchtarget += dt * cd->chf[ERLS_CH_PITCH];
	}
	else if (cd->chf[ERLS_CH_GNSSMODE] > -0.25
			&& Dev[DEV_GNSS].status == DEVSTATUS_INIT
			&& M10_HASFIX(Gnss.quality)) {
		Gnssmode = GNSSMODE_SPEED;

		Rolltarget = cd->chf[ERLS_CH_ROLL] * 4.0;
		Pitchtarget = cd->chf[ERLS_CH_PITCH] * 4.0;
	}
	else {
		Gnssmode = GNSSMODE_NONE;
	}

	// disable thrust when motors should be
	// disabled, for additional safety
	if (En < 0.5)
		setthrust(Dev + DEV_DSHOT, 0.0, 0.0, 0.0, 0.0);

	return 0;
}

/**
* @brief Store data got from NMEA message.
* @param nd data got from GNSS device through NMEA protocol
* @return always 0
*/
int m10msg(struct m10_data *nd)
{
	// got only altitude and signal quality data from GGA messages.
	if (nd->type == M10_TYPE_GGA) {
		Gnss.altitude = nd->gga.alt;
		Gnss.quality = nd->gga.quality;
		Gnss.satellites = nd->gga.sats;

		// write GGA values into log
		writelog(LOG_GNSS_ALT, Gnss.altitude);
		writelog(LOG_GNSS_QUAL, Gnss.quality);
		writelog(LOG_GNSS_SATS, Gnss.satellites);

		return 0;
	}

	// other needed data is got from RMC
	// messages. Discard all other messages.
	if (nd->type != M10_TYPE_RMC)
		return 0;

	Gnss.time = nd->rmc.time;
	memcpy(Gnss.date, nd->rmc.date, 10);

	Gnss.latmin = nd->rmc.latmin;
	Gnss.lat = nd->rmc.lat;
	Gnss.latdir = (tolower(nd->rmc.latdir) == 'n')
		? LATDIR_N : LATDIR_S;
	Gnss.declat = Gnss.lat + Gnss.latmin / 60.0;

	Gnss.lonmin = nd->gga.lonmin;
	Gnss.lon = nd->rmc.lon;
	Gnss.londir = (tolower(nd->rmc.londir) == 'e')
		? LONDIR_E : LONDIR_W;
	Gnss.declon = Gnss.lon + Gnss.lonmin / 60.0;

	Gnss.magvar = nd->rmc.magvar;
	Gnss.magvardir = (tolower(nd->rmc.magvardir) == 'e')
		? MAGVARDIR_E : MAGVARDIR_W;

	Gnss.speed = nd->rmc.speed;
	Gnss.course = circf(deg2rad(nd->rmc.course));

	// write RMC values into log
	writelog(LOG_GNSS_LAT, Gnss.declat);
	writelog(LOG_GNSS_LON, Gnss.declon);
	writelog(LOG_GNSS_SPEED, Gnss.speed);
	writelog(LOG_GNSS_COURSE, Gnss.course);

	// Calculate distance from starting point
	// through latitude and longitude in meters
	dsp_updatelpf(Lpf + LPF_LATM, (Gnss.declat - Lat0) * 111320);
	dsp_updatelpf(Lpf + LPF_LONM,
		(Gnss.declon - Lon0) * 111320 * cosf(deg2rad(Lat0)));

	writelog(LOG_CUSTOM0, dsp_getlpf(Lpf + LPF_LATM));
	writelog(LOG_CUSTOM1, dsp_getlpf(Lpf + LPF_LONM));
	writelog(LOG_CUSTOM2, deg2rad(Gnss.course));

	return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	static int elrsus = 0;

	char cmd[CMDSIZE];
	struct crsf_data cd;
	struct m10_data nd;
	int c, i;

	if (htim->Instance != pconf_schedhtim->Instance)
		return;

	if (!Init)
		return;

	c = TICKSPERSEC / PID_FREQ;

	// update periodic events timers
	for (i = 0; i < TEV_COUNT; ++i)
		updatetimev(Evs + i, c);

	// update ELRS timer
	elrsus += c;

	// check all periodic events context's and run callbacks
	// if enough time passed. Reset their timers after.
	for (i = 0; i < TEV_COUNT; ++i) {
		if (checktimev(Evs + i))
			runtimev(Evs + i);
	}

	// poll for configuration and telemetry commands
	// from debug wifi connection
	if (Dev[DEV_RF].read(Dev[DEV_RF].priv, &cmd,
			CMDSIZE) >= 0) {
		runcommand(Dev + DEV_RF, cmd);
	}

	// poll for configuration and telemtry commands
	// from debug uart connection
	if (Dev[DEV_UART].read(Dev[DEV_UART].priv, &cmd,
			UART_CMDSIZE) >= 0) {
		runcommand(Dev + DEV_UART, cmd);
	}

	// read the ELRS remote's packet
	if (Dev[DEV_CRSF].read(Dev[DEV_CRSF].priv, &cd,
			sizeof(struct crsf_data)) >= 0) {
		crsfcmd(&cd, elrsus);
		elrsus = 0;
	}

	// check the M10 messages
	if (Dev[DEV_GNSS].read(Dev[DEV_GNSS].priv, &nd,
			sizeof(struct m10_data)) >= 0) {
		m10msg(&nd);
	}
}

/**
* @brief Entry point
* @return always 0
*/
int main(void)
{
	// init periphery
	pconf_init(error_handler);
	
	// reading settings from memory current
	// memory slot, which is 0 at start
	readsettings(Curslot);

	// set IRC VTX power and frequency using values from settings
	updatevtx();

	// initialize stabilization routine
	setstabilize(1);

	// initilize periodic events
	inittimev(Evs + TEV_IMU, 0, PID_FREQ, imuupdate);
	inittimev(Evs + TEV_LOG, 0, St.log.freq,  logupdate);
	inittimev(Evs + TEV_PID, 0, PID_FREQ, stabilize);
	inittimev(Evs + TEV_CHECK,
		1 * PID_FREQ / TEV_COUNT * (TICKSPERSEC / PID_FREQ),
		CHECK_FREQ, checkconnection);
	inittimev(Evs + TEV_BARO,
		2 * PID_FREQ / TEV_COUNT * (TICKSPERSEC / PID_FREQ),
		DPS_FREQ, baroupdate);
	inittimev(Evs + TEV_MAG,
		3 * PID_FREQ / TEV_COUNT * (TICKSPERSEC / PID_FREQ),
		QMC_FREQ, magupdate);
	inittimev(Evs + TEV_TELE,
		5 * PID_FREQ / TEV_COUNT * (TICKSPERSEC / PID_FREQ),
		TELE_FREQ, telesend);
	inittimev(Evs + TEV_POWER,
		6 * PID_FREQ / TEV_COUNT * (TICKSPERSEC / PID_FREQ),
		POWER_FREQ, powercheck);
	inittimev(Evs + TEV_AUTOPILOT,
		7 * PID_FREQ / TEV_COUNT * (TICKSPERSEC / PID_FREQ),
		AUTOPILOT_FREQ, autopilotupdate);

	// initilize debug commands
	addcommand("r", rcmd);
	addcommand("info", infocmd);
	addcommand("flash", flashcmd);
	addcommand("system", systemcmd);
	addcommand("log", logcmd);
	addcommand("motor", motorcmd);
	addcommand("autopilot", autopilotcmd);
	addcommand("set", setcmd);
	addcommand("get", getcmd);
	addcommand("apply", applycmd);

	__HAL_TIM_SET_COUNTER(pconf_schedhtim, 0);

	Init = 1;

	// main control loop
	while (1) { }

	return 0;
}
