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

#include "stm32f4xx_hal.h"

#include "stm32periph.h"
#include "settings.h"
#include "dsp.h"
#include "log.h"
#include "crc.h"
#include "global.h"
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

#include "thrust.h"

/**
* @brief External interrupt callback. It calls interrupt handlers
	from drivers for devices that use external interrupts.
* @param pin number of pin triggered the callback
* @return none
*/
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	if (DEVITENABLED(Dev[DEV_ESP].status)) {
		Dev[DEV_ESP].interrupt(Dev[DEV_ESP].priv, &pin);
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
	if (DEVITENABLED(Dev[DEV_M10].status))
		Dev[DEV_M10].interrupt(Dev[DEV_M10].priv, huart);

	if (DEVITENABLED(Dev[DEV_CRSF].status))
		Dev[DEV_CRSF].interrupt(Dev[DEV_CRSF].priv, huart);

	if (DEVITENABLED(Dev[DEV_UART].status))
		Dev[DEV_UART].interrupt(Dev[DEV_UART].priv, huart);
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
}

/**
* @brief Get battery voltage from ADC.
* @return battery voltage
*/
float batteryvoltage()
{
	uint32_t v;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);

	v = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	return (v / (float) 0xfff * BAT_SCALE);
}

/**
* @brief Get ESC current from ADC.
* @return battery current in amperes
*/
float esccurrent()
{
	uint32_t v;

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1);

	v = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Stop(&hadc2);

	return (v / (float) 0xfff * St.adj.cursc + St.adj.curroff);
}

/**
* @brief Init ESC's.
* @return none
*/
static void esc_init()
{
	dshotinit();
}

/**
* @brief Init HP206c barometer.
* @return none
*/
static void dps_init()
{
	struct dps_device d;

	d.hi2c = &hi2c1;
	d.rate = DPS_RATE_32;
	d.osr = DPS_OSR_16;

	if (dps_initdevice(&d, Dev + DEV_DPS) >= 0)
		uartprintf("DPS368 initilized\r\n");
	else
		uartprintf("failed to initilize DPS368\r\n");
}

/**
* @brief Init IRC tramp video transmitter.
* @return none
*/
static void irc_init()
{
	struct irc_device d;

	d.huart = &huart5;
	d.power = St.irc.power;
	d.frequency = St.irc.freq;

	if (irc_initdevice(&d, Dev + DEV_IRC) >= 0)
		uartprintf("IRC device initilized\r\n");
	else
		uartprintf("failed to initilize IRC device\r\n");
}

/**
* @brief Init ICM-42688-P IMU.
* @return none
*/
static void icm_init()
{
	struct icm_device d;

	d.hspi = &hspi1;
	d.gpio = GPIOC;
	d.pin = GPIO_PIN_13;

	d.gyroscale = ICM_1000DPS;
	d.gyrorate = ICM_GYRO8K;
	d.gyroorder = ICM_GYROORDER3;
	d.gyrolpf = ICM_GYROLPFLL;
	d.accelscale = ICM_4G;
	d.accelrate = ICM_ACCEL8K;
	d.accellpf = ICM_ACCELLPFLL;
	d.accelorder = ICM_ACCELORDER3;

	if (icm_initdevice(&d, Dev + DEV_ICM) >= 0)
		uartprintf("ICM-42688 initilized\r\n");
	else
		uartprintf("failed to initilize ICM-42688\r\n");
}

/**
* @brief Init QMC5883L magnetometer.
* @return none
*/
static void qmc_init()
{
	struct qmc_device d;

	d.hi2c = &hi2c1;
	d.scale = QMC_SCALE_8;
	d.rate = QMC_RATE_100;
	d.osr = QMC_OSR_512;

	if (qmc_initdevice(&d, Dev + DEV_QMC) >= 0)
		uartprintf("QMC5883L initilized\r\n");
	else
		uartprintf("failed to initilize QMC5883L\r\n");
}

/**
* @brief Init ESP8285.
* @return none
*/
static void espdev_init()
{
	struct esp_device d;

	d.hspi = &hspi2;
	d.csgpio = GPIOC;
	d.cspin = GPIO_PIN_0;
	d.rstgpio = GPIOC;
	d.rstpin = GPIO_PIN_14;
	d.bootgpio = GPIOC;
	d.bootpin = GPIO_PIN_15;
	d.busygpio = GPIOB;
	d.busypin = GPIO_PIN_15;
	d.intpin = GPIO_PIN_1;

	if (esp_initdevice(&d, Dev + DEV_ESP) < 0) {
		uartprintf("failed to initilize ESP8266\r\n");
		return;
	}

	uartprintf("ESP8266 initilized\r\n");
}

/**
* @brief Init ERLS receiver driver.
* @return none
*/
static void crsfdev_init()
{
	struct crsf_device d;

	d.huart = &huart2;

	crsf_initdevice(&d, Dev + DEV_CRSF);
}

/**
* @brief Init W25Q onboard flash memory.
* @return none
*/
static void w25dev_init()
{
	struct w25_device d;

	d.hspi = &hspi1;
	d.gpio = GPIOB;
	d.pin = GPIO_PIN_3;

	if (w25_initdevice(&d, &Flashdev) < 0) {
		uartprintf("failed to initilize W25Q\r\n");
		return;
	}

	uartprintf("W25Q initilized\r\n");
}

/**
* @brief Init GNSS module.
* @return none
*/
static void m10dev_init()
{
	struct m10_device d;

	d.huart = &huart3;

	if (m10_initdevice(&d, Dev + DEV_M10) < 0) {
		uartprintf("failed to initilize GPS device\r\n");
		return;
	}

	uartprintf("GPS device initilized\r\n");
}

/**
* @brief Init UART config connection.
* @return none
*/
static void uartdev_init()
{
	struct uart_device d;

	d.huart = &huart4;

	if (uart_initdevice(&d, Dev + DEV_UART) < 0) {
		uartprintf("failed to initilize UART device\r\n");
		return;
	}

	uartprintf("UART device initilized\r\n");
}

/**
* @brief Calculate tilt compensated heading direction
	using magnetometer readings, roll value and pitch value.
* @param r roll value
* @param p pitch value
* @param x magnetometer's X axis value
* @param y magnetometer's Y axis value
* @param z magnetometer's Z axis value
* @return tilt compensated heading
*/
float qmc_heading(float r, float p, float x, float y, float z)
{
	x = x * cosf(p) + y * sinf(r) * sinf(p)
		- z * cosf(r) * sinf(p);
	y = y * cosf(r) + z * sinf(r);

	return circf(atan2f(y, x) + St.adj.magdecl);
}

/**
* @brief Stabilization loop. Callback for TEV_PID periodic event.
	It is the place where is almost all work happening.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int stabilize(int ms)
{
	float ltm, lbm, rbm, rtm;
	float roll, pitch, yaw;
	float rollcor, pitchcor, yawcor, thrustcor;
	float vx, vy, vz;
	float gy, gx, gz;
	float ay, ax, az;
	float va;
	float ht;
	float dt;

	// debug pin switching
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,
		!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)));

	// emergency disarm happened
	if (Emergencydisarm) {
		setthrust(0.0, 0.0, 0.0, 0.0);
		En = 0.0;
	}

	// get time passed from last invocation of this calback function
	dt = ms / (float) TICKSPERSEC;

	// divide-by-zero protection
	dt = (dt < 0.000001) ? 0.000001 : dt;

	// update battery voltage
	log_write(LOG_BAT, dsp_getlpf(Lpf + LPF_BAT));

	// toggle arming indication led
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
		(En > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// get accelerometer and gyroscope readings
	Dev[DEV_ICM].read(Dev[DEV_ICM].priv, &Imudata,
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
	log_write(LOG_ACC_X, Imudata.afx);
	log_write(LOG_ACC_Y, Imudata.afy);
	log_write(LOG_ACC_Z, Imudata.afz);
	log_write(LOG_GYRO_X, Imudata.gfx);
	log_write(LOG_GYRO_Y, Imudata.gfy);
	log_write(LOG_GYRO_Z, Imudata.gfz);

	// update accelerometer temperature
	dsp_updatelpf(Lpf + LPF_IMUTEMP, Imudata.ft);

	// apply accelerometer offsets
	ax = dsp_updatelpf(Lpf + LPF_ACCX, Imudata.afx);
	ay = dsp_updatelpf(Lpf + LPF_ACCY, Imudata.afy);
	az = dsp_updatelpf(Lpf + LPF_ACCZ, Imudata.afz);

	// update vertical acceleration low-pass filter
	dsp_updatelpf(Lpf + LPF_THR, Imudata.afz);

	// convert gyroscope values into radians
	gx = deg2rad(dsp_updatelpf(Lpf + LPF_GYROX, Imudata.gfx));
	gy = deg2rad(dsp_updatelpf(Lpf + LPF_GYROY, Imudata.gfy));
	gz = deg2rad(dsp_updatelpf(Lpf + LPF_GYROZ, Imudata.gfz));

	// update complimenraty filter for roll axis and get next roll
	// value. First signal (value) is signal to be integrated: it's
	// the speed of the rotation around Y axis. Second signal is
	// signal to be low-pass filtered: it's the tilt value that is
	// calculated from acceleromer readings through some
	// trigonometry.
	roll = dsp_updatelpf(Lpf + LPF_ROLL,
		dsp_updatecompl(Cmpl + CMPL_ROLL, gy * dt,
			atan2f(-ax, sqrt(ay * ay + az * az)))
				- St.adj.att0.roll);

	// same as for roll but for different axes
	pitch = dsp_updatelpf(Lpf + LPF_PITCH,
		dsp_updatecompl(Cmpl + CMPL_PITCH, gx * dt,
			atan2f(ay, sqrt(ax * ax + az * az))) 
				- St.adj.att0.pitch);

	// update complimenraty filter for yaw axis and get next yaw
	// value. First signal is the speed of the rotation around Z
	// axis. Second signal is the heading value that is
	// calculated from magnetometer readings.
	yaw = dsp_updatelpf(Lpf + LPF_YAW,
		circf(dsp_updatecirccompl(Cmpl + CMPL_YAW, -gz * dt,
		qmc_heading(roll, -pitch,
			Qmcdata.fx, Qmcdata.fy, Qmcdata.fz)) 
			- St.adj.att0.yaw));

	// calculate gravity direction vector in IMU coordination system
	// using pitch and roll values;
	vx = cos(-pitch) * sin(-roll);
	vy = -sin(-pitch);
	vz = cos(-pitch) * cos(-roll);

	// update vertical acceleration using acceleration
	// vector to gravity vector projection
	va = (vx * ax + vy * ay + vz * az)
		/ sqrtf(vx * vx + vy * vy + vz * vz);

	dsp_updatelpf(Lpf + LPF_VAU, va);
	dsp_updatelpf(Lpf + LPF_VAPT1, va);
	dsp_updatelpf(Lpf + LPF_VAAVG, va);
	
	log_write(LOG_CUSTOM0, dsp_getlpf(Lpf + LPF_VAU));

	vx = sin(-roll) * sin(-pitch);
	vy = cos(-pitch);
	vz = cos(-roll) * sin(-pitch);

	dsp_updatelpf(Lpf + LPF_FA, (vx * ax + vy * ay + vz * az)
		/ sqrtf(vx * vx + vy * vy + vz * vz));

	ht = St.adj.hoverthrottle / (cosf(-pitch) * cosf(-roll));

	// if vertical acceleration is negative, most likely
	// quadcopter is upside down, perform emergency disarm
	if (dsp_getlpf(Lpf + LPF_THR) < -0.5) {
		Emergencydisarm = 1;
		setthrust(0.0, 0.0, 0.0, 0.0);
		En = 0.0;
	}

	// write roll, pitch and yaw values into log
	log_write(LOG_ROLL, roll);
	log_write(LOG_PITCH, pitch);
	log_write(LOG_YAW, yaw);

	if (Speedpid) {
		// if in single PID loop mode for tilt
		// (called accro mode), use only rotation speed values
		// from the gyroscope. Update speed PID controllers for
		// roll and pitch and get next correction values.
		rollcor = dsp_pidbl(Pid + PID_ROLLS, Rolltarget, gy);
		pitchcor = dsp_pidbl(Pid + PID_PITCHS, Pitchtarget, gx);
	}
	else {
		// if in double loop mode for tilt (most commonly used
		// mode), first update roll and pitch POSITION PID
		// controllers using currect roll and values and targets
		// got from ERLS and get next correction values.
		rollcor = dsp_pidbl(Pid + PID_ROLLP, Rolltarget, roll);
		pitchcor = dsp_pidbl(Pid + PID_PITCHP, Pitchtarget, pitch);

		// then use this values to update roll and pitch speed
		// PID controllers and get next SPEED correction values.
		rollcor = dsp_pidbl(Pid + PID_ROLLS, rollcor, gy);
		pitchcor = dsp_pidbl(Pid + PID_PITCHS, pitchcor, gx);
	}

	if (Yawspeedpid) {
		// if single PID loop mode for yaw is used just use
		// rotation speed values around axis Z to upadte yaw PID
		// controller and get next yaw correciton value
		yawcor = dsp_pidbl(Pid + PID_YAWS, Yawtarget, gz);
	}
	else {
		// if in double loop mode for yaw, first use yaw value
		// calcualted using magnetometer and yaw target got from
		// ELRS remote to update yaw POSITION PID controller and
		// get it's next correciton value.
		yawcor = dsp_pidbl(Pid + PID_YAWP, Yawtarget, yaw);

		// then use this value to update yaw speed PID
		// controller and get next yaw SPEED correction value
		yawcor = dsp_pidbl(Pid + PID_YAWS, yawcor, gz);
	}

	if (Altmode == ALTMODE_POS) {
		// if altitude hold mode enabled, first use altitude
		// got from barometer readings and target altitude from
		// ELRS remote to update altitude PID controller and
		// get it's next correction value
		thrustcor = dsp_pidbl(Pid + PID_ALT, Thrust,
			dsp_getcompl(Cmpl + CMPL_ALT) - Alt0);

		// then use altitude correction value and climb rate
		// calculated by complimentary filter (barometer
		// differentiating and accelerometer Z-axis integration)
		// to update climb rate PID controller and get it's next
		// correction value
		thrustcor = dsp_pidbl(Pid + PID_CLIMBRATE, thrustcor,
			dsp_getcompl(Cmpl + CMPL_CLIMBRATE));
		
		// and next use climb rate correction value to update
		// vertial acceleration PID controller and get next
		// thrust correction value
		thrustcor = dsp_pidbl(Pid + PID_VA, thrustcor + 1.0,
			dsp_getlpf(Lpf + LPF_VAPT1)) + ht;	
	}
	else if (Altmode == ALTMODE_SPEED) {
		// if consttant climb rate mode, first use climb rate
		// calculated by complimentary filter (barometer
		// differentiating and accelerometer Z-axis integration)
		// and target climb rate from ELRS remote to update
		// climb rate PID controller and get it's next
		// correction value
		thrustcor = dsp_pidbl(Pid + PID_CLIMBRATE, Thrust,
			dsp_getcompl(Cmpl + CMPL_CLIMBRATE));	

		// and next use climb rate correction value to update
		// vertial acceleration PID controller and get next
		// thrust correction value
		thrustcor = dsp_pidbl(Pid + PID_VA, thrustcor + 1.0,
			dsp_getlpf(Lpf + LPF_VAPT1)) + ht;	
	}
	else {
		if (Hovermode) {
			// if no altitude hold and hover throttle mode
			// is enabled, update vertical acceleration PID
			// controller using next low-pass filtered
			// value of vertical acceleration and target
			// got from ERLS remote
			thrustcor = dsp_pidbl(Pid + PID_VA,
				Thrust + 1.0,
				dsp_getlpf(Lpf + LPF_VAPT1)) + ht;
		}
		else {
			// if no altitude hold and hover throttle mode
			// is disabled, update thrust PID controller
			// using next low-pass filtered value of thrust
			// and target got from ERLS remote
			thrustcor = dsp_pidbl(Pid + PID_VA,
				Thrust + 1.0,
				dsp_getlpf(Lpf + LPF_THR));
		}
	}

	// reset bilinear PID-controllers when disarmed
	if (En < 0.5) {
		dsp_resetpidbl(Pid + PID_PITCHP);
		dsp_resetpidbl(Pid + PID_ROLLP);
		dsp_resetpidbl(Pid + PID_PITCHS);
		dsp_resetpidbl(Pid + PID_ROLLS);
		dsp_resetpidbl(Pid + PID_YAWP);
		dsp_resetpidbl(Pid + PID_YAWS);
		dsp_resetpidbl(Pid + PID_VA);
		dsp_resetpidbl(Pid + PID_CLIMBRATE);
		dsp_resetpidbl(Pid + PID_ALT);
	}

	// disable I-term for all
	// PID-controller, if no throttle
	if ((Altmode == ALTMODE_ACCEL && Thrust < 0
			&& !Hovermode)
		|| (Altmode == ALTMODE_SPEED
			&& Thrust < -0.95 * St.ctrl.climbratemax)
		|| (Altmode == ALTMODE_POS && Thrust < 0.01)) {
		dsp_resetpidbls(Pid + PID_PITCHP);
		dsp_resetpidbls(Pid + PID_ROLLP);
		dsp_resetpidbls(Pid + PID_PITCHS);
		dsp_resetpidbls(Pid + PID_ROLLS);
		dsp_resetpidbls(Pid + PID_YAWP);
		dsp_resetpidbls(Pid + PID_YAWS);
		dsp_resetpidbls(Pid + PID_VA);
		dsp_resetpidbls(Pid + PID_CLIMBRATE);
		dsp_resetpidbls(Pid + PID_ALT);
	}

	// calculate weights for motors
	// thrust calibration values
	ltm = (1.0 + St.adj.mtrsc.r / 2) * (1.0 + St.adj.mtrsc.p / 2);
	rtm = (1.0 - St.adj.mtrsc.r / 2) * (1.0 + St.adj.mtrsc.p / 2);
	lbm = (1.0 + St.adj.mtrsc.r / 2) * (1.0 - St.adj.mtrsc.p / 2);
	rbm = (1.0 - St.adj.mtrsc.r / 2) * (1.0 - St.adj.mtrsc.p / 2);

	// if final thrust is greater than
	// limit set it to the limit
	thrustcor = thrustcor > St.ctrl.thrustmax
		? St.ctrl.thrustmax : thrustcor;

	// update motors thrust based on calculated values. For
	// quadcopter it's enought to split correction in half for
	// 3 pairs of motors: left and right for roll, top and bottom
	// for pitch and two diagonals (spinning in oposite directions)
	// for yaw.
	setthrust(En * ltm * (thrustcor + 0.5 * rollcor
			+ 0.5 * pitchcor - 0.5 * yawcor),
		En * rtm * (thrustcor - 0.5 * rollcor
			+ 0.5 * pitchcor + 0.5 * yawcor),	
		En * lbm * (thrustcor + 0.5 * rollcor
			- 0.5 * pitchcor + 0.5 * yawcor),
		En * rbm * (thrustcor - 0.5 * rollcor
			- 0.5 * pitchcor - 0.5 * yawcor));

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
		setthrust(0.0, 0.0, 0.0, 0.0);
		En = 0.0;
	}

	// since it runs every 1 second update loop counter here too
	Loopscount = Loops;
	Loops = 0;

	return 0;
}

/**
* @brief Get readings from barometer. Callback for
	TEV_DPS periodic event.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int dpsupdate(int ms)
{
	struct dps_data hd;
	float alt;
	static float prevalt = 0.0;
	float dt;

	dt = ms / (float) TICKSPERSEC;

	dt = (dt < 0.000001) ? 0.000001 : dt;

	// if barometer isn't initilized, return
	if (Dev[DEV_DPS].status != DEVSTATUS_INIT)
		return 0;

	// read barometer values
	Dev[DEV_DPS].read(Dev[DEV_DPS].priv, &hd,
		sizeof(struct dps_data));

	// write barometer temperature and altitude values into log
	log_write(LOG_BAR_TEMP, hd.tempf);
	log_write(LOG_BAR_ALT, hd.altf);

	// update altitude low-pass filter and temperature reading
	alt = dsp_updatelpf(Lpf + LPF_ALT, hd.altf);
	dsp_updatelpf(Lpf + LPF_BARTEMP, hd.tempf);

	// calculate climb rate from vertical acceleration and
	// barometric altitude defference using complimentary filter
	dsp_updatecompl(Cmpl + CMPL_CLIMBRATE,
		9.80665 * (dsp_getlpf(Lpf + LPF_VAU) + Goffset - 1.0) * dt,
			(dsp_getcompl(Cmpl + CMPL_ALT) - prevalt) / dt);
	
	// calculate presice altitiude from climb rate and
	// barometric altitude using complimentary filter
	dsp_updatecompl(Cmpl + CMPL_ALT,
		dsp_getcompl(Cmpl + CMPL_CLIMBRATE) * dt,
		alt);
	
	prevalt = dsp_getcompl(Cmpl + CMPL_ALT);

	// write climbrate and altitude values into log
	log_write(LOG_CLIMBRATE, dsp_getcompl(Cmpl + CMPL_CLIMBRATE));
	log_write(LOG_ALT, dsp_getcompl(Cmpl + CMPL_ALT));

	return 0;
}

/**
* @brief Get readings from magnetomer. Callback for TEV_QMC
	periodic event.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int qmcupdate(int ms)
{
	// if magnetometer isn't initilized, return
	if (Dev[DEV_QMC].status != DEVSTATUS_INIT)
		return 0;

	// read magnetometer values
	Dev[DEV_QMC].read(Dev[DEV_QMC].priv, &Qmcdata,
		sizeof(struct qmc_data));

	Qmcdata.fx = St.adj.magsc.x * (Qmcdata.fx + St.adj.mag0.x);
	Qmcdata.fy = St.adj.magsc.y * (Qmcdata.fy + St.adj.mag0.y);
	Qmcdata.fz = St.adj.magsc.z * (Qmcdata.fz + St.adj.mag0.z);

	Qmcdata.fx += St.adj.magthrsc.x * dsp_getlpf(Lpf + LPF_AVGTHR);
	Qmcdata.fy += St.adj.magthrsc.y * dsp_getlpf(Lpf + LPF_AVGTHR);
	Qmcdata.fz += St.adj.magthrsc.z * dsp_getlpf(Lpf + LPF_AVGTHR);

	// write magnetometer values into log
	log_write(LOG_MAG_X, Qmcdata.fx);
	log_write(LOG_MAG_Y, Qmcdata.fy);
	log_write(LOG_MAG_Z, Qmcdata.fz);

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
	log_update();

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

	Tele.lat = Gnss.lat + Gnss.latmin / 60.0;
	Tele.lon = Gnss.lon + Gnss.lonmin / 60.0;
	Tele.speed = Gnss.speed;
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

	snprintf((char *) Tele.mode, 14, "%c|%s|%s|%s",
		En > 0.5 ? 'a' : 'n',
		attmodestr[Speedpid ? 0 : 1],
		yawmodestr[Yawspeedpid ? 0 : 1], am);

	// in case of emergency disarming
	// set flight mode to "stopped"
	if (Emergencydisarm)
		strcpy((char *) Tele.mode, "stopped");

	Dev[DEV_CRSF].write(Dev[DEV_CRSF].priv, &Tele,
		sizeof(struct crsf_tele));

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
/*
		float dx, dy;
	
		Pitchtarget = M_PI / 12.0;
		
		Yawtarget = atan2f(nextpoint->forward.x,
			nextpoint->forward.y);

		dx = nextpoint->forward.x - Points[Curpoint].forward.x;
		dy = nextpoint->forward.y - Points[Curpoint].forward.y;

		if (forwardpath > sqrtf(dx * dx + dy * dy))
*/
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
	log_write(LOG_CRSFCH0, cd->chf[ERLS_CH_ROLL]);
	log_write(LOG_CRSFCH1, cd->chf[ERLS_CH_PITCH]);
	log_write(LOG_CRSFCH2, cd->chf[ERLS_CH_THRUST]);
	log_write(LOG_CRSFCH3, cd->chf[ERLS_CH_YAW]);
	log_write(LOG_CRSFCH4, cd->chf[ERLS_CH_YAWMODE]);
	log_write(LOG_CRSFCH5, cd->chf[ERLS_CH_ATTMODE]);
	log_write(LOG_CRSFCH6, cd->chf[ERLS_CH_THRMODE]);
	log_write(LOG_CRSFCH7, cd->chf[ERLS_CH_ONOFF]);

	// channel 8 on remote is used to turn on/off
	// erls control. If this channel has low state, all remote
	// commands will be ignored, but packet still continue to
	// comming
	Elrs = (cd->chf[ERLS_CH_ONOFF] > 0.5) ? 1 : 0;

	// update ERLS timeout as we got packet
	Elrstimeout = ELRS_TIMEOUT;

	if (!Elrs) {
		En = 0.0;
		setthrust(0.0, 0.0, 0.0, 0.0);

		return 0;
	}

	// get time passed from last ERLS packet
	dt = ms / (float) TICKSPERSEC;

	// channel 16 is used to select settings slot, it is
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

	if (cd->chf[ERLS_CH_AUTOPILOT] > 0.0) {
		if (Autopilot == 0)
			Curpoint = 0;

		Autopilot = 1;
	}
	else
		Autopilot = 0;

	// set magnetometer stabilization mode, if channel 4 has value
	// more than 0, set gyroscope only stabilization mode otherwise
	if (Autopilot) {
	//	Yawtarget = 0.0;
		Yawspeedpid = 0;
	}
	else 
	if (cd->chf[ERLS_CH_YAWMODE] > 0.0) {
		Yawspeedpid = 0;

		// in magnetometer stabilization mode absolute yaw
		// value is stabilized, so target should be integrated
		Yawtarget = circf(Yawtarget
			+ cd->chf[ERLS_CH_YAW] * dt * M_PI
				* St.ctrl.yawrate);
	}
	else {
		Yawspeedpid = 1;
		Yawtarget = -cd->chf[ERLS_CH_YAW]
			* M_PI * St.ctrl.yawrate;
	}

	// set altitude hold mode, if channel 7 has value more than 25,
	// set climbrate stabilization mode if channel 7 has value
	// between -25 and 25, set vertical acceleration mode if channel
	// 7 value is less than -25
	if (Autopilot) {
		Altmode = ALTMODE_POS;
	}
	else if (cd->chf[ERLS_CH_THRMODE] > 0.25) {
		Altmode = ALTMODE_POS;
		Thrust = (cd->chf[ERLS_CH_THRUST] + 1.0)
			/ 2.0 * St.ctrl.altmax;
	}
	else if (cd->chf[ERLS_CH_THRMODE] < -0.25) {
		Altmode = ALTMODE_ACCEL;

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
	else {
		Altmode = ALTMODE_SPEED;
		Thrust = cd->chf[ERLS_CH_THRUST] * St.ctrl.climbratemax;
	}

	// if channel 9 is active (it's no-fix button on remote used
	// for testing), set reference altitude from current altitude
	if (cd->chf[ERLS_CH_ALTCALIB] > 0.0) {
		Alt0 = dsp_getcompl(Cmpl + CMPL_ALT);
		Goffset = 1.0 - dsp_getlpf(Lpf + LPF_VAAVG);
	}

	// set acceleromter stabilization mode, if channel 6 has value
	// more than 25, set gyroscope only stabilization mode, if
	// channel 6 has value between -25 and 25, disarm if channel 6
	// value is less than -25
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
		En = 1;
		Speedpid = 0;

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
		En = 1;
		Speedpid = 1;

		// set pitch/roll targets based on
		// channels 1-2 values (it's a joystick on most remotes)
		Rolltarget = cd->chf[ERLS_CH_ROLL]
			* (M_PI * St.ctrl.rollrate);
		Pitchtarget = -cd->chf[ERLS_CH_PITCH]
			* (M_PI * St.ctrl.pitchrate);
	}

	// disable thrust when motors should be
	// disabled, for additional safety
	if (En < 0.5)
		setthrust(0.0, 0.0, 0.0, 0.0);

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

	Gnss.lonmin = nd->gga.lonmin;
	Gnss.lon = nd->rmc.lon;
	Gnss.londir = (tolower(nd->rmc.londir) == 'e')
		? LONDIR_E : LONDIR_W;

	Gnss.magvar = nd->rmc.magvar;
	Gnss.magvardir = (tolower(nd->rmc.magvardir) == 'e')
		? MAGVARDIR_E : MAGVARDIR_W;

	Gnss.speed = nd->rmc.speed;
	Gnss.course = nd->rmc.course;

	return 0;
}

/**
* @brief Entry point
* @return none
*/
int main(void)
{
	int elrsus;
	int i;

	// initilize HAL
	HAL_Init();

	// initilize stm32 clocks
	systemclock_config();

	// set initial status for all devices to prevent callback
	// calls on corresponding events before inittialization
	for (i = 0; i < DEV_COUNT; ++i)
		Dev[i].status = DEVSTATUS_NOINIT;

	// wait a little to let stm32 periphery and
	// board's devices power on
	HAL_Delay(1000);

	// init stm32 periphery
	periph_init();

	// reading settings from memory current
	// memory slot, which is 0 at start
	readsettings(Curslot);

	// init board's devices
	esc_init();
	icm_init();
	qmc_init();
	espdev_init();
	crsfdev_init();
	w25dev_init();
	m10dev_init();
	uartdev_init();
	dps_init();
	irc_init();

	// initialize stabilization routine
	setstabilize(1);

	// initilize periodic events
	inittimev(Evs + TEV_PID, 0, PID_FREQ, stabilize);
	inittimev(Evs + TEV_CHECK, 0, CHECK_FREQ, checkconnection);
	inittimev(Evs + TEV_DPS, 1 * DPS_FREQ / 5, DPS_FREQ, dpsupdate);
	inittimev(Evs + TEV_QMC, 2 * QMC_FREQ / 5, QMC_FREQ, qmcupdate);
	inittimev(Evs + TEV_LOG, 0, St.log.freq, logupdate);
	inittimev(Evs + TEV_TELE, 3 * TELE_FREQ / 5, 
		TELE_FREQ, telesend);
	inittimev(Evs + TEV_POWER, 4 * POWER_FREQ / 5,
		POWER_FREQ, powercheck);
	inittimev(Evs + TEV_AUTOPILOT, 0, AUTOPILOT_FREQ,
		autopilotupdate);
	
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

	// initilize ERLS timer. For now ERLS polling is not a periodic
	// event and called as frequently as possible, so it needs this
	// separate timer.
	elrsus = 0;

	// main control loop
	while (1) {
		char cmd[CMDSIZE];
		struct crsf_data cd;
		struct m10_data nd;
		int c, i;

		// reset iteration time counter
		__HAL_TIM_SET_COUNTER(&htim8, 0);

		// poll for configuration and telemetry commands
		// from debug wifi connection
		if (Dev[DEV_ESP].read(Dev[DEV_ESP].priv, &cmd,
			CMDSIZE) >= 0) {
			runcommand(Dev + DEV_ESP, cmd);
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
		if (Dev[DEV_M10].read(Dev[DEV_M10].priv, &nd,
			sizeof(struct m10_data)) >= 0) {
			m10msg(&nd);
		}

		// check all periodic events context's and run callbacks
		// if enough time passed. Reset their timers after.
		for (i = 0; i < TEV_COUNT; ++i) {
			if (checktimev(Evs + i)) {
				Evs[i].cb(Evs[i].ms);
				resettimev(Evs + i);
			}
		}

		// get microseconds passed in this iteration
		// one iteration duration should take at least
		// some time, 100us was choosen
		while ((c = __HAL_TIM_GET_COUNTER(&htim8)) < 1);

		// update periodic events timers
		for (i = 0; i < TEV_COUNT; ++i)
			updatetimev(Evs + i, c);

		// update ELRS timer
		elrsus += c;
	}

	return 0;
}
