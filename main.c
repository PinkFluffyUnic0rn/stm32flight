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
* @defgroup MODESTR
* @brief Stabilization modes short codes that is
	sent to eLRS remote inside telemetry packets
* @{
*/
const char *attmodestr[] = {"ac", "st", "sp", "ps"}; /*!< attitude mode */
const char *yawmodestr[] = {"sp", "cs"};	/*!< yaw mode */
const char *altmodestr[] = {"tr", "cl", "al"};	/*!< altitude mode */
/**
* @}
*/

/**
* @brief External interrupt callback. It calls interrupt handlers
	from drivers for devices that use external interrupts.
* @param pin number of pin triggered the callback
* @return none
*/
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	if (DEVITENABLED(dev[ESP_DEV].status)) {
		dev[ESP_DEV].interrupt(dev[ESP_DEV].priv, &pin);
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
	if (DEVITENABLED(dev[M10_DEV].status))
		dev[M10_DEV].interrupt(dev[M10_DEV].priv, huart);

	if (DEVITENABLED(dev[CRSF_DEV].status))
		dev[CRSF_DEV].interrupt(dev[CRSF_DEV].priv, huart);

	if (DEVITENABLED(dev[UART_DEV].status))
		dev[UART_DEV].interrupt(dev[UART_DEV].priv, huart);
}

/**
* @brief UART error callback. Calls UART error handlers
	from drivers for devices working through UART.
* @param huart context for UART triggered that callback.
* @return none
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (DEVITENABLED(dev[CRSF_DEV].status))
		dev[CRSF_DEV].error(dev[CRSF_DEV].priv, huart);
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

	return (v / (float) 0xfff * st.currscale + st.curroffset);
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

	if (dps_initdevice(&d, dev + DPS_DEV) >= 0)
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
	d.power = st.ircpower;
	d.frequency = st.ircfreq;

	if (irc_initdevice(&d, dev + IRC_DEV) >= 0)
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

	if (icm_initdevice(&d, dev + ICM_DEV) >= 0)
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

	if (qmc_initdevice(&d, dev + QMC_DEV) >= 0)
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

	if (esp_initdevice(&d, dev + ESP_DEV) < 0) {
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

	crsf_initdevice(&d, dev + CRSF_DEV);
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

	if (w25_initdevice(&d, &flashdev) < 0) {
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

	if (m10_initdevice(&d, dev + M10_DEV) < 0) {
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

	if (uart_initdevice(&d, dev + UART_DEV) < 0) {
		uartprintf("failed to initilize UART device\r\n");
		return;
	}

	uartprintf("UART device initilized\r\n");
}

/**
* @brief Init/set stabilization loop.
* @param init 1, if called during initilization, 0 otherwise
* @return always 0
*/
int setstabilize(int init)
{
	// init complementary filters contexts
	dsp_setcompl(&pitchcompl, st.atctcoef, PID_FREQ, init);
	dsp_setcompl(&rollcompl, st.atctcoef, PID_FREQ, init);
	dsp_setcompl(&yawcompl, st.yctcoef, PID_FREQ, init);

	dsp_setcompl(&climbratecompl, st.cctcoef, DPS_FREQ, init);
	dsp_setcompl(&altcompl, st.actcoef, DPS_FREQ, init);

	// init roll and pitch position PID controller contexts
	dsp_setpidbl(&pitchpv, st.p, st.i, st.d, PID_MAX_I, st.dpt1freq,
		PID_FREQ, init);
	dsp_setpidbl(&rollpv, st.p, st.i, st.d, PID_MAX_I, st.dpt1freq,
		PID_FREQ, init);

	// init roll, pitch and yaw speed PID controller contexts
	dsp_setpidbl(&pitchspv, st.sp, st.si, st.sd,
		PID_MAX_I, st.dpt1freq, PID_FREQ, init);
	dsp_setpidbl(&rollspv, st.sp, st.si, st.sd,
		PID_MAX_I, st.dpt1freq, PID_FREQ, init);
	dsp_setpidbl(&yawspv, st.ysp, st.ysi, st.ysd,
		PID_MAX_I, st.dpt1freq, PID_FREQ, init);

	// init yaw position PID controller's context
	dsp_setpid(&yawpv, st.yp, st.yi, st.yd, st.dpt1freq,
		PID_FREQ, init);

	// init vertical acceleration PID controller's context
	dsp_setpidbl(&tpv, st.zsp, st.zsi, st.zsd, PID_MAX_I,
		st.dpt1freq, PID_FREQ, init);

	// init climbrate PID controller's context
	dsp_setpidbl(&cpv, st.cp, st.ci, st.cd, PID_MAX_I,
		st.dpt1freq, PID_FREQ, init);

	// init altitude PID controller's context
	dsp_setpidbl(&apv, st.ap, st.ai, st.ad, PID_MAX_I,
		st.dpt1freq, PID_FREQ, init);

	// init battery voltage low-pass filter
	dsp_setlpf1f(&batlpf, BAT_CUTOFF, POWER_FREQ, init);
	dsp_setlpf1f(&currlpf, CUR_CUTOFF, POWER_FREQ, init);

	// init average thrust low-pass filter
	dsp_setunity(&avgthrustlpf, init);

	// init low-pass fitlers for altitude and vertical acceleration
	dsp_setunity(&templpf, init);
	dsp_setunity(&valpf, init);
	dsp_setlpf1t(&tlpf, st.ttcoef, PID_FREQ, init);
	dsp_setlpf1t(&vtlpf, st.ttcoef, PID_FREQ, init);
	dsp_setlpf1t(&volpf, VA_AVG_TCOEF, PID_FREQ, init);
	dsp_setunity(&flpf, init);
	dsp_setunity(&altlpf, init);

	// init low-pass fitlers for IMU temperature sensor
	dsp_setlpf1t(&atemppt1, TEMP_TCOEF, PID_FREQ, init);

	// init low-pass fitlers for accelerometer x, y and z axes
	dsp_setlpf1f(&accxpt1, st.accpt1freq, PID_FREQ, init);
	dsp_setlpf1f(&accypt1, st.accpt1freq, PID_FREQ, init);
	dsp_setlpf1f(&acczpt1, st.accpt1freq, PID_FREQ, init);

	// init low-pass fitlers for gyroscope x, y and z axes
	dsp_setlpf1f(&gyroxpt1, st.gyropt1freq, PID_FREQ, init);
	dsp_setlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ, init);
	dsp_setlpf1f(&gyrozpt1, st.gyropt1freq, PID_FREQ, init);

	// init low-pass fitlers for magnetometer x, y and z axes
	dsp_setlpf1t(&magxpt1, st.magpt1freq, QMC_FREQ, init);
	dsp_setlpf1t(&magypt1, st.magpt1freq, QMC_FREQ, init);
	dsp_setlpf1t(&magzpt1, st.magpt1freq, QMC_FREQ, init);

	return 0;
}

int initautopilot()
{
/*
	points[0].type = AUTOPILOT_POINT_START;

	points[1].takeoff.alt = 1.5;
	points[1].takeoff.t = 3.0;
	points[1].type = AUTOPILOT_POINT_TAKEOFF;

	points[2].hover.x = 0.0;
	points[2].hover.y = 1.0;
	points[2].hover.alt = 1.5;
	points[2].hover.t = 1.0;
	points[2].type = AUTOPILOT_POINT_HOVER;

	points[3].type = AUTOPILOT_POINT_LANDING;
	points[4].type = AUTOPILOT_POINT_STOP;

	pointscount = 5;
*/
	pointscount = 0;

	return 0;
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
	x = st.mxsc * (x + st.mx0);
	y = st.mysc * (y + st.my0);
	z = st.mzsc * (z + st.mz0);

	x = x * cosf(p) + y * sinf(r) * sinf(p)
		- z * cosf(r) * sinf(p);
	y = y * cosf(r) + z * sinf(r);

	return circf(atan2f(y, x) + st.magdecl);
}

/**
* @brief Stabilization loop. Callback for TEV_PID periodic event.
	It is the place where is almost all work happening.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int stabilize(int ms)
{
	struct icm_data id;
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
	if (emergencydisarm) {
		setthrust(0.0, 0.0, 0.0, 0.0);
		en = 0.0;
	}

	// get time passed from last invocation of this calback function
	dt = ms / (float) TICKSPERSEC;

	// divide-by-zero protection
	dt = (dt < 0.000001) ? 0.000001 : dt;

	// update battery voltage
	log_write(LOG_BAT, dsp_getlpf(&batlpf));

	// toggle arming indication led
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
		(en > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// get accelerometer and gyroscope readings
	dev[ICM_DEV].read(dev[ICM_DEV].priv, &id,
		sizeof(struct icm_data));

	id.afz += (id.ft - 25.0) * st.aztscale;

	// write accelerometer and gyroscope values into log
	log_write(LOG_ACC_X, id.afx);
	log_write(LOG_ACC_Y, id.afy);
	log_write(LOG_ACC_Z, id.afz);
	log_write(LOG_GYRO_X, id.gfx);
	log_write(LOG_GYRO_Y, id.gfy);
	log_write(LOG_GYRO_Z, id.gfz);

	// update accelerometer temperature
	dsp_updatelpf(&atemppt1, id.ft);

	// apply accelerometer offsets
	ax = dsp_updatelpf(&accxpt1, id.afx) - st.ax0;
	ay = dsp_updatelpf(&accypt1, id.afy) - st.ay0;
	az = dsp_updatelpf(&acczpt1, id.afz) - st.az0;

	// update vertical acceleration low-pass filter
	dsp_updatelpf(&tlpf, id.afz);

	// offset gyroscope readings by values, calculater
	// at power on and convert result into radians
	gx = deg2rad(dsp_updatelpf(&gyroxpt1, id.gfx) - st.gx0);
	gy = deg2rad(dsp_updatelpf(&gyroypt1, id.gfy) - st.gy0);
	gz = deg2rad(dsp_updatelpf(&gyrozpt1, id.gfz) - st.gz0);

	// update complimenraty filter for roll axis and get next roll
	// value. First signal (value) is signal to be integrated: it's
	// the speed of the rotation around Y axis. Second signal is
	// signal to be low-pass filtered: it's the tilt value that is
	// calculated from acceleromer readings through some
	// trigonometry.
	roll = dsp_updatecompl(&rollcompl, gy * dt,
		atan2f(-ax, sqrt(ay * ay + az * az))) - st.roll0;

	// same as for roll but for different axes
	pitch = dsp_updatecompl(&pitchcompl, gx * dt,
		atan2f(ay, sqrt(ax * ax + az * az))) - st.pitch0;

	// update complimenraty filter for yaw axis and get next yaw
	// value. First signal is the speed of the rotation around Z
	// axis. Second signal is the heading value that is
	// calculated from magnetometer readings.
	
	yaw = circf(dsp_updatecirccompl(&yawcompl, -gz * dt,
		qmc_heading(roll, -pitch,
			qmcdata.fx, qmcdata.fy, qmcdata.fz)) - st.yaw0);

	// calculate gravity direction vector in IMU coordination system
	// using pitch and roll values;
	vx = cos(-pitch) * sin(-roll);
	vy = -sin(-pitch);
	vz = cos(-pitch) * cos(-roll);

	// update vertical acceleration using acceleration
	// vector to gravity vector projection
	va = (vx * ax + vy * ay + vz * az)
		/ sqrtf(vx * vx + vy * vy + vz * vz);

	dsp_updatelpf(&valpf, va);
	dsp_updatelpf(&vtlpf, va);
	dsp_updatelpf(&volpf, va);
	
	log_write(LOG_CUSTOM0, dsp_getlpf(&valpf));

	vx = sin(-roll) * sin(-pitch);
	vy = cos(-pitch);
	vz = cos(-roll) * sin(-pitch);

	dsp_updatelpf(&flpf, (vx * ax + vy * ay + vz * az)
		/ sqrtf(vx * vx + vy * vy + vz * vz));

	forwardspeed += (dsp_getlpf(&flpf) - faoffset) * 9.80665 * dt;
	forwardspeed *= 0.9997501;
	forwardpath += forwardspeed * dt;

	log_write(LOG_CUSTOM1, (dsp_getlpf(&flpf) - faoffset) * 9.80665);
	log_write(LOG_CUSTOM2, forwardspeed);
	log_write(LOG_CUSTOM3, forwardpath);

	ht = st.hoverthrottle / (cosf(-pitch) * cosf(-roll));

	// if vertical acceleration is negative, most likely
	// quadcopter is upside down, perform emergency disarm
	if (dsp_getlpf(&tlpf) < -0.5) {
		emergencydisarm = 1;
		setthrust(0.0, 0.0, 0.0, 0.0);
		en = 0.0;
	}

	// write roll, pitch and yaw values into log
	log_write(LOG_ROLL, roll);
	log_write(LOG_PITCH, pitch);
	log_write(LOG_YAW, yaw);

	if (speedpid) {
		// if in single PID loop mode for tilt
		// (called accro mode), use only rotation speed values
		// from the gyroscope. Update speed PID controllers for
		// roll and pitch and get next correction values.
		rollcor = dsp_pidbl(&rollspv, rolltarget, gy);
		pitchcor = dsp_pidbl(&pitchspv, pitchtarget, gx);
	}
	else {
		// if in double loop mode for tilt (most commonly used
		// mode), first update roll and pitch POSITION PID
		// controllers using currect roll and values and targets
		// got from ERLS and get next correction values.
		rollcor = dsp_pidbl(&rollpv, rolltarget, roll);
		pitchcor = dsp_pidbl(&pitchpv, pitchtarget, pitch);

		// then use this values to update roll and pitch speed
		// PID controllers and get next SPEED correction values.
		rollcor = dsp_pidbl(&rollspv, rollcor, gy);
		pitchcor = dsp_pidbl(&pitchspv, pitchcor, gx);
	}

	if (yawspeedpid) {
		// if single PID loop mode for yaw is used just use
		// rotation speed values around axis Z to upadte yaw PID
		// controller and get next yaw correciton value
		yawcor = dsp_pidbl(&yawspv, yawtarget, gz);
	}
	else {
		// if in double loop mode for yaw, first use yaw value
		// calcualted using magnetometer and yaw target got from
		// ELRS remote to update yaw POSITION PID controller and
		// get it's next correciton value.
		yawcor = dsp_circpid(&yawpv, yawtarget, yaw, dt);

		// then use this value to update yaw speed PID
		// controller and get next yaw SPEED correction value
		yawcor = dsp_pidbl(&yawspv, yawcor, gz);
	}

	if (altmode == ALTMODE_POS) {
		// if altitude hold mode enabled, first use altitude
		// got from barometer readings and target altitude from
		// ELRS remote to update altitude PID controller and
		// get it's next correction value
		thrustcor = dsp_pidbl(&apv, thrust,
			dsp_getcompl(&altcompl) - alt0);

		// then use altitude correction value and climb rate
		// calculated by complimentary filter (barometer
		// differentiating and accelerometer Z-axis integration)
		// to update climb rate PID controller and get it's next
		// correction value
		thrustcor = dsp_pidbl(&cpv, thrustcor,
			dsp_getcompl(&climbratecompl));
		
		// and next use climb rate correction value to update
		// vertial acceleration PID controller and get next
		// thrust correction value
		thrustcor = dsp_pidbl(&tpv, thrustcor + 1.0,
			dsp_getlpf(&vtlpf)) + ht;	
	}
	else if (altmode == ALTMODE_SPEED) {
		// if consttant climb rate mode, first use climb rate
		// calculated by complimentary filter (barometer
		// differentiating and accelerometer Z-axis integration)
		// and target climb rate from ELRS remote to update
		// climb rate PID controller and get it's next
		// correction value
		thrustcor = dsp_pidbl(&cpv, thrust,
			dsp_getcompl(&climbratecompl));	

		// and next use climb rate correction value to update
		// vertial acceleration PID controller and get next
		// thrust correction value
		thrustcor = dsp_pidbl(&tpv, thrustcor + 1.0,
			dsp_getlpf(&vtlpf)) + ht;	
	}
	else {
		if (hovermode) {
			// if no altitude hold and hover throttle mode
			// is enabled, update vertical acceleration PID
			// controller using next low-pass filtered
			// value of vertical acceleration and target
			// got from ERLS remote
			thrustcor = dsp_pidbl(&tpv, thrust + 1.0,
				dsp_getlpf(&vtlpf)) + ht;
		}
		else {
			// if no altitude hold and hover throttle mode
			// is disabled, update thrust PID controller
			// using next low-pass filtered value of thrust
			// and target got from ERLS remote
			thrustcor = dsp_pidbl(&tpv, thrust + 1.0,
				dsp_getlpf(&tlpf));
		}
	}

	// reset bilinear PID-controllers when disarmed
	if (en < 0.5) {
		dsp_resetpidbl(&pitchpv);
		dsp_resetpidbl(&rollpv);
		dsp_resetpidbl(&pitchspv);
		dsp_resetpidbl(&rollspv);
		dsp_resetpids(&yawpv);
		dsp_resetpidbl(&yawspv);
		dsp_resetpidbl(&tpv);
		dsp_resetpidbl(&cpv);
		dsp_resetpidbl(&apv);
	}

	// disable I-term for all
	// PID-controller, if no throttle
	if ((altmode == ALTMODE_ACCEL && thrust < 0
			&& !hovermode)
		|| (altmode == ALTMODE_SPEED
			&& thrust < -0.95 * st.climbratemax)
		|| (altmode == ALTMODE_POS && thrust < 0.01)) {
		dsp_resetpidbls(&pitchpv);
		dsp_resetpidbls(&rollpv);
		dsp_resetpidbls(&pitchspv);
		dsp_resetpidbls(&rollspv);
		dsp_resetpids(&yawpv);
		dsp_resetpidbls(&yawspv);
		dsp_resetpidbls(&tpv);
		dsp_resetpidbls(&cpv);
		dsp_resetpidbls(&apv);
	}

	// calculate weights for motors
	// thrust calibration values
	ltm = (1.0 + st.rsc / 2) * (1.0 + st.psc / 2);
	rtm = (1.0 - st.rsc / 2) * (1.0 + st.psc / 2);
	lbm = (1.0 + st.rsc / 2) * (1.0 - st.psc / 2);
	rbm = (1.0 - st.rsc / 2) * (1.0 - st.psc / 2);

	// if final thrust is greater than
	// limit set it to the limit
	thrustcor = thrustcor > st.thrustmax ? st.thrustmax : thrustcor;

	// update motors thrust based on calculated values. For
	// quadcopter it's enought to split correction in half for
	// 3 pairs of motors: left and right for roll, top and bottom
	// for pitch and two diagonals (spinning in oposite directions)
	// for yaw.
	setthrust(en * ltm * (thrustcor + 0.5 * rollcor
			+ 0.5 * pitchcor - 0.5 * yawcor),
		en * rtm * (thrustcor - 0.5 * rollcor
			+ 0.5 * pitchcor + 0.5 * yawcor),	
		en * lbm * (thrustcor + 0.5 * rollcor
			- 0.5 * pitchcor + 0.5 * yawcor),
		en * rbm * (thrustcor - 0.5 * rollcor
			- 0.5 * pitchcor - 0.5 * yawcor));

	// update loops counter
	++loops;

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
	if (elrstimeout != 0)
		--elrstimeout;

	// if timeout conter reached 0 and no ERLS
	// packet came, disarm immediately
	if (elrstimeout <= 0) {
		emergencydisarm = 1;
		setthrust(0.0, 0.0, 0.0, 0.0);
		en = 0.0;
	}

	// since it runs every 1 second update loop counter here too
	loopscount = loops;
	loops = 0;

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
	if (dev[DPS_DEV].status != DEVSTATUS_INIT)
		return 0;

	// read barometer values
	dev[DPS_DEV].read(dev[DPS_DEV].priv, &hd,
		sizeof(struct dps_data));

	// write barometer temperature and altitude values into log
	log_write(LOG_BAR_TEMP, hd.tempf);
	log_write(LOG_BAR_ALT, hd.altf);

	// update altitude low-pass filter and temperature reading
	alt = dsp_updatelpf(&altlpf, hd.altf);
	dsp_updatelpf(&templpf, hd.tempf);

	// calculate climb rate from vertical acceleration and
	// barometric altitude defference using complimentary filter
	dsp_updatecompl(&climbratecompl,
		9.80665 * (dsp_getlpf(&valpf) + goffset - 1.0) * dt,
			(dsp_getcompl(&altcompl) - prevalt) / dt);
	
	// calculate presice altitiude from climb rate and
	// barometric altitude using complimentary filter
	dsp_updatecompl(&altcompl, dsp_getcompl(&climbratecompl) * dt,
		alt);
	
	prevalt = dsp_getcompl(&altcompl);

	// write climbrate and altitude values into log
	log_write(LOG_CLIMBRATE, dsp_getcompl(&climbratecompl));
	log_write(LOG_ALT, dsp_getcompl(&altcompl));

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
	if (dev[QMC_DEV].status != DEVSTATUS_INIT)
		return 0;

	// read magnetometer values
	dev[QMC_DEV].read(dev[QMC_DEV].priv, &qmcdata,
		sizeof(struct qmc_data));

	qmcdata.fx += st.mxthsc * dsp_getlpf(&avgthrustlpf);

	// write magnetometer values into log
	log_write(LOG_MAG_X, qmcdata.fx);
	log_write(LOG_MAG_Y, qmcdata.fy);
	log_write(LOG_MAG_Z, qmcdata.fz);

	dsp_updatelpf(&magxpt1, qmcdata.fx);
	dsp_updatelpf(&magypt1, qmcdata.fy);
	dsp_updatelpf(&magzpt1, qmcdata.fz);

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
	const char *am;

	tele.bat = dsp_getlpf(&batlpf);
	tele.curr = dsp_getlpf(&currlpf);

	if (tele.bat < 6.0)
		tele.batrem = (tele.bat - 3.5) / 0.7;
	else if (tele.bat < 9.0)
		tele.batrem = (tele.bat - 7.0) / 1.4;
	else if (tele.bat < 13.5)
		tele.batrem = (tele.bat - 10.5) / 2.1;
	else if (tele.bat < 18.0)
		tele.batrem = (tele.bat - 14) / 2.8;
	else
		tele.batrem = (tele.bat - 21) / 3.5;

	tele.batrem = trimf(100.0 * tele.batrem, 0.0, 100.0);

	tele.lat = gnss.lat + gnss.latmin / 60.0;
	tele.lon = gnss.lon + gnss.lonmin / 60.0;
	tele.speed = gnss.speed;
	tele.course = gnss.course;
	tele.alt = gnss.altitude;
	tele.sats = gnss.satellites;
	tele.balt = dsp_getcompl(&altcompl) - alt0;
	tele.vspeed = dsp_getcompl(&climbratecompl);
	tele.roll = dsp_getcompl(&rollcompl) - st.roll0;
	tele.pitch = dsp_getcompl(&pitchcompl) - st.pitch0;
	tele.yaw = circf(dsp_getcompl(&yawcompl) - st.yaw0);

	// fill flight mode string with arm state and
	// combination of stabilization modes codes
	if (altmode == ALTMODE_ACCEL)
		am = altmodestr[0];
	else if (altmode == ALTMODE_SPEED)
		am = altmodestr[1];
	else if (altmode == ALTMODE_POS)
		am = altmodestr[2];
	else
		am = "---";

	snprintf((char *) tele.mode, 14, "%c|%s|%s|%s",
		en > 0.5 ? 'a' : 'n',
		attmodestr[speedpid ? 0 : 1],
		yawmodestr[yawspeedpid ? 0 : 1], am);

	// in case of emergency disarming
	// set flight mode to "stopped"
	if (emergencydisarm)
		strcpy((char *) tele.mode, "stopped");

	dev[CRSF_DEV].write(dev[CRSF_DEV].priv, &tele,
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
	dsp_updatelpf(&batlpf, batteryvoltage());
	dsp_updatelpf(&currlpf, esccurrent());

	return 0;
}

/**
* @brief Move to next point in autopilot track.
* @return always 0
*/
int autopilotstep()
{
	forwardpath = 0.0;
	autopilottimer = 0;
	++curpoint;

	return 0;
}

/**
* @brief Update control values when moving in autopilot mode.
* @param ms microsecond passed from last callback invocation
* @return always 0
*/
int autopilotupdate(int ms)
{
	float dx, dy;
	struct trackpoint *nextpoint;
	float dt;

	dt = ms / (float) TICKSPERSEC;
	
	if (!autopilot || pointscount == 0 || curpoint < 0
			|| curpoint >= pointscount) {
		return 0;
	}

	nextpoint = points + curpoint + 1;
	if (nextpoint->type == AUTOPILOT_START)
		autopilottimer = 0.0;
	else if (nextpoint->type == AUTOPILOT_TAKEOFF) {
		thrust = autopilottimer * nextpoint->takeoff.alt
			/ nextpoint->takeoff.t;
		
	//	pitchtarget = 0.0;
		
		autopilottimer += dt;

		if (autopilottimer > nextpoint->takeoff.t)
			autopilotstep();
	}
	else if (nextpoint->type == AUTOPILOT_HOVER) {
		thrust = nextpoint->hover.alt;
		
	//	pitchtarget = 0.0;
		yawtarget = atan2f(nextpoint->hover.x,
			nextpoint->hover.y);
		
		autopilottimer += dt;

		if (autopilottimer > nextpoint->hover.t)
			autopilotstep();
	}
	else if (nextpoint->type == AUTOPILOT_LANDING) {
		if (dsp_getcompl(&altcompl) - alt0 > 5.0)
			thrust -= dt;
		else if (dsp_getcompl(&altcompl) - alt0 > 5.0)
			thrust -= dt * 0.5;
		else if (dsp_getcompl(&altcompl) - alt0 > 2.0)
			thrust -= dt * 0.25;
		else
			thrust -= dt * 0.125;
		
	//	pitchtarget = 0.0;
		
		if (dsp_getcompl(&altcompl) - alt0 <= 0.1)
			autopilotstep();
	}
	else if (nextpoint->type == AUTOPILOT_STOP) {
		thrust = -1.0;
	}
	else if (nextpoint->type == AUTOPILOT_FORWARD) {
		pitchtarget = M_PI / 12.0;
		
		yawtarget = atan2f(nextpoint->forward.x,
			nextpoint->forward.y);

		dx = nextpoint->forward.x - points[curpoint].forward.x;
		dy = nextpoint->forward.y - points[curpoint].forward.y;

		if (forwardpath > sqrtf(dx * dx + dy * dy))
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
	elrs = (cd->chf[ERLS_CH_ONOFF] > 0.5) ? 1 : 0;

	// update ERLS timeout as we got packet
	elrstimeout = ELRS_TIMEOUT;

	if (!elrs) {
		en = 0.0;
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
	if (slot != curslot) {
		slottimeout -= dt;

		if (slottimeout <= 0.0) {
			readsettings(slot);
			setstabilize(0);

			curslot = slot;
		}
	} else
		slottimeout = ELRS_PUSHTIMEOUT;

	if (cd->chf[ERLS_CH_AUTOPILOT] > 0.0) {
		if (autopilot == 0) {
			curpoint = 0;
			forwardspeed = 0.0;
			forwardpath = 0.0;
		}

		autopilot = 1;
	}
	else
		autopilot = 0;

	// set magnetometer stabilization mode, if channel 4 has value
	// more than 0, set gyroscope only stabilization mode otherwise
	if (autopilot) {
	//	yawtarget = 0.0;
		yawspeedpid = 0;
	}
	else 
	if (cd->chf[ERLS_CH_YAWMODE] > 0.0) {
		yawspeedpid = 0;

		// in magnetometer stabilization mode absolute yaw
		// value is stabilized, so target should be integrated
		yawtarget = circf(yawtarget
			+ cd->chf[ERLS_CH_YAW] * dt * M_PI
				* st.yawtargetspeed);
	}
	else {
		yawspeedpid = 1;
		yawtarget = -cd->chf[ERLS_CH_YAW] * M_PI * st.yawspeed;
	}

	// set altitude hold mode, if channel 7 has value more than 25,
	// set climbrate stabilization mode if channel 7 has value
	// between -25 and 25, set vertical acceleration mode if channel
	// 7 value is less than -25
	if (autopilot) {
		altmode = ALTMODE_POS;
	}
	else if (cd->chf[ERLS_CH_THRMODE] > 0.25) {
		altmode = ALTMODE_POS;
		thrust = (cd->chf[ERLS_CH_THRUST] + 1.0)
			/ 2.0 * st.altmax;
	}
	else if (cd->chf[ERLS_CH_THRMODE] < -0.25) {
		altmode = ALTMODE_ACCEL;

		if (cd->chf[ERLS_CH_HOVER] < 0.0) {
			hovermode = 0;
			thrust = (cd->chf[ERLS_CH_THRUST] + 0.5)
				/ 1.5 * st.accelmax;
		}
		else {
			hovermode = 1;

			thrust = (cd->chf[ERLS_CH_THRUST])
				/ 2.0 * st.accelmax;
		}

	}
	else {
		altmode = ALTMODE_SPEED;
		thrust = cd->chf[ERLS_CH_THRUST] * st.climbratemax;
	}

	// if channel 9 is active (it's no-fix button on remote used
	// for testing), set reference altitude from current altitude
	if (cd->chf[ERLS_CH_ALTCALIB] > 0.0) {
		alt0 = dsp_getcompl(&altcompl);
		goffset = 1.0 - dsp_getlpf(&volpf);
		forwardspeed = 0.0;
		forwardpath = 0.0;
		faoffset = dsp_getlpf(&flpf);
	}

	// set acceleromter stabilization mode, if channel 6 has value
	// more than 25, set gyroscope only stabilization mode, if
	// channel 6 has value between -25 and 25, disarm if channel 6
	// value is less than -25
/*
	if (autopilot) {
		en = 1;
		speedpid = 0;

		rolltarget = cd->chf[ERLS_CH_ROLL]
			* (M_PI * st.rollmax);
	}
	else 
	*/	
	if (cd->chf[ERLS_CH_ATTMODE] > 0.25) {
		en = 1;
		speedpid = 0;

		// set pitch/roll targets based on
		// channels 1-2 values (it's a joystick on most remotes)
		rolltarget = cd->chf[ERLS_CH_ROLL]
			* (M_PI * st.rollmax);
		pitchtarget = -cd->chf[ERLS_CH_PITCH]
			* (M_PI * st.pitchmax);
	}
	else if (cd->chf[ERLS_CH_ATTMODE] < -0.25) {
		en = 0;
		speedpid = 0;
	}
	else {
		en = 1;
		speedpid = 1;

		// set pitch/roll targets based on
		// channels 1-2 values (it's a joystick on most remotes)
		rolltarget = cd->chf[ERLS_CH_ROLL]
			* (M_PI * st.rollspeed);
		pitchtarget = -cd->chf[ERLS_CH_PITCH]
			* (M_PI * st.pitchspeed);
	}

	// disable thrust when motors should be
	// disabled, for additional safety
	if (en < 0.5)
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
		gnss.altitude = nd->gga.alt;
		gnss.quality = nd->gga.quality;
		gnss.satellites = nd->gga.sats;

		return 0;
	}

	// other needed data is got from RMC
	// messages. Discard all other messages.
	if (nd->type != M10_TYPE_RMC)
		return 0;

	gnss.time = nd->rmc.time;
	memcpy(gnss.date, nd->rmc.date, 10);

	gnss.latmin = nd->rmc.latmin;
	gnss.lat = nd->rmc.lat;
	gnss.latdir = (tolower(nd->rmc.latdir) == 'n')
		? LATDIR_N : LATDIR_S;

	gnss.lonmin = nd->gga.lonmin;
	gnss.lon = nd->rmc.lon;
	gnss.londir = (tolower(nd->rmc.londir) == 'e')
		? LONDIR_E : LONDIR_W;

	gnss.magvar = nd->rmc.magvar;
	gnss.magvardir = (tolower(nd->rmc.magvardir) == 'e')
		? MAGVARDIR_E : MAGVARDIR_W;

	gnss.speed = nd->rmc.speed;
	gnss.course = nd->rmc.course;

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
		dev[i].status = DEVSTATUS_NOINIT;

	// wait a little to let stm32 periphery and
	// board's devices power on
	HAL_Delay(1000);

	// init stm32 periphery
	periph_init();

	// reading settings from memory current
	// memory slot, which is 0 at start
	readsettings(curslot);

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

	// initilize stabilization routine
	setstabilize(0);

	// set device that is used to logging
	log_setdev(&flashdev);

	initautopilot();

	// initilize periodic events
	inittimev(evs + TEV_PID, 0, PID_FREQ, stabilize);
	inittimev(evs + TEV_CHECK, 0, CHECK_FREQ, checkconnection);
	inittimev(evs + TEV_DPS, 1 * DPS_FREQ / 5, DPS_FREQ, dpsupdate);
	inittimev(evs + TEV_QMC, 2 * QMC_FREQ / 5, QMC_FREQ, qmcupdate);
	inittimev(evs + TEV_LOG, 0, st.logfreq, logupdate);
	inittimev(evs + TEV_TELE, 3 * TELE_FREQ / 5, 
		TELE_FREQ, telesend);
	inittimev(evs + TEV_POWER, 4 * POWER_FREQ / 5,
		POWER_FREQ, powercheck);
	inittimev(evs + TEV_AUTOPILOT, 0, AUTOPILOT_FREQ,
		autopilotupdate);

	// initilize debug commands
	addcommand("r", rcmd);
	addcommand("c", ccmd);
	addcommand("info", infocmd);
	addcommand("pid", pidcmd);
	addcommand("flash", flashcmd);
	addcommand("compl", complcmd);
	addcommand("lpf", lpfcmd);
	addcommand("adj", adjcmd);
	addcommand("log", logcmd);
	addcommand("ctrl", ctrlcmd);
	addcommand("autopilot", autopilotcmd);
	addcommand("system", systemcmd);
	addcommand("irc", irccmd);
	addcommand("motor", motorcmd);
	addcommand("get", getcmd);

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
		if (dev[ESP_DEV].read(dev[ESP_DEV].priv, &cmd,
			CMDSIZE) >= 0) {
			runcommand(dev + ESP_DEV, cmd);
		}

		// poll for configuration and telemtry commands
		// from debug uart connection
		if (dev[UART_DEV].read(dev[UART_DEV].priv, &cmd,
			UART_CMDSIZE) >= 0) {
			runcommand(dev + UART_DEV, cmd);
		}

		// read the ELRS remote's packet
		if (dev[CRSF_DEV].read(dev[CRSF_DEV].priv, &cd,
			sizeof(struct crsf_data)) >= 0) {
			crsfcmd(&cd, elrsus);
			elrsus = 0;
		}

		// check the M10 messages
		if (dev[M10_DEV].read(dev[M10_DEV].priv, &nd,
			sizeof(struct m10_data)) >= 0) {
			m10msg(&nd);
		}

		// check all periodic events context's and run callbacks
		// if enough time passed. Reset their timers after.
		for (i = 0; i < TEV_COUNT; ++i) {
			if (checktimev(evs + i)) {
				evs[i].cb(evs[i].ms);
				resettimev(evs + i);
			}
		}

		// get microseconds passed in this iteration
		// one iteration duration should take at least
		// some time, 100us was choosen
		while ((c = __HAL_TIM_GET_COUNTER(&htim8)) < 1);

		// update periodic events timers
		for (i = 0; i < TEV_COUNT; ++i)
			updatetimev(evs + i, c);

		// update ELRS timer
		elrsus += c;
	}

	return 0;
}
