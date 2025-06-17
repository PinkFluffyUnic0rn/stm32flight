#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>

#include "stm32f4xx_hal.h"

#include "stm32periph.h"
#include "dsp.h"
#include "crc.h"
#include "global.h"
#include "timev.h"
#include "command.h"
#include "util.h"

#include "device.h"
#include "icm42688.h"
#include "hp206c.h"
#include "esp8266.h"
#include "qmc5883l.h"
#include "crsf.h"
#include "w25.h"
#include "m10.h"
#include "uartconf.h"
#include "irc.h"

// device numbers
#define ICM_DEV		0
#define HP_DEV		1
#define QMC_DEV		2
#define CRSF_DEV	3
#define M10_DEV		4
#define ESP_DEV		5
#define UART_DEV	6
#define IRC_DEV		7
#define DEV_COUNT	8

// Periodic events frequencies
#define PID_FREQ 4000
#define CHECK_FREQ 1
#define CALIB_FREQ 25
#define HP_FREQ 25
#define QMC_FREQ 100
#define TELE_FREQ 10
#define LOG_FREQ 64

// MCU flash address where quadcopter settings is stored
#define USER_FLASH 0x080e0000

// quadcopter setting's slot
#define USER_SETSLOTS (0x80 / sizeof(struct settings))

// maximum telemetry packets count, depends on onboard flash size
#define LOG_MAXPACKS \
	(W25_TOTALSIZE / sizeof(struct logpack))

// log packets buffer size
#define LOG_BUFSIZE (W25_PAGESIZE / sizeof(struct logpack))

// log packet values positions
#define LOG_PACKSIZE	32
#define LOG_ACC_X	0
#define LOG_ACC_Y	1
#define LOG_ACC_Z	2
#define LOG_GYRO_X	3
#define LOG_GYRO_Y	4
#define LOG_GYRO_Z	5
#define LOG_MAG_X	6
#define LOG_MAG_Y	7
#define LOG_MAG_Z	8
#define LOG_BAR_TEMP	9
#define LOG_BAR_ALT	10
#define LOG_ACCPT_X	11
#define LOG_ACCPT_Y	12
#define LOG_ACCPT_Z	13
#define LOG_ROLL	14
#define LOG_PITCH	15
#define LOG_YAW		16
#define LOG_CLIMBRATE	17
#define LOG_ALT		18
#define LOG_LT		19
#define LOG_LB		20
#define LOG_RB		21
#define LOG_RT		22
#define LOG_CRSFCH0	23
#define LOG_CRSFCH1	24
#define LOG_CRSFCH2	25
#define LOG_CRSFCH3	26
#define LOG_CRSFCH4	27
#define LOG_CRSFCH5	28
#define LOG_CRSFCH6	29
#define LOG_CRSFCH7	30
#define LOG_BAT		31

// Timer events IDs
#define TEV_PID 	0
#define TEV_CHECK 	1
#define TEV_CALIB	2
#define TEV_HP		3
#define TEV_QMC		4
#define TEV_LOG		5
#define TEV_TELE	6
#define TEV_COUNT	7

// Timeout in seconds before quadcopter disarm
// when got no data from ERLS receiver
#define ELRS_TIMEOUT 2

// set value in current log frame
// pos -- value's position inside the frame
// val -- value itself
#define logwrite(pos, val) logbuf[logbufpos].data[(pos)] = (val);

enum ALTMODE {
	ALTMODE_ACCEL	= 0,
	ALTMODE_SPEED	= 1,
	ALTMODE_POS	= 2
};

enum GNSSSTATUS {
	GNSSSTATUS_VALID = 0,
	GNSSSTATUS_INVALID = 1
};

enum LATDIR {
	LATDIR_N = 0,
	LATDIR_S = 1
};

enum LONDIR {
	LONDIR_E = 0,
	LONDIR_W = 1
};

enum MAGVARDIR {
	MAGVARDIR_E = 0,
	MAGVARDIR_W = 1
};

// Quadcopter settings structure stored in MCU's flash
struct settings {
	float mx0, my0, mz0;	// magnetometer offset values for X, Y
				// and Z axes determinted by
				// calibration procedure

	float mxsc, mysc, mzsc;	// megnetormeter scaling values for Z,
				// Y and Z axes determinted by
				// calibration procedure

	float magdecl;		// magnetic declination

	float ax0, ay0, az0;	// accelometer offset values for X, Y
				// and Z axes

	float gx0, gy0, gz0;	// gyroscope offset values for X, Y
				// and Z axes

	float rsc, psc;		// motor thrust scaling for roll and
				// pitch side motors, because not all
				// motors are abolutely equal

	float roll0, pitch0, yaw0; // roll, pitch and yaw offset values

	float thrustmax;	// maximum thrust value
	float rollmax, pitchmax; // maximum roll and pitch angles in Pi


	float rollspeed;	// roll rotation speed in Pi for
				// single loop tilt mode
	float pitchspeed;	// pitch rotation speed in Pi for
				// single loop tilt mode
	float yawspeed;		// yaw rotation speed in Pi for
				// single loop yaw mode
	float yawtargetspeed;	// yaw target change speed in Pi for
				// dual loop yaw mode
	float accelmax;		// maximum acceleration in g for
				// single loop throttle mode
	float climbratemax;	// maximum climbrate in m/s for
				// dual loop throttle mode
	float altmax;		// maximum altitude in m for triple
				// loop mode (altitude hold mode)

	float atctcoef;		// time coefficient for pitch/roll
				// complimentary filter
	float yctcoef;		// time coefficient for yaw
				// complimentary filter
	float ttcoef;		// time coefficient for vertical axis
				// acceleration pow-pass filter
	float vatcoef;		// time coefficient for vertical
				// acceleration pow-pass filter

	float accpt1freq;	// cut-off frequency for
				// accelerometer PT1 filter

	float gyropt1freq;	// cut-off frequency for
				// gyroscope PT1 filter
	
	float dpt1freq;		// cut-off frequency for PID D term

	float atcoef;		// time coefficient for altitude
				// low-pass filter
	float cctcoef;		// time coefficient for climb rate
				// complimentary filter
	float actcoef;		// time coefficient for altitude
				// complimentary filter

	int speedpid;	// 1 if single PID loop for roll/pitch is used,
			// 0 if double loop is used

	int yawspeedpid; // 1 if single PID loop for yaw is used, 0 if
			 // double loop is used

	float p, i, d;	// P/I/D values for roll/pitch (used only in
			// double roll/pitch PID loop mode)

	float sp, si, sd;	// P/I/D values for roll/pitch
				// rotation speed

	float yp, yi, yd; // P/I/D values for yaw (used only in double
			  // yaw PID loop mode)

	float ysp, ysi, ysd;	// P/I/D values for yaw rotation speed

	float zsp, zsi,	zsd; // P/I/D values for vertical acceleration
	float cp, ci, cd; // P/I/D values for climb rate
	float ap, ai, ad; // P/I/D values for altitude
};

// Values got from GNSS module using NMEA protocol
struct gnss_data {
	enum GNSSSTATUS status;		// GNSS data status
					// 1 if valid, 0 otherwise

	float time;			// seconds passed from 00:00 UTC
	char date[10];			// date in format dd.mm.yy

	uint8_t lat;			// latitude
	float latmin;			// latitude minutes
	enum LATDIR latdir;		// latitude direction, 1 if
					// south, 0 if north

	uint8_t lon;			// longitude
	float lonmin;			// longitude minutes
	enum LONDIR londir;		// longitude direction, 1 if
					// west, 0 if east

	float magvar;			// magnetic declination in
					// degrees
	enum MAGVARDIR  magvardir;	// magnetic declination
					// direction, 0 if east, 1 if
					// west

	float speed;			// speed in knots
	float course;			// course toward north pole
					// in degrees
	float altitude;			// altitude in meters

	int quality;			// link quality
	uint8_t satellites;		// satellites count
};

// telemetry packet stored in onboard flash
struct logpack {
	float data[LOG_PACKSIZE];
};

// stabilization modes short codes that is
// sent to eLRS remote inside telemetry packets
const char *attmodestr[] = {"ac", "st", "sp", "ps"};
const char *yawmodestr[] = {"sp", "cs"};
const char *altmodestr[] = {"tr", "cl", "al"};

// Flight controller board's devices drivers
struct cdevice dev[DEV_COUNT];
struct bdevice flashdev;

// DSP contexts
struct dsp_lpf batlpf;

struct dsp_lpf valpf;
struct dsp_lpf tlpf;
struct dsp_lpf altlpf;

struct dsp_lpf accxpt1;
struct dsp_lpf accypt1;
struct dsp_lpf acczpt1;

struct dsp_lpf gyroxpt1;
struct dsp_lpf gyroypt1;
struct dsp_lpf gyrozpt1;

struct dsp_lpf magxlpf;
struct dsp_lpf magylpf;
struct dsp_lpf magzlpf;

struct dsp_compl pitchcompl;
struct dsp_compl rollcompl;
struct dsp_compl yawcompl;

struct dsp_compl climbratecompl;
struct dsp_compl altcompl;

struct dsp_pidval pitchpv;
struct dsp_pidval rollpv;
struct dsp_pidval pitchspv;
struct dsp_pidval rollspv;
struct dsp_pidval yawpv;
struct dsp_pidval yawspv;
struct dsp_pidval tpv;
struct dsp_pidval cpv;
struct dsp_pidval apv;

// Global storage for sensor data
// that aquired in separate events
float temp;
struct qmc_data qmcdata;
struct gnss_data  gnss;
struct crsf_tele tele;

// Control values
float thrust = 0.0; // motors basic thrust
float rolltarget = 0.0; // roll PID target
float pitchtarget = 0.0; // pitch PID target
float yawtarget = 0.0; // yaw PID target
float ltm = 1.0; // left-top motor thrust scaling
float lbm = 1.0; // left-bot motor thrust scaling
float rtm = 1.0; // right-top motor thrust scaling
float rbm = 1.0; // right-bot motor thrust scaling
float en = 0.0; // 1.0 when motors turned on, 0.0 otherwise
enum ALTMODE altmode = 0; // ALTMODE_POS if in altitude hold mode,
			  // ALTMODE_SPEED if climbrate control mode,
			  // ALTMODE_ACCEL if acceleration control mode
int speedpid = 0;	// 1 if only gyroscope if used for yaw
			// stabilization, 0 if accelerometer is used
int yawspeedpid = 0;	// 1 if only gyroscope if used for yaw
			// stabilization, 0 if magnetometer is used

int magcalibmode = 0; // 1 when magnetometer calibration mode
		      // is enabled, 0 otherwise
int elrs = 0; // 1 when ELRS control is active (ELRS remote's channel 8
	      // is > 50)

// Pressure and altitude initial values
float alt0;

// Settings
struct settings st;

// Timer events
struct timev evs[TEV_COUNT];

// Stabilization loops counter
int loops = 0;

// Stabilization loops performed in last second
int loopscount = 0;

// Timeout counter for the ELRS reciver. Set to ELRS_TIMEOUT after
// receiving useful packet from receiver and decreased by 1 every
// second. If it falls to 0, quadcopter disarms.
int elrstimeout = ELRS_TIMEOUT;

// emergency disarm triggered, further
// arming is possible only after reboot
int emergencydisarm = 0;

// Log bufferization controlling variables
int logtotal = 0;
int logbufpos = 0;
int logflashpos = 0;
size_t logsize = 0;
struct logpack logbuf[LOG_BUFSIZE];

// Get battery voltage from ADC.
//
// hadc -- ADC context.
float batteryvoltage()
{
	uint32_t v;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);

	v = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);

	return (v / (float) 0xfff * 17.85882);
}

// external interrupt callback. It calls interrupt hadlers from
// drivers for devices that use external interrupts.
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
	if (DEVITENABLED(dev[ESP_DEV].status))
		dev[ESP_DEV].interrupt(dev[ESP_DEV].priv, &pin);
}

// UART receive callback. It calls interrupt handlers from
// drivers for devices working through UART.
//
// huart -- context for UART triggered that callback.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (DEVITENABLED(dev[M10_DEV].status))
		dev[M10_DEV].interrupt(dev[M10_DEV].priv, huart);

	if (DEVITENABLED(dev[CRSF_DEV].status))
		dev[CRSF_DEV].interrupt(dev[CRSF_DEV].priv, huart);

	if (DEVITENABLED(dev[UART_DEV].status))
		dev[UART_DEV].interrupt(dev[UART_DEV].priv, huart);
}

// UART error callback. Calls UART error handlers from
// drivers for devices working through UART.
//
// huart -- context for UART triggered that callback.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (DEVITENABLED(dev[ESP_DEV].status))
		dev[ESP_DEV].error(dev[ESP_DEV].priv, huart);
}

// Init ESC's
static void esc_init()
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4
		= (uint16_t) (0.2 * (float) PWM_MAXCOUNT);
	HAL_Delay(2000);
}

// Init HP206c barometer
static void hp_init()
{
	struct hp_device d;

	d.hi2c = &hi2c1;
	d.osr = HP_OSR_512;

	if (hp_initdevice(&d, dev + HP_DEV) >= 0)
		uartprintf("HP206C initilized\r\n");
	else
		uartprintf("failed to initilize HP206C\r\n");
}

// Init IRC tramp video transmitter
static void irc_init()
{
	struct irc_device d;

	d.huart = &huart5;
	d.power = 25;
	d.frequency = 5733;

	if (irc_initdevice(&d, dev + IRC_DEV) >= 0)
		uartprintf("IRC device initilized\r\n");
	else
		uartprintf("failed to initilize IRC device\r\n");
}

// Init ICM-42688-P IMU
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

// Init QMC5883L magnetometer
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

// Init ESP8285
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
	d.intpin = GPIO_PIN_1;

	if (esp_initdevice(&d, dev + ESP_DEV) < 0) {
		uartprintf("failed to initilize ESP8266\r\n");
		return;
	}

	uartprintf("ESP8266 initilized\r\n");
}

// Init ERLS receiver driver
static void crsfdev_init()
{
	struct crsf_device d;

	d.huart = &huart2;

	crsf_initdevice(&d, dev + CRSF_DEV);
}

// Init W25Q onboard flash memory
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

// Init GNSS module
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

// Init UART config connection
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

// write quadcopter settings into internal MCU flash
//
// slot -- offset in settings array in flash.
int writesettings(int slot)
{
	uint32_t sz;
	uint32_t *pt;
	uint32_t addr;
	int j;

	__disable_irq();
	HAL_FLASH_Unlock();

	FLASH_Erase_Sector(FLASH_SECTOR_11,  VOLTAGE_RANGE_3);

	sz = sizeof(struct settings);
	pt = (uint32_t *) &st;

	addr = USER_FLASH + sizeof(struct settings) * slot;
	for (j = 0; j < sz / 4; ++j) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, pt[j]);
		addr += 4;
	}

	HAL_FLASH_Lock();
	__enable_irq();

	return 0;
}

// Read setting from internal MCU flash.
//
// slot -- offset in settings array in flash.
int readsettings(int slot)
{
	memcpy(&st, (void *) (USER_FLASH + slot),
		sizeof(struct settings));

	return 0;
}

// Init stabilization loop.
//
// alt -- initial quadcopter altitude. Usually 0.0.
int initstabilize(float alt)
{
	// set initial altitude. Usually 0.0
	alt0 = alt;

	// init complementary filters contexts
	dsp_initcompl(&pitchcompl, st.atctcoef, PID_FREQ);
	dsp_initcompl(&rollcompl, st.atctcoef, PID_FREQ);
	dsp_initcompl(&yawcompl, st.yctcoef, PID_FREQ);

	dsp_initcompl(&climbratecompl, st.cctcoef, HP_FREQ);
	dsp_initcompl(&altcompl, st.actcoef, HP_FREQ);

	// init roll and pitch position PID controller contexts
	dsp_initpid(&pitchpv, st.p, st.i, st.d, st.dpt1freq, PID_FREQ);
	dsp_initpid(&rollpv, st.p, st.i, st.d, st.dpt1freq, PID_FREQ);

	// init roll, pitch and yaw speed PID controller contexts
	dsp_initpid(&pitchspv, st.sp, st.si, st.sd,
		st.dpt1freq, PID_FREQ);
	dsp_initpid(&rollspv, st.sp, st.si, st.sd,
		st.dpt1freq, PID_FREQ);
	dsp_initpid(&yawspv, st.ysp, st.ysi, st.ysd,
		st.dpt1freq, PID_FREQ);

	// init yaw position PID controller's context
	dsp_initpid(&yawpv, st.yp, st.yi, st.yd, st.dpt1freq, PID_FREQ);

	// init vertical acceleration PID controller's context
	dsp_initpid(&tpv, st.zsp, st.zsi, st.zsd, st.dpt1freq, PID_FREQ);

	// init climbrate PID controller's context
	dsp_initpid(&cpv, st.cp, st.ci, st.cd, st.dpt1freq, PID_FREQ);

	// init altitude PID controller's context
	dsp_initpid(&apv, st.ap, st.ai, st.ad, st.dpt1freq, PID_FREQ);

	// init battery voltage low-pass filter
	dsp_initlpf1f(&batlpf, 100.0, PID_FREQ);

	// init low-pass fitlers for altitude and vertical acceleration
	dsp_initlpf1t(&altlpf, st.atcoef, HP_FREQ);
	dsp_initlpf1t(&tlpf, st.ttcoef, PID_FREQ);
	dsp_initlpf1t(&valpf, st.vatcoef, PID_FREQ);

	// init low-pass fitlers for accelerometer x, y and z axes
	dsp_initlpf1f(&accxpt1, st.accpt1freq, PID_FREQ);
	dsp_initlpf1f(&accypt1, st.accpt1freq, PID_FREQ);
	dsp_initlpf1f(&acczpt1, st.accpt1freq, PID_FREQ);

	// init low-pass fitlers for gyroscope x, y and z axes
	dsp_initlpf1f(&gyroxpt1, st.gyropt1freq, PID_FREQ);
	dsp_initlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ);
	dsp_initlpf1f(&gyrozpt1, st.gyropt1freq, PID_FREQ);

	// init low-pass fitlers for magnetometer x, y and z axes
	dsp_initlpf1f(&magxlpf, 10.0, PID_FREQ);
	dsp_initlpf1f(&magylpf, 10.0, PID_FREQ);
	dsp_initlpf1f(&magzlpf, 10.0, PID_FREQ);

	return 0;
}

// calculate tilt compensated heading direction using magnetometer
// readings, roll value and pitch value.
//
// r -- roll value.
// p -- pitch value.
// x -- magnetometer's X axis value.
// y -- magnetometer's Y axis value.
// z -- magnetometer's Z axis value.
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

/* Set motors thrust

      rtd    ltd
        \    /
   p     \  /
   |
   v     /  \
        /    \
      rbd    lbd

         r ->
*/

// all values should be between 0.0 and 1.0.
int setthrust(float ltd, float lbd, float rbd, float rtd)
{
	if (isnan(ltd) || isnan(rtd) || isnan(rbd)
		|| isnan(lbd) || !elrs)
		ltd = rtd = rbd = lbd = 0.0;

	logwrite(LOG_LT, ltd);
	logwrite(LOG_LB, lbd);
	logwrite(LOG_RB, rbd);
	logwrite(LOG_RT, rtd);

	TIM1->CCR1 = (uint16_t) ((trimuf(ltd) * 0.8 + 0.19)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR2 = (uint16_t) ((trimuf(lbd) * 0.8 + 0.19)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR3 = (uint16_t) ((trimuf(rtd) * 0.8 + 0.19)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR4 = (uint16_t) ((trimuf(rbd) * 0.8 + 0.19)
		* (float) PWM_MAXCOUNT);

	return 0;
}

// Print quadcopter's postion and tilt data into a string.
//
// s -- output string.
// md -- accelerometer and gyroscope data.
// hd -- magnetometer data.
int sprintpos(char *s, struct icm_data *id)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r", "accel: ",
		(double) dsp_getlpf(&accxpt1),
		(double) dsp_getlpf(&accypt1),
		(double) dsp_getlpf(&acczpt1));	
	
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r", "gyro: ",
		(double) dsp_getlpf(&gyroxpt1),
		(double) dsp_getlpf(&gyroypt1),
		(double) dsp_getlpf(&gyrozpt1));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r",
		"accel corrected: ",
		(double) (dsp_getlpf(&accxpt1) - st.ax0),
		(double) (dsp_getlpf(&accypt1) - st.ay0),
		(double) (dsp_getlpf(&acczpt1) - st.az0));
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r",
		"gyro corrected: ",
		(double) (dsp_getlpf(&gyroxpt1) - st.gx0),
		(double) (dsp_getlpf(&gyroypt1) - st.gy0),
		(double) (dsp_getlpf(&gyrozpt1) - st.gz0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll: %0.3f; pitch: %0.3f; yaw: %0.3f\n\r",
		(double) (dsp_getcompl(&rollcompl) - st.roll0),
		(double) (dsp_getcompl(&pitchcompl) - st.pitch0),
		(double) circf(dsp_getcompl(&yawcompl) - st.yaw0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"z acceleration: %f\r\n",
		(double) dsp_getlpf(&tlpf));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"vertical acceleration: %f\r\n",
		(double) dsp_getlpf(&valpf));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"presice  altitude: %f\r\n",
		(double) (dsp_getcompl(&altcompl) - alt0));

	return 0;
}

// Print magmetometer data into a string.
//
// s -- output string.
// hd -- magnetometer data.
int sprintqmc(char *s, struct qmc_data *hd)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"x = %0.3f; y = %0.3f; z = %0.3f\n\r",
		(double) qmcdata.fx, (double) qmcdata.fy,
		(double) qmcdata.fz);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"corrected: x = %0.3f; y = %0.3f; z = %0.3f\n\r",
		(double) (st.mxsc * (qmcdata.fx + st.mx0)),
		(double) (st.mysc * (qmcdata.fy + st.my0)),
		(double) (st.mzsc * (qmcdata.fz + st.mz0)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"heading: %f\r\n", (double) qmc_heading(
			dsp_getcompl(&rollcompl) - st.roll0,
			-(dsp_getcompl(&pitchcompl) - st.pitch0),
			hd->fx, hd->fy, hd->fz));

	return 0;
}

// Print all devices statuses into a string
//
// s -- output string.
int sprintdevs(char *s)
{
	int i;

	s[0] = '\0';

	for (i = 0; i < DEV_COUNT; ++i) {
		const char *strstatus;

		switch (dev[i].status) {
		case DEVSTATUS_IT:
			strstatus = "interrupts enabled";
			break;
		case DEVSTATUS_INIT:
			strstatus = "initilized";
			break;
		case DEVSTATUS_FAILED:
			strstatus = "failed";
			break;
		case DEVSTATUS_NOINIT:
			strstatus = "not initilized";
			break;
		default:
			strstatus = "unknown";
			break;
		}

		sprintf(s + strlen(s), "%-15s: %s\r\n",
			dev[i].name, strstatus);
	}

	return 0;
}

// Print various configuration values into a string.
//
// s -- output string.
int sprintvalues(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"t: %.3f; r: %.3f; p: %.3f; y: %.3f\r\n",
		(double) thrust, (double) rolltarget,
		(double) pitchtarget, (double) yawtarget);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll thrust: %.3f; pitch thrust %.3f\r\n",
		(double) st.rsc, (double) st.psc);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag off: %.3f; %.3f; %.3f\r\n",
		(double) st.mx0, (double) st.my0, (double) st.mz0);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag scale: %.3f; %.3f; %.3f\r\n",
		(double) st.mxsc, (double) st.mysc, (double) st.mzsc);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag decl: %.5f\r\n",
		(double) st.magdecl);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel cor: %.3f; %.3f; %.3f\r\n",
		(double) st.ax0, (double) st.ay0, (double) st.az0);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"gyro cor: %.3f; %.3f; %.3f\r\n",
		(double) st.gx0, (double) st.gy0, (double) st.gz0);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll cor: %.3f; pitch cor: %.3f; yaw cor: %.3f\r\n",
		(double) st.roll0, (double) st.pitch0,
		(double) st.yaw0);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"battery: %0.3f\n\r",
		(double) (dsp_getlpf(&batlpf)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"motors state: %.3f\r\n", (double) en);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"loops count: %d\r\n", loopscount);

	return 0;
}

// Print all PID values into a string.
//
// s -- output string.
int sprintpid(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"pos PID: %.5f,%.5f,%.5f\r\n",
		(double) st.p, (double) st.i, (double) st.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"speed PID: %.5f,%.5f,%.5f\r\n",
		(double) st.sp, (double) st.si, (double) st.sd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw PID: %.5f,%.5f,%.5f\r\n",
		(double) st.yp, (double) st.yi, (double) st.yd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw speed PID: %.5f,%.5f,%.5f\r\n",
		(double) st.ysp, (double) st.ysi, (double) st.ysd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"thrust PID: %.5f,%.5f,%.5f\r\n",
		(double) st.zsp, (double) st.zsi, (double) st.zsd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"climb rate PID: %.5f,%.5f,%.5f\r\n",
		(double) st.cp, (double) st.ci, (double) st.cd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude PID: %.5f,%.5f,%.5f\r\n",
		(double) st.ap, (double) st.ai, (double) st.ad);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s mode\r\n", speedpid ? "single" : "dual");

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s yaw mode\r\n",
		yawspeedpid ? "single" : "dual");

	const char *mode;

	if (altmode == ALTMODE_ACCEL)		mode = "single";
	else if (altmode == ALTMODE_SPEED)	mode = "double";
	else					mode = "triple";

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s altitude mode\r\n", mode);

	return 0;
}

// Print all GNSS values into a string.
//
// s -- output string.
int sprintgnss(char *s) {
	s[0] = '\0';

	sprintf(s, "%s: %f\r\n%s: %hd\r\n%s: %hd %f %c\r\n\
%s: %hd %f %c\r\n%s: %f\r\n%s: %f\r\n\
%s: %d\r\n%s: %d\r\n%s: %f\r\n%s: %s\r\n%s: %f\r\n%s: %c\r\n",
		"time", (double) gnss.time,
		"status", gnss.status,
		"latitude", gnss.lat,
		(double) gnss.latmin,
		(gnss.latdir == LATDIR_N) ? 'N' : 'S',
		"longitude", gnss.lon,
		(double) gnss.lonmin,
		(gnss.londir == LONDIR_W) ? 'W' : 'E',
		"speed", (double) gnss.speed,
		"course", (double) gnss.course,
		"quality", gnss.quality,
		"satellites", gnss.satellites,
		"altitude", (double) gnss.altitude,
		"date", gnss.date,
		"magvar", (double) gnss.magvar,
		"magverdir",
		(gnss.magvardir == MAGVARDIR_W) ? 'W' : 'E');

	return 0;
}

// Print all control scaling values
//
// s -- output string.
int sprintfctrl(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum thrust: %.5f\r\n", (double) st.thrustmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum roll: %.5f\r\n", (double) st.rollmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum pitch: %.5f\r\n", (double) st.pitchmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll speed: %.5f\r\n",
		(double) st.rollspeed);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"pitch speed: %.5f\r\n",
		(double) st.pitchspeed);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw speed: %.5f\r\n",
		(double) st.yawspeed);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw target change speed: %.5f\r\n",
		(double) st.yawtargetspeed);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum acceleration: %.5f\r\n",
		(double) st.accelmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum climbrate: %.5f\r\n",
		(double) st.climbratemax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum altitude: %.5f\r\n",
		(double) st.altmax);

	return 0;
}

// Print filters coefficients into a string.
//
// s -- output string.
int sprintffilters(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"attitude compl tc: %.6f\r\n", (double) st.atctcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw compl tc: %.6f\r\n", (double) st.yctcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"gyro lpf cut-off: %.6f\r\n", (double) st.gyropt1freq);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel lpf cut-off: %.6f\r\n", (double) st.accpt1freq);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"d lpf cut-off: %.6f\r\n", (double) st.dpt1freq);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"thrust lpf tc: %.6f\r\n", (double) st.ttcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"vertical accel lpf tc: %.6f\r\n", (double) st.vatcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"climb rate compl tc: %.6f\r\n", (double) st.cctcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude lpf tc: %.6f\r\n", (double) st.atcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude compl tc: %.6f\r\n", (double) st.actcoef);

	return 0;
}

// Erase log flash to prepare at for writing,
// erasing starts from address 0.
//
// size -- bytes count to erase.
int eraseflash(const struct cdevice *d, size_t size)
{
	size_t pos;
	char s[255];

	// if erase size equals total flash size,
	// use chip erase command
	if (size == W25_TOTALSIZE) {
		flashdev.eraseall(flashdev.priv);
		return 0;
	}

	// else erase flash block-by-block, sector-by-sector
	for (pos = 0; pos < size; ) {
		// if remained size is more than block size
		// use block erase command,
		// use sector erase command otherwise
		if ((size - pos) >= W25_BLOCKSIZE) {
			flashdev.eraseblock(flashdev.priv, pos);
			pos += W25_BLOCKSIZE;

			sprintf(s, "erased block at %u\r\n", pos);
			d->write(d->priv, s, strlen(s));
		}
		else {
			flashdev.erasesector(flashdev.priv, pos);
			pos += W25_SECTORSIZE;

			sprintf(s, "erased sector at %u\r\n", pos);
			d->write(d->priv, s, strlen(s));
		}
	}

	return 0;
}

// Print all log values into debug connection.
//
// s -- string user as buffer.
int printlog(const struct cdevice *d, char *buf, size_t from, size_t to)
{
	int fp;

	if (from > to)
		return (-1);

	// run through all writable space in the flash
	for (fp = from; fp < to; fp += LOG_BUFSIZE) {
		int bp;

		// read batch of log frames into log buffer
		flashdev.read(flashdev.priv,
			fp * sizeof(struct logpack), logbuf,
			LOG_BUFSIZE * sizeof(struct logpack));

		// for every read frame
		for (bp = 0; bp < LOG_BUFSIZE; ++bp) {
			char *data;
			int i;

			if (fp + bp < from || fp + bp >= to)
				continue;

			data = buf + 6;

			// put all frame's values into a string
			sprintf(data, "%d ", fp + bp);

			for (i = 0; i < LOG_PACKSIZE; ++i) {
				sprintf(data + strlen(data), "%0.5f ",
					(double) logbuf[bp].data[i]);
			}

			sprintf(data + strlen(data), "\r\n");

			sprintf(buf, "%05u",
				crc16((uint8_t *) data, strlen(data)));
			buf[5] = ' ';

			// send this string into debug connection
			d->write(d->priv, buf, strlen(buf));
		}
	}

	return 1;
}

// Stabilization loop. Callback for TEV_PID periodic event. It is the
// place where is almost all work happening.
//
// ms -- microsecond passed from last callback invocation.
int stabilize(int ms)
{
	struct icm_data id;
	float ltm, lbm, rbm, rtm;
	float roll, pitch, yaw;
	float rollcor, pitchcor, yawcor, thrustcor;
	float vx, vy, vz;
	float gy, gx, gz;
	float ay, ax, az;
	float dt;

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
	logwrite(LOG_BAT, dsp_updatelpf(&batlpf, batteryvoltage()));

	// toggle arming indication led
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
		(en > 0.5) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// get accelerometer and gyroscope readings
	dev[ICM_DEV].read(dev[ICM_DEV].priv, &id,
		sizeof(struct icm_data));

	// write accelerometer and gyroscope values into log
	logwrite(LOG_ACC_X, id.afx);
	logwrite(LOG_ACC_Y, id.afy);
	logwrite(LOG_ACC_Z, id.afz);
	logwrite(LOG_GYRO_X, id.gfx);
	logwrite(LOG_GYRO_Y, id.gfy);
	logwrite(LOG_GYRO_Z, id.gfz);

	// apply accelerometer offsets
	ax = dsp_updatelpf(&accxpt1, id.afx) - st.ax0;
	ay = dsp_updatelpf(&accypt1, id.afy) - st.ay0;
	az = dsp_updatelpf(&acczpt1, id.afz) - st.az0;

	logwrite(LOG_ACCPT_X, ax);
	logwrite(LOG_ACCPT_Y, ay);
	logwrite(LOG_ACCPT_Z, az);

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
	dsp_updatelpf(&valpf, (vx * ax + vy * ay + vz * az)
		/ sqrtf(vx * vx + vy * vy + vz * vz));

	// if vertical acceleration is negative, most likely
	// quadcopter is upside down, perform emergency disarm
	if (dsp_getlpf(&tlpf) < -0.5) {
		emergencydisarm = 1;
		setthrust(0.0, 0.0, 0.0, 0.0);
		en = 0.0;
	}

	// write roll, pitch and yaw values into log
	logwrite(LOG_ROLL, roll);
	logwrite(LOG_PITCH, pitch);
	logwrite(LOG_YAW, yaw);

	if (speedpid) {
		// if in single PID loop mode for tilt
		// (called accro mode), use only rotation speed values
		// from the gyroscope. Update speed PID controllers for
		// roll and pitch and get next correction values.
		rollcor = dsp_pid(&rollspv, rolltarget, gy, dt);
		pitchcor = dsp_pid(&pitchspv, pitchtarget, gx, dt);
	}
	else {
		// if in double loop mode for tilt (most commonly used
		// mode), first update roll and pitch POSITION PID
		// controllers using currect roll and values and targets
		// got from ERLS and get next correction values.
		rollcor = dsp_pid(&rollpv, rolltarget, roll, dt);
		pitchcor = dsp_pid(&pitchpv, pitchtarget, pitch, dt);

		// then use this values to update roll and pitch speed
		// PID controllers and get next SPEED correction values.
		rollcor = dsp_pid(&rollspv, rollcor, gy, dt);
		pitchcor = dsp_pid(&pitchspv, pitchcor, gx, dt);
	}

	if (yawspeedpid) {
		// if single PID loop mode for yaw is used just use
		// rotation speed values around axis Z to upadte yaw PID
		// controller and get next yaw correciton value
		yawcor = dsp_pid(&yawspv, yawtarget, gz, dt);
	}
	else {
		// if in double loop mode for yaw, first use yaw value
		// calcualted using magnetometer and yaw target got from
		// ELRS remote to update yaw POSITION PID controller and
		// get it's next correciton value.
		yawcor = dsp_circpid(&yawpv, yawtarget, yaw, dt);

		// then use this value to update yaw speed PID
		// controller and get next yaw SPEED correction value
		yawcor = dsp_pid(&yawspv, yawcor, gz, dt);
	}

	if (altmode == ALTMODE_POS) {
		// if altitude hold mode enabled, first use altitude
		// got from barometer readings and target altitude from
		// ELRS remote to update altitude PID controller and
		// get it's next correction value
		thrustcor = dsp_pid(&apv, thrust,
			dsp_getlpf(&altlpf) - alt0, dt);

		// then use altitude correction value and climb rate
		// calculated by complimentary filter (barometer
		// differentiating and accelerometer Z-axis integration)
		// to update climb rate PID controller and get it's next
		// correction value
		thrustcor = dsp_pid(&cpv, thrustcor,
			dsp_getcompl(&climbratecompl), dt);

		// and next use climb rate correction value to update
		// vertial acceleration PID controller and get next
		// thrust correction value
		thrustcor = dsp_pid(&tpv, thrustcor + 1.0,
			dsp_getlpf(&tlpf), dt);
	}
	else if (altmode == ALTMODE_SPEED) {
		// if consttant climb rate mode, first use climb rate
		// calculated by complimentary filter (barometer
		// differentiating and accelerometer Z-axis integration)
		// and target climb rate from ELRS remote to update
		// climb rate PID controller and get it's next
		// correction value
		thrustcor = dsp_pid(&cpv, thrust,
			dsp_getcompl(&climbratecompl), dt);

		// and next use climb rate correction value to update
		// vertial acceleration PID controller and get next
		// thrust correction value
		thrustcor = dsp_pid(&tpv, thrustcor + 1.0,
			dsp_getlpf(&tlpf), dt);
	}
	else {
		// if no altitude hold, update vertical acceleration PID
		// controller using next low-pass filtered value of
		// vertical acceleration and target got from ERLS remote
		thrustcor = dsp_pid(&tpv, thrust + 1.0,
			dsp_getlpf(&tlpf), dt);
	}

	// calculate weights for motors
	// thrust calibration values
	ltm = (1.0 - st.rsc / 2) * (1.0 + st.psc / 2);
	lbm = (1.0 - st.rsc / 2) * (1.0 - st.psc / 2);
	rbm = (1.0 + st.rsc / 2) * (1.0 - st.psc / 2);
	rtm = (1.0 + st.rsc / 2) * (1.0 + st.psc / 2);

	// if final thrust is greater than
	// limit set it to the limit
	thrustcor = thrustcor > st.thrustmax ? st.thrustmax : thrustcor;

	// update motors thrust based on calculated values. For
	// quadcopter it's enought to split correction in half for
	// 3 pairs of motors: left and right for roll, top and bottom
	// for pitch and two diagonals (spinning in oposite directions)
	// for yaw.
	setthrust(en * ltm * (thrustcor - 0.5 * rollcor
			+ 0.5 * pitchcor - 0.5 * yawcor),
		en * lbm * (thrustcor - 0.5 * rollcor
			- 0.5 * pitchcor + 0.5 * yawcor),
		en * rbm * (thrustcor + 0.5 * rollcor
			- 0.5 * pitchcor - 0.5 * yawcor),
		en * rtm * (thrustcor + 0.5 * rollcor
			+ 0.5 * pitchcor + 0.5 * yawcor));

	// update loops counter
	++loops;

	return 0;
}

// Check if ERLS connection is alive, disarm if not. Callback for
// TEV_CHECK periodic event.
//
// ms -- microsecond passed from last callback invocation.
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

// If magnetometer calibration mode is enabled, send magnetometer
// readings into debug wi-fi connection. Callback for TEV_CALIB
// periodic event.
//
// ms -- microsecond passed from last callback invocation.
int magcalib(int ms)
{
	struct qmc_data hd;
	char s[INFOLEN];

	// this callback is only for magnetometer calibration
	// mode, so quit if this mode isn't enabled
	if (!magcalibmode)
		return 0;

	// if magnetometer isn't initilized, return
	if (dev[QMC_DEV].status != DEVSTATUS_INIT)
		return 0;

	// read magnetometer values
	dev[QMC_DEV].read(dev[QMC_DEV].priv, &hd,
		sizeof(struct qmc_data));

	// send them to debug wifi connection
	sprintf(s, "%f %f %f\r\n",
		(double) (st.mxsc * (hd.fx + st.mx0)),
		(double) (st.mysc * (hd.fy + st.my0)),
		(double) (st.mzsc * (hd.fz + st.mz0)));

	dev[ESP_DEV].write(dev[ESP_DEV].priv, s, strlen(s));

	return 0;
}

// Get readings from barometer. Callback for TEV_HP periodic event.
//
// ms -- microsecond passed from last callback invocation.
int hpupdate(int ms)
{
	struct hp_data hd;
	float prevalt;
	float dt;

	dt = ms / (float) TICKSPERSEC;

	dt = (dt < 0.000001) ? 0.000001 : dt;

	// if barometer isn't initilized, return
	if (dev[HP_DEV].status != DEVSTATUS_INIT)
		return 0;

	// read barometer values
	dev[HP_DEV].read(dev[HP_DEV].priv, &hd,
		sizeof(struct hp_data));

	// write barometer temperature and altitude values into log
	logwrite(LOG_BAR_TEMP, hd.tempf);
	logwrite(LOG_BAR_ALT, hd.altf);

	prevalt = dsp_getlpf(&altlpf);

	// update altitude low-pass filter and temperature reading
	dsp_updatelpf(&altlpf, hd.altf);
	temp = hd.tempf;

	// calculate climb rate from vertical acceleration and
	// barometric altitude defference using complimentary filter
	dsp_updatecompl(&climbratecompl,
		9.80665 * (dsp_getlpf(&valpf) - 1.0) * dt,
		(dsp_getlpf(&altlpf) - prevalt) / dt);

	// calculate presice altitiude from climb rate and
	// barometric altitude using complimentary filter
	dsp_updatecompl(&altcompl, dsp_getcompl(&climbratecompl) * dt,
		prevalt);

	// write climbrate and altitude values into log
	logwrite(LOG_CLIMBRATE, dsp_getcompl(&climbratecompl));
	logwrite(LOG_ALT, dsp_getlpf(&altlpf));

	return 0;
}

// Get readings from magnetomer. Callback for TEV_QMC periodic event.
//
// ms -- microsecond passed from last callback invocation.
int qmcupdate(int ms)
{
	// if magnetometer isn't initilized, return
	if (dev[QMC_DEV].status != DEVSTATUS_INIT)
		return 0;

	// read magnetometer values
	dev[QMC_DEV].read(dev[QMC_DEV].priv, &qmcdata,
		sizeof(struct qmc_data));

	// write magnetometer values into log
	logwrite(LOG_MAG_X, qmcdata.fx);
	logwrite(LOG_MAG_Y, qmcdata.fy);
	logwrite(LOG_MAG_Z, qmcdata.fz);

	return 0;
}

// Update log frame. If buffer isn't full, just move buffer pointer,
// otherwise save buffer content into flash and set buffer pointer to 0.
//
// ms -- microsecond passed from last callback invocation.
int logupdate(int ms)
{
	if (logtotal >= logsize)
		return 0;

	if (++logbufpos < LOG_BUFSIZE)
		return 0;

	logtotal += sizeof(struct logpack);

	flashdev.write(flashdev.priv, logflashpos, logbuf,
		LOG_BUFSIZE * sizeof(struct logpack));

	logflashpos += LOG_BUFSIZE * sizeof(struct logpack);
	logbufpos = 0;

	return 0;
}
	
// Send telemetry through ELRS.
//
// ms -- microsecond passed from last callback invocation.
int telesend(int ms)
{
	const char *am;

	tele.bat = dsp_getlpf(&batlpf);
	
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
	tele.balt = dsp_getlpf(&altlpf) - alt0;
	tele.vspeed = dsp_getcompl(&climbratecompl);
	tele.roll = dsp_getcompl(&rollcompl) - st.roll0;
	tele.pitch = dsp_getcompl(&pitchcompl) - st.pitch0;
	tele.yaw = circf(qmc_heading(tele.roll, -tele.pitch,
		qmcdata.fx, qmcdata.fy, qmcdata.fz) - st.yaw0);

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

// Disarm command handler.
//
// toks -- list of parsed command tokens.
int rcmd(const struct cdevice *dev, const char **toks, char *out)
{
	en = 0.0;

	return 0;
}

// Recalibrate command handler.
//
// toks -- list of parsed command tokens.
int ccmd(const struct cdevice *dev, const char **toks, char *out)
{
	initstabilize(atof(toks[1]));

	return 0;
}

// "info" command handler. Print various sensor and control values.
//
// toks -- list of parsed command tokens.
int infocmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "mpu") == 0) {
		struct icm_data id;

		dev[ICM_DEV].read(dev[ICM_DEV].priv, &id,
			sizeof(struct icm_data));

		sprintpos(out, &id);
	}
	else if (strcmp(toks[1], "qmc") == 0) {
		struct qmc_data hd;

		dev[QMC_DEV].read(dev[QMC_DEV].priv, &hd,
			sizeof(struct qmc_data));

		sprintqmc(out, &hd);
	}
	else if (strcmp(toks[1], "hp") == 0) {
		float alt;

		alt = dsp_getlpf(&altlpf) - alt0;

		snprintf(out, INFOLEN,
			"temp: %f; alt: %f; climb rate: %f\r\n",
			(double) temp, (double) alt,
			(double) dsp_getcompl(&climbratecompl));
	}
	else if (strcmp(toks[1], "dev") == 0)
		sprintdevs(out);
	else if (strcmp(toks[1], "values") == 0)
		sprintvalues(out);
	else if (strcmp(toks[1], "pid") == 0)
		sprintpid(out);
	else if (strcmp(toks[1], "gnss") == 0)
		sprintgnss(out);
	else if (strcmp(toks[1], "ctrl") == 0)
		sprintfctrl(out);
	else if (strcmp(toks[1], "filter") == 0)
		sprintffilters(out);
	else if (strcmp(toks[1], "irc") == 0) {
		struct irc_data data;

		dev[IRC_DEV].read(dev[IRC_DEV].priv, &data,
			sizeof(struct irc_data));

		snprintf(out, INFOLEN, "frequency: %d; power: %d\r\n",
			data.frequency, data.power);
	}
	else
		return (-1);

	return 1;
}

// "pid" command handler. Configure PID values.
//
// toks -- list of parsed command tokens.
int pidcmd(const struct cdevice *dev, const char **toks, char *out)
{
	float v;

	v = atof(toks[3]);

	if (strcmp(toks[1], "tilt") == 0) {
		if (strcmp(toks[2], "mode") == 0) {
			if (strcmp(toks[3], "double") == 0)
				speedpid = 0;
			else if (strcmp(toks[3], "single") == 0)
				speedpid = 1;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "p") == 0)	st.p = v;
		else if (strcmp(toks[2], "i") == 0)	st.i = v;
		else if (strcmp(toks[2], "d") == 0)	st.d = v;
		else					return (-1);

		dsp_setpid(&pitchpv, st.p, st.i, st.d,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&rollpv, st.p, st.i, st.d,
			st.dpt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "stilt") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.sp = v;
		else if (strcmp(toks[2], "i") == 0)	st.si = v;
		else if (strcmp(toks[2], "d") == 0)	st.sd = v;
		else					return (-1);

		dsp_setpid(&pitchspv, st.sp, st.si, st.sd,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&rollspv, st.sp, st.si, st.sd,
			st.dpt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "yaw") == 0) {
		if (strcmp(toks[2], "mode") == 0) {
			if (strcmp(toks[3], "double") == 0)
				st.yawspeedpid = 0;
			else if (strcmp(toks[3], "single") == 0)
				st.yawspeedpid = 1;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "p") == 0)	st.yp = v;
		else if (strcmp(toks[2], "i") == 0)	st.yi = v;
		else if (strcmp(toks[2], "d") == 0)	st.yd = v;
		else					return (-1);

		dsp_setpid(&yawpv, st.yp, st.yi, st.yd,
			st.dpt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "syaw") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.ysp = v;
		else if (strcmp(toks[2], "i") == 0)	st.ysi = v;
		else if (strcmp(toks[2], "d") == 0)	st.ysd = v;
		else					return (-1);

		dsp_setpid(&yawspv, st.ysp, st.ysi, st.ysd,
			st.dpt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "throttle") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.zsp = v;
		else if (strcmp(toks[2], "i") == 0)	st.zsi = v;
		else if (strcmp(toks[2], "d") == 0)	st.zsd = v;
		else					return (-1);

		dsp_setpid(&tpv, st.zsp, st.zsi, st.zsd,
			st.dpt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.cp = v;
		else if (strcmp(toks[2], "i") == 0)	st.ci = v;
		else if (strcmp(toks[2], "d") == 0)	st.cd = v;
		else					return (-1);

		dsp_setpid(&cpv, st.cp, st.ci, st.cd,
			st.dpt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.ap = v;
		else if (strcmp(toks[2], "i") == 0)	st.ai = v;
		else if (strcmp(toks[2], "d") == 0)	st.ad = v;
		else					return (-1);

		dsp_setpid(&apv, st.ap, st.ai, st.ad,
			st.dpt1freq, PID_FREQ);
	}
	else
		return (-1);

	return 0;
}

// "calib" command handler. Turn on/off devices calibration modes.
//
// toks -- list of parsed command tokens.
int calibcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "mag") == 0) {
		if (strcmp(toks[2], "on") == 0)
			magcalibmode = 1;
		else if (strcmp(toks[2], "off") == 0)
			magcalibmode = 0;
		else
			return (-1);
	}
	else
		return (-1);

	return 0;
}

// "flash" command handler. Write/read MCU's internal flash
// used for storing configuraton.
//
// toks -- list of parsed command tokens.
int flashcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "write") == 0)
		writesettings(atoi(toks[2]));
	else if (strcmp(toks[1], "read") == 0)
		readsettings(atoi(toks[2]));
	else
		return (-1);

	return 0;
}

// "compl" command handler. Configure pitch/roll complimentary filters.
//
// toks -- list of parsed command tokens.
int complcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "attitude") == 0) {
		st.atctcoef = atof(toks[2]);
		dsp_initcompl(&rollcompl, st.atctcoef, PID_FREQ);
		dsp_initcompl(&pitchcompl, st.atctcoef, PID_FREQ);
	}
	else if (strcmp(toks[1], "yaw") == 0) {
		st.yctcoef = atof(toks[2]);
		dsp_initcompl(&yawcompl, st.yctcoef, PID_FREQ);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		st.cctcoef = atof(toks[2]);
		dsp_initcompl(&climbratecompl, st.cctcoef, HP_FREQ);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		st.actcoef = atof(toks[2]);
		dsp_initcompl(&altcompl, st.actcoef, HP_FREQ);
	}

	return 0;
}

// "lpf" command handler. Configure low-pass filters.
//
// toks -- list of parsed command tokens.
int lpfcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "gyro") == 0) {
		st.gyropt1freq = atof(toks[2]);

		dsp_initlpf1f(&gyroxpt1, st.gyropt1freq, PID_FREQ);
		dsp_initlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ);
		dsp_initlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "accel") == 0) {
		st.accpt1freq = atof(toks[2]);

		dsp_initlpf1f(&accxpt1, st.accpt1freq, PID_FREQ);
		dsp_initlpf1f(&accypt1, st.accpt1freq, PID_FREQ);
		dsp_initlpf1f(&accypt1, st.accpt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "d") == 0) {
		st.dpt1freq = atof(toks[2]);
		
		dsp_setpid(&pitchpv, st.p, st.i, st.d,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&rollpv, st.p, st.i, st.d,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&pitchspv, st.sp, st.si, st.sd,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&rollspv, st.sp, st.si, st.sd,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&yawspv, st.ysp, st.ysi, st.ysd,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&yawpv, st.yp, st.yi, st.yd,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&tpv, st.zsp, st.zsi, st.zsd,
			st.dpt1freq, PID_FREQ);
		dsp_setpid(&cpv, st.cp, st.ci, st.cd,
			st.dpt1freq, PID_FREQ);
	}
	else if (strcmp(toks[1], "climb") == 0) {
		st.ttcoef = atof(toks[2]);

		dsp_initlpf1t(&tlpf, st.ttcoef, PID_FREQ);
	}
	else if (strcmp(toks[1], "vaccel") == 0) {
		st.vatcoef = atof(toks[2]);

		dsp_initlpf1t(&valpf, st.vatcoef, PID_FREQ);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		st.atcoef = atof(toks[2]);

		dsp_initlpf1t(&altlpf, st.atcoef, HP_FREQ);
	}
	else
		return (-1);

	return 0;
}

// "adj" command handler. Configure various offset/scale values.
//
// toks -- list of parsed command tokens.
int adjcmd(const struct cdevice *dev, const char **toks, char *out)
{
	float v;

	v = atof(toks[2]);

	if (strcmp(toks[1], "rollthrust") == 0)		st.rsc = v;
	else if (strcmp(toks[1], "pitchthrust") == 0)	st.psc = v;
	else if (strcmp(toks[1], "roll") == 0)		st.roll0 = v;
	else if (strcmp(toks[1], "pitch") == 0)		st.pitch0 = v;
	else if (strcmp(toks[1], "yaw") == 0)		st.yaw0 = v;
	else if (strcmp(toks[1], "acc") == 0) {
		if (strcmp(toks[2], "x") == 0)
			st.ax0 = atof(toks[3]);
		else if (strcmp(toks[2], "y") == 0)
			st.ay0 = atof(toks[3]);
		else if (strcmp(toks[2], "z") == 0)
			st.az0 = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "gyro") == 0) {
		if (strcmp(toks[2], "x") == 0)
			st.gx0 = atof(toks[3]);
		else if (strcmp(toks[2], "y") == 0)
			st.gy0 = atof(toks[3]);
		else if (strcmp(toks[2], "z") == 0)
			st.gz0 = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "mag") == 0) {
		if (strcmp(toks[2], "x0") == 0)
			st.mx0 = atof(toks[3]);
		else if (strcmp(toks[2], "y0") == 0)
			st.my0 = atof(toks[3]);
		else if (strcmp(toks[2], "z0") == 0)
			st.mz0 = atof(toks[3]);
		else if (strcmp(toks[2], "xscale") == 0)
			st.mxsc = atof(toks[3]);
		else if (strcmp(toks[2], "yscale") == 0)
			st.mysc = atof(toks[3]);
		else if (strcmp(toks[2], "zscale") == 0)
			st.mzsc = atof(toks[3]);
		else if (strcmp(toks[2], "decl") == 0)
			st.magdecl = atof(toks[3]);
		else
			return (-1);
	}
	else
		return (-1);

	return 0;
}

// "log" command handler. Start/stop/get flight log.
//
// toks -- list of parsed command tokens.
int logcmd(const struct cdevice *d, const char **toks, char *out)
{
	char s[INFOLEN];

	if (strcmp(toks[1], "set") == 0) {
		// flash erasing process takes time and blocks
		// other actions, so disarm for safety
		en = 0.0;
		setthrust(0.0, 0.0, 0.0, 0.0);

		// notify user when erasing is started
		sprintf(s, "erasing flash...\r\n");
		d->write(d->priv, s, strlen(s));

		// set log size (0 is valid and
		// means to disable logging)
		if (atoi(toks[2]) > W25_TOTALSIZE)
			return (-1);

		logsize = atoi(toks[2]) * sizeof(struct logpack);

		// enable writeonly mode if log writing is enabled
		flashdev.ioctl(flashdev.priv,
			(logsize == 0)
			? W25_IOCTL_READWRITE : W25_IOCTL_WRITEONLY);

		// erase log flash no
		// respond during this process
		eraseflash(d, logsize);

		// notify user when telemetry flash is erased
		sprintf(s,"erased %u bytes of flash\r\n", logsize);
		d->write(d->priv, s, strlen(s));

		// enable telemetry
		logtotal = 0;
		logflashpos = 0;
		logbufpos = 0;
	}
	else if (strcmp(toks[1], "rget") == 0) {
		// enable read/write mode if reading log
		flashdev.ioctl(flashdev.priv, W25_IOCTL_READWRITE);

		// print records from specified range
		if (printlog(d, s, atoi(toks[2]), atoi(toks[3])) < 0)
			return (-1);

		// write end marker
		sprintf(s, "-end-\r\n");
		d->write(d->priv, s, strlen(s));
	}
	else if (strcmp(toks[1], "bget") == 0) {
		const char **p;

		// enable read/write mode if reading log
		flashdev.ioctl(flashdev.priv, W25_IOCTL_READWRITE);
	
		// print every record whose number is in arguments
		for (p = toks + 2; strlen(*p) != 0; ++p) {
			if (printlog(d, s, atoi(*p), atoi(*p) + 1) < 0)
				return (-1);
		}

		// write end marker
		sprintf(s, "-end-\r\n");
		d->write(d->priv, s, strlen(s));
	}
	else
		return (-1);

	return 1;
}


// "ctrl" command handler. Configure control ranges scaling.
//
// toks -- list of parsed command tokens.
int ctrlcmd(const struct cdevice *d, const char **toks, char *out)
{
	float v;

	v = atof(toks[2]);

	if (strcmp(toks[1], "thrust") == 0)
		st.thrustmax = v;
	else if (strcmp(toks[1], "roll") == 0)
		st.rollmax = v;
	else if (strcmp(toks[1], "pitch") == 0)
		st.pitchmax = v;
	else if (strcmp(toks[1], "yaw") == 0)
		st.yawtargetspeed = v;
	else if (strcmp(toks[1], "sroll") == 0)
		st.rollspeed = v;
	else if (strcmp(toks[1], "spitch") == 0)
		st.pitchspeed = v;
	else if (strcmp(toks[1], "syaw") == 0)
		st.yawspeed = v;
	else if (strcmp(toks[1], "accel") == 0)
		st.accelmax = v;
	else if (strcmp(toks[1], "climbrate") == 0)
		st.climbratemax = v;
	else if (strcmp(toks[1], "altmax") == 0)
		st.altmax = v;
	else
		return (-1);

	return 0;
}

// "system" command handler. Run system commands.
//
// toks -- list of parsed command tokens.
int systemcmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "esp") == 0) {
		if (strcmp(toks[2], "flash") == 0)
			dev[ESP_DEV].configure(dev[ESP_DEV].priv, "flash");
		else if (strcmp(toks[2], "run") == 0)
			dev[ESP_DEV].configure(dev[ESP_DEV].priv, "run");
		else
			return (-1);
	}
	else
		return (-1);

	return 0;
}

// "irc" command handler. Configure IRC tramp video transmitter.
//
// toks -- list of parsed command tokens.
int irccmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "set") == 0) {
		if (strcmp(toks[2], "frequency") == 0) {
			dev[IRC_DEV].configure(dev[IRC_DEV].priv,
				"frequency", atoi(toks[3]));
		}
		else if (strcmp(toks[2], "power") == 0) {
			dev[IRC_DEV].configure(dev[IRC_DEV].priv,
				"power", atoi(toks[3]));
		}
		else
			return (-1);
	}
	else
		return (-1);

	return 0;
}

int getcmd(const struct cdevice *d, const char **toks, char *out)
{
	const char **p;
	char *data;
	float v;

	if (strcmp(toks[1], "pid") == 0) {
		if (strcmp(toks[2], "tilt") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.p;
			else if (strcmp(toks[3], "i") == 0)
				v = st.i;
			else if (strcmp(toks[3], "d") == 0)
				v = st.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "stilt") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.sp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.si;
			else if (strcmp(toks[3], "d") == 0)
				v = st.sd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "yaw") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.yp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.yi;
			else if (strcmp(toks[3], "d") == 0)
				v = st.yd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "syaw") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.ysp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.ysi;
			else if (strcmp(toks[3], "d") == 0)
				v = st.ysd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "throttle") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.zsp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.zsi;
			else if (strcmp(toks[3], "d") == 0)
				v = st.zsd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "climbrate") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.cp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.ci;
			else if (strcmp(toks[3], "d") == 0)
				v = st.cd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "altitude") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.ap;
			else if (strcmp(toks[3], "i") == 0)
				v = st.ai;
			else if (strcmp(toks[3], "d") == 0)
				v = st.ad;
			else
				return (-1);
		}
		else
			return (-1);
	}
	else if (strcmp(toks[1], "compl") == 0) {
		if (strcmp(toks[2], "attitude") == 0)
			v = st.atctcoef;
		else if (strcmp(toks[2], "yaw") == 0)
			v = st.yctcoef;
		else if (strcmp(toks[2], "climbrate") == 0)
			v = st.cctcoef;
		else if (strcmp(toks[2], "altitude") == 0)
			v = st.actcoef;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "lpf") == 0) {
		if (strcmp(toks[2], "gyro") == 0)
			v = st.gyropt1freq;
		else if (strcmp(toks[2], "accel") == 0)
			v = st.accpt1freq;
		else if (strcmp(toks[2], "d") == 0)
			v = st.dpt1freq;
		else if (strcmp(toks[2], "climb") == 0)
			v = st.ttcoef;
		else if (strcmp(toks[2], "vaccel") == 0)
			v = st.vatcoef;
		else if (strcmp(toks[2], "altitude") == 0)
			v = st.atcoef;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "adj") == 0) {
		if (strcmp(toks[2], "rollthrust") == 0)
			v = st.rsc;
		else if (strcmp(toks[2], "pitchthrust") == 0)
			v = st.psc;
		else if (strcmp(toks[2], "roll") == 0)
			v = st.roll0;
		else if (strcmp(toks[2], "pitch") == 0)
			v = st.pitch0;
		else if (strcmp(toks[2], "yaw") == 0)
			v = st.yaw0;
		else if (strcmp(toks[2], "acc") == 0) {
			if (strcmp(toks[3], "x") == 0)
				v = st.ax0;
			else if (strcmp(toks[3], "y") == 0)
				v = st.ay0;
			else if (strcmp(toks[3], "z") == 0)
				v = st.az0;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "gyro") == 0) {
			if (strcmp(toks[3], "x") == 0)
				v = st.gx0;
			else if (strcmp(toks[3], "y") == 0)
				v = st.gy0;
			else if (strcmp(toks[3], "z") == 0)
				v = st.gz0;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "mag") == 0) {
			if (strcmp(toks[3], "x0") == 0)
				v = st.mx0;
			else if (strcmp(toks[3], "y0") == 0)
				v = st.my0;
			else if (strcmp(toks[3], "z0") == 0)
				v = st.mz0;
			else if (strcmp(toks[3], "xscale") == 0)
				v = st.mxsc;
			else if (strcmp(toks[3], "yscale") == 0)
				v = st.mysc;
			else if (strcmp(toks[3], "zscale") == 0)
				v = st.mzsc;
			else if (strcmp(toks[3], "decl") == 0)
				v = st.magdecl;
			else
				return (-1);
		}
		else
			return (-1);
	}
	else if (strcmp(toks[1], "ctrl") == 0) {
		if (strcmp(toks[2], "thrust") == 0)
			v = st.thrustmax;
		else if (strcmp(toks[2], "roll") == 0)
			v = st.rollmax;
		else if (strcmp(toks[2], "pitch") == 0)
			v = st.pitchmax;
		else if (strcmp(toks[2], "yaw") == 0)
			v = st.yawtargetspeed;
		else if (strcmp(toks[2], "sroll") == 0)
			v = st.rollspeed;
		else if (strcmp(toks[2], "spitch") == 0)
			v = st.pitchspeed;
		else if (strcmp(toks[2], "syaw") == 0)
			v = st.yawspeed;
		else if (strcmp(toks[2], "accel") == 0)
			v = st.accelmax;
		else if (strcmp(toks[2], "climbrate") == 0)
			v = st.climbratemax;
		else if (strcmp(toks[2], "altmax") == 0)
			v = st.altmax;
		else
			return (-1);
	}
	else
		return (-1);

	data = out + 6;

	data[0] = '\0';
	for (p = toks + 1; *p != NULL; ++p)
		sprintf(data + strlen(data), "%s ", *p);

	sprintf(data + strlen(data), "%f\r\n", (double) v);

	sprintf(out, "%05u", crc16((uint8_t *) data, strlen(data)));
	out[5] = ' ';

	return 1;
}

// Set control values using CRSF packet.
//
// cd -- CRSF packet
// ms -- microsecond passed from last CRSF packet
int crsfcmd(const struct crsf_data *cd, int ms)
{
	float dt;

	// write first 8 channels values into log	
	logwrite(LOG_CRSFCH0, cd->chf[0]);
	logwrite(LOG_CRSFCH1, cd->chf[1]);
	logwrite(LOG_CRSFCH2, cd->chf[2]);
	logwrite(LOG_CRSFCH3, cd->chf[3]);
	logwrite(LOG_CRSFCH4, cd->chf[4]);
	logwrite(LOG_CRSFCH5, cd->chf[5]);
	logwrite(LOG_CRSFCH6, cd->chf[6]);
	logwrite(LOG_CRSFCH7, cd->chf[7]);

	// channel 8 on remote is used to turn on/off
	// erls control. If this channel has low state, all remote
	// commands will be ignored, but packet still continue to
	// comming
	elrs = (cd->chf[7] > 0.5) ? 1 : 0;

	// update ERLS timeout as we got packet
	elrstimeout = ELRS_TIMEOUT;

	if (!elrs) {
		en = 0.0;
		setthrust(0.0, 0.0, 0.0, 0.0);

		return 0;
	}

	// get time passed from last ERLS packet
	dt = ms / (float) TICKSPERSEC;

	if (cd->chf[4] < 0.0) {
		yawspeedpid = 0;
		yawtarget = circf(yawtarget
			+ cd->chf[3] * dt * M_PI * st.yawtargetspeed);
	}
	else {
		yawspeedpid = 1;
		yawtarget = -cd->chf[3] * M_PI * st.yawspeed;
	}

	// set altitude hold mode, if channel 7 has value more than 25,
	// set climbrate stabilization mode if channel 7 has value
	// between -25 and 25, set vertical acceleration mode if channel
	// 7 value is less than -25
	if (cd->chf[6] < -0.25) {
		altmode = ALTMODE_ACCEL;
		thrust = (cd->chf[2] + 0.75) / 1.75 * st.accelmax;;
	}
	else if (cd->chf[6] > 0.25) {
		altmode = ALTMODE_POS;
		thrust = (cd->chf[2] + 1.0) / 2.0 * st.altmax;
	}
	else {
		altmode = ALTMODE_SPEED;
		thrust = cd->chf[2] * st.climbratemax;
	}

	// if channel 9 is active (it's no-fix button on remote used
	// for testing), set reference altitude from current altitude
	if (cd->chf[8] > 0.0)
		alt0 = dsp_getlpf(&altlpf);

	// set acceleromter stabilization mode, if channel 6 has value
	// more than 25, set gyroscope only stabilization mode, if
	// channel 6 has value between -25 and 25, disarm if channel 6
	// value is less than -25
	if (cd->chf[5] < -0.25) {
		en = 0;
		speedpid = 0;
	}
	else if (cd->chf[5] > 0.25) {
		en = 1;
		speedpid = 0;

		// set pitch/roll targets based on
		// channels 1-2 values (it's a joystick on most remotes)
		rolltarget = cd->chf[0] * (M_PI * st.rollmax);
		pitchtarget = -cd->chf[1] * (M_PI * st.pitchmax);
	}
	else {
		en = 1;
		speedpid = 1;

		// set pitch/roll targets based on
		// channels 1-2 values (it's a joystick on most remotes)
		rolltarget = cd->chf[0] * (M_PI * st.rollspeed);
		pitchtarget = -cd->chf[1] * (M_PI * st.pitchspeed);
	}

	// disable thrust when motors should be
	// disabled, for additional safety
	if (en < 0.5)
		setthrust(0.0, 0.0, 0.0, 0.0);

	return 0;
}

// Store data got from NMEA message.
//
// nd -- data got from GNSS device through NMEA protocol.
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

// Entry point
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
	gpio_init();
	tim1_init();
	tim8_init();
	tim10_init();
	dma_init();
	i2c_init();
	spi1_init();
	spi2_init();
	adc1_init();
	usart1_init();
	usart2_init();
	usart3_init();
	uart4_init();
	uart5_init();

	// init board's devices
	esc_init();
	icm_init();
	qmc_init();
	espdev_init();
	crsfdev_init();
	w25dev_init();
	m10dev_init();
	uartdev_init();
	hp_init();
	irc_init();

	// reading settings from memory slot 0
	readsettings(0);

	// initilize stabilization routine
	initstabilize(0.0);

	// initilize periodic events
	inittimev(evs + TEV_PID, PID_FREQ, stabilize);
	inittimev(evs + TEV_CHECK, CHECK_FREQ, checkconnection);
	inittimev(evs + TEV_CALIB, CALIB_FREQ, magcalib);
	inittimev(evs + TEV_HP, HP_FREQ, hpupdate);
	inittimev(evs + TEV_QMC, QMC_FREQ, qmcupdate);
	inittimev(evs + TEV_LOG, LOG_FREQ, logupdate);
	inittimev(evs + TEV_TELE, TELE_FREQ, telesend);

	// initilize debug commands
	addcommand("r", rcmd);
	addcommand("c", ccmd);
	addcommand("info", infocmd);
	addcommand("pid", pidcmd);
	addcommand("calib", calibcmd);
	addcommand("flash", flashcmd);
	addcommand("compl", complcmd);
	addcommand("lpf", lpfcmd);
	addcommand("adj", adjcmd);
	addcommand("log", logcmd);
	addcommand("ctrl", ctrlcmd);
	addcommand("system", systemcmd);
	addcommand("irc", irccmd);
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
		while ((c = __HAL_TIM_GET_COUNTER(&htim8)) < 100);

		// update periodic events timers
		for (i = 0; i < TEV_COUNT; ++i)
			updatetimev(evs + i, c);

		// update ELRS timer
		elrsus += c;
	}

	return 0;
}
