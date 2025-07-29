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
#define LOG_FREQ 128
#define POWER_FREQ 500

// MCU flash address where quadcopter settings is stored
#define USER_FLASH 0x080e0000

// quadcopter setting's slot
#define USER_SETSLOTS (0x80 / sizeof(struct settings))

// log packets buffer size
#define LOG_BUFSIZE W25_PAGESIZE

// log packets per buffer
#define LOG_PACKSPERBUF (LOG_BUFSIZE / (sizeof(float) * st.logpacksize))

// log packet size
#define LOG_MAXPACKSIZE	32
#define LOG_FIELDSTRSIZE 38

// log packet values positions
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
#define LOG_ROLL	11
#define LOG_PITCH	12
#define LOG_YAW		13
#define LOG_CLIMBRATE	14
#define LOG_ALT		15
#define LOG_LT		16
#define LOG_LB		17
#define LOG_RB		18
#define LOG_RT		19
#define LOG_BAT		20
#define LOG_CUR		21
#define LOG_CRSFCH0	22
#define LOG_CRSFCH1	23
#define LOG_CRSFCH2	24
#define LOG_CRSFCH3	25
#define LOG_CRSFCH4	26
#define LOG_CRSFCH5	27
#define LOG_CRSFCH6	28
#define LOG_CRSFCH7	29
#define LOG_CRSFCH8	30
#define LOG_CRSFCH9	31
#define LOG_CRSFCH10	32
#define LOG_CRSFCH11	33
#define LOG_CRSFCH12	34
#define LOG_CRSFCH13	35
#define LOG_CRSFCH14	36
#define LOG_CRSFCH15	37

// Timer events count
#define TEV_COUNT	8

// Timer events IDs
#define TEV_PID 	0
#define TEV_CHECK 	1
#define TEV_CALIB	2
#define TEV_HP		3
#define TEV_QMC		4
#define TEV_LOG		5
#define TEV_TELE	6
#define TEV_POWER	7

// Timeout in seconds before quadcopter disarm
// when got no data from ERLS receiver
#define ELRS_TIMEOUT 2

// Time required to register button push,
// used for buttons and switches that controls
// time consuming functions
#define ELRS_PUSHTIMEOUT 0.1

// eRLS channels mapping
#define ERLS_CH_ROLL		0
#define ERLS_CH_PITCH		1
#define ERLS_CH_THRUST		2
#define ERLS_CH_YAW		3
#define ERLS_CH_YAWMODE		4
#define ERLS_CH_ATTMODE		5
#define ERLS_CH_THRMODE		6
#define ERLS_CH_ONOFF		7
#define ERLS_CH_ALTCALIB	8
#define ERLS_CH_SETSLOT		15

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
	
	int ircpower, ircfreq; // IRC Tramp VTX power and frequency
	int lt, lb, rb, rt; // motors ESC outputs numbers

	int logfreq;			// log frequency
	int logpacksize;		// log size
	int fieldid[LOG_FIELDSTRSIZE];	// id for every log field
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
	float data[LOG_MAXPACKSIZE];
};

// stabilization modes short codes that is
// sent to eLRS remote inside telemetry packets
const char *attmodestr[] = {"ac", "st", "sp", "ps"};
const char *yawmodestr[] = {"sp", "cs"};
const char *altmodestr[] = {"tr", "cl", "al"};
const char *logfieldstr[LOG_FIELDSTRSIZE] = {
	"acc_x", "acc_y", "acc_z",
	"gyro_x", "gyro_y", "gyro_z",
	"mag_x", "mag_y", "mag_z",
	"bar_temp", "bar_alt",
	"roll", "pitch", "yaw",
	"climbrate", "alt",
	"lt", "lb", "rb", "rt",
	"bat", "cur",
	"ch0", "ch1", "ch2", "ch3", "ch4", "ch5", "ch6", "ch7",
	"ch8", "ch9", "ch10", "ch11", "ch12", "ch13", "ch14", "ch15"
};

// Flight controller board's devices drivers
struct cdevice dev[DEV_COUNT];
struct bdevice flashdev;

// DSP contexts
struct dsp_lpf batlpf;
struct dsp_lpf currlpf;

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
float alt0 = 0.0;

// Current settings slot
int curslot = 0;

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
int logbufpos = 0;
int logflashpos = 0;
size_t logsize = 0;
float logbuf[LOG_BUFSIZE / sizeof(float)];

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

// Get esc current from ADC.
//
// hadc -- ADC context.
float esccurrent()
{
	uint32_t v;

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1);

	v = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Stop(&hadc2);

	return (v / (float) 0xfff);
}

int logfieldstrn(const char *s)
{
	int i;

	for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
		if (strcmp(s, logfieldstr[i]) == 0)
			break;
	}
	
	if (i >= LOG_FIELDSTRSIZE)
		return (-1);

	return i;
}

// set value in current log frame
// pos -- value's position inside the frame
// val -- value itself
void logwrite(int pos, float val)
{
	if (st.fieldid[pos] < 0 || st.fieldid[pos] > st.logpacksize)
		return;

	logbuf[logbufpos * st.logpacksize + st.fieldid[pos]] = val;
}

// DMA callback for dshot processing
//
// hdma -- DMA device that triggered this callback.
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

// Fill buffer, that contains PWM duty cycle values of a
// a DSHOT packet using 16 bit value.
//
// buf -- buffer for PWM duty cycle values that is used by DMA.
// val -- 16 bit value from 0 to 2047
// tele -- 1 if telemetry bit needed, 0 otherwise
void dshotsetbuf(uint16_t *buf, uint16_t val, int tele)
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

inline int dshotsetthrust(uint16_t *buf, float v)
{
	// convert motor thrust value from [0.0;1.0]
	// range to int value in [48; 2047] range
	// disable telemetry request bit
	dshotsetbuf(buf, (uint16_t) (trimuf(v) * 1999.0 + 0.5) + 48, 0);

	return 0;
}

// Send dshot special command to a DSHOT output.
//
// n -- a DSHOT output's number.
// cmd -- command number.
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

/* Set motors thrust

      ltd    rtd
        \    /
   p     \  /
   |
   v     /  \
        /    \
      lbd    rbd

         <- r
*/
// all values should be between 0.0 and 1.0.
int setthrust(float ltd, float rtd, float lbd, float rbd)
{
	static uint16_t dsbuf[4][18];

	if (isnan(ltd) || isnan(rtd) || isnan(rbd)
		|| isnan(lbd) || !elrs)
		ltd = rtd = rbd = lbd = 0.0;

	// put motors thrust values into log
	logwrite(LOG_LT, ltd);
	logwrite(LOG_LB, lbd);
	logwrite(LOG_RB, rbd);
	logwrite(LOG_RT, rtd);

	// construst a PWM duty cycle
	// buffers for dshot ESCs
	dshotsetthrust(dsbuf[st.lt], ltd);
	dshotsetthrust(dsbuf[st.lb], lbd);
	dshotsetthrust(dsbuf[st.rt], rtd);
	dshotsetthrust(dsbuf[st.rb], rbd);

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

// Init ESC's.
static void esc_init()
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

// Init HP206c barometer.
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

// Init IRC tramp video transmitter.
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

// Init ICM-42688-P IMU.
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

// Init QMC5883L magnetometer.
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

// Init ESP8285.
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

// Init ERLS receiver driver.
static void crsfdev_init()
{
	struct crsf_device d;

	d.huart = &huart2;

	crsf_initdevice(&d, dev + CRSF_DEV);
}

// Init W25Q onboard flash memory.
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

// Init GNSS module.
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

// Init UART config connection.
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

// write quadcopter settings into internal MCU flash.
//
// slot -- offset in settings array in flash.
int writesettings(int slot)
{
	struct settings s[6];
	uint32_t sz;
	uint32_t *pt;
	uint32_t addr;
	int j;

	memcpy(s, (void *) (USER_FLASH), sizeof(struct settings) * 6);

	__disable_irq();
	HAL_FLASH_Unlock();

	FLASH_Erase_Sector(FLASH_SECTOR_11,  VOLTAGE_RANGE_3);

	memcpy(s + slot, &st, sizeof(struct settings));

	sz = sizeof(struct settings) * 6;
	pt = (uint32_t *) s;
	
	addr = USER_FLASH;
	for (j = 0; j < sz / 4; ++j) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, pt[j]);
		addr += 4;
	}

	HAL_FLASH_Lock();
	__enable_irq();

	return 0;
}

// Validate current settings. Reset settings
// that has wrong values to default
int validatesettings()
{
	int i;

	if (!irc_ispowervalid(st.ircpower))
		st.ircpower = IRC_POWER_25;
	if (!irc_isfreqvalid(st.ircpower))
		st.ircfreq = IRC_FREQ_5733;

	if (st.lt < 0 || st.lt > 3)	st.lt = 0;
	if (st.lb < 0 || st.lb > 3)	st.lb = 1;
	if (st.rb < 0 || st.rb > 3)	st.rb = 2;
	if (st.rt < 0 || st.rt > 3)	st.rt = 3;

	if (st.logfreq < 0 || st.logfreq > 4096)
		st.logfreq = 128;

	if (st.logpacksize < 0 || st.logpacksize > LOG_MAXPACKSIZE)
		st.logpacksize = 0;

	for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
		if (st.fieldid[i] < 0)
			st.fieldid[i] = 0xff;
	}

	return 0;
}

// Read setting from internal MCU flash.
//
// slot -- offset in settings array in flash.
int readsettings(int slot)
{
	memcpy(&st, (void *) (USER_FLASH
			+ slot * sizeof(struct settings)),
		sizeof(struct settings));

	validatesettings();

	return 0;
}

// Init/set stabilization loop.
//
// init -- if called during initilization.
int setstabilize(int init)
{
	// init complementary filters contexts
	dsp_setcompl(&pitchcompl, st.atctcoef, PID_FREQ, init);
	dsp_setcompl(&rollcompl, st.atctcoef, PID_FREQ, init);
	dsp_setcompl(&yawcompl, st.yctcoef, PID_FREQ, init);

	dsp_setcompl(&climbratecompl, st.cctcoef, HP_FREQ, init);
	dsp_setcompl(&altcompl, st.actcoef, HP_FREQ, init);

	// init roll and pitch position PID controller contexts
	dsp_setpid(&pitchpv, st.p, st.i, st.d, st.dpt1freq,
		PID_FREQ, init);
	dsp_setpid(&rollpv, st.p, st.i, st.d, st.dpt1freq,
		PID_FREQ, init);

	// init roll, pitch and yaw speed PID controller contexts
	dsp_setpid(&pitchspv, st.sp, st.si, st.sd,
		st.dpt1freq, PID_FREQ, init);
	dsp_setpid(&rollspv, st.sp, st.si, st.sd,
		st.dpt1freq, PID_FREQ, init);
	dsp_setpid(&yawspv, st.ysp, st.ysi, st.ysd,
		st.dpt1freq, PID_FREQ, init);

	// init yaw position PID controller's context
	dsp_setpid(&yawpv, st.yp, st.yi, st.yd, st.dpt1freq,
		PID_FREQ, init);

	// init vertical acceleration PID controller's context
	dsp_setpid(&tpv, st.zsp, st.zsi, st.zsd, st.dpt1freq,
		PID_FREQ, init);

	// init climbrate PID controller's context
	dsp_setpid(&cpv, st.cp, st.ci, st.cd, st.dpt1freq,
		PID_FREQ, init);

	// init altitude PID controller's context
	dsp_setpid(&apv, st.ap, st.ai, st.ad, st.dpt1freq,
		PID_FREQ, init);

	// init battery voltage low-pass filter
	dsp_setlpf1f(&batlpf, 100.0, POWER_FREQ, init);
	dsp_setlpf1f(&currlpf, 100.0, POWER_FREQ, init);

	// init low-pass fitlers for altitude and vertical acceleration
	dsp_setlpf1t(&altlpf, st.atcoef, HP_FREQ, init);
	dsp_setlpf1t(&tlpf, st.ttcoef, PID_FREQ, init);
	dsp_setlpf1t(&valpf, st.vatcoef, PID_FREQ, init);

	// init low-pass fitlers for accelerometer x, y and z axes
	dsp_setlpf1f(&accxpt1, st.accpt1freq, PID_FREQ, init);
	dsp_setlpf1f(&accypt1, st.accpt1freq, PID_FREQ, init);
	dsp_setlpf1f(&acczpt1, st.accpt1freq, PID_FREQ, init);

	// init low-pass fitlers for gyroscope x, y and z axes
	dsp_setlpf1f(&gyroxpt1, st.gyropt1freq, PID_FREQ, init);
	dsp_setlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ, init);
	dsp_setlpf1f(&gyrozpt1, st.gyropt1freq, PID_FREQ, init);

	// init low-pass fitlers for magnetometer x, y and z axes
	dsp_setlpf1f(&magxlpf, 10.0, PID_FREQ, init);
	dsp_setlpf1f(&magylpf, 10.0, PID_FREQ, init);
	dsp_setlpf1f(&magzlpf, 10.0, PID_FREQ, init);

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

// Print all devices statuses into a string.
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
		"battery: %0.3f\r\n",
		(double) (dsp_getlpf(&batlpf)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"esc current: %0.3f\r\n",
		(double) (dsp_getlpf(&currlpf)));

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

// Print all control scaling values.
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

			sprintf(s, "erased block at %u\r\n", pos);
			d->write(d->priv, s, strlen(s));
			
			pos += W25_BLOCKSIZE;
		}
		else {
			flashdev.erasesector(flashdev.priv, pos);

			sprintf(s, "erased sector at %u\r\n", pos);
			d->write(d->priv, s, strlen(s));
			
			pos += W25_SECTORSIZE;
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
	for (fp = from; fp < to; fp += LOG_PACKSPERBUF) {
		int bp;

		// read batch of log frames into log buffer
		flashdev.read(flashdev.priv,
			sizeof(float) * fp * st.logpacksize, logbuf,
			LOG_BUFSIZE);

		// for every read frame
		for (bp = 0; bp < LOG_PACKSPERBUF; ++bp) {
			char *data;
			int i;
		
			if (fp + bp < from || fp + bp >= to)
				continue;

			data = buf + 6;

			// put all frame's values into a string
			sprintf(data, "%d ", fp + bp);

			for (i = 0; i < st.logpacksize; ++i) {
				int rec;

				rec = bp * st.logpacksize;
				sprintf(data + strlen(data), "%0.5f ",
					(double) logbuf[rec + i]);
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
	logwrite(LOG_BAT, dsp_getlpf(&batlpf));

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
			dsp_getlpf(&valpf), dt);
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
			dsp_getlpf(&valpf), dt);
	}
	else {
		// if no altitude hold, update vertical acceleration PID
		// controller using next low-pass filtered value of
		// vertical acceleration and target got from ERLS remote
		thrustcor = dsp_pid(&tpv, thrust + 1.0,
			dsp_getlpf(&tlpf), dt);
	}

	// disable I-term for all PID-controller,
	// if disarmeed of no throttle
	if (en < 0.5 
		|| (altmode == ALTMODE_ACCEL 
			&& thrustcor < 0.35 * st.accelmax)
		|| (altmode == ALTMODE_SPEED
			&& thrustcor < -0.95 * st.climbratemax)
		|| (altmode == ALTMODE_POS
			&& thrustcor < -0.01)) {
		dsp_resetpids(&pitchpv);
		dsp_resetpids(&rollpv);
		dsp_resetpids(&pitchspv);
		dsp_resetpids(&rollspv);
		dsp_resetpids(&yawpv);
		dsp_resetpids(&yawspv);
		dsp_resetpids(&tpv);
		dsp_resetpids(&cpv);
		dsp_resetpids(&apv);
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
			+ 0.5 * pitchcor + 0.5 * yawcor),
		en * rtm * (thrustcor - 0.5 * rollcor
			+ 0.5 * pitchcor - 0.5 * yawcor),	
		en * lbm * (thrustcor + 0.5 * rollcor
			- 0.5 * pitchcor - 0.5 * yawcor),
		en * rbm * (thrustcor - 0.5 * rollcor
			- 0.5 * pitchcor + 0.5 * yawcor));

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
	if (logflashpos >= logsize)
		return 0;
		
	if (++logbufpos < LOG_PACKSPERBUF)
		return 0;

	flashdev.write(flashdev.priv, logflashpos, logbuf, LOG_BUFSIZE);

	logflashpos += LOG_BUFSIZE;
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

int powercheck(int ms)
{
	dsp_updatelpf(&batlpf, batteryvoltage());
	dsp_updatelpf(&currlpf, esccurrent());
		
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
	setstabilize(1);

	return 0;
}

// "info" command handler. Print various sensor and control values.
//
// d -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
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
// dev -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
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
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&rollpv, st.p, st.i, st.d,
			st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "stilt") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.sp = v;
		else if (strcmp(toks[2], "i") == 0)	st.si = v;
		else if (strcmp(toks[2], "d") == 0)	st.sd = v;
		else					return (-1);

		dsp_setpid(&pitchspv, st.sp, st.si, st.sd,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&rollspv, st.sp, st.si, st.sd,
			st.dpt1freq, PID_FREQ, 0);
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
			st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "syaw") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.ysp = v;
		else if (strcmp(toks[2], "i") == 0)	st.ysi = v;
		else if (strcmp(toks[2], "d") == 0)	st.ysd = v;
		else					return (-1);

		dsp_setpid(&yawspv, st.ysp, st.ysi, st.ysd,
			st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "throttle") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.zsp = v;
		else if (strcmp(toks[2], "i") == 0)	st.zsi = v;
		else if (strcmp(toks[2], "d") == 0)	st.zsd = v;
		else					return (-1);

		dsp_setpid(&tpv, st.zsp, st.zsi, st.zsd,
			st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.cp = v;
		else if (strcmp(toks[2], "i") == 0)	st.ci = v;
		else if (strcmp(toks[2], "d") == 0)	st.cd = v;
		else					return (-1);

		dsp_setpid(&cpv, st.cp, st.ci, st.cd,
			st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.ap = v;
		else if (strcmp(toks[2], "i") == 0)	st.ai = v;
		else if (strcmp(toks[2], "d") == 0)	st.ad = v;
		else					return (-1);

		dsp_setpid(&apv, st.ap, st.ai, st.ad,
			st.dpt1freq, PID_FREQ, 0);
	}
	else
		return (-1);

	return 0;
}

// "calib" command handler. Turn on/off devices calibration modes.
//
// dev -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
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
// dev -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
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
// dev -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
int complcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "attitude") == 0) {
		st.atctcoef = atof(toks[2]);
		dsp_setcompl(&rollcompl, st.atctcoef, PID_FREQ, 0);
		dsp_setcompl(&pitchcompl, st.atctcoef, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "yaw") == 0) {
		st.yctcoef = atof(toks[2]);
		dsp_setcompl(&yawcompl, st.yctcoef, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		st.cctcoef = atof(toks[2]);
		dsp_setcompl(&climbratecompl, st.cctcoef, HP_FREQ, 0);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		st.actcoef = atof(toks[2]);
		dsp_setcompl(&altcompl, st.actcoef, HP_FREQ, 0);
	}

	return 0;
}

// "lpf" command handler. Configure low-pass filters.
//
// dev -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
int lpfcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "gyro") == 0) {
		st.gyropt1freq = atof(toks[2]);

		dsp_setlpf1f(&gyroxpt1, st.gyropt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "accel") == 0) {
		st.accpt1freq = atof(toks[2]);

		dsp_setlpf1f(&accxpt1, st.accpt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&accypt1, st.accpt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&accypt1, st.accpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "d") == 0) {
		st.dpt1freq = atof(toks[2]);
		
		dsp_setpid(&pitchpv, st.p, st.i, st.d,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&rollpv, st.p, st.i, st.d,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&pitchspv, st.sp, st.si, st.sd,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&rollspv, st.sp, st.si, st.sd,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&yawspv, st.ysp, st.ysi, st.ysd,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&yawpv, st.yp, st.yi, st.yd,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&tpv, st.zsp, st.zsi, st.zsd,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&cpv, st.cp, st.ci, st.cd,
			st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "climb") == 0) {
		st.ttcoef = atof(toks[2]);

		dsp_setlpf1t(&tlpf, st.ttcoef, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "vaccel") == 0) {
		st.vatcoef = atof(toks[2]);

		dsp_setlpf1t(&valpf, st.vatcoef, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		st.atcoef = atof(toks[2]);

		dsp_setlpf1t(&altlpf, st.atcoef, HP_FREQ, 0);
	}
	else
		return (-1);

	return 0;
}

// "adj" command handler. Configure various offset/scale values.
//
// dev -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
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
// d -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
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

		logsize = atoi(toks[2]) * sizeof(float)
			* st.logpacksize;

		// set log size (0 is valid and
		// means to disable logging)
		if (logsize > W25_TOTALSIZE)
			return (-1);

		// erase log flash no
		// respond during this process
		eraseflash(d, logsize);

		// enable writeonly mode if log writing is enabled
		flashdev.ioctl(flashdev.priv,
			(logsize == 0)
			? W25_IOCTL_READWRITE : W25_IOCTL_WRITEONLY);

		// notify user when telemetry flash is erased
		sprintf(s,"erased %u bytes of flash\r\n", logsize);
		d->write(d->priv, s, strlen(s));

		// enable telemetry
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
	else if (strcmp(toks[1], "freq") == 0) {
		st.logfreq = atoi(toks[2]);

		if (st.logfreq < 0 || st.logfreq > 4096)
			st.logfreq = 128;

		modifytimev(evs + TEV_LOG, st.logfreq);

		return 0;
	}
	else if (strcmp(toks[1], "record") == 0) {
		if (strcmp(toks[2], "size") == 0) {
			unsigned int p;

			for (p = 1; p < atoi(toks[3]); p <<= 1);

			if (p > LOG_MAXPACKSIZE)
				p = LOG_MAXPACKSIZE;

			st.logpacksize = p;
		}
		else {
			int strn;

			strn = logfieldstrn(toks[2]);

			if (strn < 0)
				return (-1);

			st.fieldid[strn] = atoi(toks[3]);
		}
		
		return 0;
	}
	else
		return (-1);

	return 1;
}


// "ctrl" command handler. Configure control ranges scaling.
//
// d -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
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
// d -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
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
// d -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
int irccmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "frequency") == 0) {
		st.ircfreq = atoi(toks[2]);

		if (dev[IRC_DEV].status != DEVSTATUS_INIT)
			return 0;

		dev[IRC_DEV].configure(dev[IRC_DEV].priv, "set",
			"frequency", atoi(toks[2]));
	}
	else if (strcmp(toks[1], "power") == 0) {
		st.ircpower = atoi(toks[2]);
	
		if (dev[IRC_DEV].status != DEVSTATUS_INIT)
			return 0;

		dev[IRC_DEV].configure(dev[IRC_DEV].priv, "set",
			"power", atoi(toks[2]));
	}
	else
		return (-1);

	return 0;
}

// "motor" command handler. Configure motors output
// number and direction.
//
// d -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
int motorcmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "lt") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.lt, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.lt, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.lt, 12);
			mdelay(35);
		}
		else
			st.lt = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "lb") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.lb, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.lb, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.lb, 12);
			mdelay(35);
		}
		else
			st.lb = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "rb") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.rb, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.rb, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.rb, 12);
			mdelay(35);
		}
		else
			st.rb = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "rt") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.rt, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.rt, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.rt, 12);
			mdelay(35);
		}
		else
			st.rt = atoi(toks[2]);
	}
	else
		return (-1);

	return 0;
}

// "get" command handler. Get current configuration values.
//
// d -- charter device device that got this command.
// toks -- list of parsed command tokens.
// out -- command's output.
int getcmd(const struct cdevice *d, const char **toks, char *out)
{
	const char **p;
	char *data;
	int isfloat;
	float v;
	int vi;

	isfloat = 1;

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
	
		isfloat = 1;
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

		isfloat = 1;
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

		isfloat = 1;
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

		isfloat = 1;
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

		isfloat = 1;
	}
	else if (strcmp(toks[1], "irc") == 0) {
		if (strcmp(toks[2], "frequency") == 0)
			vi = st.ircfreq;
		else if (strcmp(toks[2], "power") == 0)
			vi = st.ircpower;
		else
			return (-1);

		isfloat = 0;
	}
	else if (strcmp(toks[1], "log") == 0) {
		if (strcmp(toks[2], "freq") == 0)
			vi = st.logfreq;
		else if (strcmp(toks[2], "record") == 0) {
			if (strcmp(toks[3], "size") == 0)
				vi = st.logpacksize;
			else {
				int strn;

				strn = logfieldstrn(toks[3]);

				if (strn < 0)
					return (-1);

				vi = st.fieldid[strn];
			}
		}
		else
			return (-1);
			
		isfloat = 0;
	}	
	else if (strcmp(toks[1], "motor") == 0) {
		if (strcmp(toks[2], "lt") == 0)
			vi = st.lt;
		else if (strcmp(toks[2], "lb") == 0)
			vi = st.lb;
		else if (strcmp(toks[2], "rb") == 0)
			vi = st.rb;
		else if (strcmp(toks[2], "rt") == 0)
			vi = st.rt;
		
		isfloat = 0;
	}
	else
		return (-1);

	data = out + 6;

	data[0] = '\0';
	for (p = toks + 1; *p != NULL; ++p)
		sprintf(data + strlen(data), "%s ", *p);

	if (isfloat)
		sprintf(data + strlen(data), "%f\r\n", (double) v);
	else	
		sprintf(data + strlen(data), "%d\r\n", vi);

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
	static float slottimeout = ELRS_PUSHTIMEOUT;
	float dt;
	int slot;

	// write first 8 channels values into log	
	logwrite(LOG_CRSFCH0, cd->chf[ERLS_CH_ROLL]);
	logwrite(LOG_CRSFCH1, cd->chf[ERLS_CH_PITCH]);
	logwrite(LOG_CRSFCH2, cd->chf[ERLS_CH_THRUST]);
	logwrite(LOG_CRSFCH3, cd->chf[ERLS_CH_YAW]);
	logwrite(LOG_CRSFCH4, cd->chf[ERLS_CH_YAWMODE]);
	logwrite(LOG_CRSFCH5, cd->chf[ERLS_CH_ATTMODE]);
	logwrite(LOG_CRSFCH6, cd->chf[ERLS_CH_THRMODE]);
	logwrite(LOG_CRSFCH7, cd->chf[ERLS_CH_ONOFF]);

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

	// set magnetometer stabilization mode, if channel 4 has value
	// more than 0, set gyroscope only stabilization mode otherwise
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
	if (cd->chf[ERLS_CH_THRMODE] < -0.25) {
		altmode = ALTMODE_ACCEL;

		thrust = (cd->chf[ERLS_CH_THRUST] + 0.5)
			/ 1.5 * st.accelmax;;

	}
	else if (cd->chf[ERLS_CH_THRMODE] > 0.25) {
		altmode = ALTMODE_POS;
		thrust = (cd->chf[ERLS_CH_THRUST] + 1.0)
			/ 2.0 * st.altmax;
	}
	else {
		altmode = ALTMODE_SPEED;
		thrust = cd->chf[ERLS_CH_THRUST] * st.climbratemax;
	}

	// if channel 9 is active (it's no-fix button on remote used
	// for testing), set reference altitude from current altitude
	if (cd->chf[ERLS_CH_ALTCALIB] > 0.0)
		alt0 = dsp_getlpf(&altlpf);

	// set acceleromter stabilization mode, if channel 6 has value
	// more than 25, set gyroscope only stabilization mode, if
	// channel 6 has value between -25 and 25, disarm if channel 6
	// value is less than -25
	if (cd->chf[ERLS_CH_ATTMODE] < -0.25) {
		en = 0;
		speedpid = 0;
	}
	else if (cd->chf[ERLS_CH_ATTMODE] > 0.25) {
		en = 1;
		speedpid = 0;

		// set pitch/roll targets based on
		// channels 1-2 values (it's a joystick on most remotes)
		rolltarget = cd->chf[ERLS_CH_ROLL]
			* (M_PI * st.rollmax);
		pitchtarget = -cd->chf[ERLS_CH_PITCH]
			* (M_PI * st.pitchmax);
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
	hp_init();
	irc_init();

	// initilize stabilization routine
	setstabilize(0);

	// initilize periodic events
	inittimev(evs + TEV_PID, PID_FREQ, stabilize);
	inittimev(evs + TEV_CHECK, CHECK_FREQ, checkconnection);
	inittimev(evs + TEV_CALIB, CALIB_FREQ, magcalib);
	inittimev(evs + TEV_HP, HP_FREQ, hpupdate);
	inittimev(evs + TEV_QMC, QMC_FREQ, qmcupdate);
	inittimev(evs + TEV_LOG, LOG_FREQ, logupdate);
	inittimev(evs + TEV_TELE, TELE_FREQ, telesend);
	inittimev(evs + TEV_POWER, POWER_FREQ, powercheck);

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
		while ((c = __HAL_TIM_GET_COUNTER(&htim8)) < 100);

		// update periodic events timers
		for (i = 0; i < TEV_COUNT; ++i)
			updatetimev(evs + i, c);

		// update ELRS timer
		elrsus += c;
	}

	return 0;
}
