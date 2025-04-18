#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <ctype.h>

#include "main.h"
#include "device.h"
#include "uartdebug.h"
#include "icm42688.h"
#include "hp206c.h"
#include "esp8266.h"
#include "qmc5883l.h"
#include "dsp.h"
#include "crsf.h"
#include "w25.h"
#include "m10.h"

// Max length for info packet
// sent back to operator
#define INFOLEN 512

// device numbers
#define ICM_DEV		0
#define HP_DEV		1
#define QMC_DEV		2
#define CRSF_DEV	3
#define M10_DEV		4
#define ESP_DEV		5
#define DEV_COUNT	6

// timers prescaler
#define PRESCALER 128

// clock frequency
#define OCSFREQ 128000000

// period for main timer (used to timing periodic events)
#define TIMPERIOD 0xfff

// main timer ticks per second
#define TICKSPERSEC (OCSFREQ / PRESCALER)

// PWM settings
//#define PWM_MAXCOUNT 2500
#define PWM_MAXCOUNT 32000

// Periodic events frequencies
#define PID_FREQ 1000
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
#define LOG_PACKSIZE	16
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

// debug commands maximum count
#define MAX_COMMANDS 32

// debug commands maximum token count
#define MAX_CMDTOKS 12

// Timer events IDs
#define TEV_PID 	0
#define TEV_CHECK 	1
#define TEV_CALIB	2
#define TEV_HP		3
#define TEV_QMC		4
#define TEV_LOG		5
#define TEV_TELE	6
#define TEV_COUNT	7

// Debug connection port
#define SERVPORT 8880

// Timeout in seconds before quadcopter disarm
// when got no data from ERLS receiver
#define ELRS_TIMEOUT 2

// check if enough time passed to call periodic event again.
// ev -- periodic event's context.
#define checktimev(ev) ((ev)->ms > (TICKSPERSEC / (ev)->freq - (ev)->rem))

// update periodic event's counter.
// ev -- periodic event's context.
// s -- time in microseconds after last callback's call.
#define updatetimev(ev, s) (ev)->ms += (s);

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

	float rollmax, pitchmax; // maximum roll and pitch angles in Pi

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


// debug command handler structure
struct command {
	const char *name;
	int (*func)(const char **);
};

// Periodic event's context that holds event's settings
// and data needed beetween calls
struct timev {
	int ms;			// microseconds passed from
				// last triggering
	int rem;
	int freq;		// event frequency
	int (*cb)(int);		// event callback
};

// STM32 clocks and periphery devices initilization
void systemclock_config();
static void gpio_init();
static void i2c_init();
static void spi1_init();
static void dma_init();
static void tim1_init();
static void tim8_init();
static void adc1_init(void);
static void usart1_init();
static void usart2_init();
static void usart3_init();
static void uart4_init();

// Init ESC's
static void esc_init();

// Init HP206c barometer
static void hp_init();

// Init ICM-42688 IMU
static void icm_init();

// Init QMC5883L magnetometer
static void qmc_init();

// Init ESP07
static void espdev_init();

// Init ERLS receiver driver
static void crsfdev_init();

// Init W25Q onboard flash memory
static void w25dev_init();

// Init GNSS module
static void m10dev_init();

// STM32 perithery contexts
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;

// Flight controller board's devices drivers
struct cdevice dev[DEV_COUNT];
struct bdevice flashdev;

// DSP contexts
struct dsp_lpf valpf;
struct dsp_lpf tlpf;
struct dsp_lpf altlpf;

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

// Log bufferization controlling variables
int logtotal = 0;
int logbufpos = 0;
int logflashpos = 0;
size_t logsize = 0;
struct logpack logbuf[LOG_BUFSIZE];

// Debug commands handlers
struct command commtable[MAX_COMMANDS];
size_t commcount;

// Add debug command with it's handler.
//
// name -- command name
// func -- command handler
int addcommand(const char *name, int (*func)(const char **))
{
	commtable[commcount].name = name;
	commtable[commcount].func = func;

	++commcount;

	return 0;
}

// Get value from ADC, was used to monitor battery voltage.
//
// hadc -- ADC context.
uint32_t getadcv(ADC_HandleTypeDef *hadc)
{
	uint32_t v;

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1);

	v = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);

	return v;
}

// UART receive callback. It calls interrupt handlers from
// drivers for devices working through UART.
//
// huart -- context for UART triggered that callback.
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	if (DEVITENABLED(dev[ESP_DEV].status))
		dev[ESP_DEV].interrupt(dev[ESP_DEV].priv, huart);
	
	if (DEVITENABLED(dev[M10_DEV].status))
		dev[M10_DEV].interrupt(dev[M10_DEV].priv, huart);
	
	if (DEVITENABLED(dev[CRSF_DEV].status))
		dev[CRSF_DEV].interrupt(dev[CRSF_DEV].priv, huart);
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

// Initilize periodic event's context
//
// ev -- periodic event's context.
// freq -- periodic event's frequency in Hz (calls per second).
// cb -- callback that is called by this event.
int inittimev(struct timev *ev, int freq, int (*cb)(int))
{
	ev->ms = 0;
	ev->rem = 0;
	ev->freq = freq;
	ev->cb = cb;

	return 0;
}

// reset periodic event's counter. Used after every
// event's callback call.
// ev -- periodic event's context.
int resettimev(struct timev *ev)
{
	ev->rem += ev->ms - TICKSPERSEC / (ev)->freq;
	ev->ms = 0;

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

      ltd    rtd
        \    /
   ^     \  /
   |       
   p     /  \
        /    \
      lbd    rbd
        
         <- r
*/
// all values should be between 0.0 and 1.0.
int setthrust(float ltd, float lbd, float rbd, float rtd)
{
	if (isnan(ltd) || isnan(rtd) || isnan(rbd)
		|| isnan(lbd) || !elrs)
		ltd = rtd = rbd = lbd = 0.0;

	TIM1->CCR1 = (uint16_t) ((trimuf(ltd) * 0.5 + 0.49)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR2 = (uint16_t) ((trimuf(lbd) * 0.5 + 0.49)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR3 = (uint16_t) ((trimuf(rbd) * 0.5 + 0.49)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR4 = (uint16_t) ((trimuf(rtd) * 0.5 + 0.49)
		* (float) PWM_MAXCOUNT);

	return 0;
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
	dsp_initpidval(&pitchpv, st.p, st.i, st.d, 0.0);
	dsp_initpidval(&rollpv, st.p, st.i, st.d, 0.0);

	// init roll, pitch and yaw speed PID controller contexts
	dsp_initpidval(&pitchspv, st.sp, st.si, st.sd, 0.0);
	dsp_initpidval(&rollspv, st.sp, st.si, st.sd, 0.0);
	dsp_initpidval(&yawspv, st.ysp, st.ysi, st.ysd, 0.0);

	// init yaw position PID controller's context
	dsp_initpidval(&yawpv, st.yp, st.yi, st.yd, 0.0);

	// init vertical acceleration PID controller's context
	dsp_initpidval(&tpv, st.zsp, st.zsi, st.zsd, 0.0);

	// init climbrate PID controller's context
	dsp_initpidval(&cpv, st.cp, st.ci, st.cd, 0.0);

	// init altitude PID controller's context
	dsp_initpidval(&apv, st.ap, st.ai, st.ad, 0.0);

	// init low-pass fitlers for altitude and vertical acceleration
	dsp_initlpf(&altlpf, st.atcoef, HP_FREQ);
	dsp_initlpf(&tlpf, st.ttcoef, PID_FREQ);
	dsp_initlpf(&valpf, st.vatcoef, PID_FREQ);

	return 0;
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
	float dt;

	// get time passed from last invocation of this calback function
	dt = ms / (float) TICKSPERSEC;

	// divide-by-zero protection
	dt = (dt < 0.000001) ? 0.000001 : dt;

	// toggle arming indication led
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
		en ? GPIO_PIN_SET : GPIO_PIN_RESET);

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
	id.afx -= st.ax0;
	id.afy -= st.ay0;
	id.afz -= st.az0;

	// update vertical acceleration low-pass filter
	dsp_updatelpf(&tlpf, id.afz);

	// offset gyroscope readings by values, calculater
	// at power on and convert result into radians
	gx = deg2rad((id.gfx - st.gx0));
	gy = deg2rad((id.gfy - st.gy0));
	gz = deg2rad((id.gfz - st.gz0));

	// update complimenraty filter for roll axis and get next roll
	// value. First signal (value) is signal to be integrated: it's
	// the speed of the rotation around Y axis. Second signal is
	// signal to be low-pass filtered: it's the tilt value that is
	// calculated from acceleromer readings through some
	// trigonometry.
	roll = dsp_updatecompl(&rollcompl, gy * dt,
		atan2f(-id.afx,
		sqrt(id.afy * id.afy + id.afz * id.afz))) - st.roll0;

	// same as for roll but for different axes
	pitch = dsp_updatecompl(&pitchcompl, gx * dt,
		atan2f(id.afy,
		sqrt(id.afx * id.afx + id.afz * id.afz))) - st.pitch0;

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
	dsp_updatelpf(&valpf, (vx * id.afx + vy * id.afy + vz * id.afz)
		/ sqrtf(vx * vx + vy * vy + vz * vz));

	// write roll, pitch and yaw values into log
	logwrite(LOG_ROLL, roll);
	logwrite(LOG_PITCH, pitch);
	logwrite(LOG_YAW, yaw);

	if (st.speedpid) {
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

	ltm = (1.0 - st.rsc / 2) * (1.0 + st.psc / 2);
	lbm = (1.0 - st.rsc / 2) * (1.0 - st.psc / 2);
	rbm = (1.0 + st.rsc / 2) * (1.0 - st.psc / 2);
	rtm = (1.0 + st.rsc / 2) * (1.0 + st.psc / 2);

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
	dev[QMC_DEV].read(dev[QMC_DEV].priv, &qmcdata,
		sizeof(struct qmc_data));

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,
		!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8));


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

	dev[CRSF_DEV].write(dev[CRSF_DEV].priv, &tele,
		sizeof(struct crsf_tele));

	return 0;
}

// Parse configureation command got from debug wi-fi connection
// by just splitting it into tokens by spaces
//
// toks -- result array of command tokens (it's a paring result).
// maxtoks -- maximum number of tokens posible.
// data -- command to be parsed.
int parsecommand(char **toks, int maxtoks, char *data)
{
	int i;

	for (i = 0; i < maxtoks; ++i)
		toks[i] = "";

	i = 0;

	toks[i++] = strtok((char *) data, " ");

	while (i < maxtoks && (toks[i++] = strtok(NULL, " ")) != NULL);

	return (i - 1);
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
		(double) id->afx, (double) id->afy, (double) id->afz);
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r", "gyro: ",
		(double) id->gfx, (double) id->gfy, (double) id->gfz);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r",
		"accel corrected: ",
		(double) (id->afx - st.ax0),
		(double) (id->afy - st.ay0),
		(double) (id->afz - st.az0));
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r",
		"gyro corrected: ",
		(double) (id->gfx - st.gx0),
		(double) (id->gfy - st.gy0),
		(double) (id->gfz - st.gz0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll: %0.3f; pitch: %0.3f; yaw: %0.3f\n\r",
		(double) (dsp_getcompl(&rollcompl) - st.roll0),
		(double) (dsp_getcompl(&pitchcompl) - st.pitch0),
		(double) circf(dsp_getcompl(&yawcompl) - st.yaw0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"z acceleration: %f\r\n",
		(double) dsp_getlpf(&tlpf));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"vertival acceleration: %f\r\n",
		(double) dsp_getlpf(&valpf));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"presice  altitude: %f\r\n",
		(double) (dsp_getcompl(&altcompl) - alt0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"battery: %0.3f\n\r",
		getadcv(&hadc1) / (double) 0xfff * (double) 9.9276);

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
		"motors state: %.3f\r\n", (double) en);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"loops count: %d\r\n", loopscount);

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
		"accel lpf tc: %.6f\r\n", (double) st.ttcoef);

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
		"%s mode\r\n", st.speedpid ? "single" : "dual");

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

// Print all control scaling values
//
// s -- output string.
int sprintfctrl(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum roll: %.5f\r\n", (double) st.rollmax);
	
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum pitch: %.5f\r\n", (double) st.pitchmax);

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

// Erase log flash to prepare at for writing,
// erasing starts from address 0.
//
// size -- bytes count to erase.
int eraseflash(size_t size)
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
			dev[ESP_DEV].write(dev[ESP_DEV].priv, s,
				strlen(s));
		}
		else {
			flashdev.erasesector(flashdev.priv, pos);
			pos += W25_SECTORSIZE;

			sprintf(s, "erased sector at %u\r\n", pos);
			dev[ESP_DEV].write(dev[ESP_DEV].priv, s,
				strlen(s));
		}
	}

	return 0;
}

// Print all log values into debug connection.
//
// s -- string user as buffer.
int printlog(char *buf, size_t size)
{
	int fp;
	int frames;

	size = (size > W25_TOTALSIZE) ? W25_TOTALSIZE : size;

	frames = size / sizeof(struct logpack);

	// run through all writable space in the flash
	for (fp = 0; fp < frames; fp += LOG_BUFSIZE) {
		int bp;

		// read batch of log frames into log buffer
		flashdev.read(flashdev.priv,
			fp * sizeof(struct logpack), logbuf,
			LOG_BUFSIZE * sizeof(struct logpack));

		// for every read frame
		for (bp = 0; bp < LOG_BUFSIZE; ++bp) {
			int i;

			// put all frame's values into a string
			buf[0] = '\0';
			for (i = 0; i < LOG_PACKSIZE; ++i) {
				sprintf(buf + strlen(buf), "%0.5f ",
					(double) logbuf[bp].data[i]);
			}
			sprintf(buf + strlen(buf), "\r\n");

			// send this string into debug connection
			dev[ESP_DEV].write(dev[ESP_DEV].priv, buf,
				strlen(buf));
		}
	}
			
	
	sprintf(buf, "log finished, %u frames written\r\n", frames);
	dev[ESP_DEV].write(dev[ESP_DEV].priv, buf, strlen(buf));

	return 0;
}

// Disarm command handler.
//
// toks -- list of parsed command tokens.
int rcmd(const char **toks)
{
	en = 0.0;
	
	return 0;
}

// Recalibrate command handler.
//
// toks -- list of parsed command tokens.
int ccmd(const char **toks)
{
	initstabilize(atof(toks[1]));

	return 0;
}

// "info" command handler. Print various sensor and control values.
//
// toks -- list of parsed command tokens.
int infocmd(const char **toks)
{
	char s[INFOLEN];
	
	if (strcmp(toks[1], "mpu") == 0) {
		struct icm_data id;

		dev[ICM_DEV].read(dev[ICM_DEV].priv, &id,
			sizeof(struct icm_data));



		id.afx -= st.ax0;
		id.afy -= st.ay0;
		id.afz -= st.az0;

		sprintpos(s, &id);
	}
	else if (strcmp(toks[1], "qmc") == 0) {
		struct qmc_data hd;

		dev[QMC_DEV].read(dev[QMC_DEV].priv, &hd,
			sizeof(struct qmc_data));

		sprintqmc(s, &hd);	
	}
	else if (strcmp(toks[1], "hp") == 0) {
		float alt;

		alt = dsp_getlpf(&altlpf) - alt0;

		snprintf(s, INFOLEN,
			"temp: %f; alt: %f; climb rate: %f\r\n",
			(double) temp, (double) alt,
			(double) dsp_getcompl(&climbratecompl));
	}
	else if (strcmp(toks[1], "dev") == 0)
		sprintdevs(s);
	else if (strcmp(toks[1], "values") == 0)
		sprintvalues(s);
	else if (strcmp(toks[1], "pid") == 0)
		sprintpid(s);
	else if (strcmp(toks[1], "gnss") == 0)
		sprintgnss(s);
	else if (strcmp(toks[1], "ctrl") == 0)
		sprintfctrl(s);
	else if (strcmp(toks[1], "filter") == 0)
		sprintffilters(s);
	else
		return (-1);

	dev[ESP_DEV].write(dev[ESP_DEV].priv, s, strlen(s));

	return 0;
}

// "pid" command handler. Configure PID values.
//
// toks -- list of parsed command tokens.
int pidcmd(const char **toks)
{
	float v;

	v = atof(toks[3]);

	if (strcmp(toks[1], "tilt") == 0) {
		if (strcmp(toks[2], "mode") == 0) {
			if (strcmp(toks[3], "double") == 0)
				st.speedpid = 0;
			else if (strcmp(toks[3], "single") == 0)
				st.speedpid = 1;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "p") == 0)	st.p = v;
		else if (strcmp(toks[2], "i") == 0)	st.i = v;
		else if (strcmp(toks[2], "d") == 0)	st.d = v;
		else					return (-1);

		dsp_setpid(&pitchpv, st.p, st.i, st.d);
		dsp_setpid(&rollpv, st.p, st.i, st.d);
	}
	else if (strcmp(toks[1], "stilt") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.sp = v;
		else if (strcmp(toks[2], "i") == 0)	st.si = v;
		else if (strcmp(toks[2], "d") == 0)	st.sd = v;
		else					return (-1);

		dsp_setpid(&pitchspv, st.sp, st.si, st.sd);
		dsp_setpid(&rollspv, st.sp, st.si, st.sd);
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

		dsp_setpid(&yawpv, st.yp, st.yi, st.yd);
	}
	else if (strcmp(toks[1], "syaw") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.ysp = v;
		else if (strcmp(toks[2], "i") == 0)	st.ysi = v;
		else if (strcmp(toks[2], "d") == 0)	st.ysd = v;
		else					return (-1);

		dsp_setpid(&yawspv, st.ysp, st.ysi, st.ysd);
	}
	else if (strcmp(toks[1], "throttle") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.zsp = v;
		else if (strcmp(toks[2], "i") == 0)	st.zsi = v;
		else if (strcmp(toks[2], "d") == 0)	st.zsd = v;
		else					return (-1);

		dsp_setpid(&tpv, st.zsp, st.zsi, st.zsd);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.cp = v;
		else if (strcmp(toks[2], "i") == 0)	st.ci = v;
		else if (strcmp(toks[2], "d") == 0)	st.cd = v;
		else					return (-1);

		dsp_setpid(&cpv, st.cp, st.ci, st.cd);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.ap = v;
		else if (strcmp(toks[2], "i") == 0)	st.ai = v;
		else if (strcmp(toks[2], "d") == 0)	st.ad = v;
		else					return (-1);

		dsp_setpid(&apv, st.ap, st.ai, st.ad);
	}
	else
		return (-1);
	
	return 0;
}

// "calib" command handler. Turn on/off devices calibration modes.
//
// toks -- list of parsed command tokens.
int calibcmd(const char **toks)
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
int flashcmd(const char **toks)
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
int complcmd(const char **toks)
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
int lpfcmd(const char **toks)
{
	if (strcmp(toks[1], "climb") == 0) {
		st.ttcoef = atof(toks[2]);

		dsp_initlpf(&tlpf, st.ttcoef, PID_FREQ);
	}
	else if (strcmp(toks[1], "vaccel") == 0) {
		st.vatcoef = atof(toks[2]);

		dsp_initlpf(&valpf, st.vatcoef, PID_FREQ);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		st.atcoef = atof(toks[2]);

		dsp_initlpf(&altlpf, st.atcoef, HP_FREQ);
	}
	else
		return (-1);

	return 0;
}

// "adj" command handler. Configure various offset/scale values.
//
// toks -- list of parsed command tokens.
int adjcmd(const char **toks)
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
	}
	else if (strcmp(toks[1], "gyro") == 0) {
		if (strcmp(toks[2], "x") == 0)
			st.gx0 = atof(toks[3]);
		else if (strcmp(toks[2], "y") == 0)
			st.gy0 = atof(toks[3]);
		else if (strcmp(toks[2], "z") == 0)
			st.gz0 = atof(toks[3]);
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
int logcmd(const char **toks)
{
	char s[INFOLEN];
	
	if (strcmp(toks[1], "set") == 0) {
		// flash erasing process takes time and blocks
		// other actions, so disarm for safety
		en = 0.0;
		setthrust(0.0, 0.0, 0.0, 0.0);

		// notify user when erasing is started
		sprintf(s, "erasing flash...\r\n");
		dev[ESP_DEV].write(dev[ESP_DEV].priv, s,
			strlen(s));

		// set log size (0 is valid and
		// means to disable logging)
		if (atoi(toks[2]) > W25_TOTALSIZE)
			return (-1);

		logsize = atoi(toks[2]);

		// erase log flash no
		// respond during this process
		eraseflash(logsize);

		// notify user when telemetry flash is erased
		sprintf(s,"erased %u bytes of flash\r\n",
			logsize);
		dev[ESP_DEV].write(dev[ESP_DEV].priv, s,
			strlen(s));

		// enable telemetry
		logtotal = 0;
		logflashpos = 0;
		logbufpos = 0;
	}
	else if (strcmp(toks[1], "get") == 0)
		printlog(s, atoi(toks[2]));
	else
		return (-1);

	return 0;
}


// "ctrl" command handler. Start/stop/get flight log.
//
// toks -- list of parsed command tokens.
int ctrlcmd(const char **toks)
{
	float v;

	v = atof(toks[2]);

	if (strcmp(toks[1], "roll") == 0)
		st.rollmax = v;
	else if (strcmp(toks[1], "pitch") == 0)
		st.pitchmax = v;
	else if (strcmp(toks[1], "syaw") == 0)
		st.yawspeed = v;
	else if (strcmp(toks[1], "yaw") == 0)
		st.yawtargetspeed = v;
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

// Parse and execute control command bot from debug wifi-connetion.
//
// cmd -- command to be executed.
int runcommand(char *cmd)
{
	char s[INFOLEN];
	char *toks[MAX_CMDTOKS];
	int i;

	if (cmd[0] == '\0')
		return 0;

	// split a command into tokens by spaces
	parsecommand(toks, MAX_CMDTOKS, cmd);

	// perform corresponding action
	for (i = 0; i < commcount; ++i) {
		if (strcmp(toks[0], commtable[i].name) == 0) {
			if (commtable[i].func((const char **) toks) < 0)
				goto unknown;

			return 0;
		}
	}

	return 0;

unknown:
	snprintf(s, INFOLEN, "Unknown command: %s\r\n", cmd);
	dev[ESP_DEV].write(dev[ESP_DEV].priv, s, strlen(s));

	return (-1);
}

// Set control values using CRSF packet.
//
// cd -- CRSF packet
// ms -- microsecond passed from last CRSF packet
int crsfcmd(const struct crsf_data *cd, int ms)
{
	float dt;

	// channel to on remote is used to turn on/off
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

	// set pitch/roll targets based on
	// channels 1-2 values (it's a joystick on most remotes)
	rolltarget = cd->chf[0] * (M_PI * st.rollmax);
	pitchtarget = -cd->chf[1] * (M_PI * st.pitchmax);

	if (cd->chf[4] < 0.0) {
		yawspeedpid = 0;
		yawtarget = circf(yawtarget
			+ cd->chf[3] * dt * M_PI * st.yawtargetspeed);
	}
	else {
		yawspeedpid = 1;
		yawtarget = -cd->chf[3] * M_PI * st.yawspeed;
	}

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
	if (cd->chf[8] > 0.0) {
		dsp_initcompl(&climbratecompl, st.cctcoef, HP_FREQ);

		alt0 = dsp_getlpf(&altlpf);
	}

	// enable motors if channel six has value
	// more than 50, disarm immediately otherwise
	en = (cd->chf[5] > 0.5) ? 1.0 : 0.0;

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

	// other needed data is got RMC messages. Discard
	// all other messages.
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
	dma_init();
	i2c_init();
	spi1_init();
	adc1_init();
	usart1_init();
	usart2_init();
	usart3_init();
	uart4_init();

	// init board's devices
	esc_init();
	icm_init();
	qmc_init();
	espdev_init();
	crsfdev_init();
	w25dev_init();
	m10dev_init();
	hp_init();

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

	// initilize ERLS timer. For now ERLS polling is not a periodic
	// event and called as frequently as possible, so it needs this
	// separate timer.
	elrsus = 0;

	// main control loop
	while (1) {
		char cmd[ESP_CMDSIZE];
		struct crsf_data cd;
		struct m10_data nd;
		int c, i;

		// reset iteration time counter
		__HAL_TIM_SET_COUNTER(&htim8, 0);

		// poll for configureation and telemetry commands
		// from from debug wifi connection
		if (dev[ESP_DEV].read(dev[ESP_DEV].priv, &cmd,
			ESP_CMDSIZE) >= 0) {
			runcommand(cmd);
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

void systemclock_config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 128;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;

	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		error_handler();

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
		| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
		| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
			FLASH_LATENCY_4) != HAL_OK)
		error_handler();
}

static void gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,
		GPIO_PIN_RESET);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
		| GPIO_PIN_13, GPIO_PIN_RESET);
	
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void dma_init(void)
{
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
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
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
	htim1.Init.Prescaler = 0;//PRESCALER - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = PWM_MAXCOUNT;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

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

static void tim8_init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = PRESCALER - 1;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = TIMPERIOD - 1;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
		error_handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
		error_handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
		error_handler();

	HAL_TIM_Base_Start_IT(&htim8);
}

static void adc1_init(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
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

static void usart3_init()
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart3) != HAL_OK)
		error_handler();
}

static void uart4_init()
{
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 921600;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart4) != HAL_OK)
		error_handler();
}

// Init ESC's
static void esc_init()
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4
		= (uint16_t) (0.5 * (float) PWM_MAXCOUNT);
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

static void icm_init()
{
	struct icm_device d;

	d.hspi = &hspi1;
	d.gpio = GPIOC;
	d.pin = GPIO_PIN_13;

	d.gyroscale = ICM_1000DPS;
	d.gyrorate = ICM_GYRO1K;
	d.gyroorder = ICM_GYROORDER3;
	d.gyrolpf = ICM_GYROLPF4;
	d.accelscale = ICM_4G;
	d.accelrate = ICM_ACCEL1K;
	d.accellpf = ICM_ACCELLPF4;
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
	d.osr = QMC_OSR_256;

	if (qmc_initdevice(&d, dev + QMC_DEV) >= 0)
		uartprintf("QMC5883L initilized\r\n");
	else
		uartprintf("failed to initilize QMC5883L\r\n");
}

// Init ESP07
static void espdev_init()
{
	struct esp_device d;

	d.huart = &huart1;
	d.ssid = "copter";
	d.pass = "";
	d.port = SERVPORT;

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

// Init ERLS receiver driver
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

// Init GPS module
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

// MCU hardware errors handler. Disarm immediately
// in case of any of such error.
void error_handler(void)
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0;

	__disable_irq();
	while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
