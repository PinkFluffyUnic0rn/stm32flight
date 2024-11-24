#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "main.h"
#include "device.h"
#include "uartdebug.h"
#include "mpu6500.h"
#include "hp206c.h"
#include "esp8266.h"
#include "qmc5883l.h"
#include "dsp.h"
#include "crsf.h"

// Max length for info packet
// sent back to operator
#define INFOLEN 512

// device numbers
#define MPU_DEV 0
#define HP_DEV 1
#define QMC_DEV 2
#define CRSF_DEV 3
#define DEV_COUNT 4

// timers prescaler
#define PRESCALER 48

// clock frequency
#define OCSFREQ 48000000

// period for main timer (used to timing periodic events)
#define TIMPERIOD 0xfff

// main timer ticks per second
#define TICKSPERSEC (OCSFREQ / PRESCALER)

// PWM settings
#define PWM_MAXCOUNT 2500

// Periodic events frequencies
#define PID_FREQ 1000
#define CALIB_FREQ 25
#define HP_FREQ 25
#define CRSF_FREQ 100
#define QMC_FREQ 100

// MCU flash address where quadcopter settings is stored
#define USER_FLASH 0x0801f800

// quadcopter setting's slot
#define USER_SETSLOTS (0x80 / sizeof(struct settings))

// Timer events IDs
#define TEV_PID 	0
#define TEV_CHECK 	1
#define TEV_CALIB	2
#define TEV_HP		3
#define TEV_QMC		4
#define TEV_COUNT	5

// Debug connection port
#define SERVPORT 8880

// Timeout in seconds before quadcopter disarm
// when got no data from ERLS receiver
#define ELRS_TIMEOUT 2

// check if enough time passed to call periodic event again.
// ev -- periodic event's context.
#define checktimev(ev) ((ev)->ms > TICKSPERSEC / (ev)->freq)

// reset periodic event's counter. Used after every
// event's callback call.
// ev -- periodic event's context.
#define resettimev(ev) (ev)->ms = 0;

// update periodic event's counter.
// ev -- periodic event's context.
// s -- time in microseconds after last callback's call.
#define updatetimev(ev, s) (ev)->ms += (s);

// Quadcopter settings structure stored in MCU's flash
struct settings {
	float mx0, my0, mz0;	// magnetometer offset values for X, Y
				// and Z axes determinted by
				// calibration procedure

	float mxsc, mysc, mzsc;	// megnetormeter scaling values for Z,
				// Y and Z axes determinted by 
				// calibration procedure
	
	float magdecl;	// magnetic declination

	float gx0, gy0, gz0;	// gyroscope offset values for X, Y
				// and Z axes. Currently unused: these
				// values determined at power on

	float roll0, pitch0, yaw0; // roll, pitch and yaw offset values

	float tcoef;		// time coefficient for pitch/roll
				// complimentary filter
	float ttcoef;		// time coefficient for vertical axis
				// acceleration pow-pass filter
	float atcoef;		// time coefficient for pressure
				// low-pass filter
	float climbcoef;	// time coefficient for climb rate
				// low-pass filter

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
	
	float ap, ai, ad; // P/I/D values for altitude
};

// Periodic event's context that holds event's settings
// and data needed beetween calls
struct timev {
	int ms;			// microseconds passed from
				// last triggering
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
static void tim2_init();
static void adc1_init(void);
static void usart1_init();
static void usart2_init();
static void usart3_init();

// Init ESC's
static void esc_init();

// Init HP206c barometer
static void hp_init();

// Init MPU6050 IMU
static void mpu_init();

// Init QMC5883L magnetometer
static void qmc_init();

// Init ESP07
static void espdev_init();

// Init ERLS receiver driver
static void crsfdev_init();

// STM32 perithery contexts
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

// Flight controller board's devices drivers
struct cdevice dev[DEV_COUNT];
struct esp_device espdev;
struct crsf_device crsfdev;

// DSP contexts
struct dsp_lpf tlpf;
struct dsp_lpf altlpf;
struct dsp_lpf climblpf;
float temp;
struct qmc_data qmcdata;
float climbrate;

struct dsp_compl pitchcompl;
struct dsp_compl rollcompl;

struct dsp_pidval pitchpv;
struct dsp_pidval rollpv;
struct dsp_pidval pitchspv;
struct dsp_pidval rollspv;
struct dsp_pidval yawpv;
struct dsp_pidval yawspv;
struct dsp_pidval tpv;
struct dsp_pidval apv;

// Control values
float thrust = 0.0; // motors basic thrust
float rolltarget = 0.0; // roll PID target
float pitchtarget = 0.0; // pitch PID target
float yawtarget = 0.0; // yaw PID target
float en = 0.0; // 1.0 when motors turned on, 0.0 otherwise
int althold = 0; // 1 if altitude holding mode is enabled, 0 otherwise
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

// Get value from ADC, was used to monitor battery voltage
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
	esp_interrupt(&espdev, huart);
	
	if (dev[CRSF_DEV].status == DEVSTATUS_INIT)
		dev[CRSF_DEV].interrupt(dev[CRSF_DEV].priv, huart);
}

// UART error callback. Calls UART error handlers from
// drivers for devices working through UART.
//
// huart -- context for UART triggered that callback.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	esp_error(&espdev, huart);
}

// Initilize periodic event's context
//
// ev -- periodic event's context.
// freq -- periodic event's frequency in Hz (calls per second).
// cb -- callback that is called by this event.
int inittimev(struct timev *ev, int freq, int (*cb)(int))
{
	ev->ms = 0;
	ev->freq = freq;
	ev->cb = cb;

	return 0;
}

// Get IMU accelerometer and gyroscope averaged values
// accumulated for 2.5 second
//
// ax -- accelerometer X axis value.
// ay -- accelerometer Y axis value.
// az -- accelerometer Z axis value.
// ax -- gyroscope X axis value.
// ay -- gyroscope Y axis value.
// az -- gyroscope Z axis value.
int averageposition(float *ax, float *ay, float *az, float *gx,
	float *gy, float *gz)
{
	struct mpu_data md;
	float axt, ayt, azt, gxt, gyt, gzt;
	int i;

	axt = ayt = azt = gxt = gyt = gzt = 0;
	for (i = 0; i < 250; ++i) {
		dev[MPU_DEV].read(dev[MPU_DEV].priv, &md,
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
		+ z * cosf(r) * sinf(p);
	y = y * cosf(r) - z * sinf(r);

	return circf(atan2f(y, x) + st.magdecl);
}

/* Set motors thrust

      ltd    rtd
        \    /
   x     \  /
   |       
   v     /  \
        /    \
      lbd    rbd
        
         y ->
*/
// all values should be between 0.0 and 1.0.
int setthrust(float ltd, float rtd, float rbd, float lbd)
{
	if (isnan(ltd) || isnan(rtd) || isnan(rbd)
		|| isnan(lbd) || !elrs)
		ltd = rtd = rbd = lbd = 0.0;

	TIM1->CCR1 = (uint16_t) ((trimuf(ltd) * 0.05 + 0.049)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR2 = (uint16_t) ((trimuf(rtd) * 0.05 + 0.049)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR3 = (uint16_t) ((trimuf(rbd) * 0.05 + 0.049)
		* (float) PWM_MAXCOUNT);
	TIM1->CCR4 = (uint16_t) ((trimuf(lbd) * 0.05 + 0.049)
		* (float) PWM_MAXCOUNT);

	return 0;
}

// write quadcopter settings into internal MCU flash
//
// slot -- offset in settings array in flash.
int writesettings(int slot)
{
	FLASH_EraseInitTypeDef ferase;
	uint32_t serror;
	uint32_t sz;
	uint32_t *pt;
	uint32_t addr;
	int j;

	__disable_irq();
	HAL_FLASH_Unlock();

	ferase.TypeErase = FLASH_TYPEERASE_PAGES;
	ferase.PageAddress = USER_FLASH;
	ferase.NbPages = 1;
	HAL_FLASHEx_Erase(&ferase, &serror);

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
	float ax, ay, az;

	// set initial altitude. Usually 0.0
	alt0 = alt;

	// calculate initial acceleromter and gyroscope offsets. These
	// ofset's came from device inperfections (and sometimes as
	// result of harsh impacts). Accelerometer offsets are not used,
	// gyroscope values stored and used in the stabilization loop.
	averageposition(&ax, &ay, &az, &(st.gx0), &(st.gy0), &(st.gz0));

	// init complementary filters contexts
	dsp_initcompl(&pitchcompl, st.tcoef, PID_FREQ);
	dsp_initcompl(&rollcompl, st.tcoef, PID_FREQ);

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
	
	// init altitude PID controller's context
	dsp_initpidval(&apv, st.ap, st.ai, st.ad, 0.0);

	// init low-pass fitlers for altitude and vertical acceleration
	dsp_initlpf(&altlpf, st.atcoef, HP_FREQ);
	dsp_initlpf(&climblpf, st.climbcoef, HP_FREQ);
	dsp_initlpf(&tlpf, st.ttcoef, PID_FREQ);

	return 0;
}

// Stabilization loop. Callback for TEV_PID periodic event. It is the
// place where is almost all work happening.
//
// ms -- microsecond passed from last callback invocation.
int stabilize(int ms)
{
	struct mpu_data md;
	float roll, pitch, yaw;
	float rollcor, pitchcor, yawcor, thrustcor;
	float gy, gx, gz;
	float dt;
		
	// get time passed from last invocation of this calback function
	dt = ms / (float) TICKSPERSEC;

	// divide-by-zero protection
	dt = (dt < 0.000001) ? 0.000001 : dt;

	// get accelerometer and gyroscope readings
	dev[MPU_DEV].read(dev[MPU_DEV].priv, &md,
		sizeof(struct mpu_data));

	// update vertical acceleration low-pass filter
	dsp_updatelpf(&tlpf, md.afz);

	// offset gyroscope readings by values, calculater
	// at power on and convert result into radians
	gy = deg2rad((md.gfy - st.gy0));
	gx = deg2rad((md.gfx - st.gx0));
	gz = deg2rad((md.gfz - st.gz0));

	// update complimenraty filter for roll axis and get next roll
	// value. First signal (value) is signal to be integrated: it's
	// the speed of the rotation around Y axis. Second signal is
	// signal to be low-pass filtered: it's the tilt value that is
	// calculated from acceleromer readings through some
	// trigonometry.
	roll = dsp_updatecompl(&rollcompl, gy * dt,
		atan2f(-md.afx,
		sqrt(md.afy * md.afy + md.afz * md.afz))) - st.roll0;

	// same as for roll but for different axes
	pitch = dsp_updatecompl(&pitchcompl, gx * dt,
		atan2f(md.afy,
		sqrt(md.afx * md.afx + md.afz * md.afz))) - st.pitch0;

	// calcualte yaw value using last magnetometer reading, roll
	// value and pitch value, offset it by a value from settings.
	yaw = circf(qmc_heading(pitch, roll,
		qmcdata.fx, qmcdata.fy, qmcdata.fz) - st.yaw0);

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

	if (st.yawspeedpid) {
		// if single PID loop mode for yaw is used just use
		// rotation speed values around axis Z to upadte yaw PID
		// controller and get next yaw correciton value
		yawcor = dsp_pid(&yawspv, yawtarget, gz, dt);
	}
	else {
		// if in double loop mode for yaw, first use yaw value
		// calcualted using magnetometer and yaw target got from
		// ERLS remove to update yaw POSITION PID controller and
		// get it's next correciton value.
		yawcor = dsp_circpid(&yawpv, yawtarget, yaw, dt);

		// then use this value to update yaw speed PID
		// controller and get next yaw SPEED correction value
		yawcor = dsp_pid(&yawspv, yawcor, gz, dt);
	}

	if (althold) {
		thrustcor = dsp_pid(&apv, thrust,
			dsp_getlpf(&climblpf), dt);

		thrustcor = dsp_pid(&tpv, thrustcor + 1.0,
			dsp_getlpf(&tlpf), dt);
	}
	else {
		// update vertical acceleration PID controller using
		// next low-pass filtered value of vertical acceleration
		// and target got from ERLS remote
		thrustcor = dsp_pid(&tpv, thrust + 1.0,
			dsp_getlpf(&tlpf), dt);
	}

	// update motors thrust based on calculated values. For
	// quadcopter it's enought to split correction in half for 
	// 3 pairs of motors: left and right for roll, top and bottom
	// for pitch and two diagonals (spinning in oposite directions)
	// for yaw.
	setthrust(en * (thrustcor + 0.5 * rollcor
			+ 0.5 * pitchcor - 0.5 * yawcor),
		en * (thrustcor - 0.5 * rollcor
			+ 0.5 * pitchcor + 0.5 * yawcor),
		en * (thrustcor - 0.5 * rollcor
			- 0.5 * pitchcor - 0.5 * yawcor),
		en * (thrustcor + 0.5 * rollcor
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

	esp_send(&espdev, s);

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

	prevalt = dsp_getlpf(&altlpf);

	// upate altitude low-pass filter and temperature reading
	dsp_updatelpf(&altlpf, hd.altf);
	temp = hd.tempf;

	dsp_updatelpf(&climblpf, (dsp_getlpf(&altlpf) - prevalt) / dt);

	return 0;
}

// Get readings from magnetomer. Callback for TEV_QMC periodic event.
//
// ms -- microsecond passed from last callback invocation.
int qmcupdate(int ms)
{
	dev[QMC_DEV].read(dev[QMC_DEV].priv, &qmcdata,
		sizeof(struct qmc_data));

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
int sprintpos(char *s, struct mpu_data *md, struct qmc_data *hd)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r", "accel: ",
		(double) md->afx, (double) md->afy, (double) md->afz);
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\n\r", "gyro: ",
		(double) md->gfx, (double) md->gfy, (double) md->gfz);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll: %0.3f; pitch: %0.3f; yaw: %0.3f\n\r",
		(double) (dsp_getcompl(&rollcompl) - st.roll0),
		(double) (dsp_getcompl(&pitchcompl) - st.pitch0),
		(double) circf(qmc_heading(
			-(dsp_getcompl(&pitchcompl) - st.pitch0),
			-(dsp_getcompl(&rollcompl) - st.roll0),
			hd->fx, hd->fy, hd->fz) - st.yaw0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"z acceleration: %f\r\n",
		(double) dsp_getlpf(&tlpf));

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
		"heading: %f\r\n", (double) qmc_heading(
			(dsp_getcompl(&pitchcompl) - st.pitch0),
			(dsp_getcompl(&rollcompl) - st.roll0),
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
		"mag off: %.3f; %.3f; %.3f\r\n",
		(double) st.mx0, (double) st.my0, (double) st.mz0);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag scale: %.3f; %.3f; %.3f\r\n",
		(double) st.mxsc, (double) st.mysc, (double) st.mzsc);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag decl: %.5f\r\n",
		(double) st.magdecl);

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
		"tc: %.6f\r\n", (double) st.tcoef);
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel tc: %.6f\r\n", (double) st.ttcoef);
	
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"climb rate tc: %.6f\r\n", (double) st.climbcoef);
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude tc: %.6f\r\n", (double) st.atcoef);
	
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
		"altitude PID: %.5f,%.5f,%.5f\r\n",
		(double) st.ap, (double) st.ai, (double) st.ad);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s mode\r\n", st.speedpid ? "single" : "dual");

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s yaw mode\r\n",
		st.yawspeedpid ? "single" : "dual");
	
	return 0;
}

// Parse and execute control command bot from debug wifi-connetion.
//
// cmd -- command to be executed.
int controlcmd(char *cmd)
{
	char s[INFOLEN];
	char *toks[12];

	if (cmd[0] == '\0')
		return 0;

	// split a command into tokens by spaces
	parsecommand(toks, 12, cmd);

	// perform corresponding action. Here is simple
	// recursive descent is used.
	if (strcmp(toks[0], "info") == 0) {
		if (strcmp(toks[1], "mpu") == 0) {
			struct mpu_data md;
			struct qmc_data hd;

			dev[MPU_DEV].read(dev[MPU_DEV].priv, &md,
				sizeof(struct mpu_data));

			dev[QMC_DEV].read(dev[QMC_DEV].priv, &hd,
				sizeof(struct qmc_data));

			sprintpos(s, &md, &hd);

			esp_send(&espdev, s);
		}
		else if (strcmp(toks[1], "qmc") == 0) {
			struct qmc_data hd;

			dev[QMC_DEV].read(dev[QMC_DEV].priv, &hd,
				sizeof(struct qmc_data));
		
			sprintqmc(s, &hd);	
		
			esp_send(&espdev, s);
		}
		else if (strcmp(toks[1], "hp") == 0) {
			float alt;

			alt = dsp_getlpf(&altlpf) - alt0;

			snprintf(s, INFOLEN,
				"temp: %f; alt: %f; climb rate: %f\r\n",
				(double) temp, (double) alt,
				(double) dsp_getlpf(&climblpf));
			
			esp_send(&espdev, s);
		}
		else if (strcmp(toks[1], "values") == 0) {
			sprintvalues(s);
			esp_send(&espdev, s);
		}
		else if (strcmp(toks[1], "pid") == 0) {
			sprintpid(s);
			esp_send(&espdev, s);
		}
	}
	else if (strcmp(toks[0], "r") == 0)
		en = 0.0;
	else if (strcmp(toks[0], "e") == 0)
		en = 1.0;
	else if (strcmp(toks[0], "c") == 0)
		initstabilize(atof(toks[1]));
	else if (strcmp(toks[0], "calib") == 0) {
		if (strcmp(toks[1], "mag") == 0) {
			if (strcmp(toks[2], "on") == 0)
				magcalibmode = 1;
			else if (strcmp(toks[2], "off") == 0)
				magcalibmode = 0;
			else
				goto unknown;
		}
		else
			goto unknown;
	}
	else if (strcmp(toks[0], "flash") == 0) {
		if (strcmp(toks[1], "write") == 0)
			writesettings(atoi(toks[2]));	
		else if (strcmp(toks[1], "read") == 0)
			readsettings(atoi(toks[2]));
	}
	else if (strcmp(toks[0], "pid") == 0) {
		float v;
	
		v = atof(toks[3]);
		
		if (strcmp(toks[1], "tilt") == 0) {
			if (strcmp(toks[2], "mode") == 0) {
				if (strcmp(toks[3], "double") == 0)
					st.speedpid = 0;
				else if (strcmp(toks[3], "single") == 0)
					st.speedpid = 1;
				else
					goto unknown;
			}
			else if (strcmp(toks[2], "p") == 0)
				st.p = v;
			else if (strcmp(toks[2], "i") == 0)
				st.i = v;
			else if (strcmp(toks[2], "d") == 0)
				st.d = v;
			else
				goto unknown;
	
			dsp_setpid(&pitchpv, st.p, st.i, st.d);
			dsp_setpid(&rollpv, st.p, st.i, st.d);
		}
		else if (strcmp(toks[1], "stilt") == 0) {
			if (strcmp(toks[2], "p") == 0)
				st.sp = v;
			else if (strcmp(toks[2], "i") == 0)
				st.si = v;
			else if (strcmp(toks[2], "d") == 0)
				st.sd = v;
			else
				goto unknown;

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
					goto unknown;
			}
			else if (strcmp(toks[2], "p") == 0)
				st.yp = v;
			else if (strcmp(toks[2], "i") == 0)
				st.yi = v;
			else if (strcmp(toks[2], "d") == 0)
				st.yd = v;
			else
				goto unknown;
		
			dsp_setpid(&yawpv, st.yp, st.yi, st.yd);
		}
		else if (strcmp(toks[1], "syaw") == 0) {
			if (strcmp(toks[2], "p") == 0)
				st.ysp = v;
			else if (strcmp(toks[2], "i") == 0)
				st.ysi = v;
			else if (strcmp(toks[2], "d") == 0)
				st.ysd = v;
			else
				goto unknown;
		
			dsp_setpid(&yawspv, st.ysp, st.ysi, st.ysd);
		}
		else if (strcmp(toks[1], "climb") == 0) {
			if (strcmp(toks[2], "p") == 0)
				st.zsp = v;
			else if (strcmp(toks[2], "i") == 0)
				st.zsi = v;
			else if (strcmp(toks[2], "d") == 0)
				st.zsd = v;
			else
				goto unknown;

			dsp_setpid(&tpv, st.zsp, st.zsi, st.zsd);
		}
		else if (strcmp(toks[1], "altitude") == 0) {
			if (strcmp(toks[2], "p") == 0)
				st.ap = v;
			else if (strcmp(toks[2], "i") == 0)
				st.ai = v;
			else if (strcmp(toks[2], "d") == 0)
				st.ad = v;
			else
				goto unknown;

			dsp_setpid(&apv, st.ap, st.ai, st.ad);
		}
		else
			goto unknown;
	}
	else if (strcmp(toks[0], "compl") == 0) {
		st.tcoef = atof(toks[1]);
		
		dsp_initcompl(&pitchcompl, st.tcoef, PID_FREQ);
		dsp_initcompl(&rollcompl, st.tcoef, PID_FREQ);	
	}
	else if (strcmp(toks[0], "lpf") == 0) {
		if (strcmp(toks[1], "climb") == 0) {
			st.ttcoef = atof(toks[2]);
			
			dsp_initlpf(&tlpf, st.ttcoef, PID_FREQ);
		}
		else if (strcmp(toks[1], "climbrate") == 0) {
			st.climbcoef = atof(toks[2]);
			
			dsp_initlpf(&climblpf, st.climbcoef, HP_FREQ);
		}
		else if (strcmp(toks[1], "altitude") == 0) {
			st.atcoef = atof(toks[2]);
				
			dsp_initlpf(&altlpf, st.atcoef, HP_FREQ);
		}
		else
			goto unknown;
	}
	else if (strcmp(toks[0], "adj") == 0) {
		float v;
	
		v = atof(toks[2]);
		
		if (strcmp(toks[1], "roll") == 0)	st.roll0 = v;
		else if (strcmp(toks[1], "pitch") == 0)	st.pitch0 = v;
		else if (strcmp(toks[1], "yaw") == 0)	st.yaw0 = v;
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
				goto unknown;
		}
		else
			goto unknown;
	}
	else
		goto unknown;

	return 0;

unknown:
	snprintf(s, INFOLEN, "Unknown command: %s\r\n", cmd);
	esp_send(&espdev, s);

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
	// comming.
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

	// set pitch/roll/yaw/thrus targets based on
	// channels 1-4 values (it's joysticks on most remotes).
	pitchtarget = -cd->chf[0] * (M_PI / 6.0);
	rolltarget = -cd->chf[1] * (M_PI / 6.0);
	thrust = (cd->chf[2] + 0.75) / 3.5;
	yawtarget = circf(yawtarget + cd->chf[3] * dt * M_PI);

	if (cd->chf[8] > 0.0)
		alt0 = dsp_getlpf(&altlpf);

	althold = (cd->chf[4] > 0.5) ? 1 : 0;
	if (althold)
		thrust = cd->chf[2] * 1.5;

	// enable motors if channel six has value
	// more than 50, disarm immediately otherwise.
	en = (cd->chf[5] > 0.5) ? 1.0 : 0.0;

	if (en < 0.5)
		setthrust(0.0, 0.0, 0.0, 0.0);

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

	// wait a little to let stm32 periphery and
	// board's devices power on
	HAL_Delay(1000);

	// set initial status for all devices to prevent callback
	// calls on corresponding events before inittialization
	for (i = 0; i < DEV_COUNT; ++i)
		dev[i].status = DEVSTATUS_NOINIT;

	// init stm32 periphery
	gpio_init();
	tim1_init();
	tim2_init();
	dma_init();
	i2c_init();
	spi1_init();
	adc1_init();
	usart1_init();
	usart2_init();
	usart3_init();

	// init board's devices
	esc_init();
	mpu_init();
	qmc_init();
	espdev_init();
	crsfdev_init();

	// delay here is used as temporary fix. The problem is that
	// because of layout mistake, barometer is placed on oposite
	// side from ESP07 chip and gets heat from it. That heat skews
	// altitude readings, so we wait 1 second before the barometer
	// initilization to let it heat a little and use that 'skewed'
	// temperature as reference point.
	HAL_Delay(1000);
	
	hp_init();

	// reading settings from memory slot 0
	readsettings(0);

	// initilize stabilization routine
	initstabilize(0.0);

	// initilize periodic events
	inittimev(evs + TEV_PID, PID_FREQ, stabilize);
	inittimev(evs + TEV_CHECK, 1, checkconnection);
	inittimev(evs + TEV_CALIB, CALIB_FREQ, magcalib);
	inittimev(evs + TEV_HP, HP_FREQ, hpupdate);
	inittimev(evs + TEV_QMC, QMC_FREQ, qmcupdate);

	// initilize ERLS timer. For now ERLS polling is not a periodic
	// event and called as frequently as possible, so it needs this
	// separate timer.
	elrsus = 0;

	// main control loop
	while (1) {
		char cmd[ESP_CMDSIZE];
		struct crsf_data cd;
		int c, i;

		// reset iteration time counter
		__HAL_TIM_SET_COUNTER(&htim2, 0);

		// poll for configureation and telemetry commands
		// from from debug wifi connection
		if (esp_poll(&espdev, cmd) >= 0) 
			controlcmd(cmd);

		// read the ELRS remote's packet
		if (dev[CRSF_DEV].read(dev[CRSF_DEV].priv, &cd,
			sizeof(struct crsf_data)) >= 0) {
			crsfcmd(&cd, elrsus);
			elrsus = 0;
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
		c = __HAL_TIM_GET_COUNTER(&htim2);

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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
	
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		error_handler();

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			      |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
		error_handler();
	
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
			      |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
			      |RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV256;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		error_handler();
}

static void gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void dma_init(void)
{
	__HAL_RCC_DMA1_CLK_ENABLE();

	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
}

static void i2c_init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x0000020B;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
		error_handler();

	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
		error_handler();

	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	
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
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
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
	ADC_MultiModeTypeDef multimode = {0};
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
		error_handler();

	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
		error_handler();

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
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
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
	if (HAL_UART_Init(&huart2) != HAL_OK)
		error_handler();
}

static void usart3_init()
{
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 921600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	
	if (HAL_UART_Init(&huart3) != HAL_OK)
		error_handler();
}

// Init ESC's
static void esc_init()
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4
		= (uint16_t) (0.05 * (float) PWM_MAXCOUNT);
	HAL_Delay(2000);
}

// Init HP206c barometer
static void hp_init()
{
	struct hp_device d;

	d.hi2c = &hi2c1;
	d.osr = HP_OSR_1024;

	if (hp_initdevice(&d, dev + HP_DEV) >= 0)
		uartprintf("HP206C initilized\r\n");
	else
		uartprintf("failed to initilize HP206C\r\n");
}

// Init MPU6050 IMU
static void mpu_init()
{
	struct mpu_device d;

	d.hi2c = &hi2c1;
	d.devtype = MPU_DEV6050;
	d.accelscale = MPU_4G;
	d.gyroscale = MPU_1000DPS;
	d.dlpfwidth = MPU_10DLPF;

	if (mpu_initdevice(&d, dev + MPU_DEV) >= 0)
		uartprintf("MPU-6050 initilized\r\n");
	else
		uartprintf("failed to initilize MPU-6500\r\n");
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
	espdev.huart = &huart1;

	if (esp_init(&espdev, "copter", "", SERVPORT) < 0) {
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
