#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

#include "main.h"
#include "driver.h"
#include "uartdebug.h"
#include "mpu6500.h"
#include "bmp280.h"
#include "esp8266.h"
#include "hmc5883l.h"
#include "dsp.h"

// Max length for info packet
// sent back to operator
#define INFOLEN 512

// device numbers
#define MPU_DEV 0
#define BMP_DEV 1
#define HMC_DEV 2

// timer settings
#define PRESCALER 48

#define OCSFREQ 48000000
#define TIMPERIOD 0xffff
#define TICKSPERSEC (OCSFREQ / PRESCALER)

// PWM settings
#define PWM_MAXCOUNT 2500

// DSP settings
#define PID_FREQ 1000
#define CALIB_FREQ 25
#define BMP_FREQ 150

#define USER_FLASH 0x0801f800
#define UESR_SETSLOTS (0x80 / sizeof(struct settings))

// Timer events
#define TEV_PID 	0
#define TEV_CHECK 	1
#define TEV_CALIB	2
#define TEV_BMP		3
#define TEV_COUNT	4

#define WIFI_TIMEOUT 3

// Wi-Fi settings:
// 	define SSID
// 	define PASSWORD
// 	define SERIP
// 	define SERVPORT
#include "wifidefs.h"

#define checktimev(ev) ((ev)->ms > TICKSPERSEC / (ev)->freq)
#define resettimev(ev) (ev)->ms = 0;
#define updatetimev(ev, s) (ev)->ms += (s);

struct settings {
	float mx0,	my0,		mz0;
	float mxsc,	mysc,		mzsc;
	float magdecl;

	float gx0,	gy0,		gz0;
	float roll0,	pitch0,		yaw0;

	float tcoef, ttcoef, ptcoef;

	int speedpid;
	int yawspeedpid;
	float p,	i,	d;
	float sp,	si,	sd;
	float yp,	yi,	yd;
	float ysp,	ysi,	ysd;
	float zsp,	zsi,	zsd;
};

struct timev {
	int ms;
	int freq;
	int (*cb)(int);
};

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
static void esc_init();
static void mpu_init();
static void bmp_init();
static void hmc_init();
static void espdev_init();

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

// IC drivers
struct driver drivers[4];
struct device dev[4];
struct esp_device espdev;

// DSP contexts
struct dsp_lpf tlpf;
struct dsp_lpf presslpf;

struct dsp_compl pitchcompl;
struct dsp_compl rollcompl;

struct dsp_pidval pitchpv;
struct dsp_pidval rollpv;
struct dsp_pidval pitchspv;
struct dsp_pidval rollspv;
struct dsp_pidval yawpv;
struct dsp_pidval yawspv;
struct dsp_pidval tpv;

// control values
float thrust = 0.0;
float rolltarget = 0.0, pitchtarget = 0.0, yawtarget = 0.0;
float en = 0.0;
int magcalibmode = 0;

// pressure and altitude initial values
float alt0, press0;

// settings
struct settings st;

// timer events
struct timev evs[TEV_COUNT];

int loops = 0;
int loopscount = 0;

int wifitimeout = WIFI_TIMEOUT;

uint32_t getadcv(ADC_HandleTypeDef *hadc)
{
	uint32_t v;

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1);

	v = HAL_ADC_GetValue(hadc);

	HAL_ADC_Stop(hadc);

	return v;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) 
{
	esp_interrupt(&espdev, huart);
}

int inittimev(struct timev *ev, int freq, int (*cb)(int))
{
	ev->ms = 0;
	ev->freq = freq;
	ev->cb = cb;

	return 0;
}

int mpu_printuart(const struct mpu_data *md)
{
	uartprintf("%-15sx = %hd; y = %hd; z = %hd\n\r",
		"acceleration: ", md->ax, md->ay, md->az);

	uartprintf("%-15sx = %hd; y = %hd; z = %hd\n\r\n\r",
		"rotation: ", md->gx, md->gy, md->gz);

	return 0;
}

int mpu_printuartf(const struct mpu_data *md)
{
	uartprintf("%-15sx = %-.3f; y = %-.3f; z = %-.3f\n\r",
		"acceleration: ", (double) md->afx, (double) md->afy,
		(double) md->afz);

	uartprintf("%-15sx = %-.3f; y = %-.3f; z = %-.3f\n\r\n\r",
		"rotation: ", (double) md->gfx, (double) md->gfy,
		(double) md->gfz);

	return 0;
}

float groundpressure(float alt)
{
	float press;
	float c;
	int i;

	press = 0.0;
	c = 0.0;

	for (i = 0; i < 100; ++i) {
		struct bmp_data bd;
		float t, tt;

		dev[BMP_DEV].read(dev[BMP_DEV].priv, 0, &bd,
			sizeof(struct bmp_data));
	
		tt = bd.press / 100.0
			/ powf(1.0 - (alt / 44330.0), 5.255) - c;
		
		t = press + tt;
	
		c = (t - press) - tt;

		press = t;

		HAL_Delay(10);
	}

	press /= 100.0;

	return press;
}

float getalt(float press0, float press)
{
	return (44330.0 * (1.0 - powf(press / 100.0 / press0,
		1.0 / 5.225)));
}

int averageposition(float *ax, float *ay, float *az, float *gx,
	float *gy, float *gz)
{
	struct mpu_data md;
	float axt, ayt, azt, gxt, gyt, gzt;
	int i;

	axt = ayt = azt = gxt = gyt = gzt = 0;
	for (i = 0; i < 250; ++i) {
		dev[MPU_DEV].read(dev[MPU_DEV].priv, 0, &md,
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

float hmc_heading(float r, float p, float x, float y, float z)
{
	x = st.mxsc * (x + st.mx0);
	y = st.mysc * (y + st.my0);
	z = st.mzsc * (z + st.mz0);

	x = x * cosf(p) + y * sinf(r) * sinf(p)
		+ z * cosf(r) * sinf(p);
	y = y * cosf(r) - z * sinf(r);

	return circf(atan2f(y, x) + st.magdecl);
}

int setthrust(float ltd, float rtd, float rbd, float lbd)
{
	if (isnan(ltd) || isnan(rtd) || isnan(rbd) || isnan(lbd))
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

int readsettings(int slot)
{
	memcpy(&st, (void *) (USER_FLASH + slot),
		sizeof(struct settings));

	return 0;
}

int initstabilize(float alt)
{
	float ax, ay, az;

	alt0 = alt;

	if (dev[BMP_DEV].status == DEVSTATUS_INIT)
		press0 = groundpressure(alt);

	averageposition(&ax, &ay, &az, &(st.gx0), &(st.gy0), &(st.gz0));

	dsp_initcompl(&pitchcompl, st.tcoef, PID_FREQ);
	dsp_initcompl(&rollcompl, st.tcoef, PID_FREQ);

	dsp_initpidval(&pitchpv, st.p, st.i, st.d, 0.0);
	dsp_initpidval(&rollpv, st.p, st.i, st.d, 0.0);

	dsp_initpidval(&pitchspv, st.sp, st.si, st.sd, 0.0);
	dsp_initpidval(&rollspv, st.sp, st.si, st.sd, 0.0);
	dsp_initpidval(&yawspv, st.ysp, st.ysi, st.ysd, 0.0);
	dsp_initpidval(&yawpv, st.yp, st.yi, st.yd, 0.0);
	dsp_initpidval(&tpv, st.zsp, st.zsi, st.zsd, 0.0);

	dsp_initlpf(&presslpf, st.ptcoef, BMP_FREQ);
	dsp_initlpf(&tlpf, st.ttcoef, PID_FREQ);

	return 0;
}

int stabilize(int ms)
{
	struct mpu_data md;
	struct hmc_data hd;
	float roll, pitch, yaw;
	float rollcor, pitchcor, yawcor, thrustcor;
	float gy, gx, gz;
	float dt;
			
	dt = ms / (float) TICKSPERSEC;
	dt = (dt < 0.000001) ? 0.000001 : dt;

	dev[MPU_DEV].read(dev[MPU_DEV].priv, 0, &md,
		sizeof(struct mpu_data));

	dsp_updatelpf(&tlpf, md.afz);

	gy = deg2rad((md.gfy - st.gy0));
	gx = deg2rad((md.gfx - st.gx0));
	gz = deg2rad((md.gfz - st.gz0));

	roll = dsp_updatecompl(&rollcompl, gy * dt,
		atan2f(-md.afx,
		sqrt(md.afy * md.afy + md.afz * md.afz))) - st.roll0;

	pitch = dsp_updatecompl(&pitchcompl, gx * dt,
		atan2f(md.afy,
		sqrt(md.afx * md.afx + md.afz * md.afz))) - st.pitch0;

	dev[HMC_DEV].read(dev[HMC_DEV].priv, 0, &hd,
		sizeof(struct hmc_data));

	yaw = circf(hmc_heading(-pitch, -roll, hd.fx, hd.fy, hd.fz)
		- st.yaw0);

	if (st.speedpid) {
		rollcor = dsp_pid(&rollspv, rolltarget, gy, dt);
		pitchcor = dsp_pid(&pitchspv, pitchtarget, gx, dt);
	}
	else {
		rollcor = dsp_pid(&rollpv, rolltarget, roll, dt);
		pitchcor = dsp_pid(&pitchpv, pitchtarget, pitch, dt);

		rollcor = dsp_pid(&rollspv, rollcor, gy, dt);
		pitchcor = dsp_pid(&pitchspv, pitchcor, gx, dt);
	}

	if (st.yawspeedpid)
		yawcor = dsp_pid(&yawspv, yawtarget, gz, dt);
	else {
		yawcor = circf(dsp_pid(&yawpv, yawtarget, yaw, dt));
		yawcor = dsp_pid(&yawspv, yawcor, gz, dt);
	}

	thrustcor = dsp_pid(&tpv, thrust + 1.0, dsp_getlpf(&tlpf), dt);

	setthrust(en * (thrustcor + 0.5 * rollcor
			+ 0.5 * pitchcor - 0.5 * yawcor),
		en * (thrustcor - 0.5 * rollcor
			+ 0.5 * pitchcor + 0.5 * yawcor),
		en * (thrustcor - 0.5 * rollcor
			- 0.5 * pitchcor - 0.5 * yawcor),
		en * (thrustcor + 0.5 * rollcor
			- 0.5 * pitchcor + 0.5 * yawcor));
			
	++loops;

	return 0;
}

int checkconnection(int ms)
{
	if (wifitimeout != 0)
		--wifitimeout;

	if (wifitimeout == 0) {
		char s[INFOLEN];

		setthrust(0.0, 0.0, 0.0, 0.0);

		en = 0.0;

		snprintf(s, INFOLEN, "wi-fi timed out!\n\r");

		esp_send(&espdev, s);
	}

	// since it runs every 1 second update loop counter here too
	loopscount = loops;
	loops = 0;
		
	return 0;
}

int magcalib(int ms)
{
	struct hmc_data hd;
	char s[INFOLEN];

	if (!magcalibmode)
		return 0;

	dev[HMC_DEV].read(dev[HMC_DEV].priv, 0, &hd,
		sizeof(struct hmc_data));

	sprintf(s, "%f %f %f\r\n",
		(double) (st.mxsc * (hd.fx + st.mx0)),
		(double) (st.mysc * (hd.fy + st.my0)),
		(double) (st.mzsc * (hd.fz + st.mz0)));

	esp_send(&espdev, s);

	return 0;
}

int bmpupdate(int ms)
{
	struct bmp_data bd;
	
	if (dev[BMP_DEV].status != DEVSTATUS_INIT) 
		return 0;

	dev[BMP_DEV].read(dev[BMP_DEV].priv, 0, &bd,
		sizeof(struct bmp_data));

	dsp_updatelpf(&presslpf, bd.press);

	return 0;
}

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

int sprintpos(char *s, struct mpu_data *md, struct hmc_data *hd)
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
		(double) circf(hmc_heading(
			-(dsp_getcompl(&pitchcompl) - st.pitch0),
			-(dsp_getcompl(&rollcompl) - st.roll0),
			hd->fx, hd->fy, hd->fz) - st.yaw0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"z acceleration: %f\r\n",
		(double) dsp_getlpf(&tlpf));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"battery: %0.3f\n\r",
		getadcv(&hadc1) / (double) 0xfff * (double) 10.921);

	return 0;
}

int sprinthmc(char *s, struct hmc_data *hd)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"x = %0.3f; y = %0.3f; z = %0.3f\n\r",
		(double) hd->fx, (double) hd->fy, (double) hd->fz);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"heading: %f\r\n", (double) hmc_heading(
			-(dsp_getcompl(&pitchcompl) - st.pitch0),
			-(dsp_getcompl(&rollcompl) - st.roll0),
			hd->fx, hd->fy, hd->fz));

	return 0;
}

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
		"cf: %.6f\r\n", (double) st.tcoef);
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel cf: %.6f\r\n", (double) st.ttcoef);
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"pressure cf: %.6f\r\n", (double) st.ptcoef);
	
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"loops count: %d\r\n", loopscount);

	return 0;
}

int sprintpid(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"pos PID: %.3f,%.3f,%.3f\r\n",
		(double) st.p, (double) st.i, (double) st.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"speed PID: %.3f,%.3f,%.3f\r\n",
		(double) st.sp, (double) st.si, (double) st.sd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw PID: %.3f,%.3f,%.3f\r\n",
		(double) st.yp, (double) st.yi, (double) st.yd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw speed PID: %.3f,%.3f,%.3f\r\n",
		(double) st.ysp, (double) st.ysi, (double) st.ysd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"thrust PID: %.3f,%.3f,%.3f\r\n",
		(double) st.zsp, (double) st.zsi, (double) st.zsd);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s mode\r\n", st.speedpid ? "single" : "dual");

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s yaw mode\r\n",
		st.yawspeedpid ? "single" : "dual");
	
	return 0;
}

int controlcmd(char *cmd)
{
	char s[INFOLEN];
	char *toks[12];

	if (cmd[0] == '\0')
		return 0;

	parsecommand(toks, 12, cmd);

	if (strcmp(toks[0], "beep") == 0)
		wifitimeout = WIFI_TIMEOUT;
	else if (strcmp(toks[0], "info") == 0) {
		if (strcmp(toks[1], "mpu") == 0) {
			struct mpu_data md;
			struct hmc_data hd;

			dev[MPU_DEV].read(dev[MPU_DEV].priv, 0, &md,
				sizeof(struct mpu_data));

			dev[HMC_DEV].read(dev[HMC_DEV].priv, 0, &hd,
				sizeof(struct hmc_data));

			sprintpos(s, &md, &hd);

			esp_send(&espdev, s);
		}
		else if (strcmp(toks[1], "hmc") == 0) {
			struct hmc_data hd;

			dev[HMC_DEV].read(dev[HMC_DEV].priv, 0, &hd,
				sizeof(struct hmc_data));
		
			sprinthmc(s, &hd);	
		
			esp_send(&espdev, s);
		}
		else if (strcmp(toks[1], "bmp") == 0) {
			float press;

			press = dsp_getlpf(&presslpf);

			snprintf(s, INFOLEN, "bar: %f; alt: %f\r\n",
				(double) press, (double) getalt(press0, press));
			
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
	else if (strcmp(toks[0], "t") == 0) {
		float v;
	
		v = atof(toks[2]);
		
		if (strcmp(toks[1], "c") == 0)		thrust = v;
		else if (strcmp(toks[1], "r") == 0)	rolltarget = v;
		else if (strcmp(toks[1], "p") == 0)	pitchtarget = v;
		else if (strcmp(toks[1], "y") == 0)	yawtarget = v;
		else
			goto unknown;
	}
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
		else if (strcmp(toks[1], "pressure") == 0) {
			st.ptcoef = atof(toks[2]);
				
			dsp_initlpf(&presslpf, st.ptcoef, BMP_FREQ);
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

int main(void)
{
	HAL_Init();

	systemclock_config();
	
	HAL_Delay(1000);

	gpio_init();
	tim1_init();
	tim2_init();
	dma_init();
	i2c_init();
	spi1_init();
	adc1_init();
	usart1_init();
	usart2_init();
	esc_init();
	bmp_init();
	mpu_init();
	hmc_init();
	espdev_init();

	readsettings(0);

	initstabilize(0.0);

	inittimev(evs + TEV_PID, PID_FREQ, stabilize);
	inittimev(evs + TEV_CHECK, 1, checkconnection);
	inittimev(evs + TEV_CALIB, CALIB_FREQ, magcalib);
	inittimev(evs + TEV_BMP, BMP_FREQ, bmpupdate);

	wifitimeout = WIFI_TIMEOUT;
	while (1) {
		char cmd[ESP_CMDSIZE];
		int c, i;

		__HAL_TIM_SET_COUNTER(&htim2, 0);

		if (esp_poll(&espdev, cmd) >= 0)
			controlcmd(cmd);

		for (i = 0; i < TEV_COUNT; ++i) {
			if (checktimev(evs + i)) {
				evs[i].cb(evs[i].ms);
				resettimev(evs + i);
			}
		}

		c = __HAL_TIM_GET_COUNTER(&htim2);

		for (i = 0; i < TEV_COUNT; ++i)
			updatetimev(evs + i, c);
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
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
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
/*
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
		error_handler();
*/
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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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

static void esc_init()
{
	TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4
		= (uint16_t) (0.05 * (float) PWM_MAXCOUNT);
	HAL_Delay(2000);
}

static void mpu_init()
{
	struct mpu_device d;

	mpu_getdriver(drivers + MPU_DEV);

	d.hi2c = &hi2c1;
	d.devtype = MPU_DEV6050;
	d.accelscale = MPU_4G;
	d.gyroscale = MPU_1000DPS;
	d.dlpfwidth = MPU_10DLPF;

	if (drivers[0].initdevice(&d, dev + MPU_DEV) == 0)
		uartprintf("MPU-6500 initilized\r\n");
	else
		uartprintf("failed to initilize MPU-6500\r\n");
}

static void bmp_init()
{
	struct bmp_device d;

	bmp_getdriver(drivers + BMP_DEV);

	d.hi2c = &hi2c1;

	if (drivers[1].initdevice(&d, dev + BMP_DEV) == 0)
		uartprintf("BMP280 initilized\r\n");
	else
		uartprintf("failed to initilize BMP280\r\n");

}

static void hmc_init()
{
	struct hmc_device d;
	hmc_getdriver(drivers + HMC_DEV);

	d.hi2c = &hi2c1;
	d.scale = HMC_SCALE_1_3;
	d.rate = HMC_RATE_15;

	if (drivers[2].initdevice(&d, dev + HMC_DEV) == 0)
		uartprintf("HMC5883L initilized\r\n");
	else
		uartprintf("failed to initilize HMC5883L\r\n");
}

static void espdev_init()
{
	char ip[32];
	
	espdev.huart = &huart1;

	if (esp_init(&espdev, SSID, PASSWORD) < 0) {
		uartprintf("failed to initilize ESP8266\r\n");
		return;
	}

	uartprintf("ESP8266 initilized\r\n");

	uartprintf("getting ip...\r\n");

	if (esp_getip(&espdev, ip) < 0)
		uartprintf("failed to get ip\r\n");
	else
		uartprintf("got ip %s\r\n", ip);

	uartprintf("connecting to %s:%d...\r\n", SERVIP, SERVPORT);
	
	if (esp_connect(&espdev, SERVIP, SERVPORT) < 0) {
		uartprintf("failed to connect to %s %s\r\n",
			SERVIP, SERVPORT);
		return;
	}

	uartprintf("connected\r\n");

}

void error_handler(void)
{
	__disable_irq();
	while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
