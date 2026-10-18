#include <math.h>
#include "dshot.h"

#include "runvals.h"

int Init = 0;

struct timev Evs[TEV_COUNT]
	__attribute__((section(".dtcm_buffer")));
struct dsp_lpf Lpf[LPF_COUNT]
	__attribute__((section(".dtcm_buffer")));
struct dsp_filter Flt
	__attribute__((section(".dtcm_buffer")));
struct dsp_compl Cmpl[CMPL_COUNT]
	__attribute__((section(".dtcm_buffer")));
struct dsp_complv Cmplv
	__attribute__((section(".dtcm_buffer")));
struct dsp_pidblval Pid[PID_COUNT]
	__attribute__((section(".dtcm_buffer")));

const char *Evnames[TEV_COUNT] = {
	"imu", "log", "pid", "check",
	"baro", "mag", "tele", "power",
	"autopilot"
};

struct mag_data Magdata __attribute__((section(".dtcm_buffer")));
struct imu_data Imudata __attribute__((section(".dtcm_buffer")));
struct baro_data Barodata __attribute__((section(".dtcm_buffer")));
struct lidar_data Lidardata __attribute__((section(".dtcm_buffer")));
struct gnss_data Gnss __attribute__((section(".dtcm_buffer")));
struct crsf_tele Tele __attribute__((section(".dtcm_buffer")));
struct msp_osd Osd __attribute__((section(".dtcm_buffer")));

double Thrust = 0.0;
double Rolltarget = 0.0;
double Pitchtarget = 0.0;
double Yawtarget = 0.0;
double En = 0.0;
enum ALTMODE Altmode = 0;
enum GNSSMODE Gnssmode = 0;
int Speedpid = 0;
int Yawspeedpid = 0;
int Hovermode = 0;
int Elrs = 0;
int Altref = 0;
int Autopilot = 0;

double Alt0 = 0.0;
double Lat0 = 0.0;
double Lon0 = 0.0;
double Goffset = 0.0;

struct trackpoint Points[MAX_POINT_COUNT];
int Pointscount = 0;
int Curpoint = 0;
double Autopilottimer = 0.0;

double Tiltcoeflimit __attribute__((section(".dtcm_buffer")));
double Mtrscaleinv __attribute__((section(".dtcm_buffer")));
double Sr __attribute__((section(".dtcm_buffer")));
double Cr __attribute__((section(".dtcm_buffer")));
double Sp __attribute__((section(".dtcm_buffer")));
double Cp __attribute__((section(".dtcm_buffer")));
double Sy __attribute__((section(".dtcm_buffer")));
double Cy __attribute__((section(".dtcm_buffer")));

int Curslot = 0;

int Loops = 0;

int Loopscount = 0;

int Elrstimeout = ELRS_TIMEOUT;

int Emergencydisarm = 0;

int setthrust(struct cdevice *dev,
	float ltd, float rtd, float lbd, float rbd)
{
	struct dshot_data dd;
	double avgthrust;

	// incorrect thrust value protection and
	// extra check for disabled ELRS transmitter	
	if (isnan(ltd) || isnan(rtd) || isnan(rbd)
		|| isnan(lbd) || !Elrs)
		ltd = rtd = rbd = lbd = 0.0;

	// update average thrust LPF
	avgthrust = (ltd + rtd + rbd + lbd) * 0.25f;
	avgthrust = avgthrust < 0.0 ? 0.0 : avgthrust;

	dsp_updatelpf(Lpf + LPF_AVGTHR, avgthrust);
	dsp_updatelpf(Lpf + LPF_AVGTHRA, avgthrust);

	// linearize thrust
	ltd = powf(trimuff(ltd), (float) Mtrscaleinv);
	rtd = powf(trimuff(rtd), (float) Mtrscaleinv);
	lbd = powf(trimuff(lbd), (float) Mtrscaleinv);
	rbd = powf(trimuff(rbd), (float) Mtrscaleinv);

	// set values for thrust channels using
	// motor to PWM channel mapping values
	dd.thrust[St.mtr.lt] = ltd;
	dd.thrust[St.mtr.lb] = lbd;
	dd.thrust[St.mtr.rt] = rtd;
	dd.thrust[St.mtr.rb] = rbd;

	// write throttle values for each motor into log
	writelog(LOG_LT, ltd);
	writelog(LOG_LB, lbd);
	writelog(LOG_RB, rbd);
	writelog(LOG_RT, rtd);

	// write average throttle values into log
	writelog(LOG_AVGTHR, dsp_getlpf(Lpf + LPF_AVGTHR));
	
	// set thrust values using DShot output device
	dev->write(dev->priv, &dd, sizeof(struct dshot_data));

	return 0;
}
