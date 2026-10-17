#include <math.h>
#include "dshot.h"

#include "runvals.h"

int Init = 0;

struct timev Evs[TEV_COUNT];
struct dsp_lpf Lpf[LPF_COUNT];
struct dsp_filter Flt;
struct dsp_compl Cmpl[CMPL_COUNT];
struct dsp_complv Cmplv;
struct dsp_pidblval Pid[PID_COUNT];

const char *Evnames[TEV_COUNT] = {
	"imu", "log", "pid", "check",
	"baro", "mag", "tele", "power",
	"autopilot"
};

struct mag_data Magdata;
struct imu_data Imudata;
struct baro_data Barodata;
struct lidar_data Lidardata;
struct gnss_data Gnss;
struct crsf_tele Tele;
struct msp_osd Osd;

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

double Tiltcoeflimit = 1.414214;
double Mtrscaleinv = 1.0;
double Sr = 0.0, Cr = 0.0;
double Sp = 0.0, Cp = 0.0;
double Sy = 0.0, Cy = 0.0;

int Curslot = 0;

int Loops = 0;

int Loopscount = 0;

int Elrstimeout = ELRS_TIMEOUT;

int Emergencydisarm = 0;

int setthrust(struct cdevice *dev,
	double ltd, double rtd, double lbd, double rbd)
{
	struct dshot_data dd;
	double avgthrust;

	// incorrect thrust value protection and
	// extra check for disabled ELRS transmitter	
	if (isnan(ltd) || isnan(rtd) || isnan(rbd)
		|| isnan(lbd) || !Elrs)
		ltd = rtd = rbd = lbd = 0.0;

	// update average thrust LPF
	avgthrust = (ltd + rtd + rbd + lbd) * 0.25;
	avgthrust = avgthrust < 0.0 ? 0.0 : avgthrust;

	dsp_updatelpf(Lpf + LPF_AVGTHR, avgthrust);
	dsp_updatelpf(Lpf + LPF_AVGTHRA, avgthrust);

	// linearize thrust
	ltd = pow(trimuf(ltd), Mtrscaleinv);
	rtd = pow(trimuf(rtd), Mtrscaleinv);
	lbd = pow(trimuf(lbd), Mtrscaleinv);
	rbd = pow(trimuf(rbd), Mtrscaleinv);

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
