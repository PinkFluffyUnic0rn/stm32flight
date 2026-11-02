#include <math.h>
#include <string.h>

#include "dshot.h"

#include "runvals.h"

int Init = 0;

DTCMRAM struct timev Evs[TEV_COUNT];
DTCMRAM struct dsp_lpf Lpf[LPF_COUNT];
DTCMRAM struct dsp_filter Flt;
DTCMRAM struct dsp_compl Cmpl[CMPL_COUNT];
DTCMRAM struct dsp_complv Cmplv;
DTCMRAM struct dsp_pidblval Pid[PID_COUNT];

const char *Evnames[TEV_COUNT] = {
	"imu", "log", "pid", "check",
	"baro", "mag", "tele", "power",
	"autopilot"
};

DTCMRAM struct mag_data Magdata;
DTCMRAM struct imu_data Imudata;
DTCMRAM struct baro_data Barodata;
DTCMRAM struct lidar_data Lidardata;
DTCMRAM struct gnss_data Gnss;
DTCMRAM struct crsf_tele Tele;
DTCMRAM struct msp_osd Osd;

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

DTCMRAM double Tiltcoeflimit;
DTCMRAM double Mtrscaleinv;
DTCMRAM double Sr;
DTCMRAM double Cr;
DTCMRAM double Sp;
DTCMRAM double Cp;
DTCMRAM double Sy;
DTCMRAM double Cy;

int Curslot = 0;

int Loops = 0;

int Loopscount = 0;

int Elrstimeout = ELRS_TIMEOUT;

int Emergencydisarm = 0;

ITCMRAM int setthrust(struct cdevice *dev,
	float ltd, float rtd, float lbd, float rbd)
{
	struct dshot_data dd;
	struct dshot_tele td;
	double avgthrust;
	float rps1, rps2, rps3, rps4, avgrps;

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
	dev->read(dev->priv, &td, sizeof(struct dshot_tele));

	// update each motor RPS for valid RPS telemetry values
	if (td.isvalid[0])
		dsp_updatelpf(Lpf + LPF_RPS1, td.rps[0]);

	if (td.isvalid[1])
		dsp_updatelpf(Lpf + LPF_RPS2, td.rps[1]);

	if (td.isvalid[2])
		dsp_updatelpf(Lpf + LPF_RPS3, td.rps[2]);

	if (td.isvalid[3])
		dsp_updatelpf(Lpf + LPF_RPS4, td.rps[3]);


	rps1 = dsp_getlpf(Lpf + LPF_RPS1);
	rps2 = dsp_getlpf(Lpf + LPF_RPS2);
	rps3 = dsp_getlpf(Lpf + LPF_RPS3);
	rps4 = dsp_getlpf(Lpf + LPF_RPS4);

	// calculate average RPS value
	avgrps = (rps1 + rps2 + rps3 + rps4) * 0.25f;
	dsp_updatelpf(Lpf + LPF_AVGRPS, avgrps);

	// write RPS values for each motor and average RPS value
	writelog(LOG_RPS1, rps1);
	writelog(LOG_RPS2, rps2);
	writelog(LOG_RPS3, rps3);
	writelog(LOG_RPS4, rps4);
	writelog(LOG_AVGRPS, avgrps);

	return 0;
}
