#include <math.h>
#include "dshot.h"

#include "runvals.h"

struct timev Evs[TEV_COUNT];
struct dsp_lpf Lpf[LPF_COUNT];
struct dsp_compl Cmpl[CMPL_COUNT];
struct dsp_pidblval Pid[PID_COUNT];

struct qmc_data Qmcdata;
struct icm_data Imudata;
struct gnss_data Gnss;
struct crsf_tele Tele;

float Thrust = 0.0;
float Rolltarget = 0.0;
float Pitchtarget = 0.0;
float Yawtarget = 0.0;
float En = 0.0;
enum ALTMODE Altmode = 0;
int Speedpid = 0;
int Yawspeedpid = 0;
int Hovermode = 0;
int Elrs = 0;
int Autopilot = 0;

float Alt0 = 0.0;
float Goffset = 0.0;

struct trackpoint Points[MAX_POINT_COUNT];
int Pointscount = 0;
int Curpoint = 0;
float Autopilottimer = 0.0;

int Curslot = 0;

int Loops = 0;

int Loopscount = 0;

int Elrstimeout = ELRS_TIMEOUT;

int Emergencydisarm = 0;

int setthrust(struct cdevice *dev,
	float ltd, float rtd, float lbd, float rbd)
{
	struct dshot_data dd;
	float avgthrust;
	
	// incorrect thrust value protection and
	// extra check for disabled ELRS transmitter	
	if (isnan(ltd) || isnan(rtd) || isnan(rbd)
		|| isnan(lbd) || !Elrs)
		ltd = rtd = rbd = lbd = 0.0;
						
	// set values for thrust channels using
	// motor to PWM channel mapping values
	dd.thrust[St.mtr.lt] = ltd;
	dd.thrust[St.mtr.lb] = lbd;
	dd.thrust[St.mtr.rt] = rtd;
	dd.thrust[St.mtr.rb] = rbd;

	// update average thrust LPF
	avgthrust = (ltd + rtd + rbd + lbd) / 4.0;
	avgthrust = avgthrust < 0.0 ? 0.0 : avgthrust;

	dsp_updatelpf(Lpf + LPF_AVGTHR, avgthrust);

	// write thrust values for each motor into log
	writelog(LOG_LT, ltd);
	writelog(LOG_LB, lbd);
	writelog(LOG_RB, rbd);
	writelog(LOG_RT, rtd);

	// set thrust values using DShot output device
	dev->write(dev->priv, &dd, sizeof(struct dshot_data));

	return 0;
}
