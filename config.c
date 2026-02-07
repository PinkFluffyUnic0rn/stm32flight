#include "config.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "settings.h"
#include "dsp.h"
#include "log.h"
#include "crc.h"
#include "runvals.h"
#include "timev.h"
#include "command.h"
#include "util.h"

#include "icm42688.h"
#include "dps368.h"
#include "qmc5883l.h"
#include "irc.h"
#include "dshot.h"

enum NODETYPE {
	NODETYPE_PARENT = 0,
	NODETYPE_FLOAT = 1,
	NODETYPE_INT = 2,
	NODETYPE_MAP = 3
};

struct settingnode
{
	char *token;
	int type;
	union {
		struct settingnode **child;
		float *f;
		int *i;
		struct {
			int *m;
			const char **k;
		} m;
	};
};

static struct settingnode Sttree = {
	.token = "",
	.type = NODETYPE_PARENT,
	.child = (struct settingnode *[]) {
		&(struct settingnode) {
			.token = "pid",
			.type = NODETYPE_PARENT,
			.child = (struct settingnode *[]) {
				&(struct settingnode) {
					.token = "tilt",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "p",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.attpos.p)
						},
						&(struct settingnode) {
							.token = "i",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.attpos.i)
						},
						&(struct settingnode) {
							.token = "d",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.attpos.d)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "stilt",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "p",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.attrate.p)
						},
						&(struct settingnode) {
							.token = "i",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.attrate.i)
						},
						&(struct settingnode) {
							.token = "d",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.attrate.d)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "yaw",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "p",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.yawpos.p)
						},
						&(struct settingnode) {
							.token = "i",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.yawpos.i)
						},
						&(struct settingnode) {
							.token = "d",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.yawpos.d)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "syaw",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "p",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.yawrate.p)
						},
						&(struct settingnode) {
							.token = "i",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.yawrate.i)
						},
						&(struct settingnode) {
							.token = "d",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.yawrate.d)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "throttle",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "p",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.throttle.p)
						},
						&(struct settingnode) {
							.token = "i",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.throttle.i)
						},
						&(struct settingnode) {
							.token = "d",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.throttle.d)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "climbrate",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "p",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.climbrate.p)
						},
						&(struct settingnode) {
							.token = "i",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.climbrate.i)
						},
						&(struct settingnode) {
							.token = "d",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.climbrate.d)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "altitude",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "p",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.alt.p)
						},
						&(struct settingnode) {
							.token = "i",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.alt.i)
						},
						&(struct settingnode) {
							.token = "d",
							.type = NODETYPE_FLOAT,
							.f = &(St.pid.alt.d)
						},
						NULL
					}
				},
				NULL
			}
		},
		&(struct settingnode) {
			.token = "compl",
			.type = NODETYPE_PARENT,
			.child = (struct settingnode *[]) {
				&(struct settingnode) {
					.token = "attitude",
					.type = NODETYPE_FLOAT,
					.f = &(St.cmpl.att)
				},
				&(struct settingnode) {
					.token = "yaw",
					.type = NODETYPE_FLOAT,
					.f = &(St.cmpl.yaw)
				},
				&(struct settingnode) {
					.token = "climbrate",
					.type = NODETYPE_FLOAT,
					.f = &(St.cmpl.climbrate)
				},
				&(struct settingnode) {
					.token = "altitude",
					.type = NODETYPE_FLOAT,
					.f = &(St.cmpl.alt)
				},
				NULL
			}
		},
		&(struct settingnode) {
			.token = "lpf",
			.type = NODETYPE_PARENT,
			.child = (struct settingnode *[]) {
				&(struct settingnode) {
					.token = "gyro",
					.type = NODETYPE_FLOAT,
					.f = &(St.lpf.gyro)
				},
				&(struct settingnode) {
					.token = "accel",
					.type = NODETYPE_FLOAT,
					.f = &(St.lpf.acc)
				},
				&(struct settingnode) {
					.token = "mag",
					.type = NODETYPE_FLOAT,
					.f = &(St.lpf.mag)
				},
				&(struct settingnode) {
					.token = "d",
					.type = NODETYPE_FLOAT,
					.f = &(St.lpf.d)
				},
				&(struct settingnode) {
					.token = "vaccel",
					.type = NODETYPE_FLOAT,
					.f = &(St.lpf.va)
				},
				NULL
			}
		},
		&(struct settingnode) {
			.token = "adj",
			.type = NODETYPE_PARENT,
			.child = (struct settingnode *[]) {
				&(struct settingnode) {
					.token = "rollthrust",
					.type = NODETYPE_FLOAT,
					.f = &(St.adj.mtrsc.r)
				},
				&(struct settingnode) {
					.token = "pitchthrust",
					.type = NODETYPE_FLOAT,
					.f = &(St.adj.mtrsc.p)
				},
				&(struct settingnode) {
					.token = "roll",
					.type = NODETYPE_FLOAT,
					.f = &(St.adj.att0.roll)
				},
				&(struct settingnode) {
					.token = "pitch",
					.type = NODETYPE_FLOAT,
					.f = &(St.adj.att0.pitch)
				},
				&(struct settingnode) {
					.token = "yaw",
					.type = NODETYPE_FLOAT,
					.f = &(St.adj.att0.yaw)
				},
				&(struct settingnode) {
					.token = "acc",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "x",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.acc0.x)
						},
						&(struct settingnode) {
							.token = "y",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.acc0.y)
						},
						&(struct settingnode) {
							.token = "z",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.acc0.z)
						},
						&(struct settingnode) {
							.token = "xtscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.acctsc.x)
						},
						&(struct settingnode) {
							.token = "ytscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.acctsc.y)
						},
						&(struct settingnode) {
							.token = "ztscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.acctsc.z)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "gyro",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "x",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.gyro0.x)
						},
						&(struct settingnode) {
							.token = "y",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.gyro0.y)
						},
						&(struct settingnode) {
							.token = "z",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.gyro0.z)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "mag",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "x0",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.mag0.x)
						},
						&(struct settingnode) {
							.token = "y0",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.mag0.y)
						},
						&(struct settingnode) {
							.token = "z0",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.mag0.z)
						},
						&(struct settingnode) {
							.token = "xscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.magsc.x)
						},
						&(struct settingnode) {
							.token = "yscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.magsc.y)
						},
						&(struct settingnode) {
							.token = "zscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.magsc.z)
						},

						&(struct settingnode) {
							.token = "xthscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.magthrsc.x)
						},
						&(struct settingnode) {
							.token = "ythscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.magthrsc.y)
						},
						&(struct settingnode) {
							.token = "zthscale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.magthrsc.z)
						},
						&(struct settingnode) {
							.token = "decl",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.magdecl)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "curr",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "offset",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.curroff)
						},
						&(struct settingnode) {
							.token = "scale",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.cursc)
						},
						NULL
					}
				},
				&(struct settingnode) {
					.token = "althold",
					.type = NODETYPE_PARENT,
					.child = (struct settingnode *[]) {
						&(struct settingnode) {
							.token = "hover",
							.type = NODETYPE_FLOAT,
							.f = &(St.adj.hoverthrottle)
						},
						NULL
					}
				},
				NULL
			}
		},
		&(struct settingnode) {
			.token = "ctrl",
			.type = NODETYPE_PARENT,
			.child = (struct settingnode *[]) {
				&(struct settingnode) {
					.token = "thrust",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.thrustmax)
				},
				&(struct settingnode) {
					.token = "roll",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.rollmax)
				},
				&(struct settingnode) {
					.token = "pitch",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.pitchmax)
				},
				&(struct settingnode) {
					.token = "sroll",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.rollrate)
				},
				&(struct settingnode) {
					.token = "spitch",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.pitchrate)
				},
				&(struct settingnode) {
					.token = "syaw",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.yawrate)
				},
				&(struct settingnode) {
					.token = "accel",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.accelmax)
				},
				&(struct settingnode) {
					.token = "climbrate",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.climbratemax)
				},
				&(struct settingnode) {
					.token = "altmax",
					.type = NODETYPE_FLOAT,
					.f = &(St.ctrl.altmax)
				},
				NULL
			}
		},
		&(struct settingnode) {
			.token = "irc",
			.type = NODETYPE_PARENT,
			.child = (struct settingnode *[]) {
				&(struct settingnode) {
					.token = "power",
					.type = NODETYPE_INT,
					.i = &(St.irc.power)
				},
				&(struct settingnode) {
					.token = "frequency",
					.type = NODETYPE_INT,
					.i = &(St.irc.freq)
				},
				NULL
			}
		},
		&(struct settingnode) {
			.token = "motor",
			.type = NODETYPE_PARENT,
			.child = (struct settingnode *[]) {
				&(struct settingnode) {
					.token = "lt",
					.type = NODETYPE_INT,
					.i = &(St.mtr.lt)
				},
				&(struct settingnode) {
					.token = "lb",
					.type = NODETYPE_INT,
					.i = &(St.mtr.lb)
				},
				&(struct settingnode) {
					.token = "rb",
					.type = NODETYPE_INT,
					.i = &(St.mtr.rb)
				},
				&(struct settingnode) {
					.token = "rt",
					.type = NODETYPE_INT,
					.i = &(St.mtr.rt)
				},
				NULL
			}
		},
		&(struct settingnode) {
			.token = "log",
			.type = NODETYPE_PARENT,
			.child = (struct settingnode *[]) {
				&(struct settingnode) {
					.token = "freq",
					.type = NODETYPE_INT,
					.i = &(St.log.freq)
				},
				&(struct settingnode) {
					.token = "recsize",
					.type = NODETYPE_INT,
					.i = &(St.log.recsize)
				},
				&(struct settingnode) {
					.token = "record",
					.type = NODETYPE_MAP,
					.m = {
						.m = St.log.fieldid,
						.k = logfieldmap
					}
				},
				NULL
			}
		},
		NULL
	}
};

static int mapsearch(const char **map, const char *s)
{
	const char **p;
	
	for (p = map; *p != NULL; ++p) {
		if (strcmp(s, *p) == 0)
			break;
	}
	
	if (*p == NULL)
		return (-1);

	return (p - map);
}

int setstabilize(int init)
{
	// init complementary filters contexts
	dsp_setcompl(Cmpl + CMPL_PITCH, St.cmpl.att, PID_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_ROLL, St.cmpl.att, PID_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_YAW, St.cmpl.yaw, PID_FREQ, init);

	dsp_setcompl(Cmpl + CMPL_CLIMBRATE, St.cmpl.climbrate,
		DPS_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_ALT, St.cmpl.alt, DPS_FREQ, init);

	// init roll and pitch position PID controller contexts
	dsp_setpidbl(Pid + PID_PITCHP,
		St.pid.attpos.p, St.pid.attpos.i, St.pid.attpos.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);
	dsp_setpidbl(Pid + PID_ROLLP,
		St.pid.attpos.p, St.pid.attpos.i, St.pid.attpos.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	// init roll, pitch and yaw speed PID controller contexts
	dsp_setpidbl(Pid + PID_PITCHS,
		St.pid.attrate.p, St.pid.attrate.i, St.pid.attrate.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);
	dsp_setpidbl(Pid + PID_ROLLS, 
		St.pid.attrate.p, St.pid.attrate.i, St.pid.attrate.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);
	dsp_setpidbl(Pid + PID_YAWS,
		St.pid.yawrate.p, St.pid.yawrate.i, St.pid.yawrate.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	// init yaw position PID controller's context
	dsp_setpidbl(Pid + PID_YAWP,
		St.pid.yawpos.p, St.pid.yawpos.i, St.pid.yawpos.d,
		PID_MAX_I, St.lpf.d, 1, PID_FREQ, init);

	// init vertical acceleration PID controller's context
	dsp_setpidbl(Pid + PID_VA,
		St.pid.throttle.p, St.pid.throttle.i, St.pid.throttle.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	// init climbrate PID controller's context
	dsp_setpidbl(Pid + PID_CLIMBRATE,
		St.pid.climbrate.p, St.pid.climbrate.i,
		St.pid.climbrate.d, PID_MAX_I, St.lpf.d, 0, PID_FREQ,
		init);

	// init altitude PID controller's context
	dsp_setpidbl(Pid + PID_ALT,
		St.pid.alt.p, St.pid.alt.i, St.pid.alt.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	// init battery voltage low-pass filter
	dsp_setlpf1f(Lpf + LPF_BAT, BAT_CUTOFF, POWER_FREQ, init);
	dsp_setlpf1f(Lpf + LPF_CUR, CUR_CUTOFF, POWER_FREQ, init);

	// init average thrust low-pass filter
	dsp_setunity(Lpf + LPF_AVGTHR, init);

	// init low-pass fitlers for altitude and vertical acceleration
	dsp_setunity(Lpf + LPF_BARTEMP, init);
	dsp_setlpf1t(Lpf + LPF_THR, St.lpf.va, PID_FREQ, init);
	dsp_setunity(Lpf + LPF_VAU, init);
	dsp_setlpf1t(Lpf + LPF_VAPT1, St.lpf.va, PID_FREQ, init);
	dsp_setlpf1t(Lpf + LPF_VAAVG, VA_AVG_TCOEF, PID_FREQ, init);
	dsp_setunity(Lpf + LPF_FA, init);
	dsp_setunity(Lpf + LPF_ALT, init);

	// init low-pass fitlers for IMU temperature sensor
	dsp_setlpf1t(Lpf + LPF_IMUTEMP, TEMP_TCOEF, PID_FREQ, init);

	// init low-pass fitlers for accelerometer x, y and z axes
	dsp_setlpf1f(Lpf + LPF_ACCX, St.lpf.acc, PID_FREQ, init);
	dsp_setlpf1f(Lpf + LPF_ACCY, St.lpf.acc, PID_FREQ, init);
	dsp_setlpf1f(Lpf + LPF_ACCZ, St.lpf.acc, PID_FREQ, init);

	// init low-pass fitlers for gyroscope x, y and z axes
	dsp_setlpf1f(Lpf + LPF_GYROX, St.lpf.gyro, PID_FREQ, init);
	dsp_setlpf1f(Lpf + LPF_GYROY, St.lpf.gyro, PID_FREQ, init);
	dsp_setlpf1f(Lpf + LPF_GYROZ, St.lpf.gyro, PID_FREQ, init);

	// init low-pass fitlers for magnetometer x, y and z axes
	dsp_setlpf1t(Lpf + LPF_MAGX, St.lpf.mag, QMC_FREQ, init);
	dsp_setlpf1t(Lpf + LPF_MAGY, St.lpf.mag, QMC_FREQ, init);
	dsp_setlpf1t(Lpf + LPF_MAGZ, St.lpf.mag, QMC_FREQ, init);

	// init roll, pitch, yaw unity filters
	dsp_setunity(Lpf + LPF_ROLL, init);
	dsp_setunity(Lpf + LPF_PITCH, init);
	dsp_setunity(Lpf + LPF_YAW, init);

	return 0;
}

/**
* @brief Print quadcopter's postion and tilt data into a string.
* @param s output string
* @param md accelerometer and gyroscope data
* @param hd magnetometer data
* @return always 0
*/
static int sprintimu(char *s, struct icm_data *id)
{
	float ax, ay, az;

	s[0] = '\0';

	ax = dsp_getlpf(Lpf + LPF_ACCX);
	ay = dsp_getlpf(Lpf + LPF_ACCY);
	az = dsp_getlpf(Lpf + LPF_ACCZ);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"accel corrected: ",
		(double) Imudata.afx, (double) Imudata.afy,
		(double) Imudata.afz);
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"gyro corrected: ",
		(double) Imudata.gfx, (double) Imudata.gfy,
		(double) Imudata.gfz);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"accel filtered: ",
		(double) ax, (double) ay, (double) az);
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"gyro filtered: ",
		(double) (dsp_getlpf(Lpf + LPF_GYROX)),
		(double) (dsp_getlpf(Lpf + LPF_GYROY)),
		(double) (dsp_getlpf(Lpf + LPF_GYROZ)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"temp: %0.3f\r\n",
		(double) (dsp_getlpf(Lpf + LPF_IMUTEMP)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll: %0.3f; pitch: %0.3f; yaw: %0.3f\r\n",
		(double) dsp_getlpf(Lpf + LPF_ROLL),
		(double) dsp_getlpf(Lpf + LPF_PITCH),
		(double) dsp_getlpf(Lpf + LPF_YAW));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"z acceleration: %f\r\n",
		(double) dsp_getlpf(Lpf + LPF_THR));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"vertical acceleration: %f\r\n",
		(double) dsp_getlpf(Lpf + LPF_VAU));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel: %f\r\n",
			(double) sqrt(ax * ax + ay * ay + az * az));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"g offset: %f\r\n", (double) Goffset);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"forward acceleration: %f\r\n",
		(double) dsp_getlpf(Lpf + LPF_FA));

	return 0;
}

/**
* @brief Print magmetometer data into a string.
* @param s output string.
* @param hd magnetometer data.
* @return always 0
*/
static int sprintmag(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"mag corrected: ",
		(double) Qmcdata.fx, (double) Qmcdata.fy,
		(double) Qmcdata.fz);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"mag filtered: ",
		(double) (dsp_getlpf(Lpf + LPF_MAGX)),
		(double) (dsp_getlpf(Lpf + LPF_MAGY)),
		(double) (dsp_getlpf(Lpf + LPF_MAGZ)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"heading: %f\r\n",
		(double) dsp_getcompl(Cmpl + CMPL_YAW));

	return 0;
}

/**
* @brief Print all devices statuses into a string.
* @param s output string
* @return always 0
*/
static int sprintdevs(char *s)
{
	int i;

	s[0] = '\0';

	for (i = 0; i < DEV_COUNT; ++i) {
		const char *strstatus;

		switch (Dev[i].status) {
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
			Dev[i].name, strstatus);
	}

	return 0;
}

/**
* @brief Print various configuration values into a string.
* @param s output string
* @return always 0
*/
static int sprintvalues(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"t: %.3f; r: %.3f; p: %.3f; y: %.3f\r\n",
		(double) Thrust, (double) Rolltarget,
		(double) Pitchtarget, (double) Yawtarget);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll thrust: %.3f; pitch thrust %.3f\r\n",
		(double) St.adj.mtrsc.r, (double) St.adj.mtrsc.p);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag off: %.3f; %.3f; %.3f\r\n",
		(double) St.adj.mag0.x, (double) St.adj.mag0.y,
		(double) St.adj.mag0.z);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag scale: %.3f; %.3f; %.3f\r\n",
		(double) St.adj.magsc.x, (double) St.adj.magsc.y,
		(double) St.adj.magsc.z);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag decl: %.5f\r\n",
		(double) St.adj.magdecl);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel cor: %.3f; %.3f; %.3f\r\n",
		(double) St.adj.acc0.x, (double) St.adj.acc0.y,
		(double) St.adj.acc0.z);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"gyro cor: %.3f; %.3f; %.3f\r\n",
		(double) St.adj.gyro0.x, (double) St.adj.gyro0.y,
		(double) St.adj.gyro0.z);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll cor: %.3f; pitch cor: %.3f; yaw cor: %.3f\r\n",
		(double) St.adj.att0.roll, (double) St.adj.att0.pitch,
		(double) St.adj.att0.yaw);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"battery: %0.3f\r\n",
		(double) (dsp_getlpf(Lpf + LPF_BAT)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"esc current: %0.3f\r\n",
		(double) (dsp_getlpf(Lpf + LPF_CUR)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"motors state: %.3f\r\n", (double) En);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"loops count: %d\r\n", Loopscount);
	return 0;
}

/**
* @brief Print all PID values into a string.
* @param s output string
* @return always 0
*/
static int sprintpid(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"pos PID: %.5f,%.5f,%.5f\r\n",
		(double) St.pid.attpos.p, (double) St.pid.attpos.i,
		(double) St.pid.attpos.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"speed PID: %.5f,%.5f,%.5f\r\n",
		(double) St.pid.attrate.p, (double) St.pid.attrate.i,
		(double) St.pid.attrate.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw PID: %.5f,%.5f,%.5f\r\n",
		(double) St.pid.yawpos.p, (double) St.pid.yawpos.i,
		(double) St.pid.yawpos.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw speed PID: %.5f,%.5f,%.5f\r\n",
		(double) St.pid.yawrate.p, (double) St.pid.yawrate.i,
		(double) St.pid.yawrate.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"throttle PID: %.5f,%.5f,%.5f\r\n",
		(double) St.pid.throttle.p, (double) St.pid.throttle.i,
		(double) St.pid.throttle.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"climb rate PID: %.5f,%.5f,%.5f\r\n",
		(double) St.pid.climbrate.p,
		(double) St.pid.climbrate.i,
		(double) St.pid.climbrate.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude PID: %.5f,%.5f,%.5f\r\n",
		(double) St.pid.alt.p, (double) St.pid.alt.i,
		(double) St.pid.alt.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s mode\r\n", Speedpid ? "single" : "dual");

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s yaw mode\r\n",
		Yawspeedpid ? "single" : "dual");

	const char *mode;

	if (Altmode == ALTMODE_ACCEL)		mode = "single";
	else if (Altmode == ALTMODE_SPEED)	mode = "double";
	else					mode = "triple";

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%s altitude mode\r\n", mode);

	return 0;
}

/**
* @brief Print all GNSS values into a string.
* @param s output string
* @return always 0
*/
static int sprintgnss(char *s) {
	s[0] = '\0';

	sprintf(s, "%s: %f\r\n%s: %hd\r\n%s: %hd %f %c\r\n\
%s: %hd %f %c\r\n%s: %f\r\n%s: %f\r\n\
%s: %d\r\n%s: %d\r\n%s: %f\r\n%s: %s\r\n%s: %f\r\n%s: %c\r\n",
		"time", (double) Gnss.time,
		"status", Gnss.status,
		"latitude", Gnss.lat,
		(double) Gnss.latmin,
		(Gnss.latdir == LATDIR_N) ? 'N' : 'S',
		"longitude", Gnss.lon,
		(double) Gnss.lonmin,
		(Gnss.londir == LONDIR_W) ? 'W' : 'E',
		"speed", (double) Gnss.speed,
		"course", (double) Gnss.course,
		"quality", Gnss.quality,
		"satellites", Gnss.satellites,
		"altitude", (double) Gnss.altitude,
		"date", Gnss.date,
		"magvar", (double) Gnss.magvar,
		"magverdir",
		(Gnss.magvardir == MAGVARDIR_W) ? 'W' : 'E');

	return 0;
}

/**
* @brief Print all control scaling values.
* @param s output string
* @return always 0
*/
static int sprintfctrl(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum thrust: %.5f\r\n", (double) St.ctrl.thrustmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum roll: %.5f\r\n", (double) St.ctrl.rollmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum pitch: %.5f\r\n", (double) St.ctrl.pitchmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll speed: %.5f\r\n",
		(double) St.ctrl.rollrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"pitch speed: %.5f\r\n",
		(double) St.ctrl.pitchrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw speed: %.5f\r\n",
		(double) St.ctrl.yawrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum acceleration: %.5f\r\n",
		(double) St.ctrl.accelmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum climbrate: %.5f\r\n",
		(double) St.ctrl.climbratemax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum altitude: %.5f\r\n",
		(double) St.ctrl.altmax);

	return 0;
}

/**
* @brief Print filters coefficients into a string.
* @param s output string
* @return always 0
*/
static int sprintffilters(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"attitude compl tc: %.6f\r\n", (double) St.cmpl.att);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw compl tc: %.6f\r\n", (double) St.cmpl.yaw);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"climb rate compl tc: %.6f\r\n",
		(double) St.cmpl.climbrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude compl tc: %.6f\r\n", (double) St.cmpl.alt);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"gyro lpf cut-off: %.6f\r\n", (double) St.lpf.gyro);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel lpf cut-off: %.6f\r\n", (double) St.lpf.acc);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag lpf cut-off: %.6f\r\n", (double) St.lpf.mag);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"d lpf cut-off: %.6f\r\n", (double) St.lpf.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"thrust lpf tc: %.6f\r\n", (double) St.lpf.va);

	return 0;
}

/**
* @brief Print autopilot trail into a string.
* @param s output string
* @return always 0
*/
static int sprintfautopilot(char *s)
{
	int i;

	s[0] = '\0';

	for (i = 0; i < Pointscount; ++i) {
		if (Points[i].type == AUTOPILOT_START) {
			snprintf(s + strlen(s), INFOLEN - strlen(s),
				"%d %s", i, "start\r\n");
		}
		else if (Points[i].type == AUTOPILOT_TAKEOFF) {
			snprintf(s + strlen(s), INFOLEN - strlen(s),
				"%d %s alt: %f; t: %f;\r\n",
				i, "takeoff",
				(double) Points[i].takeoff.alt,
				(double) Points[i].takeoff.t);
		}
		else if (Points[i].type == AUTOPILOT_HOVER) {
			snprintf(s + strlen(s), INFOLEN - strlen(s),
				"%d %s x: %f; y: %f; alt: %f; t: %f;\r\n",
				i, "takeoff",
				(double) Points[i].hover.x,
				(double) Points[i].hover.y,
				(double) Points[i].hover.alt,
				(double) Points[i].hover.t);
		}
		else if (Points[i].type == AUTOPILOT_FORWARD) {
			snprintf(s + strlen(s), INFOLEN - strlen(s),
				"%d %s x: %f; y: %f;\r\n",
				i, "forward",
				(double) Points[i].forward.x,
				(double) Points[i].forward.y);
		}
		else if (Points[i].type == AUTOPILOT_LANDING) {
			snprintf(s + strlen(s), INFOLEN - strlen(s),
				"%d %s\r\n", i, "landing");
		}
		else if (Points[i].type == AUTOPILOT_STOP) {
			snprintf(s + strlen(s), INFOLEN - strlen(s),
				"%d %s\r\n", i, "stop");
		}
		else
			return (-1);	
	}

	return 0;
}


int rcmd(const struct cdevice *dev, const char **toks, char *out)
{
	En = 0.0;

	return 0;
}

int applycmd(const struct cdevice *dev, const char **toks, char *out)
{
	int irc, dsp, log;

	irc = dsp = log = 0;
	
	if (strcmp(toks[1], "irc") == 0)
		irc = 1;
	else if (strcmp(toks[1], "dsp") == 0)
		dsp = 1;
	else if (strcmp(toks[1], "log") == 0)
		log = 1;
	else {
		irc = dsp = log = 1;
	}

	if (irc) {
		if (Dev[DEV_IRC].status != DEVSTATUS_INIT)
			return 0;

		Dev[DEV_IRC].configure(Dev[DEV_IRC].priv, "set",
			"frequency", St.irc.freq);
	
		Dev[DEV_IRC].configure(Dev[DEV_IRC].priv, "set",
			"power", St.irc.power);
	}
	
	if (dsp)
		setstabilize(0);

	if (log)
		modifytimev(Evs + TEV_LOG, St.log.freq);

	validatesettings();

	return 0;
}

int infocmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "imu") == 0) {
		struct icm_data id;

		Dev[DEV_ICM].read(Dev[DEV_ICM].priv, &id,
			sizeof(struct icm_data));

		sprintimu(out, &id);
	}
	else if (strcmp(toks[1], "mag") == 0)
		sprintmag(out);
	else if (strcmp(toks[1], "baro") == 0) {
		struct dps_data dd;

		Dev[DEV_DPS].read(Dev[DEV_DPS].priv, &dd,
			sizeof(struct dps_data));

		snprintf(out, INFOLEN,
			"baro temp: %f; baro alt: %f\r\n",
			(double) dd.tempf, (double) dd.altf);

		snprintf(out + strlen(out), INFOLEN - strlen(out),
			"filtered temp: %f; filtered alt: %f\r\n",
			(double) dsp_getlpf(Lpf + LPF_BARTEMP),
			(double) dsp_getlpf(Lpf + LPF_ALT));

		snprintf(out + strlen(out), INFOLEN - strlen(out),
			"climb rate: %f\r\nalt: %f\r\nref alt: %f\r\n",
			(double) dsp_getcompl(Cmpl + CMPL_CLIMBRATE),
			(double) dsp_getcompl(Cmpl + CMPL_ALT),
			(double) Alt0);
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

		Dev[DEV_IRC].read(Dev[DEV_IRC].priv, &data,
			sizeof(struct irc_data));

		snprintf(out, INFOLEN, "frequency: %d; power: %d\r\n",
			data.frequency, data.power);
	}
	else if (strcmp(toks[1], "autopilot") == 0)
		sprintfautopilot(out);
	else
		return (-1);

	return 1;
}

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

int systemcmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "esp") == 0) {
		if (strcmp(toks[2], "flash") == 0)
			Dev[DEV_ESP].configure(Dev[DEV_ESP].priv, "flash");
		else if (strcmp(toks[2], "run") == 0)
			Dev[DEV_ESP].configure(Dev[DEV_ESP].priv, "run");
		else
			return (-1);
	}
	else
		return (-1);

	return 0;
}

int logcmd(const struct cdevice *d, const char **toks, char *out)
{
	char s[INFOLEN];

	if (strcmp(toks[1], "set") == 0) {
		// flash erasing process takes time and blocks
		// other actions, so disarm for safety
		En = 0.0;
		setthrust(Dev + DEV_DSHOT, 0.0, 0.0, 0.0, 0.0);

		setlog(atoi(toks[2]), d, s);
	}
	else if (strcmp(toks[1], "rget") == 0) {
		// print records from specified range
		if (printlog(d, s, atoi(toks[2]),
				atoi(toks[3])) < 0) {
			return (-1);
		}

		// write end marker
		sprintf(s, "-end-\r\n");
		d->write(d->priv, s, strlen(s));
	}
	else if (strcmp(toks[1], "bget") == 0) {
		const char **p;

		// print every record whose
		// number is in arguments
		for (p = toks + 2; strlen(*p) != 0; ++p) {
			if (printlog(d, s, atoi(*p),
					atoi(*p) + 1) < 0) {
				return (-1);
			}
		}

		// write end marker
		sprintf(s, "-end-\r\n");
		d->write(d->priv, s, strlen(s));
	}
	else
		return (-1);

	return 0;
}

int motorcmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "lt") == 0) {
		if (strcmp(toks[2], "r") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"reverse", St.mtr.lt);
		}
		else if (strcmp(toks[2], "d") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"direct", St.mtr.lt);
		}
		else if (strcmp(toks[2], "s") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"save", St.mtr.lt);
			mdelay(35);
		}
		else
			return (-1);
	}

	else if (strcmp(toks[1], "lb") == 0) {
		if (strcmp(toks[2], "r") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"reverse", St.mtr.lb);
		}
		else if (strcmp(toks[2], "d") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"direct", St.mtr.lb);
		}
		else if (strcmp(toks[2], "s") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"save", St.mtr.lb);
			mdelay(35);
		}
		else
			return (-1);
	}
	else if (strcmp(toks[1], "rb") == 0) {
		if (strcmp(toks[2], "r") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"reverse", St.mtr.rb);
		}
		else if (strcmp(toks[2], "d") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"direct", St.mtr.rb);
		}
		else if (strcmp(toks[2], "s") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"save", St.mtr.rb);
			mdelay(35);
		}
		else
			return (-1);
	}
	else if (strcmp(toks[1], "rt") == 0) {
		if (strcmp(toks[2], "r") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"reverse", St.mtr.rt);
		}
		else if (strcmp(toks[2], "d") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"direct", St.mtr.rt);
		}
		else if (strcmp(toks[2], "s") == 0) {
			Dev[DEV_DSHOT].configure(Dev[DEV_DSHOT].priv,
				"save", St.mtr.rt);
			mdelay(35);
		}
		else
			return (-1);
	}
	else
		return (-1);

	return 0;
}

int autopilotcmd(const struct cdevice *d, const char **toks, char *out)
{
	int idx;

	if (strcmp(toks[1], "count") == 0) {
		Pointscount = atoi(toks[2]);
		return 0;
	}
	else {
		idx = atoi(toks[1]);
	
		if (idx < 0 || idx >= Pointscount)
			return (-1);

		if (strcmp(toks[2], "type") == 0) {
			if (strcmp(toks[3], "start") == 0)
				Points[idx].type = AUTOPILOT_START;
			else if (strcmp(toks[3], "takeoff") == 0)
				Points[idx].type = AUTOPILOT_TAKEOFF;
			else if (strcmp(toks[3], "hover") == 0)
				Points[idx].type = AUTOPILOT_HOVER;
			else if (strcmp(toks[3], "forward") == 0)
				Points[idx].type = AUTOPILOT_FORWARD;
			else if (strcmp(toks[3], "landing") == 0)
				Points[idx].type = AUTOPILOT_LANDING;
			else if (strcmp(toks[3], "stop") == 0)
				Points[idx].type = AUTOPILOT_STOP;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "alt") == 0) {
			if (Points[idx].type == AUTOPILOT_TAKEOFF)
				Points[idx].takeoff.alt = atof(toks[3]);
			else if (Points[idx].type == AUTOPILOT_HOVER)
				Points[idx].hover.alt = atof(toks[3]);
			else
				return (-1);
		}
		else if (strcmp(toks[2], "t") == 0) {
			if (Points[idx].type == AUTOPILOT_TAKEOFF)
				Points[idx].takeoff.t = atof(toks[3]);
			else if (Points[idx].type == AUTOPILOT_HOVER)
				Points[idx].hover.t = atof(toks[3]);
			else
				return (-1);
		}
		else if (strcmp(toks[2], "x") == 0) {
			if (Points[idx].type == AUTOPILOT_HOVER)
				Points[idx].hover.x = atof(toks[3]);
			else if (Points[idx].type == AUTOPILOT_FORWARD)
				Points[idx].forward.x = atof(toks[3]);
			else
				return (-1);
		}
		else if (strcmp(toks[2], "y") == 0) {
			if (Points[idx].type == AUTOPILOT_HOVER)
				Points[idx].hover.y = atof(toks[3]);
			else if (Points[idx].type == AUTOPILOT_FORWARD)
				Points[idx].forward.y = atof(toks[3]);
			else
				return (-1);
		}
		else
			return (-1);
	}

	return 0;
}

int setcmd(const struct cdevice *d, const char **toks, char *out)
{
	const char **p;
	struct settingnode *node;

	node = &Sttree;
			
	// for every token starting from 
	// second to skip command name
	for (p = toks + 1; *p != NULL; ++p) {
		// if current node it terminal, 
		// set corresponding setting value
		if (node->type == NODETYPE_FLOAT)
			*(node->f) = atof(*(p));
		else if (node->type == NODETYPE_INT)
			*(node->i) = atoi(*(p));
		else if (node->type == NODETYPE_MAP) {
			int strn;
			const char *key;
			int v;
			int i;

			// current token is value,
			// next token is key
			key = *(p + 1);
			v = atoi(*p);
			++p;

			// if no next token, then it is a parsing error
			if (key == NULL)
				return (-1);

			// if command is to unset specific
			// value in map just return
			if (strcmp(key, "none") == 0)
				return 0;

			// search for index corresponding to key in map	
			if ((strn = mapsearch(node->m.k, key)) < 0)
				return (-1);
		
			// unset this value if it was already set	
			for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
				if (*(node->m.m + i) == v)
					*(node->m.m + i) = 99;
			}

			// set value using key's index
			St.log.fieldid[strn] = v;
		}
		else {
			struct settingnode **chd;
	
			// for every child node	
			for (chd = node->child; *chd != NULL; ++chd) {
				// if child node's token same as
				// current token go to this node
				if (strcmp((*chd)->token, *p) == 0) {
					node = *chd;
					break;
				}
			}

			if (*chd == NULL)
				return (-1);
		}
	}

	return 0;
}

int getcmd(const struct cdevice *d, const char **toks, char *out)
{
	const char **p;
	struct settingnode *node;
	char *data;
	enum CONFVALTYPE valtype;
	float vf;
	int vi;
	const char *vs;

	// set defaults for output values
	vf = 0.0;
	vi = 0;

	valtype = CONFVALTYPE_FLOAT;

	node = &Sttree;

	// for every token starting from 
	// second to skip command name
	for (p = toks + 1; *p != NULL; ++p) {
		struct settingnode **chd;
	
		// if current node is terminal, it is a parsing error	
		if (node->type != NODETYPE_PARENT)
			break;
	
		// for every child node	
		for (chd = node->child; *chd != NULL; ++chd) {
			// if child node's token same as
			// current token go to this node
			if (strcmp((*chd)->token, *p) == 0) {
				node = *chd;
				break;
			}
		}

		// if no correspondint child node
		// found, it is a parsing error
		if (*chd == NULL)
			return (-1);
	}
	// get corresponding setting value from found
	// node and set it's output type for printf
	if (node->type == NODETYPE_FLOAT) {
		vf = *(node->f);
		valtype = CONFVALTYPE_FLOAT;
	}
	else if (node->type == NODETYPE_INT) {
		vi = *(node->i);
		valtype = CONFVALTYPE_INT;
	}
	else if (node->type == NODETYPE_MAP) {
		int recn;
		int i;

		recn = atoi(*p);

		// search map for a specific value 
		for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
			if (St.log.fieldid[i] == recn)
				break;
		}

		// get key of a found value
		vs = (i >= LOG_FIELDSTRSIZE) ? "none" : node->m.k[i];

		valtype = CONFVALTYPE_STRING;
	}

	// start printing output from 7th character of output buffer
	data = out + 6;

	// reconstruct this config command from tokens
	data[0] = '\0';
	for (p = toks + 1; *p != NULL; ++p)
		sprintf(data + strlen(data), "%s ", *p);

	// print value got from config
	switch (valtype) {
	case CONFVALTYPE_FLOAT:
		sprintf(data + strlen(data), "%f\r\n", (double) vf);
		break;

	case CONFVALTYPE_INT:
		sprintf(data + strlen(data), "%d\r\n", vi);
		break;

	case CONFVALTYPE_STRING:
		sprintf(data + strlen(data), "%s\r\n", vs);
		break;
	}

	// add CRC-16 putting it into first 5 characters
	// of output buffer, put separating space to 6th
	// character of output buffer
	sprintf(out, "%05u", crc16((uint8_t *) data, strlen(data)));
	out[5] = ' ';

	return 1;
}
