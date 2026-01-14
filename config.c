#include "config.h"

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "settings.h"
#include "dsp.h"
#include "log.h"
#include "crc.h"
#include "global.h"
#include "runvals.h"
#include "timev.h"
#include "command.h"
#include "util.h"
#include "thrust.h"

#include "icm42688.h"
#include "dps368.h"
#include "qmc5883l.h"
#include "irc.h"

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
static int sprintpos(char *s, struct icm_data *id)
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
static int sprintqmc(char *s)
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

int rcmd(const struct cdevice *dev, const char **toks, char *out)
{
	En = 0.0;

	return 0;
}

int applycmd(const struct cdevice *dev, const char **toks, char *out)
{
	setstabilize(0);

	return 0;
}

int infocmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "mpu") == 0) {
		struct icm_data id;

		Dev[DEV_ICM].read(Dev[DEV_ICM].priv, &id,
			sizeof(struct icm_data));

		sprintpos(out, &id);
	}
	else if (strcmp(toks[1], "qmc") == 0)
		sprintqmc(out);
	else if (strcmp(toks[1], "hp") == 0) {
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
	else
		return (-1);

	return 1;
}

int pidcmd(const struct cdevice *dev, const char **toks, char *out)
{

	float v;

	v = atof(toks[3]);

	if (strcmp(toks[1], "tilt") == 0) {
		if (strcmp(toks[2], "p") == 0)
			St.pid.attpos.p = v;
		else if (strcmp(toks[2], "i") == 0)
			St.pid.attpos.i = v;
		else if (strcmp(toks[2], "d") == 0)
			St.pid.attpos.d = v;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "stilt") == 0) {
		if (strcmp(toks[2], "p") == 0)
			St.pid.attrate.p = v;
		else if (strcmp(toks[2], "i") == 0)
			St.pid.attrate.i = v;
		else if (strcmp(toks[2], "d") == 0)
			St.pid.attrate.d = v;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "yaw") == 0) {
		if (strcmp(toks[2], "p") == 0)
			St.pid.yawpos.p = v;
		else if (strcmp(toks[2], "i") == 0)
			St.pid.yawpos.i = v;
		else if (strcmp(toks[2], "d") == 0)
			St.pid.yawpos.d = v;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "syaw") == 0) {
		if (strcmp(toks[2], "p") == 0)
			St.pid.yawrate.p = v;
		else if (strcmp(toks[2], "i") == 0)
			St.pid.yawrate.i = v;
		else if (strcmp(toks[2], "d") == 0)
			St.pid.yawrate.d = v;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "throttle") == 0) {
		if (strcmp(toks[2], "p") == 0)
			St.pid.throttle.p = v;
		else if (strcmp(toks[2], "i") == 0)
			St.pid.throttle.i = v;
		else if (strcmp(toks[2], "d") == 0)
			St.pid.throttle.d = v;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		if (strcmp(toks[2], "p") == 0)
			St.pid.climbrate.p = v;
		else if (strcmp(toks[2], "i") == 0)
			St.pid.climbrate.i = v;
		else if (strcmp(toks[2], "d") == 0)
			St.pid.climbrate.d = v;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		if (strcmp(toks[2], "p") == 0)
			St.pid.alt.p = v;
		else if (strcmp(toks[2], "i") == 0)
			St.pid.alt.i = v;
		else if (strcmp(toks[2], "d") == 0)
			St.pid.alt.d = v;
		else
			return (-1);
	}
	else
		return (-1);
	
	return 0;
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

int complcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "attitude") == 0)
		St.cmpl.att = atof(toks[2]);
	else if (strcmp(toks[1], "yaw") == 0)
		St.cmpl.yaw = atof(toks[2]);
	else if (strcmp(toks[1], "climbrate") == 0)
		St.cmpl.climbrate = atof(toks[2]);
	else if (strcmp(toks[1], "altitude") == 0)
		St.cmpl.alt = atof(toks[2]);

	return 0;
}

int lpfcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "gyro") == 0)
		St.lpf.gyro = atof(toks[2]);
	else if (strcmp(toks[1], "accel") == 0)
		St.lpf.acc = atof(toks[2]);
	else if (strcmp(toks[1], "mag") == 0)
		St.lpf.mag = atof(toks[2]);
	else if (strcmp(toks[1], "d") == 0)
		St.lpf.d = atof(toks[2]);
	else if (strcmp(toks[1], "vaccel") == 0)
		St.lpf.va = atof(toks[2]);
	else
		return (-1);

	return 0;
}

int adjcmd(const struct cdevice *dev, const char **toks, char *out)
{
	float v;

	v = atof(toks[2]);

	if (strcmp(toks[1], "rollthrust") == 0)
		St.adj.mtrsc.r = v;
	else if (strcmp(toks[1], "pitchthrust") == 0)
		St.adj.mtrsc.p = v;
	else if (strcmp(toks[1], "roll") == 0)
		St.adj.att0.roll = v;
	else if (strcmp(toks[1], "pitch") == 0)
		St.adj.att0.pitch = v;
	else if (strcmp(toks[1], "yaw") == 0)
		St.adj.att0.yaw = v;
	else if (strcmp(toks[1], "acc") == 0) {
		if (strcmp(toks[2], "x") == 0)
			St.adj.acc0.x = atof(toks[3]);
		else if (strcmp(toks[2], "y") == 0)
			St.adj.acc0.y = atof(toks[3]);
		else if (strcmp(toks[2], "z") == 0)
			St.adj.acc0.z = atof(toks[3]);
		else if (strcmp(toks[2], "xtscale") == 0)
			St.adj.acctsc.x = atof(toks[3]);
		else if (strcmp(toks[2], "ytscale") == 0)
			St.adj.acctsc.y = atof(toks[3]);
		else if (strcmp(toks[2], "ztscale") == 0)
			St.adj.acctsc.z = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "gyro") == 0) {
		if (strcmp(toks[2], "x") == 0)
			St.adj.gyro0.x = atof(toks[3]);
		else if (strcmp(toks[2], "y") == 0)
			St.adj.gyro0.y = atof(toks[3]);
		else if (strcmp(toks[2], "z") == 0)
			St.adj.gyro0.z = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "mag") == 0) {
		if (strcmp(toks[2], "x0") == 0)
			St.adj.mag0.x = atof(toks[3]);
		else if (strcmp(toks[2], "y0") == 0)
			St.adj.mag0.y = atof(toks[3]);
		else if (strcmp(toks[2], "z0") == 0)
			St.adj.mag0.z = atof(toks[3]);
		else if (strcmp(toks[2], "xscale") == 0)
			St.adj.magsc.x = atof(toks[3]);
		else if (strcmp(toks[2], "yscale") == 0)
			St.adj.magsc.y = atof(toks[3]);
		else if (strcmp(toks[2], "zscale") == 0)
			St.adj.magsc.z = atof(toks[3]);
		else if (strcmp(toks[2], "xthscale") == 0)
			St.adj.magthrsc.x = atof(toks[3]);
		else if (strcmp(toks[2], "ythscale") == 0)
			St.adj.magthrsc.y = atof(toks[3]);
		else if (strcmp(toks[2], "zthscale") == 0)
			St.adj.magthrsc.z = atof(toks[3]);
		else if (strcmp(toks[2], "decl") == 0)
			St.adj.magdecl = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "curr") == 0) {
		if (strcmp(toks[2], "offset") == 0)
			St.adj.curroff = atof(toks[3]);
		else if (strcmp(toks[2], "scale") == 0)
			St.adj.cursc = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "althold") == 0) {
		if (strcmp(toks[2], "hover") == 0)
			St.adj.hoverthrottle = atof(toks[3]);
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
		setthrust(0.0, 0.0, 0.0, 0.0);

		log_set(atoi(toks[2]), d, s);
	}
	else if (strcmp(toks[1], "rget") == 0) {
		// print records from specified range
		if (log_print(d, s, atoi(toks[2]), atoi(toks[3])) < 0)
			return (-1);

		// write end marker
		sprintf(s, "-end-\r\n");
		d->write(d->priv, s, strlen(s));
	}
	else if (strcmp(toks[1], "bget") == 0) {
		const char **p;

		// print every record whose number is in arguments
		for (p = toks + 2; strlen(*p) != 0; ++p) {
			if (log_print(d, s, atoi(*p), atoi(*p) + 1) < 0)
				return (-1);
		}

		// write end marker
		sprintf(s, "-end-\r\n");
		d->write(d->priv, s, strlen(s));
	}
	else if (strcmp(toks[1], "freq") == 0) {
		St.log.freq = atoi(toks[2]);

		if (St.log.freq < 0 || St.log.freq > 4096)
			St.log.freq = 128;

		modifytimev(Evs + TEV_LOG, St.log.freq);

		return 0;
	}
	else if (strcmp(toks[1], "record") == 0) {
		if (strcmp(toks[2], "size") == 0) {
			unsigned int p;

			for (p = 1; p < atoi(toks[3]); p <<= 1);

			if (p > LOG_MAXRECSIZE)
				p = LOG_MAXRECSIZE;

			St.log.recsize = p;
		}
		else {
			int strn;
			int i;

			if (strcmp(toks[3], "none") == 0)
				return 0;

			if ((strn = log_fieldstrn(toks[3])) < 0)
				return (-1);

			for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
				if (St.log.fieldid[i] == atoi(toks[2]))
					St.log.fieldid[i] = 99;
			}

			St.log.fieldid[strn] = atoi(toks[2]);
		}

		return 0;
	}
	else
		return (-1);

	return 1;
}

int autopilotcmd(const struct cdevice *d, const char **toks, char *out)
{
	int idx;

	if (strcmp(toks[1], "count") == 0) {
		Pointscount = atoi(toks[2]);
		return 0;
	}
	else if (strcmp(toks[1], "type") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= Pointscount)
			return (-1);

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
	else if (strcmp(toks[1], "alt") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= Pointscount)
			return (-1);
		
		if (Points[idx].type == AUTOPILOT_TAKEOFF)
			Points[idx].takeoff.alt = atof(toks[3]);
		else if (Points[idx].type == AUTOPILOT_HOVER)
			Points[idx].hover.alt = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "t") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= Pointscount)
			return (-1);
		
		if (Points[idx].type == AUTOPILOT_TAKEOFF)
			Points[idx].takeoff.t = atof(toks[3]);
		else if (Points[idx].type == AUTOPILOT_HOVER)
			Points[idx].hover.t = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "x") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= Pointscount)
			return (-1);
		
		if (Points[idx].type == AUTOPILOT_HOVER)
			Points[idx].hover.x = atof(toks[3]);
		else if (Points[idx].type == AUTOPILOT_FORWARD)
			Points[idx].forward.x = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "y") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= Pointscount)
			return (-1);
		
		if (Points[idx].type == AUTOPILOT_HOVER)
			Points[idx].hover.y = atof(toks[3]);
		else if (Points[idx].type == AUTOPILOT_FORWARD)
			Points[idx].forward.y = atof(toks[3]);
		else
			return (-1);
	}
	else
		return (-1);

	return 0;
}

int ctrlcmd(const struct cdevice *d, const char **toks, char *out)
{
	float v;

	v = atof(toks[2]);

	if (strcmp(toks[1], "thrust") == 0)
		St.ctrl.thrustmax = v;
	else if (strcmp(toks[1], "roll") == 0)
		St.ctrl.rollmax = v;
	else if (strcmp(toks[1], "pitch") == 0)
		St.ctrl.pitchmax = v;
	else if (strcmp(toks[1], "sroll") == 0)
		St.ctrl.rollrate = v;
	else if (strcmp(toks[1], "spitch") == 0)
		St.ctrl.pitchrate = v;
	else if (strcmp(toks[1], "syaw") == 0)
		St.ctrl.yawrate = v;
	else if (strcmp(toks[1], "accel") == 0)
		St.ctrl.accelmax = v;
	else if (strcmp(toks[1], "climbrate") == 0)
		St.ctrl.climbratemax = v;
	else if (strcmp(toks[1], "altmax") == 0)
		St.ctrl.altmax = v;
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

int irccmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "frequency") == 0) {
		St.irc.freq = atoi(toks[2]);

		if (Dev[DEV_IRC].status != DEVSTATUS_INIT)
			return 0;

		Dev[DEV_IRC].configure(Dev[DEV_IRC].priv, "set",
			"frequency", atoi(toks[2]));
	}
	else if (strcmp(toks[1], "power") == 0) {
		St.irc.power = atoi(toks[2]);

		if (Dev[DEV_IRC].status != DEVSTATUS_INIT)
			return 0;

		Dev[DEV_IRC].configure(Dev[DEV_IRC].priv, "set",
			"power", atoi(toks[2]));
	}
	else
		return (-1);

	return 0;
}

int motorcmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "lt") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(St.mtr.lt, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(St.mtr.lt, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(St.mtr.lt, 12);
			mdelay(35);
		}
		else
			St.mtr.lt = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "lb") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(St.mtr.lb, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(St.mtr.lb, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(St.mtr.lb, 12);
			mdelay(35);
		}
		else
			St.mtr.lb = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "rb") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(St.mtr.rb, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(St.mtr.rb, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(St.mtr.rb, 12);
			mdelay(35);
		}
		else
			St.mtr.rb = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "rt") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(St.mtr.rt, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(St.mtr.rt, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(St.mtr.rt, 12);
			mdelay(35);
		}
		else
			St.mtr.rt = atoi(toks[2]);
	}
	else
		return (-1);

	return 0;
}

int getcmd(const struct cdevice *d, const char **toks, char *out)
{
	const char **p;
	char *data;
	enum CONFVALTYPE valtype;
	float vf;
	int vi;
	const char *vs;

	valtype = CONFVALTYPE_FLOAT;

	if (strcmp(toks[1], "pid") == 0) {
		if (strcmp(toks[2], "tilt") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = St.pid.attpos.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = St.pid.attpos.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = St.pid.attpos.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "stilt") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = St.pid.attrate.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = St.pid.attrate.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = St.pid.attrate.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "yaw") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = St.pid.yawpos.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = St.pid.yawpos.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = St.pid.yawpos.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "syaw") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = St.pid.yawrate.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = St.pid.yawrate.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = St.pid.yawrate.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "throttle") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = St.pid.throttle.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = St.pid.throttle.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = St.pid.throttle.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "climbrate") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = St.pid.climbrate.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = St.pid.climbrate.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = St.pid.climbrate.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "altitude") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = St.pid.alt.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = St.pid.alt.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = St.pid.alt.d;
			else
				return (-1);
		}
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "compl") == 0) {
		if (strcmp(toks[2], "attitude") == 0)
			vf = St.cmpl.att;
		else if (strcmp(toks[2], "yaw") == 0)
			vf = St.cmpl.yaw;
		else if (strcmp(toks[2], "climbrate") == 0)
			vf = St.cmpl.climbrate;
		else if (strcmp(toks[2], "altitude") == 0)
			vf = St.cmpl.alt;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "lpf") == 0) {
		if (strcmp(toks[2], "gyro") == 0)
			vf = St.lpf.gyro;
		else if (strcmp(toks[2], "accel") == 0)
			vf = St.lpf.acc;
		else if (strcmp(toks[2], "mag") == 0)
			vf = St.lpf.mag;
		else if (strcmp(toks[2], "d") == 0)
			vf = St.lpf.d;
		else if (strcmp(toks[2], "vaccel") == 0)
			vf = St.lpf.va;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "adj") == 0) {
		if (strcmp(toks[2], "rollthrust") == 0)
			vf = St.adj.mtrsc.r;
		else if (strcmp(toks[2], "pitchthrust") == 0)
			vf = St.adj.mtrsc.p;
		else if (strcmp(toks[2], "roll") == 0)
			vf = St.adj.att0.roll;
		else if (strcmp(toks[2], "pitch") == 0)
			vf = St.adj.att0.pitch;
		else if (strcmp(toks[2], "yaw") == 0)
			vf = St.adj.att0.yaw;
		else if (strcmp(toks[2], "acc") == 0) {
			if (strcmp(toks[3], "x") == 0)
				vf = St.adj.acc0.x;
			else if (strcmp(toks[3], "y") == 0)
				vf = St.adj.acc0.y;
			else if (strcmp(toks[3], "z") == 0)
				vf = St.adj.acc0.z;
			else if (strcmp(toks[3], "xtscale") == 0)
				vf = St.adj.acctsc.x;
			else if (strcmp(toks[3], "ytscale") == 0)
				vf = St.adj.acctsc.y;
			else if (strcmp(toks[3], "ztscale") == 0)
				vf = St.adj.acctsc.z;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "gyro") == 0) {
			if (strcmp(toks[3], "x") == 0)
				vf = St.adj.gyro0.x;
			else if (strcmp(toks[3], "y") == 0)
				vf = St.adj.gyro0.y;
			else if (strcmp(toks[3], "z") == 0)
				vf = St.adj.gyro0.z;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "mag") == 0) {
			if (strcmp(toks[3], "x0") == 0)
				vf = St.adj.mag0.x;
			else if (strcmp(toks[3], "y0") == 0)
				vf = St.adj.mag0.y;
			else if (strcmp(toks[3], "z0") == 0)
				vf = St.adj.mag0.z;
			else if (strcmp(toks[3], "xscale") == 0)
				vf = St.adj.magsc.x;
			else if (strcmp(toks[3], "yscale") == 0)
				vf = St.adj.magsc.y;
			else if (strcmp(toks[3], "zscale") == 0)
				vf = St.adj.magsc.z;
			else if (strcmp(toks[3], "xthscale") == 0)
				vf = St.adj.magthrsc.x;
			else if (strcmp(toks[3], "ythscale") == 0)
				vf = St.adj.magthrsc.y;
			else if (strcmp(toks[3], "zthscale") == 0)
				vf = St.adj.magthrsc.z;
			else if (strcmp(toks[3], "decl") == 0)
				vf = St.adj.magdecl;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "curr") == 0) {
			if (strcmp(toks[3], "offset") == 0)
				vf = St.adj.curroff;
			else if (strcmp(toks[3], "scale") == 0)
				vf = St.adj.cursc;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "althold") == 0) {
			if (strcmp(toks[3], "hover") == 0)
				vf = St.adj.hoverthrottle;
			else
				return (-1);
		}
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "ctrl") == 0) {
		if (strcmp(toks[2], "thrust") == 0)
			vf = St.ctrl.thrustmax;
		else if (strcmp(toks[2], "roll") == 0)
			vf = St.ctrl.rollmax;
		else if (strcmp(toks[2], "pitch") == 0)
			vf = St.ctrl.pitchmax;
		else if (strcmp(toks[2], "sroll") == 0)
			vf = St.ctrl.rollrate;
		else if (strcmp(toks[2], "spitch") == 0)
			vf = St.ctrl.pitchrate;
		else if (strcmp(toks[2], "syaw") == 0)
			vf = St.ctrl.yawrate;
		else if (strcmp(toks[2], "accel") == 0)
			vf = St.ctrl.accelmax;
		else if (strcmp(toks[2], "climbrate") == 0)
			vf = St.ctrl.climbratemax;
		else if (strcmp(toks[2], "altmax") == 0)
			vf = St.ctrl.altmax;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "autopilot") == 0) {
		int idx;

		if (strcmp(toks[2], "count") == 0) {
			vi = Pointscount;
			valtype = CONFVALTYPE_INT;
		}
		else if (strcmp(toks[2], "type") == 0) {
			idx = atoi(toks[3]);
		
			if (idx < 0 || idx >= Pointscount)
				return (-1);
		
			if (Points[idx].type == AUTOPILOT_START)
				vs = "start";
			else if (Points[idx].type == AUTOPILOT_TAKEOFF)
				vs = "takeoff";
			else if (Points[idx].type == AUTOPILOT_HOVER)
				vs = "hover";
			else if (Points[idx].type == AUTOPILOT_FORWARD)
				vs = "forward";
			else if (Points[idx].type == AUTOPILOT_LANDING)
				vs = "landing";
			else if (Points[idx].type == AUTOPILOT_STOP)
				vs = "stop";
			else
				return (-1);

			valtype = CONFVALTYPE_STRING;
		}
		else if (strcmp(toks[2], "alt") == 0) {
			idx = atoi(toks[3]);
			
			if (idx < 0 || idx >= Pointscount)
				return (-1);
			
			if (Points[idx].type == AUTOPILOT_TAKEOFF)
				vf = Points[idx].takeoff.alt;
			else if (Points[idx].type == AUTOPILOT_HOVER)
				vf = Points[idx].hover.alt;
			else
				return (-1);
			
			valtype = CONFVALTYPE_FLOAT;
		}
		else if (strcmp(toks[2], "t") == 0) {
			idx = atoi(toks[3]);
			
			if (idx < 0 || idx >= Pointscount)
				return (-1);
			
			if (Points[idx].type == AUTOPILOT_TAKEOFF)
				vf = Points[idx].takeoff.t;
			else if (Points[idx].type == AUTOPILOT_HOVER)
				vf = Points[idx].hover.t;
			else
				return (-1);
			
			valtype = CONFVALTYPE_FLOAT;
		}
		else if (strcmp(toks[2], "x") == 0) {
			idx = atoi(toks[3]);
				
			if (idx < 0 || idx >= Pointscount)
				return (-1);
		
			if (Points[idx].type == AUTOPILOT_HOVER)
				vf = Points[idx].hover.x;
			else if (Points[idx].type == AUTOPILOT_FORWARD)
				vf = Points[idx].forward.x;
			else
				return (-1);
			
			valtype = CONFVALTYPE_FLOAT;
		}
		else if (strcmp(toks[2], "y") == 0) {
			idx = atoi(toks[3]);
			
			if (idx < 0 || idx >= Pointscount)
				return (-1);
			
			if (Points[idx].type == AUTOPILOT_HOVER)
				vf = Points[idx].hover.y;
			else if (Points[idx].type == AUTOPILOT_FORWARD)
				vf = Points[idx].forward.y;
			else
				return (-1);
			
			valtype = CONFVALTYPE_FLOAT;
		}
		else
			return (-1);
	}
	else if (strcmp(toks[1], "irc") == 0) {
		if (strcmp(toks[2], "frequency") == 0)
			vi = St.irc.freq;
		else if (strcmp(toks[2], "power") == 0)
			vi = St.irc.power;
		else
			return (-1);

		valtype = CONFVALTYPE_INT;
	}
	else if (strcmp(toks[1], "log") == 0) {
		if (strcmp(toks[2], "freq") == 0) {
			vi = St.log.freq;
			valtype = CONFVALTYPE_INT;
		}
		else if (strcmp(toks[2], "record") == 0) {
			if (strcmp(toks[3], "size") == 0) {
				vi = St.log.recsize;
				valtype = CONFVALTYPE_INT;
			}
			else {
				int recn;
				int i;

				recn = atoi(toks[3]);

				if (recn < 0 || recn > LOG_MAXRECSIZE)
					return (-1);

				for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
					if (St.log.fieldid[i] == recn)
						break;
				}

				if (i >= LOG_FIELDSTRSIZE)
					vs = "none";
				else
					vs = logfieldstr[i];

				valtype = CONFVALTYPE_STRING;
			}
		}
		else
			return (-1);
	}	
	else if (strcmp(toks[1], "motor") == 0) {
		if (strcmp(toks[2], "lt") == 0)
			vi = St.mtr.lt;
		else if (strcmp(toks[2], "lb") == 0)
			vi = St.mtr.lb;
		else if (strcmp(toks[2], "rb") == 0)
			vi = St.mtr.rb;
		else if (strcmp(toks[2], "rt") == 0)
			vi = St.mtr.rt;

		valtype = 0;
	}
	else
		return (-1);

	data = out + 6;

	data[0] = '\0';
	for (p = toks + 1; *p != NULL; ++p)
		sprintf(data + strlen(data), "%s ", *p);

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

	sprintf(out, "%05u", crc16((uint8_t *) data, strlen(data)));
	out[5] = ' ';

	return 1;
}
