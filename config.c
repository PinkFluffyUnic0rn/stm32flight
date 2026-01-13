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

	ax = dsp_getlpf(lpf + LPF_ACCX);
	ay = dsp_getlpf(lpf + LPF_ACCY);
	az = dsp_getlpf(lpf + LPF_ACCZ);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n", "accel: ",
		(double) ax, (double) ay, (double) az);	

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n", "gyro: ",
		(double) dsp_getlpf(lpf + LPF_GYROX),
		(double) dsp_getlpf(lpf + LPF_GYROY),
		(double) dsp_getlpf(lpf + LPF_GYROZ));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"accel corrected: ",
		(double) (ax - st.adj.acc0.x),
		(double) (ay - st.adj.acc0.y),
		(double) (az - st.adj.acc0.z));
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"gyro corrected: ",
		(double) (dsp_getlpf(lpf + LPF_GYROX) - st.adj.gyro0.x),
		(double) (dsp_getlpf(lpf + LPF_GYROY) - st.adj.gyro0.y),
		(double) (dsp_getlpf(lpf + LPF_GYROZ) - st.adj.gyro0.z));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"temp: %0.3f\r\n",
		(double) (dsp_getlpf(lpf + LPF_IMUTEMP)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll: %0.3f; pitch: %0.3f; yaw: %0.3f\r\n",
		(double) (dsp_getcompl(cmpl + CMPL_ROLL)
			- st.adj.att0.roll),
		(double) (dsp_getcompl(cmpl + CMPL_PITCH)
			- st.adj.att0.pitch),
		(double) circf(dsp_getcompl(cmpl + CMPL_YAW)
			- st.adj.att0.yaw));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"z acceleration: %f\r\n",
		(double) dsp_getlpf(lpf + LPF_THR));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"vertical acceleration: %f\r\n",
		(double) dsp_getlpf(lpf + LPF_VAU));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel: %f\r\n", (double) sqrt(
			pow(ax - st.adj.acc0.x, 2.0) 
			+ pow(ay - st.adj.acc0.y, 2.0)
			+ pow(az - st.adj.acc0.z, 2.0)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"g offset: %f\r\n", (double) goffset);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"forward acceleration: %f\r\n",
		(double) dsp_getlpf(lpf + LPF_FA));

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
		"x = %0.3f; y = %0.3f; z = %0.3f\r\n",
		(double) qmcdata.fx, (double) qmcdata.fy,
		(double) qmcdata.fz);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"corrected: x = %0.3f; y = %0.3f; z = %0.3f\r\n",
		(double) (st.adj.magsc.x * (qmcdata.fx + st.adj.mag0.x)),
		(double) (st.adj.magsc.y * (qmcdata.fy + st.adj.mag0.y)),
		(double) (st.adj.magsc.z * (qmcdata.fz + st.adj.mag0.z)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"heading: %f\r\n", (double) dsp_getcompl(cmpl + CMPL_YAW));


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
		(double) thrust, (double) rolltarget,
		(double) pitchtarget, (double) yawtarget);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll thrust: %.3f; pitch thrust %.3f\r\n",
		(double) st.adj.mtrsc.r, (double) st.adj.mtrsc.p);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag off: %.3f; %.3f; %.3f\r\n",
		(double) st.adj.mag0.x, (double) st.adj.mag0.y,
		(double) st.adj.mag0.z);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag scale: %.3f; %.3f; %.3f\r\n",
		(double) st.adj.magsc.x, (double) st.adj.magsc.y,
		(double) st.adj.magsc.z);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag decl: %.5f\r\n",
		(double) st.adj.magdecl);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel cor: %.3f; %.3f; %.3f\r\n",
		(double) st.adj.acc0.x, (double) st.adj.acc0.y,
		(double) st.adj.acc0.z);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"gyro cor: %.3f; %.3f; %.3f\r\n",
		(double) st.adj.gyro0.x, (double) st.adj.gyro0.y,
		(double) st.adj.gyro0.z);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll cor: %.3f; pitch cor: %.3f; yaw cor: %.3f\r\n",
		(double) st.adj.att0.roll, (double) st.adj.att0.pitch,
		(double) st.adj.att0.yaw);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"battery: %0.3f\r\n",
		(double) (dsp_getlpf(lpf + LPF_BAT)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"esc current: %0.3f\r\n",
		(double) (dsp_getlpf(lpf + LPF_CUR)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"motors state: %.3f\r\n", (double) en);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"loops count: %d\r\n", loopscount);
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
		(double) st.pid.attpos.p, (double) st.pid.attpos.i,
		(double) st.pid.attpos.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"speed PID: %.5f,%.5f,%.5f\r\n",
		(double) st.pid.attrate.p, (double) st.pid.attrate.i,
		(double) st.pid.attrate.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw PID: %.5f,%.5f,%.5f\r\n",
		(double) st.pid.yawpos.p, (double) st.pid.yawpos.i,
		(double) st.pid.yawpos.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw speed PID: %.5f,%.5f,%.5f\r\n",
		(double) st.pid.yawrate.p, (double) st.pid.yawrate.i,
		(double) st.pid.yawrate.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"throttle PID: %.5f,%.5f,%.5f\r\n",
		(double) st.pid.throttle.p, (double) st.pid.throttle.i,
		(double) st.pid.throttle.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"climb rate PID: %.5f,%.5f,%.5f\r\n",
		(double) st.pid.climbrate.p,
		(double) st.pid.climbrate.i,
		(double) st.pid.climbrate.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude PID: %.5f,%.5f,%.5f\r\n",
		(double) st.pid.alt.p, (double) st.pid.alt.i,
		(double) st.pid.alt.d);

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

/**
* @brief Print all control scaling values.
* @param s output string
* @return always 0
*/
static int sprintfctrl(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum thrust: %.5f\r\n", (double) st.ctrl.thrustmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum roll: %.5f\r\n", (double) st.ctrl.rollmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum pitch: %.5f\r\n", (double) st.ctrl.pitchmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll speed: %.5f\r\n",
		(double) st.ctrl.rollrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"pitch speed: %.5f\r\n",
		(double) st.ctrl.pitchrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw speed: %.5f\r\n",
		(double) st.ctrl.yawrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw target change speed: %.5f\r\n",
		(double) st.ctrl.yawposrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum acceleration: %.5f\r\n",
		(double) st.ctrl.accelmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum climbrate: %.5f\r\n",
		(double) st.ctrl.climbratemax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum altitude: %.5f\r\n",
		(double) st.ctrl.altmax);

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
		"attitude compl tc: %.6f\r\n", (double) st.cmpl.att);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw compl tc: %.6f\r\n", (double) st.cmpl.yaw);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"climb rate compl tc: %.6f\r\n",
		(double) st.cmpl.climbrate);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude compl tc: %.6f\r\n", (double) st.cmpl.alt);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"gyro lpf cut-off: %.6f\r\n", (double) st.lpf.gyro);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel lpf cut-off: %.6f\r\n", (double) st.lpf.acc);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag lpf cut-off: %.6f\r\n", (double) st.lpf.mag);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"d lpf cut-off: %.6f\r\n", (double) st.lpf.d);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"thrust lpf tc: %.6f\r\n", (double) st.lpf.va);

	return 0;
}

int rcmd(const struct cdevice *dev, const char **toks, char *out)
{
	en = 0.0;

	return 0;
}

int ccmd(const struct cdevice *dev, const char **toks, char *out)
{
//	setstabilize(1);

	return 0;
}

int infocmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "mpu") == 0) {
		struct icm_data id;

		dev[DEV_ICM].read(dev[DEV_ICM].priv, &id,
			sizeof(struct icm_data));

		sprintpos(out, &id);
	}
	else if (strcmp(toks[1], "qmc") == 0)
		sprintqmc(out);
	else if (strcmp(toks[1], "hp") == 0) {
		struct dps_data dd;

		dev[DEV_DPS].read(dev[DEV_DPS].priv, &dd,
			sizeof(struct dps_data));

		snprintf(out, INFOLEN,
			"baro temp: %f; baro alt: %f\r\n",
			(double) dd.tempf, (double) dd.altf);

		snprintf(out + strlen(out), INFOLEN - strlen(out),
			"filtered temp: %f; filtered alt: %f\r\n",
			(double) dsp_getlpf(lpf + LPF_BARTEMP),
			(double) dsp_getlpf(lpf + LPF_ALT));

		snprintf(out + strlen(out), INFOLEN - strlen(out),
			"climb rate: %f\r\nalt: %f\r\nref alt: %f\r\n",
			(double) dsp_getcompl(cmpl + CMPL_CLIMBRATE),
			(double) dsp_getcompl(cmpl + CMPL_ALT),
			(double) alt0);
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

		dev[DEV_IRC].read(dev[DEV_IRC].priv, &data,
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
			st.pid.attpos.p = v;
		else if (strcmp(toks[2], "i") == 0)
			st.pid.attpos.i = v;
		else if (strcmp(toks[2], "d") == 0)
			st.pid.attpos.d = v;
		else
			return (-1);

		dsp_setpidbl(pid + PID_PITCHP,
			st.pid.attpos.p, st.pid.attpos.i,
			st.pid.attpos.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_ROLLP,
			st.pid.attpos.p, st.pid.attpos.i,
			st.pid.attpos.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "stilt") == 0) {
		if (strcmp(toks[2], "p") == 0)
			st.pid.attrate.p = v;
		else if (strcmp(toks[2], "i") == 0)
			st.pid.attrate.i = v;
		else if (strcmp(toks[2], "d") == 0)
			st.pid.attrate.d = v;
		else
			return (-1);

		dsp_setpidbl(pid + PID_PITCHS,
			st.pid.attrate.p, st.pid.attrate.i,
			st.pid.attrate.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_ROLLS,
			st.pid.attrate.p, st.pid.attrate.i,
			st.pid.attrate.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "yaw") == 0) {
		if (strcmp(toks[2], "p") == 0)
			st.pid.yawpos.p = v;
		else if (strcmp(toks[2], "i") == 0)
			st.pid.yawpos.i = v;
		else if (strcmp(toks[2], "d") == 0)
			st.pid.yawpos.d = v;
		else
			return (-1);

		dsp_setpid(&yawpv,
			st.pid.yawpos.p, st.pid.yawpos.i,
			st.pid.yawpos.d,
			st.lpf.d, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "syaw") == 0) {
		if (strcmp(toks[2], "p") == 0)
			st.pid.yawrate.p = v;
		else if (strcmp(toks[2], "i") == 0)
			st.pid.yawrate.i = v;
		else if (strcmp(toks[2], "d") == 0)
			st.pid.yawrate.d = v;
		else
			return (-1);

		dsp_setpidbl(pid + PID_YAWS,
			st.pid.yawrate.p, st.pid.yawrate.i,
			st.pid.yawrate.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "throttle") == 0) {
		if (strcmp(toks[2], "p") == 0)
			st.pid.throttle.p = v;
		else if (strcmp(toks[2], "i") == 0)
			st.pid.throttle.i = v;
		else if (strcmp(toks[2], "d") == 0)
			st.pid.throttle.d = v;
		else
			return (-1);

		dsp_setpidbl(pid + PID_VA,
			st.pid.throttle.p, st.pid.throttle.i,
			st.pid.throttle.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		if (strcmp(toks[2], "p") == 0)
			st.pid.climbrate.p = v;
		else if (strcmp(toks[2], "i") == 0)
			st.pid.climbrate.i = v;
		else if (strcmp(toks[2], "d") == 0)
			st.pid.climbrate.d = v;
		else
			return (-1);

		dsp_setpidbl(pid + PID_CLIMBRATE,
			st.pid.climbrate.p, st.pid.climbrate.i,
			st.pid.climbrate.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		if (strcmp(toks[2], "p") == 0)
			st.pid.alt.p = v;
		else if (strcmp(toks[2], "i") == 0)
			st.pid.alt.i = v;
		else if (strcmp(toks[2], "d") == 0)
			st.pid.alt.d = v;
		else
			return (-1);

		dsp_setpidbl(pid + PID_ALT,
			st.pid.alt.p, st.pid.alt.i, st.pid.alt.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
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
	if (strcmp(toks[1], "attitude") == 0) {
		st.cmpl.att = atof(toks[2]);
		dsp_setcompl(cmpl + CMPL_ROLL, st.cmpl.att, PID_FREQ, 0);
		dsp_setcompl(cmpl + CMPL_PITCH, st.cmpl.att, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "yaw") == 0) {
		st.cmpl.yaw = atof(toks[2]);
		dsp_setcompl(cmpl + CMPL_YAW, st.cmpl.yaw, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		st.cmpl.climbrate = atof(toks[2]);
		dsp_setcompl(cmpl + CMPL_CLIMBRATE, st.cmpl.climbrate,
			DPS_FREQ, 0);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		st.cmpl.alt = atof(toks[2]);
		dsp_setcompl(cmpl + CMPL_ALT, st.cmpl.alt, DPS_FREQ, 0);
	}

	return 0;
}

int lpfcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "gyro") == 0) {
		st.lpf.gyro = atof(toks[2]);

		dsp_setlpf1f(lpf + LPF_GYROX, st.lpf.gyro, PID_FREQ, 0);
		dsp_setlpf1f(lpf + LPF_GYROY, st.lpf.gyro, PID_FREQ, 0);
		dsp_setlpf1f(lpf + LPF_GYROZ, st.lpf.gyro, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "accel") == 0) {
		st.lpf.acc = atof(toks[2]);

		dsp_setlpf1f(lpf + LPF_ACCX, st.lpf.acc, PID_FREQ, 0);
		dsp_setlpf1f(lpf + LPF_ACCY, st.lpf.acc, PID_FREQ, 0);
		dsp_setlpf1f(lpf + LPF_ACCZ, st.lpf.acc, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "mag") == 0) {
		st.lpf.mag = atof(toks[2]);

		dsp_setlpf1f(lpf + LPF_MAGX, st.lpf.mag, PID_FREQ, 0);
		dsp_setlpf1f(lpf + LPF_MAGY, st.lpf.mag, PID_FREQ, 0);
		dsp_setlpf1f(lpf + LPF_MAGZ, st.lpf.mag, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "d") == 0) {
		st.lpf.d = atof(toks[2]);

		dsp_setpidbl(pid + PID_PITCHP,
			st.pid.attpos.p, st.pid.attpos.i,
			st.pid.attpos.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_ROLLP,
			st.pid.attpos.p, st.pid.attpos.i,
			st.pid.attpos.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_PITCHS,
			st.pid.attrate.p, st.pid.attrate.i,
			st.pid.attrate.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_ROLLS,
			st.pid.attrate.p, st.pid.attrate.i,
			st.pid.attrate.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_YAWS,
			st.pid.yawrate.p, st.pid.yawrate.i,
			st.pid.yawrate.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpid(&yawpv,
			st.pid.yawpos.p, st.pid.yawpos.i,
			st.pid.yawpos.d,
			st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_VA,
			st.pid.throttle.p, st.pid.throttle.i,
			st.pid.throttle.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_CLIMBRATE,
			st.pid.climbrate.p, st.pid.climbrate.i,
			st.pid.climbrate.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
		dsp_setpidbl(pid + PID_ALT,
			st.pid.alt.p, st.pid.alt.i, st.pid.alt.d,
			PID_MAX_I, st.lpf.d, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "vaccel") == 0) {
		st.lpf.va = atof(toks[2]);

		dsp_setlpf1t(lpf + LPF_THR, st.lpf.va, PID_FREQ, 0);
		dsp_setlpf1t(lpf + LPF_VAPT1, st.lpf.va, PID_FREQ, 0);
	}
	else
		return (-1);

	return 0;
}

int adjcmd(const struct cdevice *dev, const char **toks, char *out)
{
	float v;

	v = atof(toks[2]);

	if (strcmp(toks[1], "rollthrust") == 0)
		st.adj.mtrsc.r = v;
	else if (strcmp(toks[1], "pitchthrust") == 0)
		st.adj.mtrsc.p = v;
	else if (strcmp(toks[1], "roll") == 0)
		st.adj.att0.roll = v;
	else if (strcmp(toks[1], "pitch") == 0)
		st.adj.att0.pitch = v;
	else if (strcmp(toks[1], "yaw") == 0)
		st.adj.att0.yaw = v;
	else if (strcmp(toks[1], "acc") == 0) {
		if (strcmp(toks[2], "x") == 0)
			st.adj.acc0.x = atof(toks[3]);
		else if (strcmp(toks[2], "y") == 0)
			st.adj.acc0.y = atof(toks[3]);
		else if (strcmp(toks[2], "z") == 0)
			st.adj.acc0.z = atof(toks[3]);
		else if (strcmp(toks[2], "ztscale") == 0)
			st.adj.acctsc.z = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "gyro") == 0) {
		if (strcmp(toks[2], "x") == 0)
			st.adj.gyro0.x = atof(toks[3]);
		else if (strcmp(toks[2], "y") == 0)
			st.adj.gyro0.y = atof(toks[3]);
		else if (strcmp(toks[2], "z") == 0)
			st.adj.gyro0.z = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "mag") == 0) {
		if (strcmp(toks[2], "x0") == 0)
			st.adj.mag0.x = atof(toks[3]);
		else if (strcmp(toks[2], "y0") == 0)
			st.adj.mag0.y = atof(toks[3]);
		else if (strcmp(toks[2], "z0") == 0)
			st.adj.mag0.z = atof(toks[3]);
		else if (strcmp(toks[2], "xscale") == 0)
			st.adj.magsc.x = atof(toks[3]);
		else if (strcmp(toks[2], "yscale") == 0)
			st.adj.magsc.y = atof(toks[3]);
		else if (strcmp(toks[2], "zscale") == 0)
			st.adj.magsc.z = atof(toks[3]);
		else if (strcmp(toks[2], "xthscale") == 0)
			st.adj.magthrsc.x = atof(toks[3]);
		else if (strcmp(toks[2], "ythscale") == 0)
			st.adj.magthrsc.y = atof(toks[3]);
		else if (strcmp(toks[2], "zthscale") == 0)
			st.adj.magthrsc.z = atof(toks[3]);
		else if (strcmp(toks[2], "decl") == 0)
			st.adj.magdecl = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "curr") == 0) {
		if (strcmp(toks[2], "offset") == 0)
			st.adj.curroff = atof(toks[3]);
		else if (strcmp(toks[2], "scale") == 0)
			st.adj.cursc = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "althold") == 0) {
		if (strcmp(toks[2], "hover") == 0)
			st.adj.hoverthrottle = atof(toks[3]);
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
		en = 0.0;
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
		st.log.freq = atoi(toks[2]);

		if (st.log.freq < 0 || st.log.freq > 4096)
			st.log.freq = 128;

		modifytimev(evs + TEV_LOG, st.log.freq);

		return 0;
	}
	else if (strcmp(toks[1], "record") == 0) {
		if (strcmp(toks[2], "size") == 0) {
			unsigned int p;

			for (p = 1; p < atoi(toks[3]); p <<= 1);

			if (p > LOG_MAXRECSIZE)
				p = LOG_MAXRECSIZE;

			st.log.recsize = p;
		}
		else {
			int strn;
			int i;

			if (strcmp(toks[3], "none") == 0)
				return 0;

			if ((strn = log_fieldstrn(toks[3])) < 0)
				return (-1);

			for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
				if (st.log.fieldid[i] == atoi(toks[2]))
					st.log.fieldid[i] = 99;
			}

			st.log.fieldid[strn] = atoi(toks[2]);
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
		pointscount = atoi(toks[2]);
		return 0;
	}
	else if (strcmp(toks[1], "type") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= pointscount)
			return (-1);

		if (strcmp(toks[3], "start") == 0)
			points[idx].type = AUTOPILOT_START;
		else if (strcmp(toks[3], "takeoff") == 0)
			points[idx].type = AUTOPILOT_TAKEOFF;
		else if (strcmp(toks[3], "hover") == 0)
			points[idx].type = AUTOPILOT_HOVER;
		else if (strcmp(toks[3], "forward") == 0)
			points[idx].type = AUTOPILOT_FORWARD;
		else if (strcmp(toks[3], "landing") == 0)
			points[idx].type = AUTOPILOT_LANDING;
		else if (strcmp(toks[3], "stop") == 0)
			points[idx].type = AUTOPILOT_STOP;
		else
			return (-1);
	}
	else if (strcmp(toks[1], "alt") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= pointscount)
			return (-1);
		
		if (points[idx].type == AUTOPILOT_TAKEOFF)
			points[idx].takeoff.alt = atof(toks[3]);
		else if (points[idx].type == AUTOPILOT_HOVER)
			points[idx].hover.alt = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "t") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= pointscount)
			return (-1);
		
		if (points[idx].type == AUTOPILOT_TAKEOFF)
			points[idx].takeoff.t = atof(toks[3]);
		else if (points[idx].type == AUTOPILOT_HOVER)
			points[idx].hover.t = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "x") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= pointscount)
			return (-1);
		
		if (points[idx].type == AUTOPILOT_HOVER)
			points[idx].hover.x = atof(toks[3]);
		else if (points[idx].type == AUTOPILOT_FORWARD)
			points[idx].forward.x = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "y") == 0) {
		idx = atoi(toks[2]);

		if (idx < 0 || idx >= pointscount)
			return (-1);
		
		if (points[idx].type == AUTOPILOT_HOVER)
			points[idx].hover.y = atof(toks[3]);
		else if (points[idx].type == AUTOPILOT_FORWARD)
			points[idx].forward.y = atof(toks[3]);
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
		st.ctrl.thrustmax = v;
	else if (strcmp(toks[1], "roll") == 0)
		st.ctrl.rollmax = v;
	else if (strcmp(toks[1], "pitch") == 0)
		st.ctrl.pitchmax = v;
	else if (strcmp(toks[1], "yaw") == 0)
		st.ctrl.yawposrate = v;
	else if (strcmp(toks[1], "sroll") == 0)
		st.ctrl.rollrate = v;
	else if (strcmp(toks[1], "spitch") == 0)
		st.ctrl.pitchrate = v;
	else if (strcmp(toks[1], "syaw") == 0)
		st.ctrl.yawrate = v;
	else if (strcmp(toks[1], "accel") == 0)
		st.ctrl.accelmax = v;
	else if (strcmp(toks[1], "climbrate") == 0)
		st.ctrl.climbratemax = v;
	else if (strcmp(toks[1], "altmax") == 0)
		st.ctrl.altmax = v;
	else
		return (-1);

	return 0;
}

int systemcmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "esp") == 0) {
		if (strcmp(toks[2], "flash") == 0)
			dev[DEV_ESP].configure(dev[DEV_ESP].priv, "flash");
		else if (strcmp(toks[2], "run") == 0)
			dev[DEV_ESP].configure(dev[DEV_ESP].priv, "run");
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
		st.irc.freq = atoi(toks[2]);

		if (dev[DEV_IRC].status != DEVSTATUS_INIT)
			return 0;

		dev[DEV_IRC].configure(dev[DEV_IRC].priv, "set",
			"frequency", atoi(toks[2]));
	}
	else if (strcmp(toks[1], "power") == 0) {
		st.irc.power = atoi(toks[2]);

		if (dev[DEV_IRC].status != DEVSTATUS_INIT)
			return 0;

		dev[DEV_IRC].configure(dev[DEV_IRC].priv, "set",
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
			dshotcmd(st.mtr.lt, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.mtr.lt, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.mtr.lt, 12);
			mdelay(35);
		}
		else
			st.mtr.lt = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "lb") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.mtr.lb, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.mtr.lb, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.mtr.lb, 12);
			mdelay(35);
		}
		else
			st.mtr.lb = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "rb") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.mtr.rb, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.mtr.rb, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.mtr.rb, 12);
			mdelay(35);
		}
		else
			st.mtr.rb = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "rt") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.mtr.rt, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.mtr.rt, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.mtr.rt, 12);
			mdelay(35);
		}
		else
			st.mtr.rt = atoi(toks[2]);
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
				vf = st.pid.attpos.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = st.pid.attpos.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = st.pid.attpos.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "stilt") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = st.pid.attrate.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = st.pid.attrate.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = st.pid.attrate.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "yaw") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = st.pid.yawpos.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = st.pid.yawpos.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = st.pid.yawpos.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "syaw") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = st.pid.yawrate.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = st.pid.yawrate.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = st.pid.yawrate.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "throttle") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = st.pid.throttle.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = st.pid.throttle.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = st.pid.throttle.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "climbrate") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = st.pid.climbrate.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = st.pid.climbrate.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = st.pid.climbrate.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "altitude") == 0) {
			if (strcmp(toks[3], "p") == 0)
				vf = st.pid.alt.p;
			else if (strcmp(toks[3], "i") == 0)
				vf = st.pid.alt.i;
			else if (strcmp(toks[3], "d") == 0)
				vf = st.pid.alt.d;
			else
				return (-1);
		}
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "compl") == 0) {
		if (strcmp(toks[2], "attitude") == 0)
			vf = st.cmpl.att;
		else if (strcmp(toks[2], "yaw") == 0)
			vf = st.cmpl.yaw;
		else if (strcmp(toks[2], "climbrate") == 0)
			vf = st.cmpl.climbrate;
		else if (strcmp(toks[2], "altitude") == 0)
			vf = st.cmpl.alt;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "lpf") == 0) {
		if (strcmp(toks[2], "gyro") == 0)
			vf = st.lpf.gyro;
		else if (strcmp(toks[2], "accel") == 0)
			vf = st.lpf.acc;
		else if (strcmp(toks[2], "mag") == 0)
			vf = st.lpf.mag;
		else if (strcmp(toks[2], "d") == 0)
			vf = st.lpf.d;
		else if (strcmp(toks[2], "vaccel") == 0)
			vf = st.lpf.va;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "adj") == 0) {
		if (strcmp(toks[2], "rollthrust") == 0)
			vf = st.adj.mtrsc.r;
		else if (strcmp(toks[2], "pitchthrust") == 0)
			vf = st.adj.mtrsc.p;
		else if (strcmp(toks[2], "roll") == 0)
			vf = st.adj.att0.roll;
		else if (strcmp(toks[2], "pitch") == 0)
			vf = st.adj.att0.pitch;
		else if (strcmp(toks[2], "yaw") == 0)
			vf = st.adj.att0.yaw;
		else if (strcmp(toks[2], "acc") == 0) {
			if (strcmp(toks[3], "x") == 0)
				vf = st.adj.acc0.x;
			else if (strcmp(toks[3], "y") == 0)
				vf = st.adj.acc0.y;
			else if (strcmp(toks[3], "z") == 0)
				vf = st.adj.acc0.z;
			else if (strcmp(toks[3], "ztscale") == 0)
				vf = st.adj.acctsc.z;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "gyro") == 0) {
			if (strcmp(toks[3], "x") == 0)
				vf = st.adj.gyro0.x;
			else if (strcmp(toks[3], "y") == 0)
				vf = st.adj.gyro0.y;
			else if (strcmp(toks[3], "z") == 0)
				vf = st.adj.gyro0.z;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "mag") == 0) {
			if (strcmp(toks[3], "x0") == 0)
				vf = st.adj.mag0.x;
			else if (strcmp(toks[3], "y0") == 0)
				vf = st.adj.mag0.y;
			else if (strcmp(toks[3], "z0") == 0)
				vf = st.adj.mag0.z;
			else if (strcmp(toks[3], "xscale") == 0)
				vf = st.adj.magsc.x;
			else if (strcmp(toks[3], "yscale") == 0)
				vf = st.adj.magsc.y;
			else if (strcmp(toks[3], "zscale") == 0)
				vf = st.adj.magsc.z;
			else if (strcmp(toks[3], "xthscale") == 0)
				vf = st.adj.magsc.x;
			else if (strcmp(toks[3], "ythscale") == 0)
				vf = st.adj.magsc.y;
			else if (strcmp(toks[3], "zthscale") == 0)
				vf = st.adj.magsc.z;
			else if (strcmp(toks[3], "decl") == 0)
				vf = st.adj.magdecl;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "curr") == 0) {
			if (strcmp(toks[3], "offset") == 0)
				vf = st.adj.curroff;
			else if (strcmp(toks[3], "scale") == 0)
				vf = st.adj.cursc;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "althold") == 0) {
			if (strcmp(toks[3], "hover") == 0)
				vf = st.adj.hoverthrottle;
			else
				return (-1);
		}
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "ctrl") == 0) {
		if (strcmp(toks[2], "thrust") == 0)
			vf = st.ctrl.thrustmax;
		else if (strcmp(toks[2], "roll") == 0)
			vf = st.ctrl.rollmax;
		else if (strcmp(toks[2], "pitch") == 0)
			vf = st.ctrl.pitchmax;
		else if (strcmp(toks[2], "yaw") == 0)
			vf = st.ctrl.yawposrate;
		else if (strcmp(toks[2], "sroll") == 0)
			vf = st.ctrl.rollrate;
		else if (strcmp(toks[2], "spitch") == 0)
			vf = st.ctrl.pitchrate;
		else if (strcmp(toks[2], "syaw") == 0)
			vf = st.ctrl.yawrate;
		else if (strcmp(toks[2], "accel") == 0)
			vf = st.ctrl.accelmax;
		else if (strcmp(toks[2], "climbrate") == 0)
			vf = st.ctrl.climbratemax;
		else if (strcmp(toks[2], "altmax") == 0)
			vf = st.ctrl.altmax;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "autopilot") == 0) {
		int idx;

		if (strcmp(toks[2], "count") == 0) {
			vi = pointscount;
			valtype = CONFVALTYPE_INT;
		}
		else if (strcmp(toks[2], "type") == 0) {
			idx = atoi(toks[3]);
		
			if (idx < 0 || idx >= pointscount)
				return (-1);
		
			if (points[idx].type == AUTOPILOT_START)
				vs = "start";
			else if (points[idx].type == AUTOPILOT_TAKEOFF)
				vs = "takeoff";
			else if (points[idx].type == AUTOPILOT_HOVER)
				vs = "hover";
			else if (points[idx].type == AUTOPILOT_FORWARD)
				vs = "forward";
			else if (points[idx].type == AUTOPILOT_LANDING)
				vs = "landing";
			else if (points[idx].type == AUTOPILOT_STOP)
				vs = "stop";
			else
				return (-1);

			valtype = CONFVALTYPE_STRING;
		}
		else if (strcmp(toks[2], "alt") == 0) {
			idx = atoi(toks[3]);
			
			if (idx < 0 || idx >= pointscount)
				return (-1);
			
			if (points[idx].type == AUTOPILOT_TAKEOFF)
				vf = points[idx].takeoff.alt;
			else if (points[idx].type == AUTOPILOT_HOVER)
				vf = points[idx].hover.alt;
			else
				return (-1);
			
			valtype = CONFVALTYPE_FLOAT;
		}
		else if (strcmp(toks[2], "t") == 0) {
			idx = atoi(toks[3]);
			
			if (idx < 0 || idx >= pointscount)
				return (-1);
			
			if (points[idx].type == AUTOPILOT_TAKEOFF)
				vf = points[idx].takeoff.t;
			else if (points[idx].type == AUTOPILOT_HOVER)
				vf = points[idx].hover.t;
			else
				return (-1);
			
			valtype = CONFVALTYPE_FLOAT;
		}
		else if (strcmp(toks[2], "x") == 0) {
			idx = atoi(toks[3]);
				
			if (idx < 0 || idx >= pointscount)
				return (-1);
		
			if (points[idx].type == AUTOPILOT_HOVER)
				vf = points[idx].hover.x;
			else if (points[idx].type == AUTOPILOT_FORWARD)
				vf = points[idx].forward.x;
			else
				return (-1);
			
			valtype = CONFVALTYPE_FLOAT;
		}
		else if (strcmp(toks[2], "y") == 0) {
			idx = atoi(toks[3]);
			
			if (idx < 0 || idx >= pointscount)
				return (-1);
			
			if (points[idx].type == AUTOPILOT_HOVER)
				vf = points[idx].hover.y;
			else if (points[idx].type == AUTOPILOT_FORWARD)
				vf = points[idx].forward.y;
			else
				return (-1);
			
			valtype = CONFVALTYPE_FLOAT;
		}
		else
			return (-1);
	}
	else if (strcmp(toks[1], "irc") == 0) {
		if (strcmp(toks[2], "frequency") == 0)
			vi = st.irc.freq;
		else if (strcmp(toks[2], "power") == 0)
			vi = st.irc.power;
		else
			return (-1);

		valtype = CONFVALTYPE_INT;
	}
	else if (strcmp(toks[1], "log") == 0) {
		if (strcmp(toks[2], "freq") == 0) {
			vi = st.log.freq;
			valtype = CONFVALTYPE_INT;
		}
		else if (strcmp(toks[2], "record") == 0) {
			if (strcmp(toks[3], "size") == 0) {
				vi = st.log.recsize;
				valtype = CONFVALTYPE_INT;
			}
			else {
				int recn;
				int i;

				recn = atoi(toks[3]);

				if (recn < 0 || recn > LOG_MAXRECSIZE)
					return (-1);

				for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
					if (st.log.fieldid[i] == recn)
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
			vi = st.mtr.lt;
		else if (strcmp(toks[2], "lb") == 0)
			vi = st.mtr.lb;
		else if (strcmp(toks[2], "rb") == 0)
			vi = st.mtr.rb;
		else if (strcmp(toks[2], "rt") == 0)
			vi = st.mtr.rt;

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
