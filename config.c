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
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n", "accel: ",
		(double) dsp_getlpf(&accxpt1),
		(double) dsp_getlpf(&accypt1),
		(double) dsp_getlpf(&acczpt1));	

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n", "gyro: ",
		(double) dsp_getlpf(&gyroxpt1),
		(double) dsp_getlpf(&gyroypt1),
		(double) dsp_getlpf(&gyrozpt1));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"accel corrected: ",
		(double) (dsp_getlpf(&accxpt1) - st.ax0),
		(double) (dsp_getlpf(&accypt1) - st.ay0),
		(double) (dsp_getlpf(&acczpt1) - st.az0));
	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"%-7sx = %0.3f; y = %0.3f; z = %0.3f\r\n",
		"gyro corrected: ",
		(double) (dsp_getlpf(&gyroxpt1) - st.gx0),
		(double) (dsp_getlpf(&gyroypt1) - st.gy0),
		(double) (dsp_getlpf(&gyrozpt1) - st.gz0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"temp: %0.3f\r\n", (double) (dsp_getlpf(&atemppt1)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll: %0.3f; pitch: %0.3f; yaw: %0.3f\r\n",
		(double) (dsp_getcompl(&rollcompl) - st.roll0),
		(double) (dsp_getcompl(&pitchcompl) - st.pitch0),
		(double) circf(dsp_getcompl(&yawcompl) - st.yaw0));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"z acceleration: %f\r\n",
		(double) dsp_getlpf(&tlpf));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"vertical acceleration: %f\r\n",
		(double) dsp_getlpf(&valpf));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel: %f\r\n", (double) sqrt(
			pow(dsp_getlpf(&accxpt1) - st.ax0, 2.0) 
			+ pow(dsp_getlpf(&accypt1) - st.ay0, 2.0)
			+ pow(dsp_getlpf(&acczpt1) - st.az0, 2.0)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"g offset: %f\r\n", (double) goffset);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"forward acceleration: %f\r\n",
		(double) dsp_getlpf(&flpf));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"forward speed: %f\r\n", (double) forwardspeed);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"forward path: %f\r\n", (double) forwardpath);


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
		(double) (st.mxsc * (qmcdata.fx + st.mx0)),
		(double) (st.mysc * (qmcdata.fy + st.my0)),
		(double) (st.mzsc * (qmcdata.fz + st.mz0)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"heading: %f\r\n", (double) dsp_getcompl(&yawcompl));


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
		"battery: %0.3f\r\n",
		(double) (dsp_getlpf(&batlpf)));

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"esc current: %0.3f\r\n",
		(double) (dsp_getlpf(&currlpf)));

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
		"maximum thrust: %.5f\r\n", (double) st.thrustmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum roll: %.5f\r\n", (double) st.rollmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"maximum pitch: %.5f\r\n", (double) st.pitchmax);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"roll speed: %.5f\r\n",
		(double) st.rollspeed);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"pitch speed: %.5f\r\n",
		(double) st.pitchspeed);

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

/**
* @brief Print filters coefficients into a string.
* @param s output string
* @return always 0
*/
static int sprintffilters(char *s)
{
	s[0] = '\0';

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"attitude compl tc: %.6f\r\n", (double) st.atctcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"yaw compl tc: %.6f\r\n", (double) st.yctcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"gyro lpf cut-off: %.6f\r\n", (double) st.gyropt1freq);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"accel lpf cut-off: %.6f\r\n", (double) st.accpt1freq);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"mag lpf cut-off: %.6f\r\n", (double) st.magpt1freq);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"d lpf cut-off: %.6f\r\n", (double) st.dpt1freq);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"thrust lpf tc: %.6f\r\n", (double) st.ttcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"vertical accel lpf tc: %.6f\r\n", (double) st.vatcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"climb rate compl tc: %.6f\r\n", (double) st.cctcoef);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude lpf cut-off: %.6f\r\n", (double) st.apt1freq);

	snprintf(s + strlen(s), INFOLEN - strlen(s),
		"altitude compl tc: %.6f\r\n", (double) st.actcoef);

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

		dev[ICM_DEV].read(dev[ICM_DEV].priv, &id,
			sizeof(struct icm_data));

		sprintpos(out, &id);
	}
	else if (strcmp(toks[1], "qmc") == 0)
		sprintqmc(out);
	else if (strcmp(toks[1], "hp") == 0) {
		struct dps_data dd;

		dev[DPS_DEV].read(dev[DPS_DEV].priv, &dd,
			sizeof(struct dps_data));

		snprintf(out, INFOLEN,
			"baro temp: %f; baro alt: %f\r\n",
			(double) dd.tempf, (double) dd.altf);

		snprintf(out + strlen(out), INFOLEN - strlen(out),
			"filtered temp: %f; filtered alt: %f\r\n",
			(double) dsp_getlpf(&templpf),
			(double) dsp_getlpf(&altlpf));

		snprintf(out + strlen(out), INFOLEN - strlen(out),
			"climb rate: %f\r\nalt: %f\r\nref alt: %f\r\n",
			(double) dsp_getcompl(&climbratecompl),
			(double) dsp_getcompl(&altcompl),
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

		dev[IRC_DEV].read(dev[IRC_DEV].priv, &data,
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
		if (strcmp(toks[2], "mode") == 0) {
			if (strcmp(toks[3], "double") == 0)
				speedpid = 0;
			else if (strcmp(toks[3], "single") == 0)
				speedpid = 1;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "p") == 0)	st.p = v;
		else if (strcmp(toks[2], "i") == 0)	st.i = v;
		else if (strcmp(toks[2], "d") == 0)	st.d = v;
		else					return (-1);

		dsp_setpidbl(&pitchpv, st.p, st.i, st.d,
			0.5, st.dpt1freq, PID_FREQ, 0);
		dsp_setpidbl(&rollpv, st.p, st.i, st.d,
			0.5, st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "stilt") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.sp = v;
		else if (strcmp(toks[2], "i") == 0)	st.si = v;
		else if (strcmp(toks[2], "d") == 0)	st.sd = v;
		else					return (-1);

		dsp_setpidbl(&pitchspv, st.sp, st.si, st.sd,
			0.5, st.dpt1freq, PID_FREQ, 0);
		dsp_setpidbl(&rollspv, st.sp, st.si, st.sd,
			0.5, st.dpt1freq, PID_FREQ, 0);
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

		dsp_setpid(&yawpv, st.yp, st.yi, st.yd,
			st.dpt1freq, PID_FREQ, 0);

	}
	else if (strcmp(toks[1], "syaw") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.ysp = v;
		else if (strcmp(toks[2], "i") == 0)	st.ysi = v;
		else if (strcmp(toks[2], "d") == 0)	st.ysd = v;
		else					return (-1);

		dsp_setpidbl(&yawspv, st.ysp, st.ysi, st.ysd,
			0.5, st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "throttle") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.zsp = v;
		else if (strcmp(toks[2], "i") == 0)	st.zsi = v;
		else if (strcmp(toks[2], "d") == 0)	st.zsd = v;
		else					return (-1);

		dsp_setpidbl(&tpv, st.zsp, st.zsi, st.zsd,
			0.5, st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.cp = v;
		else if (strcmp(toks[2], "i") == 0)	st.ci = v;
		else if (strcmp(toks[2], "d") == 0)	st.cd = v;
		else					return (-1);

		dsp_setpidbl(&cpv, st.cp, st.ci, st.cd,
			0.5, st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		if (strcmp(toks[2], "p") == 0)		st.ap = v;
		else if (strcmp(toks[2], "i") == 0)	st.ai = v;
		else if (strcmp(toks[2], "d") == 0)	st.ad = v;
		else					return (-1);

		dsp_setpidbl(&apv, st.ap, st.ai, st.ad,
			0.5, st.dpt1freq, PID_FREQ, 0);
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
		st.atctcoef = atof(toks[2]);
		dsp_setcompl(&rollcompl, st.atctcoef, PID_FREQ, 0);
		dsp_setcompl(&pitchcompl, st.atctcoef, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "yaw") == 0) {
		st.yctcoef = atof(toks[2]);
		dsp_setcompl(&yawcompl, st.yctcoef, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "climbrate") == 0) {
		st.cctcoef = atof(toks[2]);
		dsp_setcompl(&climbratecompl, st.cctcoef, DPS_FREQ, 0);
	}
	else if (strcmp(toks[1], "altitude") == 0) {
		st.actcoef = atof(toks[2]);
		dsp_setcompl(&altcompl, st.actcoef, DPS_FREQ, 0);
	}

	return 0;
}

int lpfcmd(const struct cdevice *dev, const char **toks, char *out)
{
	if (strcmp(toks[1], "gyro") == 0) {
		st.gyropt1freq = atof(toks[2]);

		dsp_setlpf1f(&gyroxpt1, st.gyropt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&gyroypt1, st.gyropt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "accel") == 0) {
		st.accpt1freq = atof(toks[2]);

		dsp_setlpf1f(&accxpt1, st.accpt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&accypt1, st.accpt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&accypt1, st.accpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "mag") == 0) {
		st.magpt1freq = atof(toks[2]);

		dsp_setlpf1f(&magxpt1, st.magpt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&magypt1, st.magpt1freq, PID_FREQ, 0);
		dsp_setlpf1f(&magypt1, st.magpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "d") == 0) {
		st.dpt1freq = atof(toks[2]);

		dsp_setpidbl(&pitchpv, st.p, st.i, st.d,
			0.5, st.dpt1freq, PID_FREQ, 0);
		dsp_setpidbl(&rollpv, st.p, st.i, st.d,
			0.5, st.dpt1freq, PID_FREQ, 0);
		dsp_setpidbl(&pitchspv, st.sp, st.si, st.sd,
			0.5, st.dpt1freq, PID_FREQ, 0);
		dsp_setpidbl(&rollspv, st.sp, st.si, st.sd,
			0.5, st.dpt1freq, PID_FREQ, 0);
		dsp_setpidbl(&yawspv, st.ysp, st.ysi, st.ysd,
			0.5, st.dpt1freq, PID_FREQ, 0);
		dsp_setpid(&yawpv, st.yp, st.yi, st.yd,
			st.dpt1freq, PID_FREQ, 0);
		dsp_setpidbl(&tpv, st.zsp, st.zsi, st.zsd,
			0.5, st.dpt1freq, PID_FREQ, 0);
		dsp_setpidbl(&cpv, st.cp, st.ci, st.cd,
			0.5, st.dpt1freq, PID_FREQ, 0);
	}
	else if (strcmp(toks[1], "vaccel") == 0) {
		st.ttcoef = atof(toks[2]);

		dsp_setlpf1t(&tlpf, st.ttcoef, PID_FREQ, 0);
		dsp_setlpf1t(&vtlpf, st.ttcoef, PID_FREQ, 0);
	}
	else
		return (-1);

	return 0;
}

int adjcmd(const struct cdevice *dev, const char **toks, char *out)
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
		else if (strcmp(toks[2], "ztscale") == 0)
			st.aztscale = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "gyro") == 0) {
		if (strcmp(toks[2], "x") == 0)
			st.gx0 = atof(toks[3]);
		else if (strcmp(toks[2], "y") == 0)
			st.gy0 = atof(toks[3]);
		else if (strcmp(toks[2], "z") == 0)
			st.gz0 = atof(toks[3]);
		else
			return (-1);
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
		else if (strcmp(toks[2], "xthscale") == 0)
			st.mxthsc = atof(toks[3]);
		else if (strcmp(toks[2], "ythscale") == 0)
			st.mythsc = atof(toks[3]);
		else if (strcmp(toks[2], "zthscale") == 0)
			st.mzthsc = atof(toks[3]);
		else if (strcmp(toks[2], "decl") == 0)
			st.magdecl = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "curr") == 0) {
		if (strcmp(toks[2], "offset") == 0)
			st.curroffset = atof(toks[3]);
		else if (strcmp(toks[2], "scale") == 0)
			st.currscale = atof(toks[3]);
		else
			return (-1);
	}
	else if (strcmp(toks[1], "althold") == 0) {
		if (strcmp(toks[2], "hover") == 0)
			st.hoverthrottle = atof(toks[3]);
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
		st.logfreq = atoi(toks[2]);

		if (st.logfreq < 0 || st.logfreq > 4096)
			st.logfreq = 128;

		modifytimev(evs + TEV_LOG, st.logfreq);

		return 0;
	}
	else if (strcmp(toks[1], "record") == 0) {
		if (strcmp(toks[2], "size") == 0) {
			unsigned int p;

			for (p = 1; p < atoi(toks[3]); p <<= 1);

			if (p > LOG_MAXRECSIZE)
				p = LOG_MAXRECSIZE;

			st.logrecsize = p;
		}
		else {
			int strn;
			int i;

			if (strcmp(toks[3], "none") == 0)
				return 0;

			if ((strn = log_fieldstrn(toks[3])) < 0)
				return (-1);

			for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
				if (st.fieldid[i] == atoi(toks[2]))
					st.fieldid[i] = 99;
			}

			st.fieldid[strn] = atoi(toks[2]);
		}

		return 0;
	}
	else
		return (-1);

	return 1;
}


int ctrlcmd(const struct cdevice *d, const char **toks, char *out)
{
	float v;

	v = atof(toks[2]);

	if (strcmp(toks[1], "thrust") == 0)
		st.thrustmax = v;
	else if (strcmp(toks[1], "roll") == 0)
		st.rollmax = v;
	else if (strcmp(toks[1], "pitch") == 0)
		st.pitchmax = v;
	else if (strcmp(toks[1], "yaw") == 0)
		st.yawtargetspeed = v;
	else if (strcmp(toks[1], "sroll") == 0)
		st.rollspeed = v;
	else if (strcmp(toks[1], "spitch") == 0)
		st.pitchspeed = v;
	else if (strcmp(toks[1], "syaw") == 0)
		st.yawspeed = v;
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

int systemcmd(const struct cdevice *d, const char **toks, char *out)
{
	if (strcmp(toks[1], "esp") == 0) {
		if (strcmp(toks[2], "flash") == 0)
			dev[ESP_DEV].configure(dev[ESP_DEV].priv, "flash");
		else if (strcmp(toks[2], "run") == 0)
			dev[ESP_DEV].configure(dev[ESP_DEV].priv, "run");
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
		st.ircfreq = atoi(toks[2]);

		if (dev[IRC_DEV].status != DEVSTATUS_INIT)
			return 0;

		dev[IRC_DEV].configure(dev[IRC_DEV].priv, "set",
			"frequency", atoi(toks[2]));
	}
	else if (strcmp(toks[1], "power") == 0) {
		st.ircpower = atoi(toks[2]);

		if (dev[IRC_DEV].status != DEVSTATUS_INIT)
			return 0;

		dev[IRC_DEV].configure(dev[IRC_DEV].priv, "set",
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
			dshotcmd(st.lt, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.lt, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.lt, 12);
			mdelay(35);
		}
		else
			st.lt = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "lb") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.lb, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.lb, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.lb, 12);
			mdelay(35);
		}
		else
			st.lb = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "rb") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.rb, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.rb, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.rb, 12);
			mdelay(35);
		}
		else
			st.rb = atoi(toks[2]);
	}
	else if (strcmp(toks[1], "rt") == 0) {
		if (strcmp(toks[2], "r") == 0)
			dshotcmd(st.rt, 21);
		else if (strcmp(toks[2], "d") == 0)
			dshotcmd(st.rt, 20);
		else if (strcmp(toks[2], "s") == 0) {
			dshotcmd(st.rt, 12);
			mdelay(35);
		}
		else
			st.rt = atoi(toks[2]);
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
	float v;
	int vi;
	const char *si;

	valtype = CONFVALTYPE_FLOAT;

	if (strcmp(toks[1], "pid") == 0) {
		if (strcmp(toks[2], "tilt") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.p;
			else if (strcmp(toks[3], "i") == 0)
				v = st.i;
			else if (strcmp(toks[3], "d") == 0)
				v = st.d;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "stilt") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.sp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.si;
			else if (strcmp(toks[3], "d") == 0)
				v = st.sd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "yaw") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.yp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.yi;
			else if (strcmp(toks[3], "d") == 0)
				v = st.yd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "syaw") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.ysp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.ysi;
			else if (strcmp(toks[3], "d") == 0)
				v = st.ysd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "throttle") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.zsp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.zsi;
			else if (strcmp(toks[3], "d") == 0)
				v = st.zsd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "climbrate") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.cp;
			else if (strcmp(toks[3], "i") == 0)
				v = st.ci;
			else if (strcmp(toks[3], "d") == 0)
				v = st.cd;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "altitude") == 0) {
			if (strcmp(toks[3], "p") == 0)
				v = st.ap;
			else if (strcmp(toks[3], "i") == 0)
				v = st.ai;
			else if (strcmp(toks[3], "d") == 0)
				v = st.ad;
			else
				return (-1);
		}
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "compl") == 0) {
		if (strcmp(toks[2], "attitude") == 0)
			v = st.atctcoef;
		else if (strcmp(toks[2], "yaw") == 0)
			v = st.yctcoef;
		else if (strcmp(toks[2], "climbrate") == 0)
			v = st.cctcoef;
		else if (strcmp(toks[2], "altitude") == 0)
			v = st.actcoef;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "lpf") == 0) {
		if (strcmp(toks[2], "gyro") == 0)
			v = st.gyropt1freq;
		else if (strcmp(toks[2], "accel") == 0)
			v = st.accpt1freq;
		else if (strcmp(toks[2], "mag") == 0)
			v = st.magpt1freq;
		else if (strcmp(toks[2], "d") == 0)
			v = st.dpt1freq;
		else if (strcmp(toks[2], "vaccel") == 0)
			v = st.ttcoef;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "adj") == 0) {
		if (strcmp(toks[2], "rollthrust") == 0)
			v = st.rsc;
		else if (strcmp(toks[2], "pitchthrust") == 0)
			v = st.psc;
		else if (strcmp(toks[2], "roll") == 0)
			v = st.roll0;
		else if (strcmp(toks[2], "pitch") == 0)
			v = st.pitch0;
		else if (strcmp(toks[2], "yaw") == 0)
			v = st.yaw0;
		else if (strcmp(toks[2], "acc") == 0) {
			if (strcmp(toks[3], "x") == 0)
				v = st.ax0;
			else if (strcmp(toks[3], "y") == 0)
				v = st.ay0;
			else if (strcmp(toks[3], "z") == 0)
				v = st.az0;
			else if (strcmp(toks[3], "ztscale") == 0)
				v = st.aztscale;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "gyro") == 0) {
			if (strcmp(toks[3], "x") == 0)
				v = st.gx0;
			else if (strcmp(toks[3], "y") == 0)
				v = st.gy0;
			else if (strcmp(toks[3], "z") == 0)
				v = st.gz0;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "mag") == 0) {
			if (strcmp(toks[3], "x0") == 0)
				v = st.mx0;
			else if (strcmp(toks[3], "y0") == 0)
				v = st.my0;
			else if (strcmp(toks[3], "z0") == 0)
				v = st.mz0;
			else if (strcmp(toks[3], "xscale") == 0)
				v = st.mxsc;
			else if (strcmp(toks[3], "yscale") == 0)
				v = st.mysc;
			else if (strcmp(toks[3], "zscale") == 0)
				v = st.mzsc;
			else if (strcmp(toks[3], "xthscale") == 0)
				v = st.mxthsc;
			else if (strcmp(toks[3], "ythscale") == 0)
				v = st.mythsc;
			else if (strcmp(toks[3], "zthscale") == 0)
				v = st.mzthsc;
			else if (strcmp(toks[3], "decl") == 0)
				v = st.magdecl;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "curr") == 0) {
			if (strcmp(toks[3], "offset") == 0)
				v = st.curroffset;
			else if (strcmp(toks[3], "scale") == 0)
				v = st.currscale;
			else
				return (-1);
		}
		else if (strcmp(toks[2], "althold") == 0) {
			if (strcmp(toks[3], "hover") == 0)
				v = st.hoverthrottle;
			else
				return (-1);
		}
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "ctrl") == 0) {
		if (strcmp(toks[2], "thrust") == 0)
			v = st.thrustmax;
		else if (strcmp(toks[2], "roll") == 0)
			v = st.rollmax;
		else if (strcmp(toks[2], "pitch") == 0)
			v = st.pitchmax;
		else if (strcmp(toks[2], "yaw") == 0)
			v = st.yawtargetspeed;
		else if (strcmp(toks[2], "sroll") == 0)
			v = st.rollspeed;
		else if (strcmp(toks[2], "spitch") == 0)
			v = st.pitchspeed;
		else if (strcmp(toks[2], "syaw") == 0)
			v = st.yawspeed;
		else if (strcmp(toks[2], "accel") == 0)
			v = st.accelmax;
		else if (strcmp(toks[2], "climbrate") == 0)
			v = st.climbratemax;
		else if (strcmp(toks[2], "altmax") == 0)
			v = st.altmax;
		else
			return (-1);

		valtype = CONFVALTYPE_FLOAT;
	}
	else if (strcmp(toks[1], "irc") == 0) {
		if (strcmp(toks[2], "frequency") == 0)
			vi = st.ircfreq;
		else if (strcmp(toks[2], "power") == 0)
			vi = st.ircpower;
		else
			return (-1);

		valtype = CONFVALTYPE_INT;
	}
	else if (strcmp(toks[1], "log") == 0) {
		if (strcmp(toks[2], "freq") == 0) {
			vi = st.logfreq;
			valtype = CONFVALTYPE_INT;
		}
		else if (strcmp(toks[2], "record") == 0) {
			if (strcmp(toks[3], "size") == 0) {
				vi = st.logrecsize;
				valtype = CONFVALTYPE_INT;
			}
			else {
				int recn;
				int i;

				recn = atoi(toks[3]);

				if (recn < 0 || recn > LOG_MAXRECSIZE)
					return (-1);

				for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
					if (st.fieldid[i] == recn)
						break;
				}

				if (i >= LOG_FIELDSTRSIZE)
					si = "none";
				else
					si = logfieldstr[i];

				valtype = CONFVALTYPE_STRING;
			}
		}
		else
			return (-1);
	}	
	else if (strcmp(toks[1], "motor") == 0) {
		if (strcmp(toks[2], "lt") == 0)
			vi = st.lt;
		else if (strcmp(toks[2], "lb") == 0)
			vi = st.lb;
		else if (strcmp(toks[2], "rb") == 0)
			vi = st.rb;
		else if (strcmp(toks[2], "rt") == 0)
			vi = st.rt;

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
		sprintf(data + strlen(data), "%f\r\n", (double) v);
		break;

	case CONFVALTYPE_INT:
		sprintf(data + strlen(data), "%d\r\n", vi);
		break;

	case CONFVALTYPE_STRING:
		sprintf(data + strlen(data), "%s\r\n", si);
		break;
	}

	sprintf(out, "%05u", crc16((uint8_t *) data, strlen(data)));
	out[5] = ' ';

	return 1;
}
