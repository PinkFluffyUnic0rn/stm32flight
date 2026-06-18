#include <math.h>

#include "controlloop.h"
#include "runvals.h"
#include "settings.h"
#include "periphconf.h"

#include "device.h"
#include "icm42688.h"
#include "dps368.h"
#include "esp8266.h"
#include "qmc5883l.h"
#include "crsf.h"
#include "w25.h"
#include "m10.h"
#include "uartconf.h"
#include "irc.h"
#include "msp.h"
#include "dshot.h"

/**
* @brief Calculate tilt compensated heading direction
	using magnetometer readings, roll value and pitch value.
* @param r roll value
* @param p pitch value
* @param x magnetometer's X axis value
* @param y magnetometer's Y axis value
* @param z magnetometer's Z axis value
* @return tilt compensated heading
*/
static float heading(float r, float p, float x, float y, float z)
{
	x = x * cosf(p) + y * sinf(r) * sinf(p)
		+ z * cosf(r) * sinf(p);
	y = y * cosf(r) - z * sinf(r);

	return circf(atan2f(y, x) + St.adj.magdecl);
}

int setstabilize(int init)
{
	float iscale;
	
	// init complementary filters contexts
	dsp_setcompl(Cmpl + CMPL_PITCH, St.cmpl.att, PID_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_ROLL, St.cmpl.att, PID_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_YAW, St.cmpl.yaw, PID_FREQ, init);

	dsp_setcompl(Cmpl + CMPL_CLIMBRATE, St.cmpl.climbrate,
		PID_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_ALT, St.cmpl.alt, PID_FREQ, init);

	dsp_setcompl(Cmpl + CMPL_SLAT, St.cmpl.speed, PID_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_SLON, St.cmpl.speed, PID_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_LAT, St.cmpl.pos, PID_FREQ, init);
	dsp_setcompl(Cmpl + CMPL_LON, St.cmpl.pos, PID_FREQ, init);

	// init roll and pitch position PID controller contexts
	dsp_setpidbl(Pid + PID_PITCHP,
		St.pid.attpos.p, St.pid.attpos.i, St.pid.attpos.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);
	dsp_setpidbl(Pid + PID_ROLLP,
		St.pid.attpos.p, St.pid.attpos.i, St.pid.attpos.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	// init roll, pitch and yaw speed PID controller contexts
	iscale = Speedpid ? 1.0 : St.pid.feature.iscale;
	
	dsp_setpidbl(Pid + PID_PITCHS,
		St.pid.attrate.p,
		St.pid.attrate.i * iscale,
		St.pid.attrate.d,
		St.pid.feature.attimax, St.lpf.d, 0, PID_FREQ, init);
	dsp_setpidbl(Pid + PID_ROLLS, 
		St.pid.attrate.p,
		St.pid.attrate.i * iscale,
		St.pid.attrate.d,
		St.pid.feature.attimax, St.lpf.d, 0, PID_FREQ, init);
	dsp_setpidbl(Pid + PID_YAWS,
		St.pid.yawrate.p,
		St.pid.yawrate.i * iscale,
		St.pid.yawrate.d,
		St.pid.feature.attimax, St.lpf.d, 0, PID_FREQ, init);

	// init yaw position PID controller's context
	dsp_setpidbl(Pid + PID_YAWP,
		St.pid.yawpos.p, St.pid.yawpos.i, St.pid.yawpos.d,
		PID_MAX_I, St.lpf.d, 1, PID_FREQ, init);

	// init vertical acceleration PID controller's context
	dsp_setpidbl(Pid + PID_VA,
		St.pid.throttle.p, St.pid.throttle.i, St.pid.throttle.d,
		St.pid.feature.thrimax, St.lpf.d, 0, PID_FREQ, init);

	// init climbrate PID controller's context
	dsp_setpidbl(Pid + PID_CLIMBRATE,
		St.pid.climbrate.p, St.pid.climbrate.i,
		St.pid.climbrate.d, PID_MAX_I, St.lpf.d, 0, PID_FREQ,
		init);

	// init altitude PID controller's context
	dsp_setpidbl(Pid + PID_ALT,
		St.pid.alt.p, St.pid.alt.i, St.pid.alt.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	// init speed through latitude and longitude 
	// PID controllers contexts
	dsp_setpidbl(Pid + PID_SLAT,
		St.pid.speed.p, St.pid.speed.i, St.pid.speed.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	dsp_setpidbl(Pid + PID_SLON,
		St.pid.speed.p, St.pid.speed.i, St.pid.speed.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	// init latitude and longitude PID controllers contexts
	dsp_setpidbl(Pid + PID_LAT,
		St.pid.pos.p, St.pid.pos.i, St.pid.pos.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	dsp_setpidbl(Pid + PID_LON,
		St.pid.pos.p, St.pid.pos.i, St.pid.pos.d,
		PID_MAX_I, St.lpf.d, 0, PID_FREQ, init);

	// init battery voltage low-pass filter
	dsp_setlpf1f(Lpf + LPF_BAT, BAT_CUTOFF, POWER_FREQ, init);
	dsp_setlpf1f(Lpf + LPF_CUR, CUR_CUTOFF, POWER_FREQ, init);

	// init average thrust low-pass filter
	dsp_setunity(Lpf + LPF_AVGTHR, init);
	dsp_setlpf1t(Lpf + LPF_AVGTHRA, St.lpf.va, PID_FREQ, init);

	// init low-pass fitlers for altitude, accelerations and speed
	dsp_setunity(Lpf + LPF_BARTEMP, init);
	dsp_setlpf1t(Lpf + LPF_THR, St.lpf.va, PID_FREQ, init);
	dsp_setunity(Lpf + LPF_VAU, init);
	dsp_setlpf1t(Lpf + LPF_VAPT1, St.lpf.va, PID_FREQ, init);
	dsp_setlpf1t(Lpf + LPF_VAAVG, VA_AVG_TCOEF, PID_FREQ, init);
	dsp_setunity(Lpf + LPF_FA, init);
	dsp_setunity(Lpf + LPF_SA, init);
	dsp_setunity(Lpf + LPF_ALT, init);
	dsp_setunity(Lpf + LPF_SPEED, init);
	dsp_setunity(Lpf + LPF_LATM, init);
	dsp_setunity(Lpf + LPF_LONM, init);

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

int updateposition(float dt)
{
	static float prevalt = 0.0;
	float roll, pitch, yaw;
	float vx, vy, vz;
	float gvx, gvy, gvz;
	float gy, gx, gz;
	float ay, ax, az;
	float va;
	float alt;
	float altcor;

	// apply accelerometer offsets
	ax = dsp_getlpf(Lpf + LPF_ACCX);
	ay = dsp_getlpf(Lpf + LPF_ACCY);
	az = dsp_getlpf(Lpf + LPF_ACCZ);

	// update vertical acceleration low-pass filter
	dsp_updatelpf(Lpf + LPF_THR, Imudata.afz);

	// convert gyroscope values into radians
	gx = deg2rad(dsp_getlpf(Lpf + LPF_GYROX));
	gy = deg2rad(dsp_getlpf(Lpf + LPF_GYROY));
	gz = deg2rad(dsp_getlpf(Lpf + LPF_GYROZ));

	// update complimenraty filter for roll axis and get next roll
	// value. First signal (value) is signal to be integrated: it's
	// the speed of the rotation around Y axis. Second signal is
	// signal to be low-pass filtered: it's the tilt value that is
	// calculated from acceleromer readings through some
	// trigonometry.
	roll = dsp_updatelpf(Lpf + LPF_ROLL,
		dsp_updatecompl(Cmpl + CMPL_ROLL, gy * dt,
			atan2f(-ax, az) - St.adj.att0.roll));

	// same as for roll but for different axes
	pitch = dsp_updatelpf(Lpf + LPF_PITCH,
		dsp_updatecompl(Cmpl + CMPL_PITCH, gx * dt,
			atan2f(ay, sqrt(ax * ax + az * az))) 
				- St.adj.att0.pitch);

	// update complimenraty filter for yaw axis and get next yaw
	// value. First signal is the speed of the rotation around Z
	// axis. Second signal is the heading value that is
	// calculated from magnetometer readings.
	yaw = dsp_updatelpf(Lpf + LPF_YAW,
		circf(dsp_updatecirccompl(Cmpl + CMPL_YAW, -gz * dt,
		heading(roll, -pitch,
			Magdata.fx, Magdata.fy, Magdata.fz)) 
			- St.adj.att0.yaw));

	// calculate gravity direction vector in IMU coordination system
	// using pitch and roll values;
	vx = -sin(roll);
	vy = sin(pitch) * cos(roll);
	vz = cos(pitch) * cos(roll);

	gvx = (1.0 - Goffset) * vx;
	gvy = (1.0 - Goffset) * vy;
	gvz = (1.0 - Goffset) * vz;

	// update vertical acceleration using acceleration
	// vector to gravity vector projection
	va = (vx * ax + vy * ay + vz * az)
		/ sqrtf(vx * vx + vy * vy + vz * vz);

	dsp_updatelpf(Lpf + LPF_VAU, va);
	dsp_updatelpf(Lpf + LPF_VAPT1, va);
	dsp_updatelpf(Lpf + LPF_VAAVG, va);

	// write vertical acceleration into log	
	writelog(LOG_VACCEL, dsp_getlpf(Lpf + LPF_VAU));

	// calculate forward direction vector in IMU
	// coordination system using pitch and roll values;
	vx = 0;
	vy = cos(pitch);
	vz = -sin(pitch);

	// update forward acceleration using acceleration
	// vector to gravity vector projection
	dsp_updatelpf(Lpf + LPF_FA,
		(vx * (ax - gvx) + vy * (ay - gvy) + vz * (az - gvz))
		/ sqrtf(vx * vx + vy * vy + vz * vz));

	// write forward acceleration into log	
	writelog(LOG_FACCEL, dsp_getlpf(Lpf + LPF_FA));

	// calculate sideward direction vector in IMU
	// coordination system using pitch and roll values;
	vx = cos(roll);
	vy = sin(pitch) * sin(roll);
	vz = sin(roll) * cos(pitch);

	// update sideward acceleration using acceleration
	// vector to gravity vector projection
	dsp_updatelpf(Lpf + LPF_SA,
		(vx * (ax - gvx) + vy * (ay - gvy) + vz * (az - gvx))
		/ sqrtf(vx * vx + vy * vy + vz * vz));
	
	// write sideward acceleration into log	
	writelog(LOG_SACCEL, dsp_getlpf(Lpf + LPF_SA));

	if (Dev[DEV_GNSS].status == DEVSTATUS_INIT
			&& M10_HASFIX(Gnss.quality)) {
		// calculate speed through latitude from
		// forward and sideward accelerations and
		// GNSS speed using complimetary filter
		dsp_updatecompl(Cmpl + CMPL_SLAT,
			9.80665 * (
			+ dsp_getlpf(Lpf + LPF_FA) * cosf(yaw)
			- dsp_getlpf(Lpf + LPF_SA) * sinf(yaw))
			* dt, Gnss.speed / 3.6 * cosf(Gnss.course));

		// calculate speed through longitude from
		// forward and sideward accelerations and
		// GNSS speed using complimetary filter
		dsp_updatecompl(Cmpl + CMPL_SLON,
			9.80665 * (
			+ dsp_getlpf(Lpf + LPF_FA) * sinf(yaw)
			+ dsp_getlpf(Lpf + LPF_SA) * cosf(yaw))
			* dt, Gnss.speed / 3.6 * sinf(Gnss.course));

		// calculate latitude from speed and GNSS
		// latitude using complimentary filter
		dsp_updatecompl(Cmpl + CMPL_LAT,
			dsp_getcompl(Cmpl + CMPL_SLAT) * dt,
			dsp_getlpf(Lpf + LPF_LATM));

		// calculate longitude from speed and GNSS
		// latitude using complimentary filter
		dsp_updatecompl(Cmpl + CMPL_LON,
			dsp_getcompl(Cmpl + CMPL_SLON) * dt,
			dsp_getlpf(Lpf + LPF_LONM));

		// calculate horizontal speed from speed
		// values through longitude and latitude
		dsp_updatelpf(Lpf + LPF_SPEED,
			sqrtf(powf(dsp_getcompl(Cmpl + CMPL_SLAT), 2.0)
			+ powf(dsp_getcompl(Cmpl + CMPL_SLON), 2.0)));

		// write speed values through latitude and longitude,
		// horizontal speed, latitude and longitude into log
		writelog(LOG_SLAT, dsp_getcompl(Cmpl + CMPL_SLAT));
		writelog(LOG_SLON, dsp_getcompl(Cmpl + CMPL_SLON));
		writelog(LOG_SPEED, dsp_getlpf(Lpf + LPF_SPEED));
		writelog(LOG_LAT, dsp_getcompl(Cmpl + CMPL_LAT));
		writelog(LOG_LON, dsp_getcompl(Cmpl + CMPL_LON));
	}

	// get last barometric altitude
	alt = dsp_getlpf(Lpf + LPF_ALT);

	// calculate altitude thrust compensation
	altcor = St.adj.althold.alttha * dsp_getlpf(Lpf + LPF_AVGTHRA)
		+ St.adj.althold.altthb;
	altcor = (altcor < 0.0) ? 0.0 : altcor;

	// compensate thrust for altitude
	alt -= altcor;	

	// if GNSS is locked, use speed to compensate dynamic pressure
	if (Dev[DEV_GNSS].status == DEVSTATUS_INIT
			&& M10_HASFIX(Gnss.quality)) {
		float sp;

		sp = dsp_getlpf(Lpf + LPF_SPEED);

		altcor = St.adj.althold.altthc * sp * sp;
		alt -= altcor;
	}

	// calculate climb rate from vertical acceleration and
	// barometric altitude defference using complimentary filter
	dsp_updatecompl(Cmpl + CMPL_CLIMBRATE,
		9.80665 * (dsp_getlpf(Lpf + LPF_VAU) + Goffset - 1.0) * dt,
			(dsp_getcompl(Cmpl + CMPL_ALT) - prevalt) / dt);
	
	// calculate presice altitiude from climb rate and
	// barometric altitude using complimentary filter
	dsp_updatecompl(Cmpl + CMPL_ALT,
		dsp_getcompl(Cmpl + CMPL_CLIMBRATE) * dt, alt);

	// store calculated alt for next calculation
	prevalt = dsp_getcompl(Cmpl + CMPL_ALT);

	// write climbrate and altitude values into log
	writelog(LOG_CLIMBRATE, dsp_getcompl(Cmpl + CMPL_CLIMBRATE));
	writelog(LOG_ALT, dsp_getcompl(Cmpl + CMPL_ALT));

	// if vertical acceleration is negative, most likely
	// quadcopter is upside down, perform emergency disarm
	if (dsp_getlpf(Lpf + LPF_THR) < -0.5) {
		Emergencydisarm = 1;
		setthrust(Dev + DEV_DSHOT, 0.0, 0.0, 0.0, 0.0);
		En = 0.0;
	}

	// write roll, pitch and yaw values into log
	writelog(LOG_ROLL, roll);
	writelog(LOG_PITCH, pitch);
	writelog(LOG_YAW, yaw);

	return 0;
}

int updatecorrection(float dt, struct corvals *cor)
{
	float roll, pitch, yaw;
	float gy, gx, gz;
	float ht;
	float tiltcoef;

	roll = dsp_getlpf(Lpf + LPF_ROLL);
	pitch = dsp_getlpf(Lpf + LPF_PITCH);
	yaw = dsp_getlpf(Lpf + LPF_YAW);

	// convert gyroscope values into radians
	gx = deg2rad(dsp_getlpf(Lpf + LPF_GYROX));
	gy = deg2rad(dsp_getlpf(Lpf + LPF_GYROY));
	gz = deg2rad(dsp_getlpf(Lpf + LPF_GYROZ));

	// get tilt compensation coefficient
	tiltcoef = cosf(pitch) * cosf(roll);

	// divide by zero protection, tilt
	// compesation for throttle works only
	// for angles less that 45 degress
	if (tiltcoef < cosf(St.adj.althold.tiltcoefmax * M_PI))
		tiltcoef = cosf(St.adj.althold.tiltcoefmax * M_PI);

	// get pitch corrected value of the hover throttle
	ht = St.adj.althold.hoverthrottle / tiltcoef;

	if (Dev[DEV_GNSS].status == DEVSTATUS_INIT
			&& M10_HASFIX(Gnss.quality)
			&& Gnssmode == GNSSMODE_POS) {

		float loncor, latcor;

		// calculate lonogitude and latitude correction
		loncor = dsp_pidbl(Pid + PID_LON, Rolltarget,
			dsp_getcompl(Cmpl + CMPL_LON));
		latcor = dsp_pidbl(Pid + PID_LAT, Pitchtarget,
			dsp_getcompl(Cmpl + CMPL_LAT));
	
		// calculate lonogitude and latitude correction
		loncor = dsp_pidbl(Pid + PID_SLON, loncor,
			dsp_getcompl(Cmpl + CMPL_SLON));
		latcor = dsp_pidbl(Pid + PID_SLAT, latcor,
			dsp_getcompl(Cmpl + CMPL_SLAT));

		// get pitch and roll correction values
		// using covertion to local frame, inverting pitch
		cor->pitch = -cosf(yaw) * latcor - sinf(yaw) * loncor;
		cor->roll =  -sinf(yaw) * latcor + cosf(yaw) * loncor;

		cor->roll = trimf(cor->roll,
			-M_PI * St.ctrl.rollmax * 0.5,
			M_PI * St.ctrl.rollmax * 0.5);

		cor->pitch = trimf(cor->pitch,
			-M_PI * St.ctrl.pitchmax * 0.5,
			M_PI * St.ctrl.pitchmax * 0.5);

		cor->roll = dsp_pidbl(Pid + PID_ROLLP, cor->roll, roll);
		cor->pitch = dsp_pidbl(Pid + PID_PITCHP, cor->pitch, pitch);

		cor->roll = dsp_pidbl(Pid + PID_ROLLS, cor->roll, gy);
		cor->pitch = dsp_pidbl(Pid + PID_PITCHS, cor->pitch, gx);
	}
	else if (Dev[DEV_GNSS].status == DEVSTATUS_INIT
			&& M10_HASFIX(Gnss.quality)
			&& Gnssmode == GNSSMODE_SPEED) {

		float loncor, latcor;
		float lontarget, lattarget;

		// convert target from local frame to global frame	
		lontarget = Pitchtarget * sinf(yaw)
			+ Rolltarget * cosf(yaw);

		lattarget = Pitchtarget * cosf(yaw)
			- Rolltarget * sinf(yaw);
		
		// calculate longitude and latitude correction
		loncor = dsp_pidbl(Pid + PID_SLON, lontarget,
			dsp_getcompl(Cmpl + CMPL_SLON));
		latcor = dsp_pidbl(Pid + PID_SLAT, lattarget,
			dsp_getcompl(Cmpl + CMPL_SLAT));

		// get pitch and roll correction values
		// using covertion to local frame, inverting pitch
		cor->pitch = -cosf(yaw) * latcor - sinf(yaw) * loncor;
		cor->roll =  -sinf(yaw) * latcor + cosf(yaw) * loncor;

		cor->roll = trimf(cor->roll,
			-M_PI * St.ctrl.rollmax * 0.5,
			M_PI * St.ctrl.rollmax * 0.5);

		cor->pitch = trimf(cor->pitch,
			-M_PI * St.ctrl.pitchmax * 0.5,
			M_PI * St.ctrl.pitchmax * 0.5);

		cor->roll = dsp_pidbl(Pid + PID_ROLLP, cor->roll, roll);
		cor->pitch = dsp_pidbl(Pid + PID_PITCHP, cor->pitch, pitch);

		cor->roll = dsp_pidbl(Pid + PID_ROLLS, cor->roll, gy);
		cor->pitch = dsp_pidbl(Pid + PID_PITCHS, cor->pitch, gx);
	}
	else if (Speedpid) {
		// if in single PID loop mode for tilt
		// (called accro mode), use only rotation speed values
		// from the gyroscope. Update speed PID controllers for
		// roll and pitch and get next correction values.
		cor->roll = dsp_pidbl(Pid + PID_ROLLS, Rolltarget, gy);
		cor->pitch = dsp_pidbl(Pid + PID_PITCHS, Pitchtarget, gx);
	}
	else {
		// if in double loop mode for tilt (most commonly used
		// mode), first update roll and pitch POSITION PID
		// controllers using currect roll and values and targets
		// got from ERLS and get next correction values.
		cor->roll = dsp_pidbl(Pid + PID_ROLLP, Rolltarget, roll);
		cor->pitch = dsp_pidbl(Pid + PID_PITCHP, Pitchtarget, pitch);

		// then use this values to update roll and pitch speed
		// PID controllers and get next SPEED correction values.
		cor->roll = dsp_pidbl(Pid + PID_ROLLS, cor->roll, gy);
		cor->pitch = dsp_pidbl(Pid + PID_PITCHS, cor->pitch, gx);
	}
			
	if (Yawspeedpid) {
		// if single PID loop mode for yaw is used just use
		// rotation speed values around axis Z to upadte yaw PID
		// controller and get next yaw correciton value
		cor->yaw = dsp_pidbl(Pid + PID_YAWS, Yawtarget, -gz);
	}
	else {
		// if in double loop mode for yaw, first use yaw value
		// calcualted using magnetometer and yaw target got from
		// ELRS remote to update yaw POSITION PID controller and
		// get it's next correciton value.
		cor->yaw = dsp_pidbl(Pid + PID_YAWP, Yawtarget, yaw);

		// then use this value to update yaw speed PID
		// controller and get next yaw SPEED correction value
		cor->yaw = dsp_pidbl(Pid + PID_YAWS, cor->yaw, -gz);
	}

	if (Altmode == ALTMODE_POS) {
		// if altitude hold mode enabled, first use altitude
		// got from barometer readings and target altitude from
		// ELRS remote to update altitude PID controller and
		// get it's next correction value
		cor->thrust = dsp_pidbl(Pid + PID_ALT, Thrust,
			dsp_getcompl(Cmpl + CMPL_ALT) - Alt0);

		// then use altitude correction value and climb rate
		// calculated by complimentary filter (barometer
		// differentiating and accelerometer Z-axis integration)
		// to update climb rate PID controller and get it's next
		// correction value
		cor->thrust = dsp_pidbl(Pid + PID_CLIMBRATE, cor->thrust,
			dsp_getcompl(Cmpl + CMPL_CLIMBRATE));
		
		// and next use climb rate correction value to update
		// vertial acceleration PID controller and get next
		// thrust correction value
		cor->thrust = dsp_pidbl(Pid + PID_VA, cor->thrust + 1.0,
			dsp_getlpf(Lpf + LPF_VAPT1)) / tiltcoef + ht;	
	}
	else if (Altmode == ALTMODE_SPEED) {
		// if consttant climb rate mode, first use climb rate
		// calculated by complimentary filter (barometer
		// differentiating and accelerometer Z-axis integration)
		// and target climb rate from ELRS remote to update
		// climb rate PID controller and get it's next
		// correction value
		cor->thrust = dsp_pidbl(Pid + PID_CLIMBRATE, Thrust,
			dsp_getcompl(Cmpl + CMPL_CLIMBRATE));	

		// and next use climb rate correction value to update
		// vertial acceleration PID controller and get next
		// thrust correction value
		cor->thrust = dsp_pidbl(Pid + PID_VA, cor->thrust + 1.0,
			dsp_getlpf(Lpf + LPF_VAPT1)) / tiltcoef + ht;
	}
	else {
		if (Hovermode) {
			// if no altitude hold and hover throttle mode
			// is enabled, update vertical acceleration PID
			// controller using next low-pass filtered
			// value of vertical acceleration and target
			// got from ERLS remote
			cor->thrust = dsp_pidbl(Pid + PID_VA,
				Thrust + 1.0,
				dsp_getlpf(Lpf + LPF_VAPT1))
					/ tiltcoef + ht;
		}
		else {
			// if no altitude hold and hover throttle mode
			// is disabled, update thrust PID controller
			// using next low-pass filtered value of thrust
			// and target got from ERLS remote
			cor->thrust = dsp_pidbl(Pid + PID_VA,
				Thrust + 1.0,
				dsp_getlpf(Lpf + LPF_THR));
		}
	}

	// reset bilinear PID-controllers when disarmed
	if (En < 0.5) {
		dsp_resetpidbl(Pid + PID_PITCHP);
		dsp_resetpidbl(Pid + PID_ROLLP);
		dsp_resetpidbl(Pid + PID_PITCHS);
		dsp_resetpidbl(Pid + PID_ROLLS);
		dsp_resetpidbl(Pid + PID_YAWP);
		dsp_resetpidbl(Pid + PID_YAWS);
		dsp_resetpidbl(Pid + PID_VA);
		dsp_resetpidbl(Pid + PID_CLIMBRATE);
		dsp_resetpidbl(Pid + PID_ALT);
		dsp_resetpidbl(Pid + PID_SLAT);
		dsp_resetpidbl(Pid + PID_SLON);
		dsp_resetpidbl(Pid + PID_LAT);
		dsp_resetpidbl(Pid + PID_LON);
	}

	// disable I-term for all
	// PID-controller, if no throttle
	if ((Altmode == ALTMODE_ACCEL && Thrust < 0
			&& !Hovermode)
		|| (Altmode == ALTMODE_SPEED
			&& Thrust < -0.95 * St.ctrl.climbratemax)
		|| (Altmode == ALTMODE_POS && Thrust < 0.01)) {
		dsp_resetpidbls(Pid + PID_PITCHP);
		dsp_resetpidbls(Pid + PID_ROLLP);
		dsp_resetpidbls(Pid + PID_PITCHS);
		dsp_resetpidbls(Pid + PID_ROLLS);
		dsp_resetpidbls(Pid + PID_YAWP);
		dsp_resetpidbls(Pid + PID_YAWS);
		dsp_resetpidbls(Pid + PID_VA);
		dsp_resetpidbls(Pid + PID_CLIMBRATE);
		dsp_resetpidbls(Pid + PID_ALT);
		dsp_resetpidbls(Pid + PID_SLAT);
		dsp_resetpidbls(Pid + PID_SLON);
		dsp_resetpidbls(Pid + PID_LAT);
		dsp_resetpidbls(Pid + PID_LON);
	}

	return 0;
}

int applythrust(const struct corvals *cor)
{
	float ltm, lbm, rbm, rtm;
	float tc;

	// calculate weights for motors
	// thrust calibration values
	ltm = (1.0 + St.adj.mtrsc.r / 2) * (1.0 + St.adj.mtrsc.p / 2);
	rtm = (1.0 - St.adj.mtrsc.r / 2) * (1.0 + St.adj.mtrsc.p / 2);
	lbm = (1.0 + St.adj.mtrsc.r / 2) * (1.0 - St.adj.mtrsc.p / 2);
	rbm = (1.0 - St.adj.mtrsc.r / 2) * (1.0 - St.adj.mtrsc.p / 2);

	// if final thrust is greater than
	// limit set it to the limit
	tc = cor->thrust > St.ctrl.thrustmax
		? St.ctrl.thrustmax : cor->thrust;

	// update motors thrust based on calculated values. For
	// quadcopter it's enought to split correction in half for
	// 3 pairs of motors: left and right for roll, top and bottom
	// for pitch and two diagonals (spinning in oposite directions)
	// for yaw.
	setthrust(Dev + DEV_DSHOT,
		En * ltm * (tc + 0.5 * cor->roll
			+ 0.5 * cor->pitch + 0.5 * cor->yaw),
		En * rtm * (tc - 0.5 * cor->roll
			+ 0.5 * cor->pitch - 0.5 * cor->yaw),
		En * lbm * (tc + 0.5 * cor->roll
			- 0.5 * cor->pitch - 0.5 * cor->yaw),
		En * rbm * (tc - 0.5 * cor->roll
			- 0.5 * cor->pitch + 0.5 * cor->yaw));

	return 0;
}
