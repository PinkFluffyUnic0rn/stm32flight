#ifndef SETTINGS_H
#define SETTINGS_H

#include "global.h"
#include "log.h"

// MCU flash address where quadcopter settings is stored
#define USER_FLASH 0x080e0000

// default settings values
#define LTDEFNUM 0
#define LBDEFNUM 1
#define RBDEFNUM 2
#define RTDEFNUM 3

#define LOGDEFFREQ 128
#define LOGDEFRECSIZE 4
#define LOGFIELDDEFPOS 99
#define IRCDEFPOWER IRC_POWER_25
#define IRCDEFFREQ IRC_FREQ_5733

// quadcopter setting's slot
#define USER_SETSLOTS (0x80 / sizeof(struct settings))

// Quadcopter settings structure stored in MCU's flash
struct settings {
	float mx0, my0, mz0;	// magnetometer offset values for X, Y
				// and Z axes determinted by
				// calibration procedure

	float mxsc, mysc, mzsc;	// megnetormeter scaling values for Z,
				// Y and Z axes determinted by
				// calibration procedure

	float magdecl;		// magnetic declination

	float ax0, ay0, az0;	// accelometer offset values for X, Y
				// and Z axes

	float gx0, gy0, gz0;	// gyroscope offset values for X, Y
				// and Z axes

	float rsc, psc;		// motor thrust scaling for roll and
				// pitch side motors, because not all
				// motors are abolutely equal

	float roll0, pitch0, yaw0; // roll, pitch and yaw offset values

	float thrustmax;	// maximum thrust value
	float rollmax, pitchmax; // maximum roll and pitch angles in Pi


	float rollspeed;	// roll rotation speed in Pi for
				// single loop tilt mode
	float pitchspeed;	// pitch rotation speed in Pi for
				// single loop tilt mode
	float yawspeed;		// yaw rotation speed in Pi for
				// single loop yaw mode
	float yawtargetspeed;	// yaw target change speed in Pi for
				// dual loop yaw mode
	float accelmax;		// maximum acceleration in g for
				// single loop throttle mode
	float climbratemax;	// maximum climbrate in m/s for
				// dual loop throttle mode
	float altmax;		// maximum altitude in m for triple
				// loop mode (altitude hold mode)

	float atctcoef;		// time coefficient for pitch/roll
				// complimentary filter
	float yctcoef;		// time coefficient for yaw
				// complimentary filter
	float ttcoef;		// time coefficient for vertical axis
				// acceleration pow-pass filter
	float vatcoef;		// time coefficient for vertical
				// acceleration pow-pass filter

	float accpt1freq;	// cut-off frequency for
				// accelerometer PT1 filter

	float gyropt1freq;	// cut-off frequency for
				// gyroscope PT1 filter
	
	float dpt1freq;		// cut-off frequency for PID D term

	float atcoef;		// time coefficient for altitude
				// low-pass filter
	float cctcoef;		// time coefficient for climb rate
				// complimentary filter
	float actcoef;		// time coefficient for altitude
				// complimentary filter

	int speedpid;	// 1 if single PID loop for roll/pitch is used,
			// 0 if double loop is used

	int yawspeedpid; // 1 if single PID loop for yaw is used, 0 if
			 // double loop is used

	float p, i, d;	// P/I/D values for roll/pitch (used only in
			// double roll/pitch PID loop mode)

	float sp, si, sd;	// P/I/D values for roll/pitch
				// rotation speed

	float yp, yi, yd; // P/I/D values for yaw (used only in double
			  // yaw PID loop mode)

	float ysp, ysi, ysd;	// P/I/D values for yaw rotation speed

	float zsp, zsi,	zsd; // P/I/D values for vertical acceleration
	float cp, ci, cd; // P/I/D values for climb rate
	float ap, ai, ad; // P/I/D values for altitude

	float curroffset;	// ESC's current meter offset
	float currscale;	// ESC's current meter scale

	int ircpower, ircfreq; // IRC Tramp VTX power and frequency
	int lt, lb, rb, rt; // motors ESC outputs numbers

	int logfreq;			// log frequency
	int logrecsize;			// log size
	int fieldid[LOG_FIELDSTRSIZE];	// id for every log field
};

// Settings
extern struct settings st;

// write quadcopter settings into internal MCU flash.
//
// slot -- offset in settings array in flash.
int writesettings(int slot);

// Validate current settings. Reset settings
// that has wrong values to default
int validatesettings();

// Read setting from internal MCU flash.
//
// slot -- offset in settings array in flash.
int readsettings(int slot);

#endif
