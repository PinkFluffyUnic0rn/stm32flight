/**
* @file settings.h
* @brief Settings management functions
*/

#ifndef SETTINGS_H
#define SETTINGS_H

#include "global.h"
#include "log.h"

/**
* @brief MCU flash address where quadcopter settings is stored
*/
#define USER_FLASH 0x080e0000

/**
* @defgroup DEF default settings values
* @{
*/
#define LTDEFNUM 0	/*!< left-top motor default output number */
#define LBDEFNUM 1	/*!< left-bottom motor default output number */
#define RBDEFNUM 2	/*!< right-bottom motor default output number */
#define RTDEFNUM 3	/*!< right-top motor default output number */

#define LOGDEFFREQ 128			/*!< default log frequency */
#define LOGDEFRECSIZE 4			/*!< default log size */
#define LOGFIELDDEFPOS 99		/*!< default log
					field position */
#define IRCDEFPOWER IRC_POWER_25	/*!< default video TX power */
#define IRCDEFFREQ IRC_FREQ_5733	/*!< default video
					TX frequency */
/**
* @}
*/

/**
* @brief quadcopter setting's slot
*/
#define USER_SETSLOTS (0x80 / sizeof(struct settings))

/**
* @defgroup COMP settings that set at compile time
* @{
*/
#define PID_MAX_I 0.5		/*!< maximum PID I-term value */
#define BAT_CUTOFF 100.0	/*!< battery voltage sensor
				filter cut-off frequency */
#define CUR_CUTOFF 100.0	/*!< battery current sensor
				filter cut-off frequency */
#define TEMP_TCOEF 0.5		/*!< PCB temperature filter
				time coefficient */
#define VA_AVG_TCOEF 2.0	/*!< averaging low-pass filter
				time coefficient for vectical
				acceleration */
#define BAT_SCALE 17.85882	/*!< battery sensor voltage scale */
/**
* @}
*/

struct settings
{
	struct {
		struct { float x, y, z; } acc0;
		struct { float x, y, z; } acctsc;
		struct { float x, y, z; } gyro0;

		struct { float roll, pitch, yaw; } att0;
		struct { float r, p; } mtrsc;

		float hoverthrottle;
		float curroff;
		float cursc;

		struct { float x, y, z; } mag0;
		struct { float x, y, z; } magsc;
		struct { float x, y, z; } magthrsc;
		float magdecl;
	} adj;

	struct {
		float thrustmax;
		float rollmax;
		float pitchmax;
		float accelmax;
		float climbratemax;
		float altmax;

		float rollrate;
		float pitchrate;
		float yawrate;
		float climbrate;
	} ctrl;

	struct {
		float att;
		float yaw;
		float climbrate;
		float alt;
	} cmpl;

	struct {
		float gyro;
		float acc;
		float mag;
		float va;
		float d;
	} lpf;

	struct {
		struct { float p, i, d; } attpos;
		struct { float p, i, d; } attrate;
		struct { float p, i, d; } yawrate;
		struct { float p, i, d; } yawpos;
		struct { float p, i, d; } throttle;
		struct { float p, i, d; } climbrate;
		struct { float p, i, d; } alt;
	} pid;

	struct {
		int power;
		int freq;
	} irc;

	struct {
		int lt;
		int lb;
		int rb;
		int rt;
	} mtr;

	struct {
		int freq;
		int recsize;
		int fieldid[LOG_FIELDSTRSIZE];
	} log;
};

/**
* @brief settings
*/
extern struct settings St;

/**
* @brief write quadcopter settings into internal MCU flash.
* @param slot offset in settings array in flash.
* @return always 0
*/
int writesettings(int slot);

/**
* @brief Validate current settings. Reset settings
	that has wrong values to default.
* @return always 0
*/
int validatesettings();

/**
* @brief Read setting from internal MCU flash.
* @param slot offset in settings array in flash
* @return always 0
*/
int readsettings(int slot);

#endif
