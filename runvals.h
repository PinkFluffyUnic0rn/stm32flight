/**
* @file runvalues.c
* @brief Global values that can be changed during run and dsp contexts
*/

#ifndef RUNVALUES_H
#define RUNVALUES_H

#include "settings.h"
#include "dsp.h"
#include "log.h"
#include "crc.h"
#include "runvals.h"
#include "timev.h"
#include "command.h"
#include "util.h"

#include "icm42688.h"
#include "qmc5883l.h"
#include "crsf.h"

/**
* @defgroup EVENTFREQUENCIES
* @brief Periodic events frequencies
* @{
*/
#define PID_FREQ 4000		/*!< PID event frequency */
#define CHECK_FREQ 1		/*!< connection check event frequency */
#define DPS_FREQ 128		/*!< barometer update event frequency */
#define QMC_FREQ 100		/*!< magnetometer update event frequency */
#define TELE_FREQ 10		/*!< telemetry send event frequency */
#define POWER_FREQ 500		/*!< battery power update event frequency */
#define AUTOPILOT_FREQ 32	/*!< autopilot update event frequency */
/**
* @}
*/

/**
* @brief timeout in seconds before quadcopter disarm
	when got no data from ERLS receiver
*/
#define ELRS_TIMEOUT 2

/**
* @brief time required to register button push,
	used for buttons and switches that controls
	time consuming functions
*/
#define ELRS_PUSHTIMEOUT 0.1

/**
* @defgroup ERLSCHANNELS
* @brief eRLS channels mapping
* @{
*/
#define ERLS_CH_ROLL		0	/*!< roll channel */
#define ERLS_CH_PITCH		1	/*!< pitch channel */
#define ERLS_CH_THRUST		2	/*!< throttle channel */
#define ERLS_CH_YAW		3	/*!< yaw channel */
#define ERLS_CH_YAWMODE		4	/*!< yaw mode channel */
#define ERLS_CH_ATTMODE		5	/*!< attitude mode channel */
#define ERLS_CH_THRMODE		6	/*!< altitude mode channel */
#define ERLS_CH_ONOFF		7	/*!< on/off channel */
#define ERLS_CH_ALTCALIB	8	/*!< recalibration channel */
#define ERLS_CH_HOVER		10	/*!< hover mode channel */
#define ERLS_CH_AUTOPILOT	11	/*!< autopilot mode channel */
#define ERLS_CH_SETSLOT		15	/*!< settings slot channel */
/**
* @}
*/

#define MAX_POINT_COUNT 16

/**
* @defgroup DEVIDS
* @brief Character devices IDs
* @{
*/
enum DEV_ID {
	DEV_ICM		= 0,	/*!< IMU device number */
	DEV_DPS		= 1,	/*!< barometer device number */
	DEV_QMC		= 2,	/*!< magnetometer device number */
	DEV_CRSF	= 3,	/*!< eLRS device number */
	DEV_M10		= 4,	/*!< GNSS device number */
	DEV_ESP		= 5,	/*!< ESP device number */
	DEV_UART	= 6,	/*!< UART debug device number */
	DEV_IRC		= 7,	/*!< video TX device number */
	DEV_DSHOT	= 8,	/*!< DShot-300 device number */
	DEV_COUNT	= 9	/*!< character devices count */
};
/**
* @}
*/

/**
* @defgroup TIMEREVENTIDS
* @brief Timer events IDs
* @{
*/
enum TEV_ID { 
	TEV_PID		= 0,	/*!< PID event ID */
	TEV_CHECK	= 1,	/*!< connection check event ID */
	TEV_DPS		= 2,	/*!< barometer update event ID */
	TEV_QMC		= 3,	/*!< magnetometer update event ID */
	TEV_LOG		= 4,	/*!< log update event ID */
	TEV_TELE	= 5,	/*!< telemetry send event ID */
	TEV_POWER	= 6,	/*!< battery power update event ID */
	TEV_AUTOPILOT	= 7,	/*!< autopilot event ID */
	TEV_COUNT	= 8	/*!< timer events count */
};
/**
* @}
*/

/**
* @defgroup LPFIDS
* @brief low-pass filters IDs
* @{
*/
enum LPF_ID {
	LPF_BAT		= 0,	/*!< battery voltage low-pass filter */
	LPF_CUR		= 1,	/*!< battery voltage low-pass filter */
	LPF_AVGTHR	= 2,	/*!< average motors thrust filter */
	LPF_VAU		= 3,	/*!< vertical acceleration unity filter */
	LPF_THR		= 4,	/*!< trust acceleration low-pass filter */
	LPF_VAPT1	= 5,	/*!< vertical acceleration low-pass filter */
	LPF_VAAVG	= 6,	/*!< vertical acceleration averaging filter */
	LPF_FA		= 7,	/*!< forward acceleration low-pass filter */
	LPF_ALT		= 8,	/*!< altitude low-pass filter */
	LPF_BARTEMP	= 9,	/*!< temperature low-pass filter */
	LPF_IMUTEMP	= 10,	/*!< IMU temperature unity filter */
	LPF_ACCX	= 11,	/*!< accelerometer x low-pass filter */
	LPF_ACCY	= 12,	/*!< accelerometer y low-pass filter */
	LPF_ACCZ	= 13,	/*!< accelerometer z low-pass filter */
	LPF_GYROX	= 14,	/*!< gyroscope x low-pass filter */
	LPF_GYROY	= 15,	/*!< gyroscope y low-pass filter */
	LPF_GYROZ	= 16,	/*!< gyroscope z low-pass filter */
	LPF_MAGX	= 17,	/*!< gyroscope x low-pass filter */
	LPF_MAGY	= 18,	/*!< gyroscope y low-pass filter */
	LPF_MAGZ	= 19,	/*!< gyroscope z low-pass filter */
	LPF_ROLL	= 20,	/*!< roll unity filter */
	LPF_PITCH	= 21,	/*!< pitch unity filter */
	LPF_YAW		= 22,	/*!< yaw unity filter */
	LPF_COUNT	= 23	/*!< low-pass filters count */
};
/**
* @}
*/

/**
* @defgroup CMPLIDS
* @brief complimentary filters IDs
* @{
*/
enum CMPL_ID {
	CMPL_PITCH 	= 0,	/*!< pitch low-pass filter */
	CMPL_ROLL 	= 1,	/*!< roll low-pass filter */
	CMPL_YAW	= 2,	/*!< yaw low-pass filter */
	CMPL_CLIMBRATE	= 3,	/*!< climb rate complimentary filter */
	CMPL_ALT 	= 4,	/*!< altitude complimentary filter */
	CMPL_COUNT 	= 5	/*!< complimentary filters count */
};
/**
* @}
*/

/**
* @defgroup PIDIDS
* @brief PID controllers IDs
* @{
*/
enum PID_ID {
	PID_PITCHP	= 0,	/*!< pitch PID context */
	PID_ROLLP	= 1,	/*!< roll PID context */
	PID_YAWP	= 2,	/*!< pitch speed PID context */
	PID_PITCHS	= 3,	/*!< roll speed PID context */
	PID_ROLLS	= 4,	/*!< yaw PID context */
	PID_YAWS	= 5,	/*!< yaw speed PID context */
	PID_VA		= 6,	/*!< vertical acceleration PID context */
	PID_CLIMBRATE	= 7,	/*!< climb rate PID context */
	PID_ALT		= 8,	/*!< altitude PID context */
	PID_COUNT	= 9	/*!< pid controllers count */
};
/**
* @}
*/


/**
* @brief Altitude control mode.
*/
enum ALTMODE {
	ALTMODE_ACCEL	= 0,	/*!< acceleration stabilization */
	ALTMODE_SPEED	= 1,	/*!< climb rate stabilization */
	ALTMODE_POS	= 2	/*!< altitude stabilization */
};

/**
* @brief GNSS data status.
*/
enum GNSSSTATUS {
	GNSSSTATUS_VALID	= 0,	/*!< data is valid */
	GNSSSTATUS_INVALID	= 1	/*!< data is invalid */
};

/**
* @brief Latitude direction.
*/
enum LATDIR {
	LATDIR_N	= 0,	/*!< north */
	LATDIR_S	= 1,	/*!< south */
};

/**
* @brief Longitude direction.
*/
enum LONDIR {
	LONDIR_E	= 0,	/*!< east */
	LONDIR_W	= 1	/*!< west */
};

/**
* @brief Magnetic declination direction.
*/
enum MAGVARDIR {
	MAGVARDIR_E	= 0,	/*!< east */
	MAGVARDIR_W	= 1	/*!< west */
};

/**
* @brief Configuration value type.
*/
enum CONFVALTYPE {
	CONFVALTYPE_INT		= 0,	/*!< integer */
	CONFVALTYPE_FLOAT	= 1,	/*!< float */
	CONFVALTYPE_STRING	= 2	/*!< string */
};

/**
* @brief Autopilot track point type
*/
enum AUTOPILOT_TYPE {
	AUTOPILOT_START 	= 0,	/*!< starting point */
	AUTOPILOT_TAKEOFF	= 1,	/*!< perform takeoff */
	AUTOPILOT_HOVER		= 2,	/*!< hover */
	AUTOPILOT_FORWARD	= 3,	/*!< move to pointed location */
	AUTOPILOT_LANDING	= 4,	/*!< perform landing */
	AUTOPILOT_STOP		= 5	/*!< stop point */
};

/**
* @brief Values got from GNSS module using NMEA protocol
*/
struct gnss_data {
	enum GNSSSTATUS status;		/*!< GNSS data status 
					1 if valid, 0 otherwise */

	float time;			/*!< seconds passed from
					00:00 UTC */
	char date[10];			/*!< date in format dd.mm.yy */

	uint8_t lat;			/*!< latitude */
	float latmin;			/*!< latitude minutes */
	enum LATDIR latdir;		/*!< latitude direction, 1 if
					south, 0 if north */

	uint8_t lon;			/*!< longitude */
	float lonmin;			/*!< longitude minutes */
	enum LONDIR londir;		/*!< longitude direction, 1 if */
					/*!< west, 0 if east */

	float magvar;			/*!< magnetic declination in
					degrees */
	enum MAGVARDIR  magvardir;	/*!< magnetic declination
					direction, 0 if east, 1 if
					west */

	float speed;			/*!< speed in knots */
	float course;			/*!< course toward north pole
					in degrees */
	float altitude;			/*!< altitude in meters */

	int quality;			/*!< link quality */
	uint8_t satellites;		/*!< satellites count */
};

/**
* @brief Autopilot track point
*/
struct trackpoint {
	union {
		struct {
			float alt;	/*!< target altitude */
			float t;	/*!< time to take off */
		} takeoff;		/*!< take off point description */
		struct {
			float x, y;	/*!< coordinates to look at */
			float alt;	/*!< target altitude */
			float t;	/*!< hovering time */
		} hover;		/*!< hover point description */
		struct {
			float x, y;	/*!< coordinates to move to */
		} forward;		/*!< move point description */
	};

	enum AUTOPILOT_TYPE type; /*!< point type */
};

/**
* @brief Timer events
*/
extern struct timev Evs[TEV_COUNT];

/**
* @brief Flight controller board's character devices drivers
*/
extern struct cdevice Dev[DEV_COUNT];

/**
* @brief Flight controller board's block device (memory chip)
*/
extern struct bdevice Flashdev;

/**
* @brief Low-pass filters
*/
extern struct dsp_lpf Lpf[LPF_COUNT];

/**
* @brief Complimentary filters
*/
extern struct dsp_compl Cmpl[CMPL_COUNT];

/**
* @brief PID controllers
*/
extern struct dsp_pidblval Pid[PID_COUNT];

/**
* @defgroup GLOBALSTORAGE
* @brief Global storage for sensor
	data that aquired in separate events
* @{
*/
extern struct qmc_data Qmcdata;		/*!< magnetometer data */
extern struct icm_data Imudata;		/*!< IMU data */
extern struct gnss_data Gnss;		/*!< GNSS data */
extern struct crsf_tele Tele;		/*!< telemetry values */
/**
* @}
*/

/**
* @defgroup CONTROLVALUES
* @brief Control values
* @{
*/
extern float Thrust; /*!< motors basic thrust */
extern float Rolltarget; /*!< roll PID target */
extern float Pitchtarget; /*!< pitch PID target */
extern float Yawtarget; /*!< yaw PID target */
extern float En; /*!< 1.0 when motors turned on, 0.0 otherwise */
extern enum ALTMODE Altmode; /*!< ALTMODE_POS if in altitude hold mode,
				ALTMODE_SPEED if climbrate control mode,
				ALTMODE_ACCEL if acceleration control mode */
extern int Speedpid;	/*!<  1 if only gyroscope if used for yaw
			stabilization, 0 if accelerometer is used */
extern int Yawspeedpid;	/*!< 1 if only gyroscope if used for yaw
			stabilization, 0 if magnetometer is used */
extern int Hovermode; 	/*!< hover mode, when throttle is
			controlled relative to hover throttle */
extern int Autopilot;	/*!< autopilot mode, 1 when autopilot
			is enabled, 0 otherwise */
extern int Elrs; /*!< 1 when ELRS control is active (ELRS remote's
			channel 8 is > 50) */
/**
* @}
*/

extern float Alt0;	/*!< reference altitude */
extern float Goffset;	/*!< free fall acceleration (g) value offset */

/**
* @brief autopilot track points
*/
extern struct trackpoint Points[MAX_POINT_COUNT];
extern int Pointscount;		/*!< autopilot track points count */
extern int Curpoint;		/*!< current autopilot track point */
extern float Autopilottimer;	/*!< autopilot timer */

extern int Curslot; /*!< current settings slot */

extern int Loops; /*!< stabilization loops counter */

extern int Loopscount; /*!< stabilization loops performed in last second */

/**
* @brief Timeout counter for the ELRS reciver. Set to
	ELRS_TIMEOUT after receiving useful packet from receiver
	and decreased by 1 every second. If it falls to 0,
	quadcopter disarms.
*/
extern int Elrstimeout;

/**
* @brief emergency disarm triggered, further
	arming is possible only after reboot
*/
extern int Emergencydisarm;

/**
* @brief Set motors thrust. All values should be between 0.0 and 1.0.
* @param dev DShot output device
* @param ltd left-top motor thrust
* @param rtd right-top motor thrust
* @param lbd left-bottom motor thrust
* @param rbd right-bottom motor thrust
* @return always 0
*/
int setthrust(struct cdevice *dev,
	float ltd, float rtd, float lbd, float rbd);

#endif
