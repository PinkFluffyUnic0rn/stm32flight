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
#include "global.h"
#include "runvals.h"
#include "timev.h"
#include "command.h"
#include "util.h"

#include "qmc5883l.h"
#include "crsf.h"

/**
* @defgroup DEVNUMBERS
* @brief Character devices numbers
* @{
*/
#define ICM_DEV		0	/*!< IMU device number */
#define DPS_DEV		1	/*!< barometer device number */
#define QMC_DEV		2	/*!< magnetometer device number */
#define CRSF_DEV	3	/*!< eLRS device number */
#define M10_DEV		4	/*!< GNSS device number */
#define ESP_DEV		5	/*!< ESP device number */
#define UART_DEV	6	/*!< UART debug device number */
#define IRC_DEV		7	/*!< video TX device number */
/**
* @}
*/

#define DEV_COUNT	8	/*!< character devices count */

/**
* @defgroup TIMEREVENTIDS
* @brief Timer events IDs
* @{
*/
#define TEV_PID 	0	/*!< PID event ID */
#define TEV_CHECK 	1	/*!< connection check event ID */
#define TEV_DPS		2	/*!< barometer update event ID */
#define TEV_QMC		3	/*!< magnetometer update event ID */
#define TEV_LOG		4	/*!< log update event ID */
#define TEV_TELE	5	/*!< telemetry send event ID */
#define TEV_POWER	6	/*!< battery power update event ID */
/**
* @}
*/

#define TEV_COUNT	7	/*!< Timer events count */

/**
* @defgroup EVENTFREQUENCIES
* @brief Periodic events frequencies
* @{
*/
#define PID_FREQ 4000	/*!< PID event frequency */
#define CHECK_FREQ 1	/*!< connection check event frequency */
#define DPS_FREQ 128	/*!< barometer update event frequency */
#define QMC_FREQ 100	/*!< magnetometer update event frequency */
#define TELE_FREQ 10	/*!< telemetry send event frequency */
#define POWER_FREQ 500	/*!< battery power update event frequency */
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
#define ERLS_CH_SETSLOT		15	/*!< settings slot channel */
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
	GNSSSTATUS_VALID = 0,	/*!< data is valid */
	GNSSSTATUS_INVALID = 1	/*!< data is invalid */
};

/**
* @brief Latitude direction.
*/
enum LATDIR {
	LATDIR_N = 0,	/*!< north */
	LATDIR_S = 1,	/*!< south */
};

/**
* @brief Longitude direction.
*/
enum LONDIR {
	LONDIR_E = 0,	/*!< east */
	LONDIR_W = 1	/*!< west */
};

/**
* @brief Magnetic declination direction.
*/
enum MAGVARDIR {
	MAGVARDIR_E = 0,	/*!< east */
	MAGVARDIR_W = 1		/*!< west */
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
* @brief Flight controller board's character devices drivers
*/
extern struct cdevice dev[DEV_COUNT];

/**
* @brief Flight controller board's block device (memory chip)
*/
extern struct bdevice flashdev;

/**
* @defgroup DSPCONTEXTS
* @brief DSP contexts
* @{
*/
extern struct dsp_lpf batlpf;	/*!< battery voltage low-pass filter */
extern struct dsp_lpf currlpf;	/*!< battery voltage low-pass filter */

extern struct dsp_lpf avgthrustlpf; /*!< average motors thrust filter */

extern struct dsp_lpf valpf;	/*!< vertical acceleration unity filter */
extern struct dsp_lpf tlpf;	/*!< trust acceleration low-pass filter */
extern struct dsp_lpf vtlpf;	/*!< vertical acceleration low-pass filter */
extern struct dsp_lpf volpf;	/*!< vertical acceleration filter for g offset */

extern struct dsp_lpf altlpf;	/*!< altitude low-pass filter */
extern struct dsp_lpf templpf;	/*!< temperature low-pass filter */

extern struct dsp_lpf atemppt1;	/*!< IMU temperature unity filter */

extern struct dsp_lpf accxpt1;	/*!< accelerometer x low-pass filter */
extern struct dsp_lpf accypt1;	/*!< accelerometer y low-pass filter */
extern struct dsp_lpf acczpt1;	/*!< accelerometer z low-pass filter */

extern struct dsp_lpf gyroxpt1;	/*!< gyroscope x low-pass filter */
extern struct dsp_lpf gyroypt1;	/*!< gyroscope y low-pass filter */
extern struct dsp_lpf gyrozpt1;	/*!< gyroscope z low-pass filter */

extern struct dsp_lpf magxpt1;	/*!< gyroscope x low-pass filter */
extern struct dsp_lpf magypt1;	/*!< gyroscope y low-pass filter */
extern struct dsp_lpf magzpt1;	/*!< gyroscope z low-pass filter */

extern struct dsp_compl pitchcompl;	/*!< pitch low-pass filter */
extern struct dsp_compl rollcompl;	/*!< roll low-pass filter */
extern struct dsp_compl yawcompl;	/*!< yaw low-pass filter */

extern struct dsp_compl climbratecompl; /*!< climb rate complimentary filter */
extern struct dsp_compl altcompl; 	/*!< altitude complimentary filter */

extern struct dsp_pidblval pitchpv;	/*!< pitch PID context */
extern struct dsp_pidblval rollpv;	/*!< roll PID context */
extern struct dsp_pidblval pitchspv;	/*!< pitch speed PID context */
extern struct dsp_pidblval rollspv;	/*!< roll speed PID context */
extern struct dsp_pidval yawpv;		/*!< yaw PID context */
extern struct dsp_pidblval yawspv;	/*!< yaw speed PID context */
extern struct dsp_pidblval tpv;		/*!< vertical acceleration PID context */
extern struct dsp_pidblval cpv;		/*!< climb rate PID context */
extern struct dsp_pidblval apv;		/*!< altitude PID context */

/**
* @}
*/

/**
* @defgroup GLOBALSTORAGE
* @brief Global storage for sensor
	data that aquired in separate events
* @{
*/
extern struct qmc_data qmcdata;		/*!< magnetometer data */
extern struct gnss_data gnss;		/*!< GNSS data */
extern struct crsf_tele tele;		/*!< telemetry values */
/**
* @}
*/

/**
* @defgroup CONTROLVALUES
* @brief Control values
* @{
*/
extern float thrust; /*!< motors basic thrust */
extern float rolltarget; /*!< roll PID target */
extern float pitchtarget; /*!< pitch PID target */
extern float yawtarget; /*!< yaw PID target */
extern float ltm; /*!< left-top motor thrust scaling */
extern float lbm; /*!< left-bot motor thrust scaling */
extern float rtm; /*!< right-top motor thrust scaling */
extern float rbm; /*!< right-bot motor thrust scaling */
extern float en; /*!< 1.0 when motors turned on, 0.0 otherwise */
extern enum ALTMODE altmode; /*!< ALTMODE_POS if in altitude hold mode,
				ALTMODE_SPEED if climbrate control mode,
				ALTMODE_ACCEL if acceleration control mode */
extern int speedpid;	/*!<  1 if only gyroscope if used for yaw
			stabilization, 0 if accelerometer is used */
extern int yawspeedpid;	/*!< 1 if only gyroscope if used for yaw
			stabilization, 0 if magnetometer is used */
extern int hovermode; 	/*!< hover mode, when throttle is
			controlled relative to hover throttle */
extern int elrs; /*!< 1 when ELRS control is active (ELRS remote's
			channel 8 is > 50) */
/**
* @}
*/

extern float alt0;	/*!< reference altitude */
extern float goffset;	/*!< free fall acceleration (g) value offset */

extern int curslot; /*!< current settings slot */

extern struct timev evs[TEV_COUNT]; /*!< timer events */

extern int loops; /*!< stabilization loops counter */

extern int loopscount; /*!< stabilization loops performed in last second */

/**
* @brief Timeout counter for the ELRS reciver. Set to
	ELRS_TIMEOUT after receiving useful packet from receiver
	and decreased by 1 every second. If it falls to 0,
	quadcopter disarms.
*/
extern int elrstimeout;

/**
* @brief emergency disarm triggered, further
	arming is possible only after reboot
*/
extern int emergencydisarm;

#endif
