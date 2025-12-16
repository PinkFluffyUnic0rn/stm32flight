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

/**
* @brief Quadcopter settings structure stored in MCU's flash
*/
struct settings {

	/**
	* @defgroup MAGOFFSET
	* @brief magnetometer offset values
		determinted by calibration procedure
	* @{
	*/
	float mx0 /*! x */, my0 /*! y */, mz0 /*! z */;  
	/**
	* @}
	*/

	/**
	* @defgroup MAGSCALE
	* @brief magnetormeter scaling values
		determinted by calibration procedure
	* @{
	*/
	float mxsc /*! x */, mysc /*! y */, mzsc /*! z */;
	/**
	* @}
	*/

	/**
	* @defgroup MAGTHROFFSET
	* @brief magnetormeter thrust offset scale
		determinted by calibration procedure
	* @{
	*/
	float mxthsc /*! x */, mythsc /*! y */, mzthsc /*! z */;
	/**
	* @}
	*/

	float magdecl;		/*!< magnetic declination */

	/**
	* @defgroup ACCOFFSET
	* @brief accelometer offset values
	* @{
	*/
	float ax0 /*! x */, ay0 /*! y */, az0 /*! z */;
	/**
	* @}
	*/
	
	float aztscale /*!< accelerometer z axis thermal scaling */;

	/**
	* @defgroup GYROOFFSET
	* @brief gyroscope offset values
	* @{
	*/
	float gx0 /*! x */, gy0 /*! y */, gz0 /*! z */;
	/**
	* @}
	*/

	/**
	* @defgroup THRUSTSCALE
	* @brief motor thrust scaling for roll and
				pitch side motors, because not all
				motors are abolutely equal
	* @{
	*/
	float rsc /*! roll */, psc /*! pitch */; 
	/**
	* @}
	*/

	/**
	* @defgroup ROTATEOFFSET
	* @brief rotation offset values
	* @{
	*/
	float roll0 /*! roll */, pitch0 /*! pitch */,  yaw0 /*! yaw */; 
	/**
	* @}
	*/

	float thrustmax;		/*!< maximum thrust value */

	/**
	* @defgroup ROTATEMAX
	* @brief maxiumum rotation values
	* @{
	*/
	float rollmax /*! roll */, pitchmax /*! pitch */;
	/**
	* @}
	*/

	float rollspeed;	/*!< roll rotation speed in Pi for
				single loop tilt mode */
	float pitchspeed;	/*!< pitch rotation speed in Pi for
				single loop tilt mode */
	float yawspeed;		/*!< yaw rotation speed in Pi for
				single loop yaw mode */
	float yawtargetspeed;	/*!< yaw target change speed in Pi for
				dual loop yaw mode */
	float accelmax;		/*!< maximum acceleration in g for
				single loop throttle mode */
	float climbratemax;	/*!< maximum climbrate in m/s for
				dual loop throttle mode */
	float altmax;		/*!< maximum altitude in m for triple
				loop mode (altitude hold mode) */

	float atctcoef;		/*!< time coefficient for pitch/roll
				complimentary filter */
	float yctcoef;		/*!< time coefficient for yaw
				complimentary filter */
	float ttcoef;		/*!< time coefficient for vertical axis
				acceleration pow-pass filter */
	float vatcoef;		/*!< time coefficient for vertical
				acceleration pow-pass filter */

	float accpt1freq;	/*!< cut-off frequency for
				accelerometer PT1 filter */

	float gyropt1freq;	/*!< cut-off frequency for
				gyroscope PT1 filter */
	
	float magpt1freq;	/*!< time coefficient for
				magnetometer low-pass filter */
	
	float dpt1freq;		/*!< cut-off frequency for PID D term */

	float apt1freq;		/*!< time coefficient for altitude
				low-pass filter */
	float cctcoef;		/*!< time coefficient for climb rate
				complimentary filter */
	float actcoef;		/*!< time coefficient for altitude
				complimentary filter */

	int speedpid;	/*!< 1 if single PID loop for roll/pitch
			is used, 0 if double loop is used */

	int yawspeedpid; /*!< 1 if single PID loop for yaw is used,
			0 if double loop is used */

	/**
	* @defgroup PIDATTITUDE
	* @brief P/I/D values for roll/pitch (used only in
		double roll/pitch PID loop mode)
	* @{
	*/
	float p /*! p */, i /*! i */, d /*! d */;
	/**
	* @}
	*/

	/**
	* @defgroup PIDROTATION
	* @brief P/I/D values for roll/pitch rotation speed
	* @{
	*/
	float sp /*! p */, si /*! i */, sd /*! d */;
	/**
	* @}
	*/

	/**
	* @defgroup PIDYAW
	* @brief P/I/D values for yaw (used only in double
		yaw PID loop mode)
	* @{
	*/
	float yp /*! p */, yi /*! i */, yd /*! d */;
	/**
	* @}
	*/

	/**
	* @defgroup PIDYAWROTATION
	* @brief P/I/D values for yaw rotation speed
	* @{
	*/
	float ysp /*! p */, ysi /*! p */, ysd /*! p */;
	/**
	* @}
	*/

	/**
	* @defgroup PIDYAWROTATION
	* @brief P/I/D values for vertical acceleration
	* @{
	*/
	float zsp /*! p */, zsi /*! i */, zsd /*! d */;
	/**
	* @}
	*/

	/**
	* @defgroup PIDYAWROTATION
	* @brief P/I/D values for climb rate
	* @{
	*/
	float cp /*! p */, ci /*! i */, cd /*! d */;
	/**
	* @}
	*/

	/**
	* @defgroup PIDYAWROTATION
	* @brief P/I/D values for altitude
	* @{
	*/
	float ap /*! p */, ai /*! i */, ad /*! d */;
	/**
	* @}
	*/

	float curroffset;	/*!< ESC's current meter offset */
	float currscale;	/*!< ESC's current meter scale */

	float hoverthrottle;	/*!< hover throttle value */

	/**
	* @defgroup PIDYAWROTATION
	* @brief IRC Tramp VTX power and frequency
	* @{
	*/
	int ircpower /*! power */, ircfreq /*! frequency */;
	/**
	* @}
	*/

	/**
	* @defgroup PIDYAWROTATION
	* @brief motors ESC outputs numbers
	* @{
	*/
	int lt /*! left-top */, lb /*! left-bottom */,
	    rb /*! right-bottom */, rt /*! right-top */;
	/**
	* @}
	*/

	int logfreq;			/*!< log frequency */
	int logrecsize;			/*!< log size */
	int fieldid[LOG_FIELDSTRSIZE];	/*!< id for every log field */
};

/**
* @brief settings
*/
extern struct settings st;

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
