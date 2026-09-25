/**
* @file log.h
* @brief Flight log functions
*/

#ifndef LOG_H
#define LOG_H

#include "device.h"
#include "w25.h"

/**
* @brief Log records buffer size
*/
#define LOG_BUFSIZE (W25_PAGESIZE)

/**
* @brief Log maximum frequency
*/
#define LOG_MAXFREQ 8000

/**
* @brief Log record size
*/
#define LOG_MAXRECSIZE	32

/**
* @brief Log value id's count
*/
#define LOG_FIELDSTRSIZE 77

/**
* @defgroup LOG log values id
* @{
*/
enum LOG_FIELD {
	LOG_ACC_X	= 0,
	LOG_ACC_Y	= 1,
	LOG_ACC_Z	= 2,
	LOG_GYRO_X	= 3,
	LOG_GYRO_Y	= 4,
	LOG_GYRO_Z	= 5,
	LOG_MAG_X	= 6,
	LOG_MAG_Y	= 7,
	LOG_MAG_Z	= 8,
	LOG_BAR_TEMP	= 9,
	LOG_BAR_ALT	= 10,
	LOG_LIDAR_ALT	= 11,
	LOG_LIDAR_VALID	= 12,
	LOG_ROLL	= 13,
	LOG_PITCH	= 14,
	LOG_YAW		= 15,
	LOG_FACCEL	= 16,
	LOG_SACCEL	= 17,
	LOG_VACCEL	= 18,
	LOG_CLIMBRATE	= 19,
	LOG_ALT		= 20,
	LOG_GNDALT	= 21,
	LOG_LT		= 22,
	LOG_LB		= 23,
	LOG_RB		= 24,
	LOG_RT		= 25,
	LOG_AVGTHR	= 26,
	LOG_BAT		= 27,
	LOG_CUR		= 28,
	LOG_PITCH_PID	= 29,
	LOG_ROLL_PID	= 30,
	LOG_YAW_PID	= 31,
	LOG_PITCHS_PID	= 32,
	LOG_PITCHS_PIDI	= 33,
	LOG_ROLLS_PID	= 34,
	LOG_ROLLS_PIDI	= 35,
	LOG_YAWS_PID	= 36,
	LOG_VA_PID	= 37,
	LOG_VA_PIDI	= 38,
	LOG_CRATE_PID	= 39,
	LOG_ALT_PID	= 40,
	LOG_SLAT_PID	= 41,
	LOG_SLON_PID	= 42,
	LOG_LAT_PID	= 43,
	LOG_LON_PID	= 44,
	LOG_CRSFCH0	= 45,
	LOG_CRSFCH1	= 46,
	LOG_CRSFCH2	= 47,
	LOG_CRSFCH3	= 48,
	LOG_CRSFCH4	= 49,
	LOG_CRSFCH5	= 50,
	LOG_CRSFCH6	= 51,
	LOG_CRSFCH7	= 52,
	LOG_CRSFCH8	= 53,
	LOG_CRSFCH9	= 54,
	LOG_CRSFCH10	= 55,
	LOG_CRSFCH11	= 56,
	LOG_CRSFCH12	= 57,
	LOG_CRSFCH13	= 58,
	LOG_CRSFCH14	= 59,
	LOG_CRSFCH15	= 60,
	LOG_GNSS_QUAL	= 61,
	LOG_GNSS_LAT	= 62,
	LOG_GNSS_LON	= 63,
	LOG_GNSS_SPEED	= 64,
	LOG_GNSS_COURSE	= 65,
	LOG_GNSS_ALT	= 66,
	LOG_GNSS_SATS	= 67,
	LOG_SPEED	= 68,
	LOG_SLAT	= 69,
	LOG_SLON	= 70,
	LOG_LAT		= 71,
	LOG_LON		= 72,
	LOG_CUSTOM0	= 73,
	LOG_CUSTOM1	= 74,
	LOG_CUSTOM2	= 75,
	LOG_CUSTOM3	= 76
};
/**
* @}
*/

/**
* @brief Log records per buffer
*/
#define LOG_RECSPERBUF (LOG_BUFSIZE \
	/ (sizeof(float) * Strun.log.recsize))

/**
* @brief Log values names
*/
extern const char *logfieldmap[LOG_FIELDSTRSIZE + 1];

/**
* @brief Set value in current log frame.
* @param pos value's position inside the frame
* @param val value itself
* @return none
*/
void writelog(int pos, float val);

/**
* @brief Print all log values into character device.
* @param d character device to write log values
* @param buf buffer to store temporary data, should be INFOLEN size
* @param from record to start from
* @param to record to end at
* @return -1 on error, 1 otherwise
*/
int printlog(const struct cdevice *d, char *buf,
	size_t from, size_t to);

/**
* @brief Update log frame. If buffer isn't full, just move buffer
	pointer, otherwise save buffer content into flash and set buffer
	pointer to 0
* @return always 0
*/
int updatelog();

/**
* @brief Set log size and start or stop logging.
* @param size log size in records. If size is greater than 0 logging
	will be started or restarted, if size is 0, logging 
	will be stopped
* @param d character device to print operation status info
* @param s buffer of INFOLEN size to store temporary inforation when
 	writing to device d
* @return -1 on error, 0 otherwise
*/
int setlog(int size, const struct cdevice *d, char *s);

#endif
