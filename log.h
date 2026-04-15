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
#define LOG_BUFSIZE (W25_PAGESIZE / 2)

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
#define LOG_FIELDSTRSIZE 53

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
	LOG_ROLL	= 11,
	LOG_PITCH	= 12,
	LOG_YAW		= 13,
	LOG_FACCEL	= 14,
	LOG_SACCEL	= 15,
	LOG_VACCEL	= 16,
	LOG_CLIMBRATE	= 17,
	LOG_ALT		= 18,
	LOG_LT		= 19,
	LOG_LB		= 20,
	LOG_RB		= 21,
	LOG_RT		= 22,
	LOG_AVGTHR	= 23,
	LOG_BAT		= 24,
	LOG_CUR		= 25,
	LOG_CRSFCH0	= 26,
	LOG_CRSFCH1	= 27,
	LOG_CRSFCH2	= 28,
	LOG_CRSFCH3	= 29,
	LOG_CRSFCH4	= 30,
	LOG_CRSFCH5	= 31,
	LOG_CRSFCH6	= 32,
	LOG_CRSFCH7	= 33,
	LOG_CRSFCH8	= 34,
	LOG_CRSFCH9	= 35,
	LOG_CRSFCH10	= 36,
	LOG_CRSFCH11	= 37,
	LOG_CRSFCH12	= 38,
	LOG_CRSFCH13	= 39,
	LOG_CRSFCH14	= 40,
	LOG_CRSFCH15	= 41,
	LOG_GNSS_QUAL	= 42,
	LOG_GNSS_LAT	= 43,
	LOG_GNSS_LON	= 44,
	LOG_GNSS_SPEED	= 45,
	LOG_GNSS_COURSE	= 46,
	LOG_GNSS_ALT	= 47,
	LOG_GNSS_SATS	= 48,
	LOG_CUSTOM0	= 49,
	LOG_CUSTOM1	= 50,
	LOG_CUSTOM2	= 51,
	LOG_CUSTOM3	= 52
};
/**
* @}
*/

/**
* @brief Log records per buffer
*/
#define LOG_RECSPERBUF (LOG_BUFSIZE / (sizeof(float) * St.log.recsize))

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
