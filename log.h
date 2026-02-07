/**
* @file log.h
* @brief Flight log functions
*/

#ifndef LOG_H
#define LOG_H

#include "device.h"
#include "w25.h"

/**
* @brief log records buffer size
*/
#define LOG_BUFSIZE W25_PAGESIZE

/**
* @brief log maximum frequency
*/
#define LOG_MAXFREQ 4096

/**
* @brief log record size
*/
#define LOG_MAXRECSIZE	32

/**
* @brief log value id's count
*/
#define LOG_FIELDSTRSIZE 42

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
	LOG_CLIMBRATE	= 14,
	LOG_ALT		= 15,
	LOG_LT		= 16,
	LOG_LB		= 17,
	LOG_RB		= 18,
	LOG_RT		= 19,
	LOG_BAT		= 20,
	LOG_CUR		= 21,
	LOG_CRSFCH0	= 22,
	LOG_CRSFCH1	= 23,
	LOG_CRSFCH2	= 24,
	LOG_CRSFCH3	= 25,
	LOG_CRSFCH4	= 26,
	LOG_CRSFCH5	= 27,
	LOG_CRSFCH6	= 28,
	LOG_CRSFCH7	= 29,
	LOG_CRSFCH8	= 30,
	LOG_CRSFCH9	= 31,
	LOG_CRSFCH10	= 32,
	LOG_CRSFCH11	= 33,
	LOG_CRSFCH12	= 34,
	LOG_CRSFCH13	= 35,
	LOG_CRSFCH14	= 36,
	LOG_CRSFCH15	= 37,
	LOG_CUSTOM0	= 38,
	LOG_CUSTOM1	= 39,
	LOG_CUSTOM2	= 40,
	LOG_CUSTOM3	= 41
};
/**
* @}
*/

/**
* @brief log records per buffer
*/
#define LOG_RECSPERBUF (LOG_BUFSIZE / (sizeof(float) * St.log.recsize))

/**
* @brief log values names
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
*
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
* pointer, otherwise save buffer content into flash and set buffer
* pointer to 0.
* @return always 0
*/
int updatelog();

/**
* Set log size and start or stop logging.
* @param size log size in records. If size is greater than 0 logging
* will be started or restarted, if size is 0, logging will be stopped
* @param d character device to print operation status info
* @param s buffer of INFOLEN size to store temporary inforation when
* 	writing to device d.
* @return -1 on error, 0 otherwise
*/
int setlog(int size, const struct cdevice *d, char *s);

#endif
