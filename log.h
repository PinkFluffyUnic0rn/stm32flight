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
#define LOG_ACC_X	0	/*!< accelerometer x */
#define LOG_ACC_Y	1	/*!< accelerometer y */
#define LOG_ACC_Z	2	/*!< accelerometer z */
#define LOG_GYRO_X	3	/*!< gyroscope x */
#define LOG_GYRO_Y	4	/*!< gyroscope y */
#define LOG_GYRO_Z	5	/*!< gyroscope z */
#define LOG_MAG_X	6	/*!< magnetometer x */
#define LOG_MAG_Y	7	/*!< magnetometer y */
#define LOG_MAG_Z	8	/*!< magnetometer z */
#define LOG_BAR_TEMP	9	/*!< barometer temperature */
#define LOG_BAR_ALT	10	/*!< barometer altitude */
#define LOG_ROLL	11	/*!< roll */
#define LOG_PITCH	12	/*!< pitch */
#define LOG_YAW		13	/*!< yaw */
#define LOG_CLIMBRATE	14	/*!< climb rate */
#define LOG_ALT		15	/*!< altitude */
#define LOG_LT		16	/*!< left-top motor throttle */
#define LOG_LB		17	/*!< left-bottom motor throttle */
#define LOG_RB		18	/*!< right-top motor throttle */
#define LOG_RT		19	/*!< right-bottom motor throttle */
#define LOG_BAT		20	/*!< battery voltage */
#define LOG_CUR		21	/*!< battery current */
#define LOG_CRSFCH0	22	/*!< CRSF channel 0 */
#define LOG_CRSFCH1	23	/*!< CRSF channel 1 */
#define LOG_CRSFCH2	24	/*!< CRSF channel 2 */
#define LOG_CRSFCH3	25	/*!< CRSF channel 3 */
#define LOG_CRSFCH4	26	/*!< CRSF channel 4 */
#define LOG_CRSFCH5	27	/*!< CRSF channel 5 */
#define LOG_CRSFCH6	28	/*!< CRSF channel 6 */
#define LOG_CRSFCH7	29	/*!< CRSF channel 7 */
#define LOG_CRSFCH8	30	/*!< CRSF channel 8 */
#define LOG_CRSFCH9	31	/*!< CRSF channel 9 */
#define LOG_CRSFCH10	32	/*!< CRSF channel 10 */
#define LOG_CRSFCH11	33	/*!< CRSF channel 11 */
#define LOG_CRSFCH12	34	/*!< CRSF channel 12 */
#define LOG_CRSFCH13	35	/*!< CRSF channel 13 */
#define LOG_CRSFCH14	36	/*!< CRSF channel 14 */
#define LOG_CRSFCH15	37	/*!< CRSF channel 15 */
#define LOG_CUSTOM0	38	/*!< custom value 0 for debug */
#define LOG_CUSTOM1	39	/*!< custom value 1 for debug */
#define LOG_CUSTOM2	40	/*!< custom value 2 for debug */
#define LOG_CUSTOM3	41	/*!< custom value 3 for debug */
/**
* @}
*/

/**
* @brief log records per buffer
*/
#define LOG_RECSPERBUF (LOG_BUFSIZE / (sizeof(float) * st.logrecsize))

/**
* @brief log values names
*/
extern const char *logfieldstr[LOG_FIELDSTRSIZE];

/**
* @brief Get value's id by it's name.
*
* @param s value's name
* @return value's id if fould, -1 otherwise
*/
int log_fieldstrn(const char *s);

/**
* @brief Set value in current log frame.
* @param pos value's position inside the frame
* @param val value itself
* @return none
*/
void log_write(int pos, float val);

/**
* @brief Erase log flash to prepare at for writing,
* erasing starts from address 0.
*
* @param d character device to write status information
* @param size bytes count to erase
* @return always 0
*/
int eraseflash(const struct cdevice *d, size_t size);

/**
* @brief Print all log values into character device.
*
* @param d character device to write log values
* @param buf buffer to store temporary data, should be INFOLEN size
* @param from record to start from
* @param to record to end at
* @return -1 on error, 1 otherwise
*/
int log_print(const struct cdevice *d, char *buf,
	size_t from, size_t to);

/**
* @brief Update log frame. If buffer isn't full, just move buffer
* pointer, otherwise save buffer content into flash and set buffer
* pointer to 0.
* @return always 0
*/
int log_update();

/**
* @brief Set w25q device that will beused for logging.
* @param fd flash device context pointer
*/
int log_setdev(struct bdevice *fd);

/**
* Set log size and start or stop logging.
* @param size log size in records. If size is greater than 0 logging
* will be started or restarted, if size is 0, logging will be stopped
* @param d character device to print operation status info
* @param s buffer of INFOLEN size to store temporary inforation when
* 	writing to device d.
* @return -1 on error, 0 otherwise
*/
int log_set(int size, const struct cdevice *d, char *s);

#endif
