#ifndef LOG_H
#define LOG_H

#include "device.h"
#include "w25.h"

// log records buffer size
#define LOG_BUFSIZE W25_PAGESIZE

// log maximum frequency
#define LOG_MAXFREQ 4096

// log record size
#define LOG_MAXRECSIZE	32

// log value id's count
#define LOG_FIELDSTRSIZE 38

// log values ids
#define LOG_ACC_X	0
#define LOG_ACC_Y	1
#define LOG_ACC_Z	2
#define LOG_GYRO_X	3
#define LOG_GYRO_Y	4
#define LOG_GYRO_Z	5
#define LOG_MAG_X	6
#define LOG_MAG_Y	7
#define LOG_MAG_Z	8
#define LOG_BAR_TEMP	9
#define LOG_BAR_ALT	10
#define LOG_ROLL	11
#define LOG_PITCH	12
#define LOG_YAW		13
#define LOG_CLIMBRATE	14
#define LOG_ALT		15
#define LOG_LT		16
#define LOG_LB		17
#define LOG_RB		18
#define LOG_RT		19
#define LOG_BAT		20
#define LOG_CUR		21
#define LOG_CRSFCH0	22
#define LOG_CRSFCH1	23
#define LOG_CRSFCH2	24
#define LOG_CRSFCH3	25
#define LOG_CRSFCH4	26
#define LOG_CRSFCH5	27
#define LOG_CRSFCH6	28
#define LOG_CRSFCH7	29
#define LOG_CRSFCH8	30
#define LOG_CRSFCH9	31
#define LOG_CRSFCH10	32
#define LOG_CRSFCH11	33
#define LOG_CRSFCH12	34
#define LOG_CRSFCH13	35
#define LOG_CRSFCH14	36
#define LOG_CRSFCH15	37

// log records per buffer
#define LOG_RECSPERBUF (LOG_BUFSIZE / (sizeof(float) * st.logrecsize))

// log values names
extern const char *logfieldstr[LOG_FIELDSTRSIZE];

// get value's id by it's name
//
// s -- value's name
int log_fieldstrn(const char *s);

// set value in current log frame
// pos -- value's position inside the frame
// val -- value itself
void log_write(int pos, float val);

// Erase log flash to prepare at for writing,
// erasing starts from address 0.
//
// size -- bytes count to erase.
int eraseflash(const struct cdevice *d, size_t size);

// Print all log values into debug connection.
//
// s -- string user as buffer.
int log_print(const struct cdevice *d, char *buf,
	size_t from, size_t to);

// Update log frame. If buffer isn't full, just move buffer pointer,
// otherwise save buffer content into flash and set buffer pointer to 0.
int log_update();

// Set w25q device that will beused for logging
//
// fd -- flash device context pointer
int log_setdev(struct bdevice *fd);

// Set log size and start or stop logging.
//
// size -- log size in records. If size is greater than 0 logging will
// be started or restarted, if size is 0, logging will be stopped
//
// d -- character device to print operation status info
// s -- buffer of INFOLEN size to store temporary inforation when
// 	writing to device d.
int log_set(int size, const struct cdevice *d, char *s);

#endif
