#include <string.h>
#include <stdio.h>

#include "runvals.h"
#include "settings.h"
#include "crc.h"

#include "log.h"

static int logbufpos = 0;
static int logflashpos = 0;
static size_t logsize = 0;
static float logbuf[LOG_BUFSIZE / sizeof(float)];

const char *logfieldmap[LOG_FIELDSTRSIZE + 1] = {
	"acc_x", "acc_y", "acc_z",
	"gyro_x", "gyro_y", "gyro_z",
	"mag_x", "mag_y", "mag_z",
	"bar_temp", "bar_alt",
	"roll", "pitch", "yaw",
	"climbrate", "alt",
	"lt", "lb", "rb", "rt",
	"bat", "cur",
	"ch0", "ch1", "ch2", "ch3", "ch4", "ch5", "ch6", "ch7",
	"ch8", "ch9", "ch10", "ch11", "ch12", "ch13", "ch14", "ch15",
	"custom0", "custom1", "custom2", "custom3", NULL
};

/**
* @brief Erase log flash to prepare at for writing,
* erasing starts from address 0.
*
* @param d character device to write status information
* @param size bytes count to erase
* @return always 0
*/
static int eraseflash(const struct cdevice *d, size_t size)
{
	size_t pos;
	char s[255];

	// if erase size equals total flash size,
	// use chip erase command
	if (size == W25_TOTALSIZE) {
		Flashdev.eraseall(Flashdev.priv);
		return 0;
	}

	// else erase flash block-by-block, sector-by-sector
	for (pos = 0; pos < size; ) {
		// if remained size is more than block size
		// use block erase command,
		// use sector erase command otherwise
		if ((size - pos) >= W25_BLOCKSIZE) {
			Flashdev.eraseblock(Flashdev.priv, pos);

			sprintf(s, "erased block at %u\r\n", pos);
			d->write(d->priv, s, strlen(s));
			
			pos += W25_BLOCKSIZE;
		}
		else {
			Flashdev.erasesector(Flashdev.priv, pos);

			sprintf(s, "erased sector at %u\r\n", pos);
			d->write(d->priv, s, strlen(s));
			
			pos += W25_SECTORSIZE;
		}
	}

	return 0;
}

void writelog(int pos, float val)
{
	if (St.log.fieldid[pos] < 0
			|| St.log.fieldid[pos] >= St.log.recsize) {
		return;
	}

	logbuf[logbufpos * St.log.recsize + St.log.fieldid[pos]] = val;
}

int printlog(const struct cdevice *d, char *buf,
	size_t from, size_t to)
{
	int fp;

	if (from > to)
		return (-1);

	// enable read/write mode if reading log
	Flashdev.ioctl(Flashdev.priv, W25_IOCTL_READWRITE);

	// run through all writable space in the flash
	for (fp = from; fp < to; fp += LOG_RECSPERBUF) {
		int bp;

		// read batch of log frames into log buffer
		Flashdev.read(Flashdev.priv,
			sizeof(float) * fp * St.log.recsize, logbuf,
			LOG_BUFSIZE);

		// for every read frame
		for (bp = 0; bp < LOG_RECSPERBUF; ++bp) {
			char *data;
			int i;
		
			if (fp + bp < from || fp + bp >= to)
				continue;

			data = buf + 6;

			// put all frame's values into a string
			sprintf(data, "%d ", fp + bp);

			for (i = 0; i < St.log.recsize; ++i) {
				int rec;

				rec = bp * St.log.recsize;
				sprintf(data + strlen(data), "%0.5f ",
					(double) logbuf[rec + i]);
			}

			sprintf(data + strlen(data), "\r\n");

			sprintf(buf, "%05u",
				crc16((uint8_t *) data, strlen(data)));
			buf[5] = ' ';

			// send this string into debug connection
			d->write(d->priv, buf, strlen(buf));
		}
	}

	return 1;
}

int updatelog()
{
	if (logflashpos >= logsize)
		return 0;

	if (++logbufpos < LOG_RECSPERBUF) {
		memcpy(logbuf + logbufpos * St.log.recsize,
			logbuf + (logbufpos - 1) * St.log.recsize,
			St.log.recsize * sizeof(float));

		return 0;
	}

	Flashdev.write(Flashdev.priv, logflashpos,
		logbuf, LOG_BUFSIZE);

	memcpy(logbuf, logbuf + (LOG_RECSPERBUF - 1) * St.log.recsize,
		St.log.recsize * sizeof(float));

	logflashpos += LOG_BUFSIZE;
	logbufpos = 0;

	return 0;
}

int setlog(int size, const struct cdevice *d, char *s)
{
	// notify user when erasing is started
	if (d != NULL) {
		sprintf(s, "erasing flash...\r\n");
		d->write(d->priv, s, strlen(s));
	}

	logsize = size * sizeof(float) * St.log.recsize;

	// set log size (0 is valid and
	// means to disable logging)
	if (logsize > W25_TOTALSIZE)
		return (-1);

	// erase log flash no
	// respond during this process
	eraseflash(d, logsize);

	// enable writeonly mode if log writing is enabled
	Flashdev.ioctl(Flashdev.priv,
		(logsize == 0)
		? W25_IOCTL_READWRITE : W25_IOCTL_WRITEONLY);

	// notify user when telemetry flash is erased
	if (d != NULL) {
		sprintf(s,"erased %u bytes of flash\r\n", logsize);
		d->write(d->priv, s, strlen(s));
	}

	// enable telemetry
	logflashpos = 0;
	logbufpos = 0;

	return 0;
}
