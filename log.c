#include <string.h>
#include <stdio.h>

#include "settings.h"
#include "crc.h"

#include "log.h"

static struct bdevice *flashdev;

static int logbufpos = 0;
static int logflashpos = 0;
static size_t logsize = 0;
static float logbuf[LOG_BUFSIZE / sizeof(float)];

const char *logfieldstr[LOG_FIELDSTRSIZE] = {
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
	"custom0", "custom1", "custom2", "custom3"
};

static int log_eraseflash(const struct cdevice *d, size_t size)
{
	size_t pos;
	char s[255];

	// if erase size equals total flash size,
	// use chip erase command
	if (size == W25_TOTALSIZE) {
		flashdev->eraseall(flashdev->priv);
		return 0;
	}

	// else erase flash block-by-block, sector-by-sector
	for (pos = 0; pos < size; ) {
		// if remained size is more than block size
		// use block erase command,
		// use sector erase command otherwise
		if ((size - pos) >= W25_BLOCKSIZE) {
			flashdev->eraseblock(flashdev->priv, pos);

			sprintf(s, "erased block at %u\r\n", pos);
			d->write(d->priv, s, strlen(s));
			
			pos += W25_BLOCKSIZE;
		}
		else {
			flashdev->erasesector(flashdev->priv, pos);

			sprintf(s, "erased sector at %u\r\n", pos);
			d->write(d->priv, s, strlen(s));
			
			pos += W25_SECTORSIZE;
		}
	}

	return 0;
}

int log_fieldstrn(const char *s)
{
	int i;

	for (i = 0; i < LOG_FIELDSTRSIZE; ++i) {
		if (strcmp(s, logfieldstr[i]) == 0)
			break;
	}
	
	if (i >= LOG_FIELDSTRSIZE)
		return (-1);

	return i;
}

void log_write(int pos, float val)
{
	if (st.log.fieldid[pos] < 0
			|| st.log.fieldid[pos] >= st.log.recsize) {
		return;
	}

	logbuf[logbufpos * st.log.recsize + st.log.fieldid[pos]] = val;
}

int log_print(const struct cdevice *d, char *buf,
	size_t from, size_t to)
{
	int fp;

	if (from > to)
		return (-1);

	// enable read/write mode if reading log
	flashdev->ioctl(flashdev->priv, W25_IOCTL_READWRITE);

	// run through all writable space in the flash
	for (fp = from; fp < to; fp += LOG_RECSPERBUF) {
		int bp;

		// read batch of log frames into log buffer
		flashdev->read(flashdev->priv,
			sizeof(float) * fp * st.log.recsize, logbuf,
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

			for (i = 0; i < st.log.recsize; ++i) {
				int rec;

				rec = bp * st.log.recsize;
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

int log_update()
{
	if (logflashpos >= logsize)
		return 0;

	if (++logbufpos < LOG_RECSPERBUF) {
		memcpy(logbuf + logbufpos * st.log.recsize,
			logbuf + (logbufpos - 1) * st.log.recsize,
			st.log.recsize * sizeof(float));

		return 0;
	}

	flashdev->write(flashdev->priv, logflashpos,
		logbuf, LOG_BUFSIZE);

	memcpy(logbuf, logbuf + (LOG_RECSPERBUF - 1) * st.log.recsize,
		st.log.recsize * sizeof(float));

	logflashpos += LOG_BUFSIZE;
	logbufpos = 0;

	return 0;
}

int log_setdev(struct bdevice *fd)
{
	flashdev = fd;
	
	return 0;
}

int log_set(int size, const struct cdevice *d, char *s)
{
	// notify user when erasing is started
	if (d != NULL) {
		sprintf(s, "erasing flash...\r\n");
		d->write(d->priv, s, strlen(s));
	}

	logsize = size * sizeof(float) * st.log.recsize;

	// set log size (0 is valid and
	// means to disable logging)
	if (logsize > W25_TOTALSIZE)
		return (-1);

	// erase log flash no
	// respond during this process
	log_eraseflash(d, logsize);

	// enable writeonly mode if log writing is enabled
	flashdev->ioctl(flashdev->priv,
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
