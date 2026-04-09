#include <termios.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <unistd.h>
#include <limits.h>
#include <libgen.h>
#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "../../crc.h"

#include "uavconf.h"

/**
* @brief UDP connection port
*/
#define PORT 8880

/**
* @brief UDP connection address
*/
#define ADDR "192.168.3.1"

/**
* @brief socket send/receive buffer size
*/
#define BUFSZ 1024

/**
* @brief UDP packet receive timeout
*/
#define UDPTIMEOUT 1000000

/**
* @brief minimum and maximum wait time for
	UAV configuration command to execute
*/
#define MINWAIT 5000
#define MAXWAIT 1000000
#define FLASHWAIT 2000000
#define IRCWAIT 1000000

/**
* @brief maximum UAV command size
*/
#define CMDMAXSZ 60

/**
* @brief log size in records
*/
#define LOGSIZE (1024 * 16)

/**
* @brief maximum number of log records in
	batch when reading whole log
*/
#define BATCHSIZE 5

/**
* @brief userdata structure passed to server-side
	part of log download command
*/
struct loggetdata {
	int reccnt;		/*!< number of log records to get */
	char *buf;		/*!< raw data buffer to hold all data */
				/*!< got from UDP socket */
	size_t bufoffset;	/*!< raw data buffer offset */
	size_t maxsz;		/*!< raw data buffer allocated size */
	size_t sz;		/*!< raw data size got after call */
};

/**
* @brief Common function for sending data into debug connection,
	which can be UART or UDP socket.
* @param fd debug connection file descriptor
* @param buf a data buffer to send
* @param len the data buffer length
* @param flags flags to pass into sendto in case of UDP socket
* @param addr UDP peer address, should be 0 in case of UART connection
* @param addrlen UDP peer address len
* @return write or sendto return code
*/
static ssize_t sendtochan(int fd, const void *buf, size_t len,
	int flags, const struct sockaddr *addr, socklen_t addrlen)
{
	if (addr == NULL) {
		char bufr[BUFSZ];
		size_t slen;

		slen = strlen(buf);

		strcpy(bufr, buf);

		if (slen < BUFSZ - 1)
			strcat(bufr, "\r");

		return write(fd, bufr, len + 1);
	}

	return sendto(fd, buf, len, flags, addr, addrlen);
}

/*
* @brief Common function for sending data into debug connection,
	which can be UART or UDP socket.
* @param fd debug connection file descriptor
* @param buf a data buffer to send
* @param len the data buffer length
* @param flags flags to pass into recvfrom in case of UDP socket
* @param addr UDP peer address, should be 0 in case of UART connection
* @param addrlen UDP peer address len
* @return write or sendto return code
*/
static ssize_t recvfromchan(int fd, void *buf, size_t len,
	int flags, struct sockaddr *addr, socklen_t *addrlen)
{
	if (addr == NULL)
		return read(fd, buf, len);

	return recvfrom(fd, buf, len, flags, addr, addrlen);
}

/**
* @brief Server-side part of log download command.
* @param lsfd UDP socket for configuration connection
* @param rsi remote IP address
* @param cmd command was sent
* @param outfunc function used to process command output
* @param data userdata passed to sendcmd
* @return always 0
*/
static int logget(int lsfd, const struct sockaddr_in *rsi,
	const char *cmd, void (*outfunc) (void *, const char *),
	void *data)
{
	struct loggetdata *d;
	char *logbuf;
	int i;

	(void)(cmd);
	(void)(outfunc);

	d = data;

	// read requested log records through UDP
	d->sz = 0;
	for (i = 0; i < d->reccnt; ++i) {
		socklen_t rsis;
		size_t offset;
		char c;
		char *s;
		int rsz;

		// reallocate the raw data buffer if it is to
		// small to hold next packet
		if (d->sz + d->bufoffset + BUFSZ >= d->maxsz) {
			d->maxsz = (d->bufoffset + d->sz + BUFSZ + 1) * 2;
			d->buf = realloc(d->buf, d->maxsz);
		}

		// add offset to raw data buffer. This offset needed
		// because the raw data buffer can be used between many
		// consecutive call.
		logbuf = d->buf + d->bufoffset;

		rsis = sizeof(rsi);
		if ((rsz = recvfromchan(lsfd, logbuf + d->sz,
				BUFSZ, 0, (struct sockaddr *) rsi,
				&rsis)) < 0) {
			break;
		}

		// temporally null terminate data got from UDP socket
		c = logbuf[d->sz + rsz];
		logbuf[d->sz + rsz] = '\0';

		// search for end marker in in currently got packet
		// with offset such that end of previous packet changed
		// too. This is needed because end marker can be divided
		// between two packets.
		offset = (d->sz < strlen("-end"))
			? d->sz : (d->sz - strlen("-end"));

		s = strstr(logbuf + offset, "-end-");

		// put back character that was
		// repalced by null character
		logbuf[d->sz + rsz] = c;

		// if end marker found, stop receiving data and
		// return indicating successful command execution.
		if (s != NULL) {
			// increase received raw data size
			d->sz += rsz;
			break;
		}

		// increase received raw data size
		d->sz += rsz;
	}

	return 0;
}

/*
* Parse, CRC check and store log record that has format:
	[crc] [number] [vals]
* @param logbuf raw data buffer
* @param logbufoffset raw data buffer offset
* @param records output array that stores log records sorted by
	their number
* @return always 0
*/
static int parserecords(char *logbuf, size_t logbufoffset, int *records)
{
	char *b, *e;

	// start from raw data buffer with offset
	e = b = logbuf + logbufoffset;

	// all records separated by newline character,
	// so iterate through raw data buffer by them
	while ((e = strchr(e, '\n')) != NULL) {
		char *num, *val;
		uint16_t crc;
		size_t n;
		char *endptr;

		// on every iteration assume that begin of a record is
		// position after end of a previous record and end of
		// this record is found newline character

		// if record is less than 5 characters,
		// skip it as corrupted
		if (e - b < 7)
			goto skip;

		// calculate record's CRC
		crc = crc16((uint8_t *) b + 6, e - b - 6 + 1);

		// if it is not the last record, replace newline
		// character with null character making separate
		// string of the current record.
		*e = '\0';

		// make separate string of first 6 characters of the
		// record. It contains record's CRC calculated on remote
		// side, as decimal string
		b[5] = '\0';

		// set pointer to characters after
		// CRC, it is record's number
		num = b + 6;

		// set pointer to first whitespace character after
		// record's number, it is begin of record's values
		val = strchr(num, ' ');

		// if record's values are empty, skip it as corrupted
		if (val == NULL)
			goto skip;

		// null terminate record's number and move record's
		// values pointer making it point to start of record's
		// values string
		*(val++) = '\0';

		// compare calculated CRC and CRC got from remote.
		// If they don't match, skip record as corrupted.
		if (strtol(b, &endptr, 10) != crc
				|| *b == '\0' || *endptr != '\0')
			goto skip;

		// convert record number from string to integer
		n = strtol(num, &endptr, 10);
		if (*num == '\0' || *endptr != '\0')
			goto skip;

		// if record number is correct, store current record's
		// values offset in raw data buffer to ordered records
		// output array
		records[n] = val - logbuf;

skip:
		// set begin of next record's string to first
		// character after end of current record
		b = ++e;
	}

	return 0;
}

/**
* @brief Server-side part of log download command.
* @param lsfd UDP socket for configuration connection
* @param rsi remote IP address
* @param cmd command to request log records
* @param d log records request structure
* @param logtotalsz total size of read log records
* @param recs output array that stores log records sorted by
	their number
* @return always 0
*/
static int reqlogrecords(int lsfd, const struct sockaddr_in *rsi,
	const char *cmd, struct loggetdata *d, size_t *logtotalsz,
	int *recs)
{
	// set raw data buffer offset
	// beyond it's end
	d->bufoffset = *logtotalsz;

	// send current batch command
	sendcmd(lsfd, rsi, cmd, logget, NULL, d);

	// null-terminate data got
	// from UDP socket
	d->buf[d->bufoffset + d->sz] = '\0';

	// parse raw data, got after
	// command sent earlier
	parserecords(d->buf, d->bufoffset, recs);

	// update total raw data buffer size
	*logtotalsz += d->sz + 1;

	return 0;
}

int inituart(int *lsfd, const char *path)
{
	struct termios tty;

	// open UART device file
	if ((*lsfd = open(path, O_RDWR | O_NOCTTY)) < 0) {
		fprintf(stderr, "cannot file\n"); //////
		exit(1);
	}

	// get UART device attributes
	tcgetattr(*lsfd, &tty);

	// set baudrate to 921600
	cfsetispeed(&tty, B921600);
	cfsetospeed(&tty, B921600);

	// set raw mode
	cfmakeraw(&tty);

	// set blocking for CMDMAXSZ bytes, and
	// set timeout between bytes to 0.1s
	tty.c_cc[VMIN] = CMDMAXSZ;
	tty.c_cc[VTIME] = 1;

	// set new UART device attributes
	tcsetattr(*lsfd, TCSANOW, &tty);

	return 0;
}

int initsocks(int *lsfd, struct sockaddr_in *rsi)
{
	struct timeval tv;
	struct sockaddr_in lsi;

	// create UDP socket
	if ((*lsfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		fprintf(stderr, "cannot create socket\n");
		exit(1);
	}

	// set timeout for receiving UDP packet
	tv.tv_sec = UDPTIMEOUT / 1000000;
	tv.tv_usec = UDPTIMEOUT % 1000000;
	if (setsockopt(*lsfd, SOL_SOCKET, SO_RCVTIMEO,
			&tv, sizeof(tv)) < 0) {
		fprintf(stderr, "cannot open set socket option\n");
		exit(1);
	}

	// bind UDP socket to it's address and port
	memset((char *) &lsi, 0, sizeof(lsi));

	lsi.sin_family = AF_INET;
	lsi.sin_addr.s_addr = INADDR_ANY;
	lsi.sin_port = htons(PORT);

	if (bind(*lsfd, (const struct sockaddr *) &lsi,
			sizeof(lsi)) < 0) {
		fprintf(stderr, "cannot bind socket\n");
		exit(1);
	}

	// fill remote address structure for sending
	memset((char *) rsi, 0, sizeof(*rsi));
	rsi->sin_family = AF_INET;
	rsi->sin_port = htons(PORT);

	if (inet_pton(AF_INET, ADDR, &(rsi->sin_addr.s_addr)) < 0) {
		fprintf(stderr, "cannot convert remote addr\n");
		exit(1);
	}

	return 0;
}

int sendcmd(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	int (*serverfunc)(int, const struct sockaddr_in *,
		const char *,
		void (*) (void *, const char *), void *),
		void (*outfunc) (void *, const char *), void *data)
{
	char scmd[BUFSZ];

	// calculate command's string CRC-16 value and add it as decimal
	// string to begin of the command.
	snprintf(scmd, BUFSZ, "%05u %s", crc16((uint8_t *) cmd,
		strlen(cmd)), cmd);
	scmd[BUFSZ - 1] = '\0';

	// do while serverfunc doesn't return non-negative value
	// indication success completion of command. If no serverfunc
	// passed, only one iteration is performed.
	do {
		// send command into UDP socket
		sendtochan(lsfd, scmd, strlen(scmd), MSG_CONFIRM,
			(const struct sockaddr *) rsi,
			sizeof(struct sockaddr_in));

		// if no serverfunc passed, stop loop
		if (serverfunc == NULL)
			break;

		// call serverfunc to perform server-side part of a
		// command and check command's sucessful completion.
		if (serverfunc(lsfd, rsi, scmd, outfunc, data) >= 0)
			break;
	} while (1);

	return 0;
}

int infofunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data)
{
	static int wait = MINWAIT; // wait time static variable
				   // is used to adjust wait time
				   // dinamically
	(void)(data);
	(void)(lsfd);
	(void)(rsi);
	(void)(outfunc);

	// wait command to execute: fixed waiting time
	// for some long commands, dynamic wait for fast commands
	if (strncmp(cmd + 6, "flash", strlen("flash")) == 0)
		usleep(FLASHWAIT);
	else
		usleep(wait);

	return 0;
}

int getfunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data)
{
	char out[BUFSZ + 1];
	static int wait = MINWAIT; // wait time static variable
				   // is used to adjust wait time
				   // dinamically
	char *endptr;
	socklen_t rsis;
	int rsz;

	(void)(data);

	// wait command to execute: fixed waiting time
	// for some long commands, dynamic wait for fast commands
	if (strncmp(cmd + 6, "flash", strlen("flash")) == 0)
		usleep(FLASHWAIT);
	else if (strncmp(cmd + 6, "irc", strlen("irc")) == 0
		|| strncmp(cmd + 6, "get irc", strlen("get irc")) == 0)
		usleep(IRCWAIT);
	else
		usleep(wait);

	// try to receive response from UDP socket
	rsis = sizeof(rsi);
	if ((rsz = recvfromchan(lsfd, out, BUFSZ, 0,
			(struct sockaddr *) rsi, &rsis)) <= 0) {
		
		// if no response and maximum wait time haven't
		// already reached, increase it
		wait = (wait < MAXWAIT) ? (wait * 15 / 10) : wait;

		return (-1);
	}
	else
		wait = (wait > MINWAIT) ? (wait * 10 / 15) : wait;

	// CRC-16 check output	
	out[5] = '\0';
	if (strtol(out, &endptr, 10)
			!= crc16((uint8_t *) out + 6, rsz - 6)
		|| *out == '\0' || *endptr != '\0')
		return (-1);

	// zero-terminate string got from network
	out[rsz] = '\0';

	if (outfunc != NULL)
		outfunc(data, out + 6);

	return 0;
}


int conffunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data)
{
	char out[BUFSZ + 1];
	static int wait = MINWAIT; // wait time static variable
				   // is used to adjust wait time
				   // dinamically
	socklen_t rsis;
	int rsz;

	(void)(data);

	// wait command to execute: fixed waiting time
	// for some long commands, dynamic wait for fast commands
	if (strncmp(cmd + 6, "flash", strlen("flash")) == 0)
		usleep(FLASHWAIT);
	else if (strncmp(cmd + 6, "irc", strlen("irc")) == 0
		|| strncmp(cmd + 6, "get irc", strlen("get irc")) == 0)
		usleep(IRCWAIT);
	else
		usleep(wait);

	// try to receive response from UDP socket
	rsis = sizeof(rsi);
	if ((rsz = recvfromchan(lsfd, out, BUFSZ, 0,
			(struct sockaddr *) rsi, &rsis)) <= 0) {	
		// if no response and maximum wait time haven't
		// already reached, increase it
		wait = (wait < MAXWAIT) ? (wait * 15 / 10) :  wait;

		return (-1);
	}

	// zero-terminate string got from network
	out[rsz] = '\0';

	outfunc(data, out);

	// if response string doesn't equal command string sent, then
	// command wasn't executed successfuly or response is corrupted
	if (strncmp(cmd, out, strlen(cmd)) != 0)
		return (-1);

	return 0;
}

int getlog(int lsfd, const struct sockaddr_in *rsi,
	int loadfrom, int loadto,
	char **output, size_t *outsize,
	void (*outfunc) (void *, const char *), void *data)
{
	struct loggetdata d;
	int *recs;
	char cmd[CMDMAXSZ];
	int reccount;
	int check;
	size_t logtotalsz;
	int outoffset;
	int i;

	if ((recs = malloc(sizeof(int) * loadto)) == NULL)
		return (-1);

	// initilize userdata structure for
	// server-side log download function
	d.reccnt = loadto;
	d.buf = NULL;
	d.sz = 0;
	d.maxsz = 0;
	d.bufoffset = 0;

	logtotalsz = 0;

	// set all records in ordered records array to
	// negative, marking they wasn't gotten yeat
	for (i = 0; i < loadto; ++i)
		recs[i] = -1;

	// while new log records ranges were
	// downloaded in previous interation
	check = 1;
	while (check) {
		// reset downloaded records flag
		check = 0;

		for (i = loadfrom; i < loadto; ++i) {
			int rb, re;

			// if record with this number
			// isn't downloaded yet
			if (recs[i] >= 0)
				continue;

			// starting from current record
			// check all records until first already
			// downloaded record not found.
			rb = i;
			for (re = i + 1; re < loadto; ++re) {
				if (recs[re] >= 0)
					break;
			}

			// if distance between current record and first
			// already downloaded record after it is less
			// that one go to next iteration
			if (re - rb <= 1)
				continue;

			// send current range command
			sprintf(cmd, "log rget %d %d\r\n", rb, re + 1);

			if (outfunc != NULL) {
				cmd[strlen(cmd) - 2] = '\0';
				outfunc(data, cmd);
			}

			// request log records
			reqlogrecords(lsfd, rsi, cmd,
				&d, &logtotalsz, recs);

			// set downloaded records flag
			check = 1;

			// set next record to check to
			// record after just requested range
			i = re;
		}
	}

	// while new log records were
	// downloaded in previous interation
	check = 1;
	while (check) {
		// reset downloaded records flag
		check = 0;

		// reset record batch size counter
		reccount = 0;

		// build first part of a batch load command
		sprintf(cmd, "log bget");

		// for every log record's number
		for (i = loadfrom; i < loadto; ++i) {
			// if record with this number
			// isn't downloaded yet
			if (recs[i] < 0) {
				// add this number to batch download
				// command
				snprintf(cmd + strlen(cmd),
					CMDMAXSZ - strlen(cmd), " %d", i);

				// set downloaded records flag
				check = 1;

				// increase record batch size counter
				++reccount;
			}

			// if current batch size is enough or it is a
			// last record and there is at least one record
			// number in batch
			if (reccount == BATCHSIZE || (i == (loadto - 1)
					&& reccount != 0)) {
				if (outfunc != NULL)
					outfunc(data, cmd);

				// add newline character
				// to current batch command
				sprintf(cmd + strlen(cmd), "\n");

				// request log records
				reqlogrecords(lsfd, rsi, cmd,
					&d, &logtotalsz, recs);

				// build first part of a
				// next batch load command
				sprintf(cmd, "log bget");

				// reset record batch size counter
				reccount = 0;
			}
		}
	}

	// write all log records in their order
	*outsize = 0;
	*output = NULL;
	outoffset = 0;
	for (i = loadfrom; i < loadto; ++i) {
		if (outoffset + strlen(d.buf + recs[i]) + 16 > *outsize) {
			*outsize = (outoffset + strlen(d.buf + recs[i]) + 16) * 2;
			*output = realloc(*output, *outsize);
		}

		outoffset += sprintf(*output + outoffset, "%d %s\n", i, d.buf + recs[i]);
	}

	// free raw data buffer
	free(d.buf);

	return 0;
}

int recvoutput(int lsfd, const struct sockaddr_in *rsi, char *buf)
{
	fd_set rfds;
	struct timeval timeout;

	FD_ZERO(&rfds);
	FD_SET(lsfd, &rfds);

	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	if (select(lsfd + 1, &rfds, NULL, NULL, &timeout) <= 0)
		return (-1);

	if (FD_ISSET(lsfd, &rfds)) {
		socklen_t rsis;
		int rsz;

		rsis = sizeof(rsi);
		if ((rsz = recvfromchan(lsfd, buf, BUFSZ, 0,
			(struct sockaddr *) rsi, &rsis)) <= 0) {
			return (-1);
		}

		buf[rsz] = '\0';
		
		return 0;
	}

	return 1;
}
