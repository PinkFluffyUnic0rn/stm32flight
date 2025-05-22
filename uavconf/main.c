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

#include "../crc.h"

// UDP connection port
#define PORT 8880

// UDP connection address
#define ADDR "192.168.3.1"

// socket send/receive buffer size
#define BUFSZ 1024

// UDP packet receive timeout
#define UDPTIMEOUT 200000

// minimum and maximum wait time for
// UAV configuration command to execute
#define MINWAIT 5000
#define MAXWAIT 1000000
#define FLASHWAIT 2000000

// maximum UAV command size
#define CMDMAXSZ 60

// log size in records
#define LOGSIZE (1024 * 64)

// maximum number of log records in
// batch when reading whole log
#define BATCHSIZE 7

// userdata structure passed to server-side
// part of log download command
struct loggetdata {
	int reccnt;		// number of log records to get
	char *buf;		// raw data buffer to hold all data
				// got from UDP socket
	size_t bufoffset;	// raw data buffer offset
	size_t maxsz;		// raw data buffer allocated size
	size_t sz;		// raw data size got after call
};

// Init configuration connection sockets.
//
// lsfd -- UDP socket for configuration connection.
// rsi -- quadcopter's esp07 IP address.
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
	tv.tv_sec = 0;
	tv.tv_usec = UDPTIMEOUT;
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

// Send command into configuration connection.
//
// lsfd -- UDP socket for configuration connection.
// rsi -- remote IP address.
// cmd -- command to send.
// serverfunc -- function that needs to be called on server-side after
// 	command was sent.
// data -- userdata passed to serverfunc as it's fourth argument.
int sendcmd(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	int (*serverfunc)(int, const struct sockaddr_in *,
		const char *, void *), void *data)
{
	char scmd[BUFSZ];

	// calculate command's string CRC-8 value and add it as decimal
	// string to begin of the command.
	snprintf(scmd, BUFSZ, "%03hu %s", crc8((uint8_t *) cmd,
		strlen(cmd)), cmd);
	scmd[BUFSZ - 1] = '\0';

	// do while serverfunc doesn't return non-negative value
	// indication success completion of command. If no serverfunc
	// passed, only one iteration is performed.
	do {
		// send command into UDP socket
		sendto(lsfd, scmd, strlen(scmd), MSG_CONFIRM,
			(const struct sockaddr *) rsi,
			sizeof(struct sockaddr_in));

		// if no serverfunc passed, stop loop
		if (serverfunc == NULL)
			break;

		// call serverfunc to perform server-side part of a
		// command and check command's sucessful completion.
		if (serverfunc(lsfd, rsi, scmd, data) >= 0)
			break;
	} while (1);

	return 0;
}

// Server-side part of configuration command got from UAV
// configuration file passed to this program as argument.
//
// lsfd -- UDP socket for configuration connection.
// rsi -- remote IP address.
// cmd -- command was sent.
// data -- userdata passed to sendcmd.
int conffunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void *data)
{
	char out[BUFSZ + 1];
	static int wait = MINWAIT; // wait time static variable
				   // is used to adjust wait time
				   // dinamically
	socklen_t rsis;
	int rsz;

	// wait command to execute: fixed waiting time
	// for some long commands, dynamic wait for fast commands
	if (strncmp(cmd + 4, "flash", strlen("flash")) == 0)
		usleep(FLASHWAIT);
	else
		usleep(wait);

	// try to receive response from UDP socket
	rsis = sizeof(rsi);
	if ((rsz = recvfrom(lsfd, out, BUFSZ, 0,
			(struct sockaddr *) &rsi, &rsis)) <= 0) {
		
		// it no response and maximum wait time haven't
		// already reached, increase it
		wait = (wait < MAXWAIT) ? (wait * 15 / 10) :  wait;

		return (-1);
	}

	// zero-terminate string got from network
	out[rsz] = '\0';
	printf("%s", out);

	// if response string doesn't equal command string sent, then
	// command wasn't executed successfuly or response is corrupted
	if (strncmp(cmd, out, strlen(cmd)) != 0)
		return (-1);

	return 0;
}

// Server-side part of log download command.
//
// lsfd -- UDP socket for configuration connection.
// rsi -- remote IP address.
// cmd -- command was sent.
// data -- userdata passed to sendcmd.
int logget(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void *data)
{
	struct loggetdata *d;
	char *logbuf;
	int i;

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
		if ((rsz = recvfrom(lsfd, logbuf + d->sz,
				BUFSZ, 0, (struct sockaddr *) &rsi,
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

// Parse, CRC check and store log record that has format:
// 	[crc] [number] [vals]
//
// 	logbuf -- raw data buffer.
// 	logbufoffset -- raw data buffer offset.
// 	records -- output array that stores log records sorted by
// 		their number.
int parserecords(char *logbuf, size_t logbufoffset, int *records)
{
	char *b, *e;

	// start from raw data buffer with offset
	e = b = logbuf + logbufoffset;

	// all records separated by newline character,
	// so iterate through raw data buffer by them
	while ((e = strchr(e, '\n')) != NULL) {
		char *num, *val;
		uint8_t crc;
		size_t n;
		char *endptr;

		// on every iteration assume that begin of a record is
		// position after end of a previous record and end of
		// this record is found newline character

		// if record is less than 5 characters,
		// skip it as corrupted
		if (e - b < 5)
			goto skip;

		// calculate record's CRC
		crc = crc8((uint8_t *) b + 4, e - b - 4 + 1);

		// if it is not the last record, replace newline
		// character with null character making separate
		// string of the current record.
		*e = '\0';

		// make separate string of first 4 characters of the
		// record. It contains record's CRC calculated on remote
		// side, as decimal string
		b[3] = '\0';

		// set pointer to characters after
		// CRC, it is record's number
		num = b + 4;

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
		if (n >= 0 && n < LOGSIZE)
			records[n] = val - logbuf;

skip:
		// set begin of next record's string to first
		// character after end of current record
		b = ++e;
	}

	return 0;
}

int reqlogrecords(int lsfd, const struct sockaddr_in *rsi,
	const char *cmd, struct loggetdata *d, size_t *logtotalsz,
	int *recs)
{
	// set raw data buffer offset
	// beyond it's end
	d->bufoffset = *logtotalsz;

	// send current batch command
	sendcmd(lsfd, rsi, cmd, logget, d);

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

// Download whole log from UDP socket.
//
// lsfd -- UDP socket for configuration connection.
// rsi -- remote IP address.
int getlog(int lsfd, const struct sockaddr_in *rsi)
{
	struct loggetdata d;
	int recs[LOGSIZE];
	char cmd[CMDMAXSZ];
	int reccount;
	int check;
	size_t logtotalsz;
	int i;

	// initilize userdata structure for
	// server-side log download function
	d.reccnt = LOGSIZE;
	d.buf = NULL;
	d.sz = 0;
	d.maxsz = 0;
	d.bufoffset = 0;

	logtotalsz = 0;

	// set all records in ordered records array to
	// negative, marking they wasn't gotten yeat
	for (i = 0; i < LOGSIZE; ++i)
		recs[i] = -1;

	// while new log records ranges were
	// downloaded in previous interation
	check = 1;
	while (check) {
		// reset downloaded records flag
		check = 0;

		for (i = 0; i < LOGSIZE; ++i) {
			int rb, re;

			// if record with this number
			// isn't downloaded yet
			if (recs[i] >= 0)
				continue;

			// starting from current record
			// check all records until first already
			// downloaded record not found.
			rb = i;
			for (re = i + 1; re < LOGSIZE; ++re) {
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
		for (i = 0; i < LOGSIZE; ++i) {
			// if record with this number
			// isn't downloaded yet
			if (recs[i] < 0) {
				// add this number to batch download
				// command
				snprintf(cmd + strlen(cmd),
					CMDMAXSZ, " %d", i);

				// set downloaded records flag
				check = 1;

				// increase record batch size counter
				++reccount;
			}

			// if current batch size is enough or it is a
			// last record and there is at least one record
			// number in batch
			if (reccount == BATCHSIZE || (i == (LOGSIZE - 1)
					&& reccount != 0)) {
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
	for (i = 0; i < LOGSIZE; ++i)
		printf("%d %s\n", i, d.buf + recs[i]);

	// flush stdout to ensure everything ot written
	// when standart output is redirrected to file
	fflush(stdout);

	// free raw data buffer
	free(d.buf);

	return 0;
}

// Handle command got from terminal.
//
// event -- key press event.
// lsfd -- UDP socket for configuration connection.
// rsi -- quadcopter's esp8285 IP address.
int handlecmd(const char *cmd, int lsfd, const struct sockaddr_in *rsi)
{
	char buf[CMDMAXSZ];

	// if got one of known commands, perform corresponding actions.
	// In other case just send it into UDP socket.
	if (strcmp(cmd, "e") == 0)
		return 1;
	else if (strcmp(cmd, "m") == 0)
		sendcmd(lsfd, rsi, "info mpu\n", NULL, NULL);
	else if (strcmp(cmd, "p") == 0)
		sendcmd(lsfd, rsi, "info pid\n", NULL, NULL);
	else if (strcmp(cmd, "v") == 0)
		sendcmd(lsfd, rsi, "info values\n", NULL, NULL);
	else if (strcmp(cmd, "h") == 0)
		sendcmd(lsfd, rsi, "info qmc\n", NULL, NULL);
	else if (strcmp(cmd, "b") == 0)
		sendcmd(lsfd, rsi, "info hp\n", NULL, NULL);
	else if (strcmp(cmd, "g") == 0)
		sendcmd(lsfd, rsi, "info gnss\n", NULL, NULL);
	else if (strcmp(cmd, "d") == 0)
		sendcmd(lsfd, rsi, "info dev\n", NULL, NULL);
	else if (strcmp(cmd, "t") == 0)
		sendcmd(lsfd, rsi, "info ctrl\n", NULL, NULL);
	else if (strcmp(cmd, "f") == 0)
		sendcmd(lsfd, rsi, "info filter\n", NULL, NULL);
	else if (strcmp(cmd, "w") == 0) {
		sprintf(buf, "log set %d\r\n", LOGSIZE);
		sendcmd(lsfd, rsi, buf, NULL, NULL);
	}
	else if (strcmp(cmd, "s") == 0)
		sendcmd(lsfd, rsi, "log set 0\n", NULL, NULL);
	else if (strcmp(cmd, "r") == 0)
		getlog(lsfd, rsi);
	else if (strcmp(cmd, "c") == 0)
		sendcmd(lsfd, rsi, "c 0.0\n", NULL, NULL);
	else {
		sprintf(buf, "%s\r\n", cmd);
		sendcmd(lsfd, rsi, buf, NULL, NULL);
	}

	return 0;
}

// Entry point.
int main(int argc, char *argv[])
{
	struct sockaddr_in rsi;
	char *buf;
	size_t bufsz;
	int lsfd;

	// initilize connection sockets
	initsocks(&lsfd, &rsi);

	// if has a command line argument, apply configuration file
	if (argc > 1) {
		FILE *f;
		char s[CMDMAXSZ];

		// open configuration file
		if ((f = fopen(argv[1], "r")) == NULL) {
			fprintf(stderr, "cannot open file %s\n",
				argv[1]);
			exit(1);
		}

		// read opened file line-by-line sending
		// every line as a configuration command
		while (fgets(s, CMDMAXSZ, f) != NULL)
			sendcmd(lsfd, &rsi, s, conffunc, NULL);
	}

	// poll for standant input and UDP socket for incoming data.
	buf = NULL;
	bufsz = 0;
	while (1) {
		fd_set rfds;

		// use select for input multiplexing
		FD_ZERO(&rfds);
		FD_SET(0, &rfds);
		FD_SET(lsfd, &rfds);

		if (select(lsfd + 1, &rfds, NULL, NULL, NULL) <= 0)
			continue;

		// if got data on standart input, read
		// one line and interpret it as command
		if (FD_ISSET(0, &rfds)) {
			ssize_t l;
			l = getline(&buf, &bufsz, stdin);

			if (buf[l - 1] == '\n')
				buf[l - 1] = '\0';

			if (handlecmd(buf, lsfd, &rsi) != 0)
				break;
		}

		// uf got data on UDP socket, print
		// it to standart output.
		if (FD_ISSET(lsfd, &rfds)) {
			char buf[BUFSZ + 1];
			socklen_t rsis;
			int rsz;

			rsis = sizeof(rsi);
			if ((rsz = recvfrom(lsfd, buf, BUFSZ, 0,
				(struct sockaddr *) &rsi, &rsis)) > 0) {
				buf[rsz] = '\0';
				printf("%s", buf);
			}
		}
	}

	// free buffer used by getline
	free(buf);

	return 0;
}
