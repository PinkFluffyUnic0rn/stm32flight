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

#define PORT 3333
#define ADDR "192.168.3.1"
#define BUFSZ 1024
#define UDPTIMEOUT 200000

#define MINWAIT 5000
#define MAXWAIT 500000
#define CMDMAXSZ 60
#define LOGSIZE (1024 * 64)
#define BATCHSIZE 7

struct loggetdata {
	int reccnt;
	char *buf;
	size_t bufoffset;
	size_t sz;
	size_t maxsz;
};

// Init configuration connection sockets
//
// lsfd -- UDP socket for configuration connection.
// rsi -- quadcopter's esp07 IP address.
int initsocks(int *lsfd, struct sockaddr_in *rsi)
{
	struct timeval tv;
	struct sockaddr_in lsi;

	// create and bind local socket for receiving
	if ((*lsfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		fprintf(stderr, "cannot create socket\n");
		exit(1);
	}

	memset((char *) &lsi, 0, sizeof(lsi));

	lsi.sin_family = AF_INET;
	lsi.sin_addr.s_addr = INADDR_ANY;
	lsi.sin_port = htons(PORT);

	tv.tv_sec = 0;
	tv.tv_usec = UDPTIMEOUT;
	if (setsockopt(*lsfd, SOL_SOCKET, SO_RCVTIMEO,
			&tv, sizeof(tv)) < 0) {
		fprintf(stderr, "cannot open set socket option\n");
		exit(1);
	}

	if (bind(*lsfd, (const struct sockaddr *) &lsi,
			sizeof(lsi)) < 0) {
		fprintf(stderr, "cannot bind socket\n");
		exit(1);
	}

	// create remote address for sending
	memset((char *) rsi, 0, sizeof(*rsi));
	rsi->sin_family = AF_INET;
	rsi->sin_port = htons(PORT);
	
	if (inet_pton(AF_INET, ADDR, &(rsi->sin_addr.s_addr)) < 0) {
		fprintf(stderr, "cannot convert remote addr\n");
		exit(1);
	}

	return 0;
}

// Send comand into configuration connection.
//
// lsfd -- UDP socket for configuration connection.
// rsi -- quadcopter's esp07 IP address.
// fmt, ... -- format line and arguments, like in printf function.
int sendcmd(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	int (*serverfunc)(int, const struct sockaddr_in *,
		const char *, void *), void *data)
{
	char scmd[BUFSZ];

	snprintf(scmd, BUFSZ, "%03hu %s", crc8((uint8_t *) cmd,
		strlen(cmd)), cmd);
	scmd[BUFSZ - 1] = '\0';

	do {
		sendto(lsfd, scmd, strlen(scmd), MSG_CONFIRM,
			(const struct sockaddr *) rsi,
			sizeof(struct sockaddr_in));

		if (serverfunc == NULL)
			break;

		if (serverfunc(lsfd, rsi, scmd, data) >= 0)
			break;
	} while (1);

	return 0;
}

int conffunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void *data)
{
	char out[BUFSZ + 1];
	static int wait = MINWAIT;
	socklen_t rsis;
	int rsz;

	usleep(wait);
	
	rsis = sizeof(rsi);

	if ((rsz = recvfrom(lsfd, out, BUFSZ, 0,
			(struct sockaddr *) &rsi, &rsis)) < 0) {
	
		wait = (wait < MAXWAIT) ? wait * 1.5 : wait;

		return (-1);
	}

	out[rsz] = '\0';
	printf("%s", out);

	if (strncmp(cmd, out, strlen(cmd)) != 0)
		return (-1);

	return 0;
}

int logget(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void *data)
{
	struct loggetdata *d;
	char *logbuf;
	int i;

	d = data;
		
	// read requested log lines through UDP
	d->sz = 0;
	for (i = 0; i < d->reccnt; ++i) {
		socklen_t rsis;
		size_t offset;
		char c;
		char *s;
		int rsz;

		if (d->sz + d->bufoffset + BUFSZ >= d->maxsz) {
			d->maxsz = (d->bufoffset + d->sz + BUFSZ) * 2;
			d->buf = realloc(d->buf, d->maxsz);
		}
	
		logbuf = d->buf + d->bufoffset;

		rsis = sizeof(rsi);
		if ((rsz = recvfrom(lsfd, logbuf + d->sz,
				BUFSZ, 0, (struct sockaddr *) &rsi,
				&rsis)) < 0) {
			break;
		}

		c = logbuf[d->sz + rsz];

		logbuf[d->sz + rsz] = '\0';

		offset = (d->sz < strlen("-end"))
			? d->sz : (d->sz - strlen("-end"));

		s = strstr(logbuf + offset, "-end-");
		
		logbuf[d->sz + rsz] = c;

		if (s != NULL) {
			d->sz += rsz;
			break;	
		}	

		d->sz += rsz;
	}

	return 0;
}

int parserecords(char *logbuf, size_t logbufoffset, int *records)
{	
	char *b, *e;
	
	e = b = logbuf + logbufoffset;
	while ((e = strchr(e, '\n')) != NULL) {
		char *num, *val;
		uint8_t crc;
		size_t n;

		if (e - b < 5)
			goto skip;

		crc = crc8((uint8_t *) b + 4, e - b - 4 + 1);

		if (*e != '\0')
			*(e++) = '\0';

		b[3] = '\0';
		num = b + 4;

		val = strchr(num, ' ');

		if (val == NULL)
			goto skip;

		*(val++) = '\0';

		if (atoi(b) != crc)
			goto skip;

		n = atoi(num);

		if (n >= 0 && n < LOGSIZE)
			records[n] = val - logbuf;

skip:
		b = e;
	}

	return 0;
}

int startlog(int lsfd, const struct sockaddr_in *rsi)
{
	char cmd[CMDMAXSZ];
	
	sprintf(cmd, "log set %d\r\n", LOGSIZE);

	sendcmd(lsfd, rsi, cmd, NULL, NULL);

	return 0;
}

int getlog(int lsfd, const struct sockaddr_in *rsi)
{
	struct loggetdata d;
	int recs[LOGSIZE];
	char cmd[CMDMAXSZ];
	int reccount;
	int check;
	size_t logtotalsz;
	int i;

	d.reccnt = LOGSIZE;
	d.buf = NULL;
	d.sz = 0;
	d.maxsz = 0;
	d.bufoffset = 0;

	sprintf(cmd, "log rget 0 %d\r\n", LOGSIZE);
	sendcmd(lsfd, rsi, cmd, logget, &d);
	d.buf[d.bufoffset + d.sz] = '\0';
	
	logtotalsz = d.sz + 1;

	for (i = 0; i < LOGSIZE; ++i)
		recs[i] = -1;
	
	parserecords(d.buf, d.bufoffset, recs);

	check = 1;
	while (check) {
		check = 0;
		reccount = 0;

		sprintf(cmd, "log bget");

		for (i = 0; i < LOGSIZE; ++i) {
			if (recs[i] < 0) {
				snprintf(cmd + strlen(cmd),
					CMDMAXSZ, " %d", i);
				check = 1;
				++reccount;
			}

			if (reccount == BATCHSIZE || (i == (LOGSIZE - 1)
					&& check)) {
				d.bufoffset = logtotalsz;

				sprintf(cmd + strlen(cmd), "\n");

				sendcmd(lsfd, rsi, cmd, logget, &d);
				d.buf[d.bufoffset + d.sz] = '\0';

				parserecords(d.buf, d.bufoffset, recs);
				
				logtotalsz += d.sz + 1;

				sprintf(cmd, "log bget");
				reccount = 0;
			}
		}
	}

	for (i = 0; i < LOGSIZE; ++i) {
		if (recs[i] < 0) {
			printf("NULL\n");
			continue;
		}

		printf("%d %s\n", i, d.buf + recs[i]);
	}

	fflush(stdout);

	free(d.buf);

	return 0;
}

// Handle key press event.
//
// event -- key press event.
// lsfd -- UDP socket for configuration connection.
// rsi -- quadcopter's esp8285 IP address.
int handlecmd(const char *cmd, int lsfd, const struct sockaddr_in *rsi)
{
	if (strncmp(cmd, "exit", strlen("exit")) == 0)
		return 1;
	if (strncmp(cmd, "m", strlen("m")) == 0)
		sendcmd(lsfd, rsi, "info mpu\n", NULL, NULL);
	else if (strncmp(cmd, "p", strlen("p")) == 0)
		sendcmd(lsfd, rsi, "info pid\n", NULL, NULL);
	else if (strncmp(cmd, "v", strlen("v")) == 0)
		sendcmd(lsfd, rsi, "info values\n", NULL, NULL);
	else if (strncmp(cmd, "h", strlen("h")) == 0)
		sendcmd(lsfd, rsi, "info qmc\n", NULL, NULL);
	else if (strncmp(cmd, "b", strlen("b")) == 0)
		sendcmd(lsfd, rsi, "info hp\n", NULL, NULL);
	else if (strncmp(cmd, "g", strlen("g")) == 0)
		sendcmd(lsfd, rsi, "info gnss\n", NULL, NULL);
	else if (strncmp(cmd, "d", strlen("d")) == 0)
		sendcmd(lsfd, rsi, "info dev\n", NULL, NULL);
	else if (strncmp(cmd, "c", strlen("c")) == 0)
		sendcmd(lsfd, rsi, "info ctrl\n", NULL, NULL);
	else if (strncmp(cmd, "f", strlen("f")) == 0)
		sendcmd(lsfd, rsi, "info filter\n", NULL, NULL);
	else if (strncmp(cmd, "w", strlen("w")) == 0)
		startlog(lsfd, rsi);
	else if (strncmp(cmd, "s", strlen("s")) == 0)
		sendcmd(lsfd, rsi, "log set 0\n", NULL, NULL);
	else if (strncmp(cmd, "r", strlen("r")) == 0) 
		getlog(lsfd, rsi);
	else if (strncmp(cmd, "c", strlen("c")) == 0)
		sendcmd(lsfd, rsi, "c 0.0\n", conffunc, NULL);
	else
		sendcmd(lsfd, rsi, cmd, NULL, NULL);

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

	// if has a command line argument, read configuration file,
	// which path is stored in this argument
	if (argc > 1) {
		FILE *f;
		char s[CMDMAXSZ];
		if ((f = fopen(argv[1], "r")) == NULL) {
			fprintf(stderr, "cannot open file %s\n",
				argv[1]);
			exit(1);
		}

		while (fgets(s, CMDMAXSZ, f) != NULL) {
			sendcmd(lsfd, &rsi, s, conffunc, NULL);
		}
	}

	buf = NULL;
	bufsz = 0;
	while (1) {
		fd_set rfds;

		FD_ZERO(&rfds);
		FD_SET(0, &rfds);
		FD_SET(lsfd, &rfds);

		if (select(lsfd + 1, &rfds, NULL, NULL, NULL) <= 0)
			continue;
		
		if (FD_ISSET(0, &rfds)) {
			getline(&buf, &bufsz, stdin);

			if (handlecmd(buf, lsfd, &rsi) != 0)
				break;
		}
		
		if (FD_ISSET(lsfd, &rfds)) {
			char buf[BUFSZ + 1];
			socklen_t rsis;
			int rsz;

			rsis = sizeof(rsi);

			// if got data on configuration
			// connection print it to stdout
			if ((rsz = recvfrom(lsfd, buf, BUFSZ, 0,
				(struct sockaddr *) &rsi, &rsis)) > 0) {
				buf[rsz] = '\0';
				printf("%s", buf);
			}
		}
	}

	free(buf);

	return 0;
}
