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

#include "../api/uavconf.h"

// log size in records
#define LOGSIZE (1024 * 64)

// maximum UAV command size
#define CMDMAXSZ 60

// socket send/receive buffer size
#define BUFSZ 1024

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
		sendcmd(lsfd, rsi, "info mpu\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "p") == 0)
		sendcmd(lsfd, rsi, "info pid\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "v") == 0)
		sendcmd(lsfd, rsi, "info values\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "h") == 0)
		sendcmd(lsfd, rsi, "info qmc\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "b") == 0)
		sendcmd(lsfd, rsi, "info hp\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "g") == 0)
		sendcmd(lsfd, rsi, "info gnss\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "d") == 0)
		sendcmd(lsfd, rsi, "info dev\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "t") == 0)
		sendcmd(lsfd, rsi, "info ctrl\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "f") == 0)
		sendcmd(lsfd, rsi, "info filter\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "i") == 0)
		sendcmd(lsfd, rsi, "info irc\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "w") == 0) {
		sprintf(buf, "log set %d\n", LOGSIZE);
		sendcmd(lsfd, rsi, buf, NULL, NULL, NULL);
	}
	else if (strcmp(cmd, "s") == 0)
		sendcmd(lsfd, rsi, "log set 0\n", NULL, NULL, NULL);
	else if (strcmp(cmd, "r") == 0) {
		char *output;
		size_t outsize;
		
		getlog(lsfd, rsi, 0, LOGSIZE, &output, &outsize);

		printf("%s", output);
	}
	else if (strcmp(cmd, "c") == 0)
		sendcmd(lsfd, rsi, "c 0.0\n", NULL, NULL, NULL);
	else {
		sprintf(buf, "%s\n", cmd);
		sendcmd(lsfd, rsi, buf, NULL, NULL, NULL);
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
			sendcmd(lsfd, &rsi, s, conffunc, NULL, NULL);
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
