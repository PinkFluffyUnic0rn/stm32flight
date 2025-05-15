#include <SDL2/SDL.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <unistd.h>
#include <limits.h>
#include <libgen.h>
#include <errno.h>

#define LOCAL_PORT 3333
#define REMOTE_PORT 3333
#define REMOTE_ADDR "192.168.3.1"

#define BUFSZ 1024

SDL_Surface *screen;
SDL_Renderer *render;

static const uint8_t crsf_crc8tbl [] = {
	0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 
	0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d, 
	0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 
	0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f, 
	0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 
	0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9, 
	0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 
	0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b, 
	0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 
	0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0, 
	0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 
	0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2, 
	0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 
	0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44, 
	0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 
	0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16, 
	0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 
	0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92, 
	0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 
	0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0, 
	0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 
	0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36, 
	0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 
	0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64, 
	0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 
	0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f, 
	0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 
	0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d, 
	0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 
	0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab, 
	0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 
	0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9
};

uint8_t crc8(const uint8_t *data, uint8_t len)
{
	uint8_t crc;
	int i;

	crc = 0x00;
	for (i = 0; i < len; ++i)
		crc = crsf_crc8tbl[crc ^ *data++];

	return crc;
}

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
		fprintf(stderr, "cannot create local socket\n");
		exit(1);
	}

	memset((char *) &lsi, 0, sizeof(lsi));

	lsi.sin_family = AF_INET;
	lsi.sin_addr.s_addr = INADDR_ANY;
	lsi.sin_port = htons(LOCAL_PORT);

	tv.tv_sec = 1;
	tv.tv_usec = 0;
	if (setsockopt(*lsfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
		fprintf(stderr, "cannot open set local socket option\n");
		exit(1);
	}


	if (bind(*lsfd, (const struct sockaddr *) &lsi, sizeof(lsi)) < 0) {
		fprintf(stderr, "cannot bind local socket\n");
		exit(1);
	}

	// create remote address for sending
	memset((char *) rsi, 0, sizeof(*rsi));
	rsi->sin_family = AF_INET;
	rsi->sin_port = htons(REMOTE_PORT);
	
	if (inet_pton(AF_INET, REMOTE_ADDR, &(rsi->sin_addr.s_addr)) < 0) {
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
int sendcmd(int lsfd, const struct sockaddr_in *rsi,
	const char *cmd , int (*serverfunc)(const char *))
{
	char scmd[BUFSZ];
	char out[BUFSZ];
	socklen_t rsis;
	int rsz;

	sprintf(scmd, "%03hu %s", crc8((uint8_t *) cmd,
		strlen(cmd)), cmd);

	do {
		sendto(lsfd, scmd, strlen(scmd), MSG_CONFIRM,
			(const struct sockaddr *) rsi,
			sizeof(struct sockaddr_in));

		serverfunc(scmd);

		rsis = sizeof(rsi);

		if ((rsz = recvfrom(lsfd, out, BUFSZ, 0,
			(struct sockaddr *) &rsi, &rsis)) > 0) {
			out[rsz] = '\0';
			printf("%s", out);
		
			if (strncmp(scmd, out, strlen(scmd)) == 0)
				break;
		}
		else if (rsz < 0)
			printf("error: %s; %d\r\n", strerror(errno), errno);
	} while (1);

	return 0;
}

int waitfunc(const char *cmd)
{
//	printf("in serverside function! |%s|\r\n", cmd);
	usleep(1000);

	return 0;
}

// Handle key press event.
//
// event -- key press event.
// lsfd -- UDP socket for configuration connection.
// rsi -- quadcopter's esp07 IP address.
int handlekeys(const char *cmd, int lsfd, const struct sockaddr_in *rsi)
{
	if (strncmp(cmd, "m", strlen("m")) == 0)
		sendcmd(lsfd, rsi, "info mpu\n", waitfunc);
	else if (strncmp(cmd, "p", strlen("p")) == 0)
		sendcmd(lsfd, rsi, "info pid\n", waitfunc);
	else if (strncmp(cmd, "v", strlen("v")) == 0)
		sendcmd(lsfd, rsi, "info values\n", waitfunc);
	else if (strncmp(cmd, "h", strlen("h")) == 0)
		sendcmd(lsfd, rsi, "info qmc\n", waitfunc);
	else if (strncmp(cmd, "b", strlen("b")) == 0)
		sendcmd(lsfd, rsi, "info hp\n", waitfunc);
	else if (strncmp(cmd, "g", strlen("g")) == 0)
		sendcmd(lsfd, rsi, "info gnss\n", waitfunc);
	else if (strncmp(cmd, "d", strlen("d")) == 0)
		sendcmd(lsfd, rsi, "info dev\n", waitfunc);
	else if (strncmp(cmd, "c", strlen("c")) == 0)
		sendcmd(lsfd, rsi, "info ctrl\n", waitfunc);
	else if (strncmp(cmd, "f", strlen("f")) == 0)
		sendcmd(lsfd, rsi, "info filter\n", waitfunc);
	else if (strncmp(cmd, "w", strlen("w")) == 0)
		sendcmd(lsfd, rsi, "log set 4194304\n", waitfunc);
	else if (strncmp(cmd, "s", strlen("s")) == 0)
		sendcmd(lsfd, rsi, "log set 0\n", waitfunc);
	else if (strncmp(cmd, "r", strlen("r")) == 0)
		sendcmd(lsfd, rsi, "log get 4194304\n", waitfunc);
	else if (strncmp(cmd, "c", strlen("c")) == 0)
		sendcmd(lsfd, rsi, "c 0.0\n", waitfunc);

	return 0;
}

// Entry point.
int main(int argc, char *argv[])
{
	struct sockaddr_in rsi;
	int lsfd;

	// initilize connection sockets
	initsocks(&lsfd, &rsi);

	// if has a command line argument, read configuration file,
	// which path is stored in this argument
	if (argc > 1) {
		FILE *f;
		char s[256];
		if ((f = fopen(argv[1], "r")) == NULL) {
			fprintf(stderr, "cannot open file %s\n",
				argv[1]);
			exit(1);
		}

		while (fgets(s, 256, f) != NULL) {
			sendcmd(lsfd, &rsi, s, waitfunc);
		}
	}

	while (1) {
		fd_set rfds;

		FD_ZERO(&rfds);
		FD_SET(0, &rfds);
		FD_SET(lsfd, &rfds);

		if (select(lsfd + 1, &rfds, NULL, NULL, NULL) > 0) {
			if (FD_ISSET(0, &rfds)) {
				char buf[BUFSZ];
				
				scanf("%s", buf);

				handlekeys(buf, lsfd, &rsi);
			}
			
			if (FD_ISSET(lsfd, &rfds)) {
				char buf[BUFSZ];
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
	}

	return 0;
}
