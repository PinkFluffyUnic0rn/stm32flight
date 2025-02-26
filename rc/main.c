#include <SDL2/SDL.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdarg.h>
#include <unistd.h>
#include <limits.h>
#include <libgen.h>

#define LOCAL_PORT 8880
#define REMOTE_PORT 8880
#define REMOTE_ADDR "192.168.3.1"

#define BUFSZ 1024

SDL_Surface *screen;
SDL_Renderer *render;

// Init configuration connection sockets
//
// lsfd -- UDP socket for configuration connection.
// rsi -- quadcopter's esp07 IP address.
int initsocks(int *lsfd, struct sockaddr_in *rsi)
{
	struct sockaddr_in lsi;
	int flags;

	// create and bind local socket for receiving
	if ((*lsfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		fprintf(stderr, "cannot create local socket\n");
		exit(1);
	}

	memset((char *) &lsi, 0, sizeof(lsi));

	lsi.sin_family = AF_INET;
	lsi.sin_addr.s_addr = INADDR_ANY;
	lsi.sin_port = htons(LOCAL_PORT);

	if ((flags = fcntl(*lsfd, F_GETFL, 0)) < 0) {
		fprintf(stderr, "cannot open get local socket flags\n");
		exit(1);
	}

	fcntl(*lsfd, F_SETFL, flags | O_NONBLOCK, 0);

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
	const char *fmt, ...)
{
	char buf[BUFSZ];
	va_list va;

	va_start(va, fmt);

	vsprintf(buf, fmt, va);

	sendto(lsfd, buf, strlen(buf), MSG_CONFIRM,
		(const struct sockaddr *) rsi,
		sizeof(struct sockaddr_in));
	
	va_end(va);

	return 0;
}

// Handle key press event.
//
// event -- key press event.
// lsfd -- UDP socket for configuration connection.
// rsi -- quadcopter's esp07 IP address.
int handlekeys(SDL_Event *event, int lsfd, const struct sockaddr_in *rsi)
{
	if (event->type == SDL_KEYUP) {
		if (event->key.keysym.sym == SDLK_m)
			sendcmd(lsfd, rsi, "info mpu\n");
		else if (event->key.keysym.sym == SDLK_p)
			sendcmd(lsfd, rsi, "info pid\n");
		else if (event->key.keysym.sym == SDLK_v)
			sendcmd(lsfd, rsi, "info values\n");
		else if (event->key.keysym.sym == SDLK_h)
			sendcmd(lsfd, rsi, "info qmc\n");
		else if (event->key.keysym.sym == SDLK_b)
			sendcmd(lsfd, rsi, "info hp\n");
		else if (event->key.keysym.sym == SDLK_w)
			sendcmd(lsfd, rsi, "log set 65536\n");
		else if (event->key.keysym.sym == SDLK_s)
			sendcmd(lsfd, rsi, "log set 0\n");
		else if (event->key.keysym.sym == SDLK_g)
			sendcmd(lsfd, rsi, "log get 65536\n");
		else if (event->key.keysym.sym == SDLK_SPACE)
			sendcmd(lsfd, rsi, "c 0.0\n");
	}

	return 0;
}

// Entry point.
int main(int argc, char *argv[])
{
	SDL_Window *win;
	struct sockaddr_in rsi;
	int lsfd;

	// initilize connection sockets
	initsocks(&lsfd, &rsi);

	// create SDL2 window
	if ((win = SDL_CreateWindow("RC", SDL_WINDOWPOS_UNDEFINED,
		SDL_WINDOWPOS_UNDEFINED, 1000, 500, 0)) == NULL) {
		fprintf(stderr, "cannot create window\n");
		exit(1);
	}

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
			sendcmd(lsfd, &rsi, s);
			usleep(10000);
		}
	}

	// show SDL2 window
	screen = SDL_GetWindowSurface(win);
	render = SDL_GetRenderer(win);

	while (1) {
		SDL_Event event;
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

		// process key presses
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT)
				exit(0);
		
			handlekeys(&event, lsfd, &rsi); 
		}

		SDL_RenderPresent(render);
	}

	SDL_DestroyWindow(win);
	SDL_Quit();
	return 0;
}
