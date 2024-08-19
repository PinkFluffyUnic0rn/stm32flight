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
//#define REMOTE_ADDR "192.168.1.39"
#define REMOTE_ADDR "192.168.1.49"

#define BUFSZ 1024

SDL_Surface *screen;
SDL_Renderer *render;

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

struct timerparam {
	struct sockaddr_in *rsi;
	int lsfd;
};

Uint32 timercb(Uint32 interval, void *param)
{
	struct timerparam *p;

	p = param;

	sendcmd(p->lsfd, p->rsi, "beep\n");

	return interval;
}

int handlekeys(SDL_Event *event, int lsfd, const struct sockaddr_in *rsi)
{
	if (event->type == SDL_KEYUP) {
		if (event->key.keysym.sym == SDLK_q) {}
		if (event->key.keysym.sym == SDLK_w) {}
		if (event->key.keysym.sym == SDLK_a) {}
		if (event->key.keysym.sym == SDLK_s) {}
		if (event->key.keysym.sym == SDLK_z) {}
		if (event->key.keysym.sym == SDLK_x) {}
		if (event->key.keysym.sym == SDLK_e) {}
		if (event->key.keysym.sym == SDLK_r) {}
		if (event->key.keysym.sym == SDLK_d) {}
		if (event->key.keysym.sym == SDLK_f) {}
		if (event->key.keysym.sym == SDLK_c) {}
		if (event->key.keysym.sym == SDLK_v) {}
		if (event->key.keysym.sym == SDLK_t) {}
		if (event->key.keysym.sym == SDLK_y) {}
		if (event->key.keysym.sym == SDLK_g) {}
		if (event->key.keysym.sym == SDLK_h) {}
		if (event->key.keysym.sym == SDLK_b) {}
		if (event->key.keysym.sym == SDLK_n) {}
		if (event->key.keysym.sym == SDLK_1) {}
		if (event->key.keysym.sym == SDLK_2) {}
		if (event->key.keysym.sym == SDLK_SPACE) {
			sendcmd(lsfd, rsi, "c 0.0\n");
		}
	}

	return 0;
}

#define AXISSCALE (M_PI / 6.0)

int handlepad(SDL_Event *event, int lsfd, const struct sockaddr_in *rsi)
{
	if (event->type == SDL_CONTROLLERBUTTONUP) {
		if (event->cbutton.button == SDL_CONTROLLER_BUTTON_A)
			sendcmd(lsfd, rsi, "md\n");
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_B)
			sendcmd(lsfd, rsi, "pd\n");
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_X)
			sendcmd(lsfd, rsi, "vd\n");
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_Y)
			sendcmd(lsfd, rsi, "hd\n");
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_LEFTSHOULDER)
			sendcmd(lsfd, rsi, "r\n");
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_RIGHTSHOULDER)
			sendcmd(lsfd, rsi, "e\n");
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_UP) {
		}
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_DOWN) {
		}
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_LEFT) {
		}
		else if (event->cbutton.button == SDL_CONTROLLER_BUTTON_DPAD_RIGHT) {
		}
	}
	else if (event->type == SDL_CONTROLLERAXISMOTION) {
		if (event->caxis.axis == SDL_CONTROLLER_AXIS_TRIGGERRIGHT)
			sendcmd(lsfd, rsi, "yt %f\n", event->caxis.value / 32767.0 * M_PI);
		else if (event->caxis.axis == SDL_CONTROLLER_AXIS_TRIGGERLEFT)
			sendcmd(lsfd, rsi, "yt %f\n", -event->caxis.value / 32767.0 * M_PI);
		else if (event->caxis.axis == SDL_CONTROLLER_AXIS_LEFTX)
			sendcmd(lsfd, rsi, "pt %f\n", -event->caxis.value / 32767.0 * AXISSCALE);
		else if (event->caxis.axis == SDL_CONTROLLER_AXIS_LEFTY)
			sendcmd(lsfd, rsi, "rt %f\n", event->caxis.value / 32767.0 * AXISSCALE);
		else if (event->caxis.axis == SDL_CONTROLLER_AXIS_RIGHTX) {
		
		} else if (event->caxis.axis == SDL_CONTROLLER_AXIS_RIGHTY)
			sendcmd(lsfd, rsi, "tt %f\n", -event->caxis.value / 32767.0 * 1.0);
	}

	return 0;
}

int main(int argc, char *argv[])
{
	SDL_Window *win;
	SDL_GameController *pad;
	SDL_TimerID tid;
	struct timerparam tp;
	struct sockaddr_in rsi;
	char gcpath[PATH_MAX];
	int lsfd;

	initsocks(&lsfd, &rsi);

	sprintf(gcpath, "%s/gamecontrollerdb.txt", dirname(argv[0]));

	if (SDL_GameControllerAddMappingsFromFile(gcpath) < 0) {
		fprintf(stderr, "cannot open controller mapping\n");
		exit(1);
	}

	if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_TIMER) < 0) {
		fprintf(stderr, "cannot init controller\n");
		exit(1);
	}

	if ((pad = SDL_GameControllerOpen(0)) == NULL) {
		fprintf(stderr, "cannot open controller\n");
		exit(1);
	}

	if ((win = SDL_CreateWindow("RC", SDL_WINDOWPOS_UNDEFINED,
		SDL_WINDOWPOS_UNDEFINED, 1000, 500, 0)) == NULL) {
		fprintf(stderr, "cannot create window\n");
		exit(1);
	}

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
			usleep(100000);
		}
	}
	

	tp.lsfd = lsfd;
	tp.rsi = &rsi;
	tid = SDL_AddTimer(1000, timercb, &tp);

	screen = SDL_GetWindowSurface(win);
	render = SDL_GetRenderer(win);

	while (1) {
		SDL_Event event;
		char buf[BUFSZ];
		socklen_t rsis;
		int rsz;

		rsis = sizeof(rsi);

		if ((rsz = recvfrom(lsfd, buf, BUFSZ, 0,
			(struct sockaddr *) &rsi, &rsis)) > 0) {
			buf[rsz] = '\0';
			printf("%s\n", buf);
		}

		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT)
				exit(0);
		
			handlepad(&event, lsfd, &rsi);
			handlekeys(&event, lsfd, &rsi); 
		}

		SDL_RenderPresent(render);
	}

	SDL_RemoveTimer(tid);
	SDL_GameControllerClose(pad);
	SDL_DestroyWindow(win);
	SDL_Quit();
	return 0;
}
