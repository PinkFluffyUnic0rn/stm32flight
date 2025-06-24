#ifndef UAVCONF_H
#define UAVCONF_H

#define BUFSZ 1024

int initsocks(int *lsfd, struct sockaddr_in *rsi);

int infofunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data);

int getfunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data);

int conffunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data);

int sendcmd(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	int (*serverfunc)(int, const struct sockaddr_in *,
		const char *, void (*) (void *, const char *), void *),
		void (*outfunc) (void *, const char *), void *data);

int getlog(int lsfd, const struct sockaddr_in *rsi,
	int loadfrom, int loadto, char **output, size_t *outsize);

int recvoutput(int lsfd, const struct sockaddr_in *rsi, char *buf);

#endif
