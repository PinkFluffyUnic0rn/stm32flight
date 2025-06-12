#ifndef UAVCONF_H
#define UAVCONF_H

int initsocks(int *lsfd, struct sockaddr_in *rsi);

int getfunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data);

int conffunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data);

int sendcmd(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	int (*serverfunc)(int, const struct sockaddr_in *,
		const char *, void (*) (void *, const char *), void *),
		void (*outfunc) (void *, const char *), void *data);

int getlog(int lsfd, const struct sockaddr_in *rsi);

#endif
