/**
* @file uavconf.h
* @brief API to access UAV debug connection
*/

#ifndef UAVCONF_H
#define UAVCONF_H

#define BUFSZ 1024

/**
* @brief Init configuration connection sockets.

* @param lsfd UART file descriptor for configuration connection
* @param rsi UART device file path
* @return always 0
*/
int inituart(int *lsfd, const char *path);

/**
* Init configuration connection sockets.
*
* @param lsfd UDP socket for configuration connection
* @param rsi UAV's ESP32 chip IP address
* @return always 0
*/
int initsocks(int *lsfd, struct sockaddr_in *rsi);

/**
* @brief Server-side part of configuration command got from UAV
	configuration file passed to this program as argument.
*
* @param lsfd UDP socket for configuration connection
* @param rsi remote IP address, or NULL, if using UART
* @param cmd command was sent
* @param outfunc function used to process command output
* @param data userdata passed to sendcmd
* @return always 0
*/
int infofunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data);

/**
* @brief Server-side part of configuration command got from UAV
	configuration file passed to this program as argument.
*
* @param lsfd UDP socket for configuration connection
* @param rsi remote IP address, or NULL, if using UART
* @param cmd command was sent
* @param outfunc function used to process command output
* @param data userdata passed to sendcmd
* @return -1 on error, 0 otherwise
*/
int getfunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data);

/**
* @brief Server-side part of configuration command got from UAV
	configuration file passed to this program as argument.
*
* @param lsfd UDP socket for configuration connection
* @param rsi remote IP address, or NULL, if using UART
* @param cmd command was sent
* @param outfunc function used to process command output
* @param data userdata passed to sendcmd
* @return -1 on error, 0 otherwise
*/
int conffunc(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	void (*outfunc) (void *, const char *), void *data);

/**
* @brief Send command into configuration connection.
*
* @param lsfd UDP socket for configuration connection
* @param rsi remote IP address, or NULL, if using UART
* @param cmd command to send
* @param serverfunc function that needs to be called on server-side
	after command was sent
* @param data userdata passed to serverfunc as it's fourth argument
* @return always 0
*/
int sendcmd(int lsfd, const struct sockaddr_in *rsi, const char *cmd,
	int (*serverfunc)(int, const struct sockaddr_in *,
		const char *, void (*) (void *, const char *), void *),
		void (*outfunc) (void *, const char *), void *data);

/**
* @brief Download whole log from UDP socket.
*
* @param lsfd file for configuration connection
* @param rsi remote IP address, or NULL, if using UART
* @param loadfrom starting log record
* @param loadto ending log record + 1
* @param output buffer where to store pointer to output data
* @param outsize size of output data
* @param outfunc function, that takes performed one UAV command and
	user data as arguments and can perform action on it
* @param data user data to pass into outfunc
* @return always 0
*/
int getlog(int lsfd, const struct sockaddr_in *rsi,
	int loadfrom, int loadto, char **output, size_t *outsize,
	void (*outfunc) (void *, const char *), void *data);

/**
* @brief Check if new pending on configuration connection
* file and return it if it does.
*
* @param lsfd file for configuration connection
* @param rsi remote IP address, or NULL, if using UART
* @param output buffer, should be as least BUFSZ size
* @return 0, if has data, 1 otherwise
*/
int recvoutput(int lsfd, const struct sockaddr_in *rsi, char *buf);

#endif
