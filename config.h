/**
* @file config.h
* @brief Configuration commands
*/

#ifndef CONFIG_H
#define CONFIG_H

#include "device.h"

/**
* @brief Disarm command handler.
* @param toks list of parsed command tokens
* @param out command's output
* @return always 0
*/
int rcmd(const struct cdevice *dev, const char **toks, char *out);

/**
* @brief Recalibrate command handler.
* @param toks list of parsed command tokens
* @param out command's output
* @return always 0
*/
int ccmd(const struct cdevice *dev, const char **toks, char *out);

/**
* @brief "info" command handler. Print various
	sensor and control values.
* @param d charter device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 1 otherwise
*/
int infocmd(const struct cdevice *d, const char **toks, char *out);

/**
* @brief "pid" command handler. Configure PID values.
* @param d charter device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int pidcmd(const struct cdevice *dev, const char **toks, char *out);

/**
* @brief "flash" command handler. Write/read MCU's
	internal flash used for storing configuraton.
* @param d charter device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int flashcmd(const struct cdevice *dev, const char **toks, char *out);

/**
* @brief "compl" command handler. Configure
	pitch/roll complimentary filters.
* @param d charter device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int complcmd(const struct cdevice *dev, const char **toks, char *out);

/**
* @brief "lpf" command handler. Configure low-pass filters.
* @param d character device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int lpfcmd(const struct cdevice *dev, const char **toks, char *out);

/**
* @brief "adj" command handler. Configure various offset/scale values.
* @param d character device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int adjcmd(const struct cdevice *dev, const char **toks, char *out);

/**
* @brief "log" command handler. Start/stop/get flight log.
* @param d character device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int logcmd(const struct cdevice *d, const char **toks, char *out);

/**
* @brief "ctrl" command handler. Configure control ranges scaling.
* @param d character device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int ctrlcmd(const struct cdevice *d, const char **toks, char *out);

/**
* @brief "system" command handler. Run system commands.
* @param d character device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int systemcmd(const struct cdevice *d, const char **toks, char *out);

/**
* @brief "irc" command handler. Configure IRC tramp video transmitter.
* @param d character device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int irccmd(const struct cdevice *d, const char **toks, char *out);

/**
* @brief "motor" command handler. Configure motors
	output number and direction.
* @param d character device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 0 otherwise
*/
int motorcmd(const struct cdevice *d, const char **toks, char *out);

/**
* @brief "get" command handler. Get current configuration values.
* @param d character device device that got this command
* @param toks list of parsed command tokens
* @param out command's output
* @return -1 on error, 1 otherwise
*/
int getcmd(const struct cdevice *d, const char **toks, char *out);

#endif
