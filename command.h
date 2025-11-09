/**
* @file command.h
* @brief Configuration commands processing
*/

#ifndef COMMAND_H
#define COMMAND_H

#include "device.h"

/**
* @brief debug commands maximum count
*/
#define MAX_COMMANDS 32

/**
* @brief debug commands maximum token count
*/
#define MAX_CMDTOKS 12

/**
* @brief Add debug command with it's handler.
* @param name command name
* @param func command handler
* @return always 0
*/
int addcommand(const char *name,
	int (*func)(const struct cdevice *, const char **, char *));

/**
* @brief Parse and execute control command got
* from debug wifi-connection.
* @param cmd command to be executed
* @return -1 on error, 0 otherwise
*/
int runcommand(const struct cdevice *d, char *cmd);

#endif
