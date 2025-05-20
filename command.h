#ifndef COMMAND_H
#define COMMAND_H

#include "device.h"

// debug commands maximum count
#define MAX_COMMANDS 32

// debug commands maximum token count
#define MAX_CMDTOKS 12

// Add debug command with it's handler.
//
// name -- command name
// func -- command handler
int addcommand(const char *name,
	int (*func)(const struct cdevice *, const char **, char *));

// Parse and execute control command got from debug wifi-connection.
//
// cmd -- command to be executed.
int runcommand(const struct cdevice *d, char *cmd);

#endif
