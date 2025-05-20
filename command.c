#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "command.h"
#include "device.h"
#include "global.h"
#include "crc.h"

// debug command handler structure
struct command {
	const char *name;
	int (*func)(const struct cdevice*, const char **, char *);
};

// Debug commands handlers
static struct command commtable[MAX_COMMANDS];
static size_t commcount;

// Parse configureation command got from debug wi-fi connection
// by just splitting it into tokens by spaces. Last token is
// the terminating token and always is an empty string.
//
// toks -- result array of command tokens (it's a paring result).
// maxtoks -- maximum number of tokens posible.
// data -- command to be parsed.
static int parsecommand(char **toks, int maxtoks, char *data)
{
	int i;

	if (data[strlen(data) - 1] == '\n')
		data[strlen(data) - 1] = '\0';

	for (i = 0; i < maxtoks; ++i)
		toks[i] = "";

	i = 0;

	toks[i++] = strtok((char *) data, " ");

	while (i < (maxtoks - 1)
		&& (toks[i++] = strtok(NULL, " ")) != NULL);

	return (i - 1);
}

int addcommand(const char *name,
	int (*func)(const struct cdevice *, const char **, char *))
{
	commtable[commcount].name = name;
	commtable[commcount].func = func;

	++commcount;

	return 0;
}

int runcommand(const struct cdevice *d, char *cmd)
{
	char buf[CMDSIZE];
	char out[INFOLEN];
	char *toks[MAX_CMDTOKS];
	uint8_t crc;
	int toksoff;
	int i;

	if (cmd[0] == '\0')
		return 0;

	memcpy(buf, cmd, CMDSIZE);

	// split a command into tokens by spaces
	parsecommand(toks, MAX_CMDTOKS, buf);

	toksoff = 0;

	if (strncmp(d->name, "uart", strlen("uart")) != 0) {
		toksoff = 1;

		crc = crc8((uint8_t *) cmd + 4, strlen(cmd) - 4);

		if (atoi(toks[0]) != crc)
			goto crcfail;
	}

	// perform corresponding action
	for (i = 0; i < commcount; ++i) {
		if (strcmp(toks[toksoff], commtable[i].name) == 0) {
			int r;

			out[0] = '\0';

			if ((r = commtable[i].func(d,
					(const char **) toks + toksoff,
					out)) < 0)
				goto unknown;

			// r > 0, then it's a printing command and
			// it does not transmission confirm
			if (r == 0) {
				char hdr[CMDSIZE];

				memcpy(hdr, cmd, CMDSIZE);
				d->write(d->priv, hdr, strlen(hdr));
			}

			if (strlen(out) != 0)
				d->write(d->priv, out, strlen(out));

			return 0;
		}
	}

	return 0;

unknown:
	snprintf(out, INFOLEN, "Unknown command: %s\r\n", cmd + 4);
	d->write(d->priv, out, strlen(out));

	return (-1);

crcfail:
	snprintf(out, INFOLEN, "CRC check fail, got %hu: %s\r\n", crc,
		cmd + 4);

	d->write(d->priv, out, strlen(out));

	return (-1);
}
