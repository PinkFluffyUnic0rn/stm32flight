#ifndef DEVICE_H
#define DEVICE_H

#include <stdint.h>
#include <stddef.h>

#define DEVNAMEMAX 32

enum DEVSTATUS {
	DEVSTATUS_INIT = 0,
	DEVSTATUS_FAILED = 1,
	DEVSTATUS_NOINIT = 2
};

struct cdevice {
	char name[DEVNAMEMAX];
	
	int (*read)(void *dev, void *data, size_t sz);
	int (*write)(void *dev, void *data, size_t sz);
	int (*interrupt)(void *dev, const void *h);
	int status;

	void *priv;
};

#endif
