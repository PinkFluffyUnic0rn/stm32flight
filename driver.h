#ifndef DRIVER_H
#define DRIVER_H

#include <stdint.h>
#include <stddef.h>

#define DEVNAMEMAX 32

enum DEVTYPE {
	DEVTYPE_CHAR	= 0,
	DEVTYPE_BLOCK	= 1
};

enum DEVSTATUS {
	DEVSTATUS_INIT = 0,
	DEVSTATUS_FAILED = 1
};

struct device {
	char name[DEVNAMEMAX];
	enum DEVTYPE type;
	int (*read)(void *dev, size_t addr, void *data, size_t sz);
	int (*write)(void *dev, size_t addr, const void *data,
		size_t sz);
	int status;

	void *priv;
};

struct driver {
	int (*initdevice)(void *, struct device *dev);
};

#endif
