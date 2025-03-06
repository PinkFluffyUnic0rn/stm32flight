#ifndef DEVICE_H
#define DEVICE_H

#include <stdint.h>
#include <stddef.h>

#define DEVNAMEMAX 32

#define DEVITENABLED(dev) ((dev) == DEVSTATUS_IT \
	|| (dev) == DEVSTATUS_INIT)

enum DEVSTATUS {
	DEVSTATUS_IT = 0,
	DEVSTATUS_INIT = 1,
	DEVSTATUS_FAILED = 2,
	DEVSTATUS_NOINIT = 3
};

struct cdevice {
	char name[DEVNAMEMAX];
	
	int (*read)(void *dev, void *data, size_t sz);
	int (*write)(void *dev, void *data, size_t sz);
	int (*interrupt)(void *dev, const void *h);
	int (*configure)(void *dev, const char *cmd, ...);
	int status;

	void *priv;
};

struct bdevice {
	char name[DEVNAMEMAX];
	int (*read)(void *dev, size_t addr, void *data, size_t sz);
	int (*write)(void *dev, size_t addr, const void *data,
		size_t sz);
	int (*ioctl)(void *dev, int req, ...);

	int (*eraseall)(void *dev);
	int (*erasesector)(void *dev, size_t addr);
	int (*eraseblock)(void *dev, size_t addr);
	int (*writesector)(void *dev, size_t addr, const void *data,
		size_t sz);
	int status;

	size_t writesize;
	size_t sectorsize;
	size_t totalsize;

	void *priv;
};

#endif
