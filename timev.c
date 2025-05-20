#include "timev.h"

int inittimev(struct timev *ev, int freq, int (*cb)(int))
{
	ev->ms = 0;
	ev->rem = 0;
	ev->freq = freq;
	ev->cb = cb;

	return 0;
}

int resettimev(struct timev *ev)
{
	ev->rem += ev->ms - TICKSPERSEC / (ev)->freq;
	ev->ms = 0;

	return 0;
}
