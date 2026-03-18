#include <math.h>

#include "settings.h"

#include "timev.h"

int inittimev(struct timev *ev, int phase, int freq, int (*cb)(int))
{
	ev->ms = phase;
	ev->rem = 0;
	ev->freq = freq;
	ev->cb = cb;

	ev->tc = (1.0 / (freq * 2.0 * M_PI));

	return 0;
}

int modifytimev(struct timev *ev, int freq)
{
	inittimev(ev, 0, freq, ev->cb);

	return 0;
}

int resettimev(struct timev *ev)
{
	ev->rem += ev->ms - TICKSPERSEC / (ev)->freq;
	ev->ms = 0;

	return 0;
}

int runtimev(struct timev *ev)
{
	volatile uint32_t start, end;
	uint32_t cnt;
	float d;

	if (PROFILER_ENABLED)
		start = DWT->CYCCNT;

	ev->cb(ev->ms);
	resettimev(ev);

	if (PROFILER_ENABLED) {
		end = DWT->CYCCNT;

		cnt = end - start;

		ev->avg = ev->tc * (float) cnt
			+ (1.0 - ev->tc) * ev->avg;

		d = cnt - ev->avg;

		ev->devi = (1.0 - ev->tc) * (ev->devi + ev->tc * d * d);
	}

	return 0;
}
