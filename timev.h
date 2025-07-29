#ifndef TIMEV_H
#define TIMEV_H

#include "global.h"

// check if enough time passed to call periodic event again.
// ev -- periodic event's context.
#define checktimev(ev) ((ev)->ms > (TICKSPERSEC / (ev)->freq - (ev)->rem))

// update periodic event's counter.
// ev -- periodic event's context.
// s -- time in microseconds after last callback's call.
#define updatetimev(ev, s) (ev)->ms += (s);

// Periodic event's context that holds event's settings
// and data needed beetween calls
struct timev {
	int ms;			// microseconds passed from
				// last triggering
	int rem;
	int freq;		// event frequency
	int (*cb)(int);		// event callback
};

// Initilize periodic event's context
//
// ev -- periodic event's context.
// freq -- periodic event's frequency in Hz (calls per second).
// cb -- callback that is called by this event.
int inittimev(struct timev *ev, int freq, int (*cb)(int));

// Update periodic event's frequency
//
// ev -- periodic event's context.
// freq -- periodic event's frequency in Hz (calls per second).
int modifytimev(struct timev *ev, int freq);

// reset periodic event's counter. Used after every
// event's callback call.
// ev -- periodic event's context.
int resettimev(struct timev *ev);

#endif
