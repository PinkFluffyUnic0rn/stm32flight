/**
* @file timev.h
* @brief Timer based periodic event context
*/

#ifndef TIMEV_H
#define TIMEV_H

#include "global.h"

/**
* @brief Check if enough time passed to call periodic event again.
* @param ev periodic event's context.
* @return 1, if periodic event should be called, 0 otherwise
*/
#define checktimev(ev) ((ev)->ms > (TICKSPERSEC / (ev)->freq - (ev)->rem))

/**
* @brief Update periodic event's counter.
* @param ev periodic event's context.
* @param s time in microseconds after last callback's call.
* @return time from last call
*/
#define updatetimev(ev, s) (ev)->ms += (s);

/**
* @brief Periodic event's context that holds event's
	settings and data needed beetween calls
*/
struct timev {
	int ms;			// microseconds passed from
				// last triggering
	int rem;
	int freq;		// event frequency
	int (*cb)(int);		// event callback
};

/**
* @brief Initilize periodic event's context
*
* @param ev periodic event's context.
* @param freq periodic event's frequency in Hz (calls per second).
* @param cb callback that is called by this event.
* @return always 0
*/
int inittimev(struct timev *ev, int phase, int freq, int (*cb)(int));

/**
* @brief Update periodic event's frequency
* @param ev periodic event's context.
* @param freq periodic event's frequency in Hz (calls per second).
* @return always 0
*/
int modifytimev(struct timev *ev, int freq);

/**
* @brief reset periodic event's counter. Used after every
	event's callback call.
* @param ev periodic event's context.
* @return always 0
*/
int resettimev(struct timev *ev);

#endif
