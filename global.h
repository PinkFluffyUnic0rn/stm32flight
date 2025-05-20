#ifndef GLOBAL_H
#define GLOBAL_H

// timers prescaler
#define PRESCALER 128

// clock frequency
#define OCSFREQ 128000000

// period for main timer (used to timing periodic events)
#define TIMPERIOD 0xfff

// main timer ticks per second
#define TICKSPERSEC (OCSFREQ / PRESCALER)

// maximum length for info packet
// sent back to operator
#define INFOLEN 512

// maximum length for control command
#define CMDSIZE 64

#endif
