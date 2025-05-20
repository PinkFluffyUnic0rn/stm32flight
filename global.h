#ifndef GLOBAL_H
#define GLOBAL_H

// clock frequency
#define OCSFREQ 128000000

// periodic event timer prescaler
#define PRESCALER 128

// periodic event timer period
#define TIMPERIOD 0xfff

// periodic event timer ticks per second
#define TICKSPERSEC (OCSFREQ / PRESCALER)

// delay timer prescaler
#define DELAYPRESCALER 2

// maximum length for info packet
// sent back to operator
#define INFOLEN 512

// maximum length for control command
#define CMDSIZE 64

#endif
