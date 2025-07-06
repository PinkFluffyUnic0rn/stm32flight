#ifndef GLOBAL_H
#define GLOBAL_H

// clock frequency
#define OCSFREQ 128000000

// periodic event timer prescaler
#define PRESCALER 128

// PWM settings
#define PWM_MAXCOUNT 3200

// DSHOT-300 values
#define DSHOT_BITLEN 427
#define DSHOT_0 160
#define DSHOT_1 320

// periodic event timer period
#define TIMPERIOD 0xfff

// periodic event timer ticks per second
#define TICKSPERSEC (OCSFREQ / PRESCALER)

// delay timer prescaler
//#define DELAYPRESCALER 2
#define DELAYPRESCALER 128

// maximum length for info packet
// sent back to operator
#define INFOLEN 512

// maximum length for control command
#define CMDSIZE 64

#endif
