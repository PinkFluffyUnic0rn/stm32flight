/**
* @file periphdef.h
* @brief choose peripheral definition file depending on target board
*/

#ifdef F4FLIGHT
#include "periphdef/f4flight.h"
#elif H7FLIGHT
#include "periphdef/h7flight.h"
#endif
