#ifndef COMMON_H
#define COMMON_H

#ifdef ARM
    #ifndef QUADCOPTER
        #define QUADCOPTER 1
    #endif
#else
    #ifndef PC_TERMINAL
        #define PC_TERMINAL 1
    #endif
#endif

#if DEBUG
    #ifdef QUADCOPTER
        #define DEBUG_PRINT(fmt, ...)
    #elif defined(PC_TERMINAL)
        #include <stdio.h>
        #define DEBUG_PRINT(fmt, ...)   printf(fmt, ##__VA_ARGS__)
    #endif
#else
    #define DEBUG_PRINT(fmt, ...)
#endif

#include "serialcomm.h"
#include "fixedpoint.h"
#include "modes.h"

#endif // COMMON_H