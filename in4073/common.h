#ifndef COMMON_H
#define COMMON_H

#ifdef NRF51
    #ifndef QUADCOPTER
        // QUADCOPTER is defined if the code is being compiled for the QC
        #define QUADCOPTER 1
    #endif
#else
    #ifndef PC_TERMINAL
        // PC_TERMINAL is defined if the code is being compiled for the PC terminal
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
#include "qc_mode.h"
#include "qc_state.h"

#endif // COMMON_H