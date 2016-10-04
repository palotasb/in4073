#ifndef PRINTF_H
#define PRINTF_H

#include <stdio.h>

#ifdef SIMULATION
    #include "simulation/simulation.h"
    #define printf  simulation_printf
#endif // SIMULATION

#endif // PRINTF_H
