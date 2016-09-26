#ifndef __PC_TERMINAL_H
#define __PC_TERMINAL_H

#include <inttypes.h>
#include <stdbool.h>
#include "../common.h"

#include <stdio.h>
#include "serial.h"
#include "console.h"
#include "joystick.h"
#include "pc_command.h"
#include "pc_log.h"

#define JS_DEV	"/dev/input/js0"

#ifndef SERIAL_DEV
	#ifndef __WINDOWS
        #ifndef __MACH__
		  #define SERIAL_DEV	"/dev/ttyUSB0"
        #else
            #define SERIAL_DEV  "/dev/cu.usbserial-DN00P2T1"
        #endif
	#else
		#define SERIAL_DEV	"\\\\.\\COM3"
	#endif
#endif

int min(int a, int b);
int max(int a, int b);

void print_help(void);
void run_terminal(char* serial, char* js);
void read_keyboard(pc_command_t* command);

#endif
