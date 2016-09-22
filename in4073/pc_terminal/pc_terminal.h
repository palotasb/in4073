#ifndef __PC_TERMINAL_H
#define __PC_TERMINAL_H

#include <inttypes.h>
#include <stdbool.h>
#include "../common.h"
#include "serial.h"
#include "console.h"
#include "joystick.h"

#define JS_DEV	"/dev/input/js0"

#ifndef SERIAL_DEV
	#ifndef __WINDOWS
		#define SERIAL_DEV	"/dev/ttyUSB0"
	#else
		#define SERIAL_DEV	"\\\\.\\COM3"
	#endif
#endif

int min(int a, int b);
int max(int a, int b);

void print_help(void);
void run_terminal(char* serial, char* js);

#endif
