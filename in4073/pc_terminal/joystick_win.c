#include "pc_terminal.h"
#include "joystick.h"
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

/******************************
open_joystick()
*******************************
Description:
	Opens the I/O file for the joystick

Inputs:
	const char *path
		contains the path to the joysticks I/O file.
	joystick_state *js
		contains a pointer to the joystick state. 
		after a successfull call, the fd is set to
		represent the right filedescriptor

Returns:
	zero if succesfull or nonzero when it fails

Author:
	Koos Eerden
*******************************/
int open_joystick(const char *path) {
	return -1;
}

/******************************
close_joystick()
*******************************
Description:
	Closes the joystick

Inputs:
	-	joystick_state *state:
			pointer to the current state structure
Returns:
	zero on success otherwise nonzero
Author:
	Koos Eerden
*******************************/
int close_joystick(void) {
	return 0;
}

int read_js_events(struct js_event* evt) {
	return 0;
}