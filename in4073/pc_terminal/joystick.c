#include "pc_terminal.h"
#include "joystick.h"
#include "console.h"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

/******************************
init_joystick_state()
*******************************
Description:
	Initialises the joystick state struct.

inputs:
	joystick_state *js
		pointer to joystick_state structure.
		After the call the flag abort and update are set to false,
		other values are set to zero.

Author:
	Koos Eerden
*******************************/

void init_joystick_state(joystick_state *js) {
	js->fd = 0;
	js->updated = true;
	js->abort = false;
	js->lift = 0;
	js->roll = 0;
	js->pitch = 0;
	js->yaw = 0;
}

/******************************
zero_joystick_state()
*******************************
Description:
	Sets all controlled values in the 
	joystick state struct to zero.

inputs:
	joystick_state *js
		pointer to joystick_state structure.
		After the call the roll pitch yaw and 
		lift values are set to 0

Author:
	Koos Eerden
*******************************/

void zero_joystick_state(joystick_state *js) {
	js->lift = 0;
	js->roll = 0;
	js->pitch = 0;
	js->yaw = 0;
}

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
int open_joystick(const char *path, joystick_state *js) {

	int err = 0;
	if ((js->fd = open(path, O_RDONLY)) < 0) {
		err = js->fd;
		js->fd = 0;
	}
	fcntl(js->fd, F_SETFL, O_NONBLOCK);

	return err;
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
int close_joystick(joystick_state * state) {
	int err = close(state->fd);
	if(!err)
		state->fd = 0;

	return err;
}
/******************************
read_joystick()
*******************************
Description:
	Checks if joystick has an event, and updates the status.

Inputs:
	-	joystick_state * state:
			Pointer to the state structure.
			the structure will be updated if there was an joystick event.
			if this is the case, the update flag is set. 
		

Returns:
	Zero if read was succesfull otherwise nonzero

Author:
	This is an modified piece of code from the joystick example.
	Modified by Koos Eerden
*******************************/
int read_joystick(joystick_state *state)
{
	struct js_event js;
	while (read(state->fd, &js, sizeof(struct js_event)) == 
	    			sizeof(struct js_event))  {
		state->updated = true;		
		switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				switch (js.number){
					case 0:
						state->abort = (js.value == 1);
					break;
				}
				break;

			case JS_EVENT_AXIS:
				switch (js.number){ 
					case 0:	//x axis inverted
						state->roll = - (js.value /256 );
						
						break;
					case 1: //y axis, pitch inverted
						state->pitch = (js.value /256);
						break;			
					case 2: //z axis, yaw
						state->yaw = (js.value /256);
						break;	
					case 3: //throttle, lift
						state->lift = -(js.value /256);
						break;		
				}
				break;	
		}
	}
	return errno != EAGAIN;
}

/******************************
print_joystick_state()
*******************************
Description:
	Prints the contents of the joystick_state structure

Inputs:
	-	joystick_state *state
			pointer to the current state structure
Author:
	Koos Eerden
*******************************/

void print_joystick_state(joystick_state *state) {
	fprintf(stderr,"lift: %d ",state->lift);
	fprintf(stderr,"roll: %d ",state->roll);
	fprintf(stderr,"pitch: %d ",state->pitch);
	fprintf(stderr,"yaw: %d ",state->yaw);
	fprintf(stderr,"abort: %d ",state->abort);
	fprintf(stderr,"updated: %d ",state->updated);
	fprintf(stderr,"fd: %d \n",state->fd);
}
