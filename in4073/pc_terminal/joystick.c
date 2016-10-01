#include <errno.h>
#include "joystick.h"
#include "../qc_mode.h"
#include "pc_terminal.h"

/******************************
read_joystick()
*******************************
Description:
	Checks if joystick has an event, and updates the status.

Inputs:
	-	pc_command_t * command:
			Pointer to the state structure.
			the structure will be updated if there was an joystick event.
			if this is the case, the update flag is set. 
		

Returns:
	Zero if read was succesfull otherwise nonzero

Author:
	This is an modified piece of code from the joystick example.
	Modified by Koos Eerden
*******************************/
int read_joystick(pc_command_t *command)
{
	struct js_event js;

	while (read_js_events(&js)) {	
		switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				switch (js.number){
					case 0:
						if (js.value == 1) {
							command->mode = MODE_1_PANIC;
							command->mode_panic_status = 1;
							command->mode_updated = true;
						}
						break;
				}
				break;

			case JS_EVENT_AXIS:
				switch (js.number){ 
					case 0:	//x axis inverted
						command->orient_js.roll = max((js.value / 256 ) + 127, 0);
						break;
					case 1: //y axis, pitch inverted
						command->orient_js.pitch = (js.value / 256);
						break;			
					case 2: //z axis, yaw
						command->orient_js.yaw = (js.value / 256);
						break;	
					case 3: //throttle, lift
						command->orient_js.lift = -(js.value / 256);
						break;		
				}
				command->orient_updated = true;	
				break;	
		}
	}
	return errno != EAGAIN;
}