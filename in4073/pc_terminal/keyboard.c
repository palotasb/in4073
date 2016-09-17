#include "pc_terminal.h"
#include "console.h"
#include "joystick.h"
#include <stdio.h>
#include <stdlib.h>


/******************************
init_keyboard_state()
*******************************
Description:
	Initialises a keyboard state struct.

Returns:
	keyboad_state structure with mode set to SAFEMODE,
	the updated flag is set and all other
	values are set to zero.

Author:
	Koos Eerden
*******************************/
keyboad_state init_keyboard_state() {
	keyboad_state result;
	result.updated = true;
	result.mode_change = true;
	result.mode 	= SAFEMODE;
	result.lift 	= 0;
	result.roll 	= 0;
	result.pitch 	= 0;
	result.yaw 		= 0;
	result.P 		= 0;
	result.P1 		= 0;
	result.P2 		= 0;
	return result;
}

/******************************
print_keyboard_state()
*******************************
Description:
	Prints the contents of the keyboard_state structure

Inputs:
	-	keyboad_state
			Variable holding the current state structure
Author:
	Koos Eerden
*******************************/

void print_keyboard_state(keyboad_state state) {
	fprintf(stderr,"mode: %d ",state.mode);
	fprintf(stderr,"lift: %d ",state.lift);
	fprintf(stderr,"roll: %d ",state.roll);
	fprintf(stderr,"pitch: %d ",state.pitch);
	fprintf(stderr,"yaw: %d ",state.yaw);
	fprintf(stderr,"P: %d ",state.P);
	fprintf(stderr,"P1: %d ",state.P1);
	fprintf(stderr,"P2: %d \n",state.P2);
}



/******************************
read_keyboard()
*******************************
Description:
	Checks if keys were pressed, and updates the status.
	If keys were pressed, the updated flag in the status is set.

Inputs:
	-	keyboad_state current:
			Variable holding the current state structure

Returns:
	the same keyboad_stat struct as the input when no keys were pressed,
	or an updated keyboad_state struct with the updated flag set 

Author:
	Koos Eerden
*******************************/

keyboad_state read_keyboard(keyboad_state current) {

	keyboad_state result = current;
	char c, c2, c3;

	if ((c = term_getchar_nb()) != -1) {
		switch (c) {
			case 'a':		//lift up
					result.lift = min(result.lift + 1, 127);
					break;
			case 'z':		//lift down
					result.lift = max(result.lift - 1, -128);
					break;
			case 'q':		//yaw down
					result.yaw = max(result.yaw - 1, -128);
					break;
			case 'w':		//yaw up
					result.yaw = min(result.yaw + 1, 127);
					break;
			case 'u':		//yaw control P up
					result.P = min(result.P + 1, 127);
					break;
			case 'j':		//yaw control P down
					result.P = max(result.P - 1, -128);
					break;
			case 'i':		//roll/pitch P1 control up
					result.P1 = min(result.P1 + 1, 127);
					break;
			case 'k':		//roll/pitch P1 control down
					result.P1 = max(result.P1 - 1, -128);
					break;
			case 'o':		//roll/pitch P2 control up
					result.P2 = min(result.P2 + 1, 127);
					break;
			case 'l':		//roll/pitch P2 control down
					result.P2 = max(result.P2 - 1, -128);
					break;
			case '0':
					result.mode = SAFEMODE;
					result.mode_change = true;
					break;
			case '1':
					result.mode = PANICMODE;
					result.mode_change = true;
					break;
			case '2':
					result.mode = MANUALMODE;
					result.mode_change = true;
					break;
			case '3':
					result.mode = CALMODE;
					result.mode_change = true;
					break;
			case '4':
					result.mode = YAWCTRLMODE;
					result.mode_change = true;
					break;
			case '5':
					result.mode = FULLCRTLMODE;
					result.mode_change = true;
					break;
			case '6':
					result.mode = YAWMODE;
					result.mode_change = true;
					break;
			case '7':
					result.mode = HEIGHTMODE;
					result.mode_change = true;
					break;
			case '8':
					result.mode = WIRELESSMODE;
					result.mode_change = true;
					break;

			case 27:			/* escape,  start of control sequence
									esc [ A    arrow up
									esc [ B    arrow down
									esc [ C    arrow right
									esc [ D    arrow left

									if no second character is available, the press of the esc key is assumed
									Unknown control sequenced are ignored
								*/
					c2 = term_getchar_nb();
					
					if(c2 == -1){
						result.mode = PANICMODE;
						result.mode_change = true;
					}else if(c2 == '['){
						c3 = term_getchar_nb();	
						switch(c3) {
							case 'A':	//up arrow: pitch down
								result.pitch = max(result.pitch - 1, -128);
							break;
							case 'B':  //down arrow: pitch up
								result.pitch = min(result.pitch + 1, 127);
							break;
							case 'C':	//right arrow: roll down
								result.roll = max(result.roll - 1, -128);
							break;
							case 'D':	//left arrow: roll up
								result.roll = min(result.roll + 1, 127);
							break;
						}
					}
			break;
		}
		result.updated = true;
	}
			
	return result;
}
