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

Output:
	- keyboad_state *state
		mode is set to INVALIDMODE,
		the updated flag is set and all other
		values are set to zero.

Author:
	Koos Eerden
*******************************/
void init_keyboard_state(keyboad_state *state) {
	state->updated = true;
	state->mode 	= SAFEMODE;
	state->lift 	= 0;
	state->roll 	= 0;
	state->pitch 	= 0;
	state->yaw 		= 0;
	state->P 		= 0;
	state->P1 		= 0;
	state->P2 		= 0;
}

/******************************
zero_keyboard_state()
*******************************
Description:
	resets a keyboard state struct.

Output:
	- keyboad_state *state
		All control values are set to 0

Author:
	Koos Eerden
*******************************/
void zero_keyboard_state(keyboad_state *state) {
	state->lift 	= 0;
	state->roll 	= 0;
	state->pitch 	= 0;
	state->yaw 		= 0;
	state->P 		= 0;
	state->P1 		= 0;
	state->P2 		= 0;
}

/******************************
print_keyboard_state()
*******************************
Description:
	Prints the contents of the keyboard_state structure

Inputs:
	-	keyboad_state *state
			Pointer to the current state structure
Author:
	Koos Eerden
*******************************/

void print_keyboard_state(keyboad_state *state) {
	fprintf(stderr,"mode: %d ",state->mode);
	fprintf(stderr,"lift: %d ",state->lift);
	fprintf(stderr,"roll: %d ",state->roll);
	fprintf(stderr,"pitch: %d ",state->pitch);
	fprintf(stderr,"yaw: %d ",state->yaw);
	fprintf(stderr,"P: %d ",state->P);
	fprintf(stderr,"P1: %d ",state->P1);
	fprintf(stderr,"P2: %d \n",state->P2);
}



/******************************
read_keyboard()
*******************************
Description:
	Checks if keys were pressed, and updates the status.
	If keys were pressed, the updated flag in the status is set.

Outputs:
	-	keyboad_state *state:
			pointer to the current state structure
			this state is updated when a key press is detecten

Author:
	Koos Eerden
*******************************/

void read_keyboard(keyboad_state *state) {

	char c, c2, c3;

	if ((c = term_getchar_nb()) != -1) {
		switch (c) {
			case 'a':		//lift up
					state->lift = min(state->lift + 1, 127);
					break;
			case 'z':		//lift down
					state->lift = max(state->lift - 1, -128);
					break;
			case 'q':		//yaw down
					state->yaw = max(state->yaw - 1, -128);
					break;
			case 'w':		//yaw up
					state->yaw = min(state->yaw + 1, 127);
					break;
			case 'u':		//yaw control P up
					state->P = min(state->P + 1, 127);
					break;
			case 'j':		//yaw control P down
					state->P = max(state->P - 1, -128);
					break;
			case 'i':		//roll/pitch P1 control up
					state->P1 = min(state->P1 + 1, 127);
					break;
			case 'k':		//roll/pitch P1 control down
					state->P1 = max(state->P1 - 1, -128);
					break;
			case 'o':		//roll/pitch P2 control up
					state->P2 = min(state->P2 + 1, 127);
					break;
			case 'l':		//roll/pitch P2 control down
					state->P2 = max(state->P2 - 1, -128);
					break;
			case '0':
					state->mode = SAFEMODE;
					break;
			case '1':
					state->mode = PANICMODE;
					break;
			case '2':
					state->mode = MANUALMODE;
					break;
			case '3':
					state->mode = CALMODE;
					break;
			case '4':
					state->mode = YAWCTRLMODE;
					break;
			case '5':
					state->mode = FULLCRTLMODE;
					break;
			case '6':
					state->mode = YAWMODE;
					break;
			case '7':
					state->mode = HEIGHTMODE;
					break;
			case '8':
					state->mode = WIRELESSMODE;
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
						state->mode = PANICMODE;
					}else if(c2 == '['){
						c3 = term_getchar_nb();	
						switch(c3) {
							case 'A':	//up arrow: pitch down
								state->pitch = max(state->pitch - 1, -128);
							break;
							case 'B':  //down arrow: pitch up
								state->pitch = min(state->pitch + 1, 127);
							break;
							case 'C':	//right arrow: roll down
								state->roll = max(state->roll - 1, -128);
							break;
							case 'D':	//left arrow: roll up
								state->roll = min(state->roll + 1, 127);
							break;
						}
					}
			break;
		}
		state->updated = true;
	}
}
