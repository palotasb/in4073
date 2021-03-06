#include "pc_terminal.h"
#include "console.h"
#include "joystick.h"
#include "pc_command.h"
#include "../mode_constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>


enum keyboard_state {HANDLE_PRESSES, LOG_MASK, TELE_MASK};

static enum keyboard_state kb_state = HANDLE_PRESSES;

static void handle_keypress(pc_command_t* command);
static void read_log_mask(pc_command_t* command);
static void read_tele_mask(pc_command_t* command);


/******************************
read_keyboard()
*******************************
Description:
	handles keybard events and updates the pc_command structure

Outputs:
	-	pc_command_t *command:
			pointer to the current state structure
			this state is updated when a key press is detecten

Author:
	Koos Eerden
*******************************/

void read_keyboard(pc_command_t* command) {
	switch(kb_state){
		case HANDLE_PRESSES:
			handle_keypress(command);
			break;
		case LOG_MASK:
			read_log_mask(command);
			break;
		case TELE_MASK:
			read_tele_mask(command);
			break;
	}
}


/******************************
handle_keypress()
*******************************
Description:
	Checks if keys were pressed, and updates the status.
	If keys were pressed, the updated flag in the status is set.

Outputs:
	-	pc_command_t *command:
			pointer to the current state structure
			this state is updated when a key press is detecten

Author:
	Koos Eerden
*******************************/

static void handle_keypress(pc_command_t* command) {
	char c, c2;
#ifndef WINDOWS
	char c3;
#endif

	if ((c = term_getchar_nb()) != -1) {
		switch (c) {

			// ----------------------------------
			// Orientation setpoint control
			// ----------------------------------

			case 'a':		//lift up
				command->orient_kb.lift = min(command->orient_kb.lift + 1, 127);
				command->orient_updated = true;
				break;
			case 'y':		//to be used with Hungarian keyboard layout
			case 'z':		//lift down
				command->orient_kb.lift = max(command->orient_kb.lift - 1, -128);
				command->orient_updated = true;
				break;
			case 'q':		//yaw down
				command->orient_kb.yaw = max(command->orient_kb.yaw - 1, -128);
				command->orient_updated = true;
				break;
			case 'w':		//yaw up
				command->orient_kb.yaw = min(command->orient_kb.yaw + 1, 127);
				command->orient_updated = true;
				break;
			case 27:	/* escape,  start of control sequence
							esc [ A    arrow up
							esc [ B    arrow down
							esc [ C    arrow right
							esc [ D    arrow left

							if no second character is available, the press of the esc key is assumed
							Unknown control sequenced are ignored
							*/
				c2 = term_getchar_nb();
				if (c2 == -1 || c2 == 27) {
					command->mode = MODE_1_PANIC;
					command->mode_panic_status = 1;
					command->mode_updated = true;
				}
#ifndef WINDOWS
				else if(c2 == '['){
					c3 = term_getchar_nb();	
					switch(c3) {
						case 'A':	//up arrow: pitch down
							command->orient_kb.pitch = max(command->orient_kb.pitch - 1, -128);
							command->orient_updated = true;
							break;
						case 'B':  //down arrow: pitch up
							command->orient_kb.pitch = min(command->orient_kb.pitch + 1, 127);
							command->orient_updated = true;
							break;
						case 'C':	//right arrow: roll down
							command->orient_kb.roll = max(command->orient_kb.roll - 1, -128);
							command->orient_updated = true;
							break;
						case 'D':	//left arrow: roll up
							command->orient_kb.roll = min(command->orient_kb.roll + 1, 127);
							command->orient_updated = true;
							break;
					}
				} else {
					fprintf(stderr, "Can't recognize character %d.\n", c);
				}
#endif
				break;
#ifdef WINDOWS
				case -32:	// EBCDIC escape code on Windows command line
					c2 = term_getchar_nb();
					// This one uses IBM scan codes: http://www.lookuptables.com/ebcdic_scancodes.php
					switch (c2) {						
						case 72:	//up arrow: pitch down
							command->orient_kb.pitch = max(command->orient_kb.pitch - 1, -128);
							command->orient_updated = true;
							break;
						case 80:  //down arrow: pitch up
							command->orient_kb.pitch = min(command->orient_kb.pitch + 1, 127);
							command->orient_updated = true;
							break;
						case 77:	//right arrow: roll down
							command->orient_kb.roll = max(command->orient_kb.roll - 1, -128);
							command->orient_updated = true;
							break;
						case 75:	//left arrow: roll up
							command->orient_kb.roll = min(command->orient_kb.roll + 1, 127);
							command->orient_updated = true;
							break;
					}
					break;
#endif

			// ----------------------------------
			// Mode switching
			// ----------------------------------

			case -61: // first of ö sequence on Linux:
				c2 = term_getchar_nb();
				if (c2 != -74)
					break;
				// Otherwise fallthrough.
			case '`':
			case '0':
			case -108: //ö on Windows
				command->mode = MODE_0_SAFE;
				command->mode_updated = true;
				break;
			case '1':
				command->mode = MODE_1_PANIC;
				command->mode_panic_status = 1;
				command->mode_updated = true;
				break;
			case '2':
				command->mode = MODE_2_MANUAL;
				command->mode_updated = true;
				break;
			case '3':
				command->mode = MODE_3_CALIBRATE;
				command->mode_updated = true;
				break;
			case '4':
				command->mode = MODE_4_YAW;
				command->mode_updated = true;
				break;
			case '5':
				command->mode = MODE_5_FULL_CONTROL;
				command->mode_updated = true;
				break;

			// ----------------------------------
			// Trimming
			// ----------------------------------

			case 'u':		//yaw control P up
				command->trim.yaw_p = min(command->trim.yaw_p + 1, YAWP_MAX);
				command->trim_updated = true;
				break;
			case 'j':		//yaw control P down
				command->trim.yaw_p = max(command->trim.yaw_p - 1, YAWP_MIN);
				command->trim_updated = true;
				break;
			case 'i':		//roll/pitch P1 control up
				command->trim.p1 = min(command->trim.p1 + 1, P1_MAX);
				command->trim_updated = true;
				break;
			case 'k':		//roll/pitch P1 control down
				command->trim.p1 = max(command->trim.p1 - 1, P1_MIN);
				command->trim_updated = true;
				break;
			case 'o':		//roll/pitch P2 control up
				command->trim.p2 = min(command->trim.p2 + 1, P2_MAX);
				command->trim_updated = true;
				break;
			case 'l':		//roll/pitch P2 control down
				command->trim.p2 = max(command->trim.p2 - 1, P2_MIN);
				command->trim_updated = true;
				break;

			// ----------------------------------
			// Option control
			// ----------------------------------

			case 'e':		// Motor enable
				command->option_number = 1;
				command->option_set = true;
				command->option_clear = false;
				command->option_toggle = false;
				break;
			case 'r':		// Motor off
				command->option_number = 1;
				command->option_set = false;
				command->option_clear = true;
				command->option_toggle = false;
				break;
			case '6':		// Option 6 raw mode toggle
				command->option_number = 6;
				command->option_set = false;
				command->option_clear = false;
				command->option_toggle = true;
				break;
			case '7':		// Option 7 height control mode toggle
				command->option_number = 7;
				command->option_set = false;
				command->option_clear = false;
				command->option_toggle = true;
				break;
			case '8':		// Option 8 wireless mode toggle
				command->option_number = 8;
				command->option_set = false;
				command->option_clear = false;
				command->option_toggle = true;
				break;

			// ----------------------------------
			// Logging
			// ----------------------------------

			case 'f':		// Log set mask
				fprintf(stderr, "Enter LOG MASK: 0x0\b");
				kb_state = LOG_MASK;
				command->log_mask = 0;
				break;
			case 'c':		// Log start
				command->log_start = true;
				break;
			case 'v':		// Log stop
				command->log_stop = true;
				break;
			case 'b':		// Log read
				command->log_read = true;
				break;
			case 'n':		// Log reset
				command->log_reset = true;
				break;

			// ----------------------------------
			// Miscellaneous
			// ----------------------------------

			case 'g':		// Telemetry set mask
				fprintf(stderr, "Enter TELEMETRY MASK: 0x0\b");
				kb_state = TELE_MASK;
				command->telemetry_mask = 0;
				break;
			case 'x':		// Reboot
				command->reboot = true;
				break;
			case 'h':
				print_run_help();
				break;
			default:
				fprintf(stderr, "Unknown key (%d)\n", c);
				break;
		}
	}
}

/******************************
read_hex()
*******************************
Description:
	Reads a hexadecimal number from the keyboard

Outputs:
	-	uint32_t* hex_number:
			Pointer to where the number esult will be stored
Returns:
	True on success, false on failure 

Author:
	 Boldizsar Palotas
*******************************/

static bool read_hex(uint32_t* hex_number) {
	char c = term_getchar_nb();
	int8_t digit = -1;
	uint32_t copy;
	if ('0' <= c && c <= '9') digit = c - '0';
	if ('a' <= c && c <= 'f') digit = c - 'a' + 10;
	if ('A' <= c && c <= 'F') digit = c - 'A' + 10;
	if (0 <= digit) {
		if (digit == 0 && *hex_number == 0) {
			return false;
		}
		if ((UINT_MAX - digit) / 16 < *hex_number) {
			return false;
		}
		copy = *hex_number;
		*hex_number *= 16;
		*hex_number += digit;
		while (copy != 0) {
			fprintf(stderr, "\b \b");
			copy /= 16;
		}
		fprintf(stderr, "\b\b");
		fprintf(stderr, "%#x", *hex_number);
	} else {
		switch (c) {
			case '\n':
			case '\r':
				kb_state = HANDLE_PRESSES;
				fprintf(stderr, "\n");
				return true;
				break;
			case 127:  // Ascii delete works on linux
			case '\b': // ^H delete or Backspace on Windows
				if (*hex_number == 0)
					break;
				*hex_number /= 16;
				fprintf(stderr, "\b \b");
				break;
			case 27: // Escape
				kb_state = HANDLE_PRESSES;
				while (*hex_number != 0) {
					fprintf(stderr, "\b \b");
					*hex_number /= 16;
				}
				fprintf(stderr, "\b\b[Cancelled.]\n");
			default:
				break;
		}
	}
	return false;
}

/******************************
void read_log_mask()
*******************************
Description:
	Reads a hexadecimal logmask from the keyboard 
	and stores it in the command struct.
	log_mask_updated is set after the call.
	

Outputs:
-	pc_command_t *command:
			pointer to the current state structure
			structure is updated after the call
Author:
	 Boldizsar Palotas
*******************************/

static void read_log_mask(pc_command_t* command) {
	if (read_hex(&command->log_mask)) {
		command->log_mask_updated = true;
	}
}
/******************************
void read_tele_mask()
*******************************
Description:
	Reads a hexadecimal telemetry mask from the keyboard 
	and stores it in the command struct.
	log_mask_updated is set after the call.
	

Outputs:
-	pc_command_t *command:
			pointer to the current state structure
			structure is updated after the call
Author:
	 Boldizsar Palotas
*******************************/
static void read_tele_mask(pc_command_t* command) {
	if (read_hex(&command->telemetry_mask)) {
		command->telemetry_mask_updated = true;
	}
}


