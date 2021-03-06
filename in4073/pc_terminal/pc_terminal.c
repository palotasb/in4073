#include "pc_terminal.h"
#include "console.h"
#include "serial.h"
#include "../common.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>

/** PC TERMINAL BLOCK DIAGRAM
 *  =========================
 *
 *	 +----------------------------------------------
 *   | PC terminal
 *   |
 *   |   +----------+        +----------+
 *   |   | Joystick |-> + -->| Setpoint |
 *   |   +----------+   ^    +----------+
 *   |   +----------+   |    +----------+
 *   |   | Keyboard |---+--->| Send     |
 *   |   +----------+        | commands |
 *   |                       +----------+
 *   |
**/

void pc_rx_complete(message_t*);
void pc_tx_byte(uint8_t);
unsigned long long timespec_ms(struct timespec*);

pc_command_t	command;
pc_log_t		pc_log;
pc_log_t		pc_telemetry;



/******************************
print_help()
*******************************
Description:
	Prints a message about the usage of this program


Author:
	 Koos Eerden
*******************************/

void print_help(void) {
	term_puts("\nUsage: pc-terminal [SERIAL] [JOYSTICK] \n");
	term_puts("\n\tSERIAL:\n\t\t-s <path to serial device>\n\t\t-ns for no serial communication.\n");
	term_puts("\n\t\tIf ommitted ");
	term_puts(SERIAL_DEV);
	term_puts(" is used\n ");
	term_puts("\n\tJOYSTICK:\n\t\t-j <path to joystick device>\n\t\t-nj for no joystick.\n");
	term_puts("\n\t\tIf ommitted ");
	term_puts(JS_DEV);
	term_puts(" is used\n");
}

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char *argv[])
{
	char *serial = SERIAL_DEV;
	char *js = NULL;
	char *virtual_out = NULL;
	char *virtual_in = NULL;

	bool printhelp = false;

	opterr = 0;
	int c;
	while ((c = getopt (argc, argv, "s::j::n:vh")) != -1) {
		switch (c) {
		case 's':
			if (optarg)
				serial = optarg;
			else
				serial = SERIAL_DEV;
			break;
		case 'j':
			if (optarg)
				js = optarg;
			else
				js = JS_DEV;
			break;
		case 'n':
			fprintf(stderr, "Arg: -n%s\n", optarg);
			if (optarg[0] == 's')
				serial = NULL;
			else if (optarg[0] == 'j')
				js = NULL;
			else
				{ fprintf(stderr, "Unknown option -n%s.\n", optarg); printhelp = true; }
			break;
		case 'v':
			virtual_in = VIRTUAL_IN_DEV;
			virtual_out = VIRTUAL_OUT_DEV;
			break;
		case 'h':
			printhelp = true;
			break;
		case '?':
			if (optopt == 's' || optopt == 'j' || optopt == 'n')
				fprintf (stderr, "Option -%c requires an argument.\n", optopt);
			else if (isprint(optopt))
				fprintf (stderr, "Unknown option `-%c'.\n", optopt);
			else
				fprintf (stderr, "Unknown option character `\\x%x'.\n",	optopt);
			printhelp = true;
			break;
		default:
			return 2;
		}
	}

	if(printhelp) {
		print_help();
	} else {
		run_terminal(serial, js, virtual_in, virtual_out);
  	}
	return 0;
}


/******************************
print_run_help()
*******************************
Description:
	Prints a help message during runtime


Author:
	  Boldizsar Palotas
*******************************/

void print_run_help(void) {
	fprintf(stderr, "========================================================\n");
	fprintf(stderr, "Terminal program - Embedded Real-Time Systems\n");
	fprintf(stderr, "--------------------------------------------------------\n\n");
	fprintf(stderr, "Press ESC to PANIC or the number keys to enter modes.\n");
	fprintf(stderr, "Motors - E: enable R: disable\n\n");
	fprintf(stderr, "Logging (telemetry) - F (G) to select what to log (enter sum)\n");
	for (int i = 0; i <= 11; i++) {
		fprintf(stderr, "%#10x: %s\n", 1u<<i, message_id_to_pc_name(i));
	}
	fprintf(stderr, "C: start V: pause B: readback (safe mode only) N: reset\n\n");
	fprintf(stderr, "Press X to REBOOT Quadcopter and EXIT terminal program.\n");
	fprintf(stderr, "========================================================\n\n");
}


/******************************
run_terminal()
*******************************
Description:
	Runs the terminal program on the pc. 

Parameters
	char* serial
		-	path to the serial device, NULL if no serial device is used
	char* js
		-	path to the joystick device, NULL if no joystick is used
	char* virt_in
		-	Path to a input pipe in order to simulate the quadcopter, NULL when not used
	char* virt_out
		-	Path to a out pipe in order to simulate the quadcopter, NULL when not used

Author:
	 Koos Eerden
*******************************/


void run_terminal(char* serial, char* js, char* virt_in, char* virt_out) {

	bool do_serial, do_js, do_virt;
	bool error = false;
	bool abort = false;
	char* errormsg = "";
	serialcomm_t sc;
	frame_t rx_frame;
	frame_t tx_frame;
	int c;

	pc_command_init(&command);

	FILE* pc_log_file = fopen("pc_log.txt", "a");
	pc_log_init(&pc_log, pc_log_file);
	pc_log_init(&pc_telemetry, stdout);

	do_virt = virt_in != NULL && virt_out != NULL;
	do_serial = (serial != NULL) || do_virt;
	do_js = (js != NULL);

	if (do_virt)
		fprintf(stderr, "Starting in virtual mode. IN: %s OUT: %s\n", virt_in, virt_out);

	if(do_serial){
		if(!do_virt) {
			if (rs232_open(serial)){
				fprintf(stderr, "Error: could not open serial device\n");
				exit(1);
			}
		} else {
			if (virt_open(virt_in, virt_out)) {
				fprintf(stderr, "Error opening virtual serial pipes.\n");
				exit(1);
			}
		}
		// Serial communication protocol initialisation
		 serialcomm_init(&sc);
		 sc.tx_frame             = &tx_frame;
		 sc.rx_frame             = &rx_frame;
		 sc.rx_complete_callback = &pc_rx_complete;
		 if (!do_virt)
		 	sc.tx_byte              = (void (*)(uint8_t)) &rs232_putchar;
		 else
		 	sc.tx_byte 				= (void (*)(uint8_t)) &virt_putchar;
		 serialcomm_send_start(&sc);
		 serialcomm_send_restart_request(&sc);	
	}

	if (do_js){
		if(open_joystick(js)){
			fprintf(stderr, "Error: could not open joystick\n");
			exit(1);
		}
	}

	print_run_help();
	
	/* send & receive
	 */

	term_initio();

	FILE* masks_file;
	uint32_t tmsk=0, lmsk=0;
	if(do_serial) {
		masks_file = fopen(".masks-log.txt", "r+");
		int result1;
		if ((result1 = fscanf(masks_file, "%"SCNx32" %"SCNx32, &tmsk, &lmsk)) == 2)
		{
			fprintf(stderr, "Setting TELEMETRY MASK to 0x%"PRIx32" and LOG MASK to 0x%"PRIx32".\n", tmsk, lmsk);
			serialcomm_quick_send(&sc, MESSAGE_SET_TELEMSK_ID, tmsk, 0);
			serialcomm_quick_send(&sc, MESSAGE_SET_LOGMSK_ID, lmsk, 0);
		} else {
			fprintf(stderr, "Couldn't read saved masks, error = %d\n", result1);
		}
	}
	
	unsigned long long last_msg;

	while (!error && !abort) 
	{
		//check for keyboard presses
		read_keyboard(&command);

		//check joystick
		if (do_js){
			if (read_joystick(&command)) {
				errormsg = "Error reading joystick\n";
				error = true;
				break;
			}
		}
		
		//handle input
		if(do_serial){
			if (!do_virt) {
				if ((c = rs232_getchar_nb()) >= 0) {
					serialcomm_receive_char(&sc, (uint8_t) c);
				} else {
					usleep(500);
				}
			} else {
				if ((c = virt_getchar_nb()) >= 0) {
					serialcomm_receive_char(&sc, (uint8_t) c);
				} else {
					usleep(500);
				}
			}

			while (pc_command_get_message(&command, &tx_frame.message)) {
				while (time_get_ms() - last_msg < 1) { usleep(500); }
				if (tx_frame.message.ID == MESSAGE_SET_P12_ID)
					fprintf(stderr, "yawp: %d, p1: %d, p2: %d\n",
						command.trim.yaw_p, command.trim.p1, command.trim.p2);
				serialcomm_send(&sc);
				//fprintf(stderr, "< Sending %s v32:[%d %d] v16:[%hd %hd %hd %hd] v8:[%hhd %hhd %hhd %hhd  %hhd %hhd %hhd %hhd]\n",
				//	message_id_to_qc_name(tx_frame.message.ID),
				//	tx_frame.message.value.v32[0], tx_frame.message.value.v32[1],
				//	tx_frame.message.value.v16[0], tx_frame.message.value.v16[1], tx_frame.message.value.v16[2], tx_frame.message.value.v16[3],
				//	tx_frame.message.value.v8[0], tx_frame.message.value.v8[1], tx_frame.message.value.v8[2], tx_frame.message.value.v8[3], tx_frame.message.value.v8[4], tx_frame.message.value.v8[5], tx_frame.message.value.v8[6], tx_frame.message.value.v8[7]);
				if (tx_frame.message.ID == MESSAGE_REBOOT_ID) {
					fprintf(stderr, "Exiting terminal.\n");
					abort = true;
					break;
				}
				last_msg = time_get_ms();
				// Don't completely block communications...
				if (!do_virt) {
					if ((c = rs232_getchar_nb()) >= 0) 
						serialcomm_receive_char(&sc, (uint8_t) c);
				} else {
					if ((c = virt_getchar_nb()) >= 0)
						serialcomm_receive_char(&sc, (uint8_t) c);
				}
				read_keyboard(&command);
				if (tx_frame.message.ID == MESSAGE_SET_TELEMSK_ID)
					tmsk = tx_frame.message.value.v32[0];
				if (tx_frame.message.ID == MESSAGE_SET_LOGMSK_ID)
					lmsk = tx_frame.message.value.v32[0];
			}

			if (150 < time_get_ms() - last_msg) {
				serialcomm_quick_send(&sc, MESSAGE_KEEP_ALIVE_ID, 0, 0);
				last_msg = time_get_ms();
			}
		}


	}

	if(error){
		fprintf(stderr, "Error: %s\n", errormsg);
	}

	while (time_get_ms() - last_msg < 250) { }

	if(do_serial && !do_virt)
		rs232_close();
	if (do_serial) {
		rewind(masks_file);
		fprintf(masks_file, "%08"PRIx32" %08"PRIx32"\n", tmsk, lmsk);
		fclose(masks_file);
	}
	if(do_virt)
		virt_close();
	if(do_js)
		close_joystick();
	
	fprintf(stderr, "\n<exit>\n");
	term_exitio();
}

/*----------------------------------------------------------------
 * pc_rx_complete -- Process message received from the Quadcopter
 *----------------------------------------------------------------
 *  Parameters:
 *      - message: pointer to the received message
 *  Returns: void
 *  Author: Boldizsar Palotas
 */
void pc_rx_complete(message_t* message) {
	char str_buf[9] = {'\0'};

	// Pass everything to logging first
	if (command.in_log_not_telemetry) {
		pc_log_receive(&pc_log, message);
		if (message->ID != MESSAGE_LOG_END_ID)
			return; // Don't handle anything received in log mode
	} else {
		pc_log_receive(&pc_telemetry, message);
	}

    // Special handling

	static mode_t last_mode = MODE_0_SAFE;
    switch (message->ID) {
    	case MESSAGE_TIME_MODE_VOLTAGE_ID:
			if (MESSAGE_MODE_VALUE(message) != last_mode) {
				last_mode = MESSAGE_MODE_VALUE(message);
    			fprintf(stderr, "Entered mode %d\n", last_mode);
			}
    		if (MESSAGE_MODE_VALUE(message) == MODE_1_PANIC) {
    			command.mode_panic_status = 0;
    		}
    		break;
    	case MESSAGE_TEXT_ID:
    		memcpy(str_buf, message->value.v8, 8);
    		fprintf(stderr, "%s", str_buf);
    		break;
    	case MESSAGE_LOG_END_ID:
    		fprintf(stderr, "End of log.\n");
    		command.in_log_not_telemetry = false;
    		break;
    	case MESSAGE_LOG_START_ID:
    		fprintf(stderr, "Start of log.\n");
    		command.in_log_not_telemetry = true;
    		break;
        default:
        	break;
    }
}



inline int min(int a, int b) {
    return (a < b) ? a : b;
}

inline int max(int a, int b) {
    return (a < b) ? b : a;
}
