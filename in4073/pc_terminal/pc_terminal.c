#include "pc_terminal.h"
#include "console.h"
#include "serial.h"
#include "../common.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <time.h>

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
	char *serial = NULL;
	char *js = NULL;
	bool printhelp = false;

	switch (argc){
		case 1:
			serial = SERIAL_DEV;
			js = JS_DEV;
			break;
		case 2:
			if(!strcmp(argv[1],"-ns")) {
				js = JS_DEV;
			}else if (!strcmp(argv[1],"-nj")) {
				serial = SERIAL_DEV;
			}else {
				printhelp = true;
			}			
			break;
		case 3:
			if(!strcmp(argv[1],"-s")) {
				js = JS_DEV;
				serial = argv[2];
			}else if (!strcmp(argv[1],"-j")) {
				serial = SERIAL_DEV;
				js = argv[2];
			}else if (!strcmp(argv[1],"-ns")  && !strcmp(argv[2],"-nj")){
				js = NULL;
				serial = NULL;
			} else {
				printhelp = true;
			}
			break;
		case 4:
			if(!strcmp(argv[1],"-s") && !strcmp(argv[3],"-nj")) {
				js = NULL;
				serial = argv[2];
			}else if(!strcmp(argv[1],"-ns") && !strcmp(argv[2],"-j")) {
				serial = NULL;
				js = argv[3];
			}else {
				printhelp = true;
			}	
			break;
		case 5:
			if(!strcmp(argv[1],"-s") && !strcmp(argv[3],"-j")) {
				js = argv[4];
				serial = argv[2];
			}else {
				printhelp = true;
			}	
			break;
		default:
			printhelp = true;
	}
	
	if(printhelp) {
		print_help();
	} else {
		run_terminal(serial, js);
  	}
	return 0;
}

void run_terminal(char* serial, char* js) {

	bool do_serial, do_js;
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

	do_serial = (serial != NULL);
	do_js = (js != NULL);

	if(do_serial){
		if(rs232_open(serial)){
			fprintf(stderr, "Error: could not open serial device\n");
			exit(1);
		}
		// Serial communication protocol initialisation
		 serialcomm_init(&sc);
		 sc.tx_frame             = &tx_frame;
		 sc.rx_frame             = &rx_frame;
		 sc.rx_complete_callback = &pc_rx_complete;
		 sc.tx_byte              = (void (*)(uint8_t)) &rs232_putchar;
		 serialcomm_send_start(&sc);
		 serialcomm_send_restart_request(&sc);	
	}

	if (do_js){
		if(open_joystick(js)){
			fprintf(stderr, "Error: could not open joystick\n");
			exit(1);
		}
	}

	fprintf(stderr, "========================================================\n");
	fprintf(stderr, "Terminal program - Embedded Real-Time Systems\n");
	fprintf(stderr, "--------------------------------------------------------\n\n");
	fprintf(stderr, "Press the number keys to enter modes.\n");
	fprintf(stderr, "Press ESC or 1 (one) to enter PANIC mode.\n");
	fprintf(stderr, "Press E to enable motors (after startup and each panic).\n");
	fprintf(stderr, "Press R to disable motors manually.\n\n");
	fprintf(stderr, "Logging data:\n");
	fprintf(stderr, "Press C to start, V to stop, B to read back, N to reset.\n");
	fprintf(stderr, "(Reading back only in safe mode.)\n\n");
	fprintf(stderr, "How to select data for logging (telemetry):\n");
	fprintf(stderr, "Press F (G) and enter the sum of the relevant value IDs:\n");
	for (int i = 0; i <= 11; i++) {
		fprintf(stderr, "\t%6u = %#8x: %s\n", 1u<<i, 1u<<i, message_id_to_pc_name(i));
	}
	fprintf(stderr, "Press F, ENTER (G, ENTER) to log nothing by default.\n\n");
	fprintf(stderr, "Press X to REBOOT Quadcopter and EXIT terminal program.\n");
	fprintf(stderr, "========================================================\n\n");
	
	/* send & receive
	 */

	term_initio();
	
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
			if ((c = rs232_getchar_nb()) >= 0) {
				serialcomm_receive_char(&sc, (uint8_t) c);
			} /*else if (c == -1) {
				error = true;
				errormsg = "couldn't read serial device";
			}*/

			while (pc_command_get_message(&command, &tx_frame.message)) {
				while (time_get_ms() - last_msg < 1) { }
				serialcomm_send(&sc);
				//fprintf(stderr, "< Sending %s v32:[%d %d] v16:[%hd %hd %hd %hd] v8:[%hd %hd %hd %hd  %hd %hd %hd %hd]\n",
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
				if ((c = rs232_getchar_nb()) >= 0) 
					serialcomm_receive_char(&sc, (uint8_t) c);
				read_keyboard(&command);
			}

			if (250 < time_get_ms() - last_msg) {
				serialcomm_quick_send(&sc, MESSAGE_KEEP_ALIVE_ID, 0, 0);
				last_msg = time_get_ms();
			}
		}


	}

	if(error){
		fprintf(stderr, "Error: %s\n", errormsg);
	}

	if(do_serial)
		rs232_close();
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
    switch (message->ID) {
    	case MESSAGE_TIME_MODE_VOLTAGE_ID:
    		fprintf(stderr, "Entered mode %d\n", MESSAGE_MODE_VALUE(message));
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
