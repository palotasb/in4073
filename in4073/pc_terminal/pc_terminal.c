#include "pc_terminal.h"
#include "console.h"
#include "serial.h"
#include "../common.h"
#include <stdlib.h>
#include <string.h>
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

	pc_log_init(&pc_log, "pc_log.txt");
	pc_log_init(&pc_telemetry, "pc_telemetry.txt");

	do_serial = (serial != NULL);
	do_js = (js != NULL);

	if(do_serial){
		if(rs232_open(serial)){
			term_puts("Error: could not open serial device\n");
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
			term_puts("Error: could not open joystick\n");
			exit(1);
		}
	}

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");
	term_puts("Press esc to abort\n");
	
	/* send & receive
	 */

	term_initio();
	
	while (!error && !abort) 
	{
		//handle input
		if(do_serial){
			if ((c = rs232_getchar_nb()) >= 0) 
				serialcomm_receive_char(&sc, (uint8_t) c);
			else if (c == -1) {
				error = true;
				errormsg = "couldn't read serial device";
			}
			while (pc_command_get_message(&command, &tx_frame.message)) {
				serialcomm_send(&sc);
				// Don't completely block communications...
				if ((c = rs232_getchar_nb()) >= 0) 
					serialcomm_receive_char(&sc, (uint8_t) c);
				read_keyboard(&command);
			}
		}

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
	}

	if(error){
		term_puts("\nError: ");
		term_puts(errormsg);
	}

	//this part is never executed but is still here for estetic reasons.
	if(do_serial)
		rs232_close();
	if(do_js)
		close_joystick();
	
	term_puts("\n<exit>\n");
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
	// Pass everything to logging first
	if (command.in_log_not_telemetry) {
		pc_log_receive(&pc_log, message);
	} else {
		pc_log_receive(&pc_telemetry, message);
	}
    // Special handling
    switch (message->ID) {
    	case MESSAGE_TIME_MODE_VOLTAGE_ID:
    		if (MESSAGE_MODE_VALUE(message) == MODE_1_PANIC)
    			command.mode_panic_status = 0;
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