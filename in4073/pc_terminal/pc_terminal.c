#include "pc_terminal.h"
#include "console.h"
#include "serial.h"
#include "../common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


inline int min(int a, int b) {
    if (a > b)
        return b;
    return a;
}
inline int max(int a, int b) {
    if (a < b)
        return b;
    return a;
}

void pc_rx_complete(message_t*);
void pc_tx_byte(uint8_t);
void mon_delay_ms(unsigned int ms);

void    mon_delay_ms(unsigned int ms)
{
        struct timespec req, rem;

        req.tv_sec = ms / 1000;
        req.tv_nsec = 1000000 * (ms % 1000);
        nanosleep(&req,&rem);
}



/******************************
init_manualmode_state()
*******************************
Description:
	Initialises a manualmode state struct.

Returns:
	manualmode_state structure with values set to zero.

Author:
	Koos Eerden
*******************************/
manualmode_state init_manualmode_state(){
	manualmode_state result;
	result.lift 	= 0;
	result.roll 	= 0;
	result.pitch 	= 0;
	result.yaw 		= 0;
	return result;
}

/******************************
print_manualmode_state()
*******************************
Description:
	Prints the contents of the manualmode_state structure

Inputs:
	-	manualmode_state
			Variable holding the current state structure
Author:
	Koos Eerden
*******************************/

void print_manualmode_state(manualmode_state state) {
	fprintf(stderr,"lift: %d ",state.lift);
	fprintf(stderr,"roll: %d ",state.roll);
	fprintf(stderr,"pitch: %d ",state.pitch);
	fprintf(stderr,"yaw: %d\n",state.yaw);
}



/******************************
update_manualmode_state()
*******************************
Description:
	Updates the manualmode state struct with the values from the keyboard and joystick states

Inputs:
	-	keyboad_state kb
			Variable holding the current keyboard state structure
	- joystick_state js
			Variable holding the current joystick state structure
Returns:
	an updated manualmode_state struct with keyboard values added to the joystick values

Author:
	Koos Eerden
*******************************/
manualmode_state update_manualmode_state(keyboad_state kb, joystick_state js) {
	manualmode_state result;
	result.roll = max(min(kb.roll + js.roll, 127), -128);
	result.pitch = max(min(kb.pitch + js.pitch, 127), -128);
	result.yaw = max(min(kb.yaw + js.yaw, 127), -128);
	result.lift = max(min(kb.lift + js.lift, 127), -128);
	return result;
}


void print_help() {
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
	
	if(printhelp)
		print_help();
	else {
		run_terminal(serial, js);
  	}
	return 0;
}

void run_terminal(char* serial, char* js) {

	bool do_serial, do_js;
	bool error = false;
	bool abort = false;
	char* errormsg = "";
	uint8_t mode = SAFEMODE;
	serialcomm_t sc;
	frame_t rx_frame;
	int c;

	keyboad_state kb_state; 
	joystick_state js_state;
	manualmode_state mm_state;



	do_serial = (serial != NULL);
	do_js = (js != NULL);

	mm_state = init_manualmode_state();
	init_keyboard_state(&kb_state);
	init_joystick_state(&js_state);

	if(do_serial){
		if(rs232_open(serial)){
			term_puts("Error: could not open serial device\n");
			exit(1);
		}
		// Serial communication protocol initialisation
		 serialcomm_init(&sc);
		 sc.rx_frame             = &rx_frame;
		 sc.rx_complete_callback = &pc_rx_complete;
		 sc.tx_byte              = (void (*)(uint8_t)) &rs232_putchar;
		 serialcomm_send_start(&sc);
		 serialcomm_send_restart_request(&sc);	
	}

	if (do_js){
		if(open_joystick(js, &js_state)){
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
		}

		//check for keyboard presses
		read_keyboard(&kb_state);


		//check joystick
		if(do_js){
			
			if(read_joystick(&js_state)){
				errormsg = "Error reading joystick\n";
				error = true;
				break;
			}
		}
		//calculate new output values if needed and send a command
		if(kb_state.updated || js_state.updated){
			js_state.updated = false;
			kb_state.updated = false;

		//check if mode is changed
			if(kb_state.mode != INVALIDMODE){
				mode = kb_state.mode;
				kb_state.mode = INVALIDMODE;
				//issue mode command
				serialcomm_quick_send(	&sc, 
												MESSAGE_SET_MODE_ID, 
												(mode & 0xFF),
												0);

			}
			if(js_state.abort){
				mode = PANICMODE;
				js_state.abort = false;
				//issue switchmode command
				serialcomm_quick_send(	&sc, 
												MESSAGE_SET_MODE_ID, 
												(mode & 0xFF),
												0);
			}
				
			switch(mode){
				case SAFEMODE:
					term_puts("\nSafemode active. controls disabled\n");
						//ensure all controls stay zero when switching back
					zero_keyboard_state(&kb_state);
					zero_joystick_state(&js_state);

					break; 
				case PANICMODE:
					abort = true;
					break; 
				case MANUALMODE:
				default:
					mm_state =  update_manualmode_state(kb_state, js_state);
					serialcomm_quick_send(	&sc, 
													MESSAGE_SET_LIFT_ROLL_PITCH_YAW_ID, 
													(mm_state.lift & 0xFFFF) | (mm_state.roll << 16),
													(mm_state.pitch & 0xFFFF) | (mm_state.yaw << 16));
			}


		}

	}
	if(error){
		term_puts("\nError: ");
		term_puts(errormsg);
	}

	//if this part is executed, either something went wrong or the program is aborted on purpose
	//this means that panicmode is issued until the program is terminated
	term_puts("\n########## !!PANIC MODE ACTIVATED!!##########\n");
	term_puts("\nSending panicmode commands to drone until the program is terminalted (<CTRL + C>)");
	//send panic
	while(1){
		serialcomm_quick_send(&sc,	MESSAGE_SET_MODE_ID,	PANICMODE,0);
		mon_delay_ms(20);
	}

	//this part is never executed but is still here for estetic reasons.
	if(do_serial)
		rs232_close();
	if(do_js)
		close_joystick(&js_state);
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
    /*  Display the message: T<time> M<quadcopter_mode> V<voltage>
     *  and so on...
     */
    switch (message->ID) {
        case MESSAGE_TIME_MODE_VOLTAGE_ID:
            printf("t%10u M%2d V%5d | ",
            	MESSAGE_TIME_VALUE(message),
            	MESSAGE_MODE_VALUE(message),
            	MESSAGE_VOLTAGE_VALUE(message));
            break;
        case MESSAGE_SPQR_ID:
        	printf("P%6d Q%6d R%6d | ",
        		MESSAGE_SP_VALUE(message),
        		MESSAGE_SQ_VALUE(message),
        		MESSAGE_SR_VALUE(message));
        	break;
        case MESSAGE_AE1234_ID:
        	printf("A %3d %3d %3d %3d | ",
        		MESSAGE_AE1_VALUE(message),
        		MESSAGE_AE2_VALUE(message),
        		MESSAGE_AE3_VALUE(message),
        		MESSAGE_AE4_VALUE(message));
        	break;
        case MESSAGE_TEMP_PRESSURE_ID:
        	printf("T%4d P%6d",
        		MESSAGE_TEMP_VALUE(message),
                MESSAGE_PRESSURE_VALUE(message));
        	break;
        case MESSAGE_PHI_THETA_PSI_ID:
        	printf("ph%6d th%6d ps%6d | ",
        		MESSAGE_PHI_VALUE(message),
        		MESSAGE_THETA_VALUE(message),
        		MESSAGE_PSI_VALUE(message));
        	break;
        case MESSAGE_TEXT_ID:
        	printf("| msg: %c \n",
        		MESSAGE_TEXT_VALUE(message));
        	break;



        default:
        	break;
    }
}


