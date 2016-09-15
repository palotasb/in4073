/*------------------------------------------------------------
 * Terminal program for communication with Quadcopter.
 *------------------------------------------------------------
 * Adapted code from:
 *       Sujay Narayana
 *       Embedded Software Group
 *       11-09-2016
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <limits.h>
#include <inttypes.h>
#include "./pc_terminal_common.h"
#include "../common.h"

void pc_rx_complete(message_t*);
void pc_tx_byte(uint8_t);
serialcomm_t sc;
frame_t rx_frame;

int main()
{
	int c;
	printf("\r\nTerminal program - Embedded Real-Time Systems\r\n");
	rs232_open();
    term_initio();

    // Serial communication protocol initialisation
    serialcomm_init(&sc);
    sc.rx_frame             = &rx_frame;
    sc.rx_complete_callback = &pc_rx_complete;
    sc.tx_byte              = (void (*)(uint8_t)) &rs232_putchar;
    serialcomm_send_start(&sc);
    serialcomm_send_restart_request(&sc);

	for (;;)
	{
        // Process keyes pressed in the terminal application
		while ((c = term_getchar_nb()) != -1) {
            // Send any key pressed as a SET_KEYCODE message
			serialcomm_quick_send(&sc, MESSAGE_SET_KEYCODE_ID, c, 0);
            // Send the 'x' character out of band to test error recovery
			if (c == 'x') {
				rs232_putchar('x');
			}
		}

        // Process any bytes in the serial line buffer
		if ((c = rs232_getchar_nb()) != -1) {
			serialcomm_receive_char(&sc, (uint8_t) c);
		}
	
	}

	rs232_close();
	printf("\r\n<exit>\r\n");
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
        	printf("T%4d P%6d\n",
        		MESSAGE_TEMP_VALUE(message),
                MESSAGE_PRESSURE_VALUE(message));
        	break;
        case MESSAGE_PHI_THETA_PSI_ID:
        	printf("ph%6d th%6d ps%6d | ",
        		MESSAGE_PHI_VALUE(message),
        		MESSAGE_THETA_VALUE(message),
        		MESSAGE_PSI_VALUE(message));
        	break;
        default:
        	break;
    }
}
