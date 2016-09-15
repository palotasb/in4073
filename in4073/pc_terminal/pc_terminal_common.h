/*------------------------------------------------------------------
 * pc_terminal_common - Common functions for a PC-side terminal
 * application connected to the Quadcopter
 *------------------------------------------------------------------
 * Author: Boldizsar Palotas
 */

#ifndef PC_TERMINAL_COMMON_H
#define PC_TERMINAL_COMMON_H

// The handle of the FILE that is associated with the RS232 serial line.
#ifndef PC_SERIAL_HANDLE
#define PC_SERIAL_HANDLE  "/dev/ttyUSB0"
// on mac it might be "/dev/tty.usbserial-DN00P2T1"
// on windows it might be "\\\\.\\COM3"
#endif

void rs232_open(void);
void rs232_close(void);
char rs232_getchar(void);
int rs232_getchar_nb(void);
void rs232_putchar(char);

void term_initio(void);
void term_exitio(void);
char term_getchar(void);
int term_getchar_nb(void);

#endif // PC_TERMINAL_COMMON_H
