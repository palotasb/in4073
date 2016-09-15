#include "pc_terminal_common.h"

#include <termios.h>
#include <ctype.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <assert.h>
#include <time.h>
#include <string.h>

// Most code taken or adapted from:
/*------------------------------------------------------------
 * Simple pc terminal in C
 * 
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

/*------------------------------------------------------------
 * Serial I/O 
 * 8 bits, 1 stopbit, no parity, 
 * 115,200 baud
 *------------------------------------------------------------
 */

int serial_device = 0;
int fd_RS232;

/*------------------------------------------------------------------
 * rs232_open -- Opens the serial line to the quadcopter
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: void
 * Author: copied from demo project (Boldizsar Palotas)
 */
void rs232_open(void)
{
    char        *name;
    int         result;  
    struct termios  tty;

    fd_RS232 = open(PC_SERIAL_HANDLE, O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime

    assert(fd_RS232>=0);

    result = isatty(fd_RS232);
    assert(result == 1);

    name = ttyname(fd_RS232);
    assert(name != 0);

    result = tcgetattr(fd_RS232, &tty); 
    assert(result == 0);

    tty.c_iflag = IGNBRK; /* ignore break condition */
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
    tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */       

    cfsetospeed(&tty, B115200); 
    cfsetispeed(&tty, B115200); 

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // added timeout

    tty.c_iflag &= ~(IXON|IXOFF|IXANY);

    result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

    tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */
}

/*------------------------------------------------------------------
 * rs232_close -- Closes the serial line to the quadcopter
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: void
 * Author: copied from demo project (Boldizsar Palotas)
 */
void rs232_close(void)
{
    int     result;

    result = close(fd_RS232);
    assert (result==0);
}

/*------------------------------------------------------------------
 * rs232_getchar_nb -- reads a character from the serial line if
 *  available.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: the read character or -1 is none is available
 * Author: copied from demo project (Boldizsar Palotas)
 */
int rs232_getchar_nb()
{
    int         result;
    unsigned char   c;

    result = read(fd_RS232, &c, 1);

    if (result == 0) 
        return -1;
    
    else 
    {
        assert(result == 1);   
        return (int) c;
    }
}

/*------------------------------------------------------------------
 * rs232_putchar -- Transmits a character on the serial line
 *------------------------------------------------------------------
 * Parameters:
 *  - c: the byte to transmit
 * Returns: void
 * Author: copied from demo project (Boldizsar Palotas)
 */
int rs232_putchar(char c)
{ 
    int result;

    do {
        result = (int) write(fd_RS232, &c, 1);
    } while (result == 0);   

    assert(result == 1);
    return result;
}

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios savetty;

/*------------------------------------------------------------------
 * term_initio -- Terminal initialization.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: void
 * Author: Boldizsar Palotas
 */
void term_initio()
{
    struct termios tty;

    tcgetattr(0, &savetty);
    tcgetattr(0, &tty);

    tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    tcsetattr(0, TCSADRAIN, &tty);
}

/*------------------------------------------------------------------
 * term_exitio -- Terminal uninitialisation.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: void
 * Author: Boldizsar Palotas
 */
void term_exitio()
{
    tcsetattr(0, TCSADRAIN, &savetty);
}

/*------------------------------------------------------------------
 * term_getchar_nb -- reads a character from the terminal if
 * available.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: the read character or -1 is none is available
 * Author: copied from demo project (Boldizsar Palotas)
 */
int term_getchar_nb() 
{ 
    unsigned char line [2];
    if (read(0, line, 1)) // note: destructive read
        return (int) line[0];
    return -1;
}
