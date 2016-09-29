#include "serial.h"
#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/select.h>


/*------------------------------------------------------------
 * Serial I/O 
 * 8 bits, 1 stopbit, no parity, 
 * 115,200 baud
 *------------------------------------------------------------
 */

int serial_device = 0;
int fd_RS232;
fd_set read_fds, write_fds, except_fds;

int rs232_open(char *dev)
{
    char        *name;
    int         result;  
    struct termios  tty;

        fd_RS232 = open(dev, O_RDWR | O_NOCTTY);  
    if(fd_RS232<0) return 1;

    result = isatty(fd_RS232);
    if(result != 1) return 2;

    name = ttyname(fd_RS232);
    if(name == 0) return 3;

    result = tcgetattr(fd_RS232, &tty); 
    if(result != 0) return 4;

    tty.c_iflag = IGNBRK; /* ignore break condition */
    tty.c_oflag = 0;
    tty.c_lflag = 0;

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
    tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */       

    cfsetospeed(&tty, B115200); 
    cfsetispeed(&tty, B115200); 

    tty.c_cc[VMIN]= 0;
    tty.c_cc[VTIME] = 1; // added timeout

    tty.c_iflag &= ~(IXON|IXOFF|IXANY);

    result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

    tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */

	// Initialize file descriptor sets

	FD_ZERO(&read_fds);
	FD_ZERO(&write_fds);
	FD_ZERO(&except_fds);
	FD_SET(fd_RS232, &read_fds);

    return 0;
}

int     rs232_close(void)
{
    return  close(fd_RS232);
}

int rs232_getchar_nb()
{
    int         result;
    unsigned char   c;
    struct timeval timeout;

    timeout.tv_sec = 0; 
    timeout.tv_usec = 1000;

    // Wait for input to become ready or until the time out; the first parameter is
    // 1 more than the largest file descriptor in any of the sets
    if (select(fd_RS232 + 1, &read_fds, &write_fds, &except_fds, &timeout) == 1)
    {
        result = read(fd_RS232, &c, 1);
	 } else {
        return -2;
	 }

    result = (result == 0)? -2 : result;
    
    return (result >= 0) ? (int) c : result;
    
}

int     rs232_putchar(char c)
{ 
    int result;

    do {
        result = (int) write(fd_RS232, &c, 1);
    } while (result == 0);   
    return result;
}
