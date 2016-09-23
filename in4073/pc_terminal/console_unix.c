#include "console.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <time.h>


/*------------------------------------------------------------
 * Simple pc terminal in C
 * 
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */


/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios  savetty;

void    term_initio()
{
    struct termios tty;

    tcgetattr(0, &savetty);
    tcgetattr(0, &tty);

    tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN] = 0;

    tcsetattr(0, TCSADRAIN, &tty);
}

void    term_exitio()
{
    tcsetattr(0, TCSANOW, &savetty);
}

int term_getchar_nb() 
{ 
        static unsigned char    line [2];

        if (read(0,line,1)) // note: destructive read
                return (int) line[0];
        
        return -1;
}

unsigned long long time_get_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return ts.tv_sec * 1000ull + ts.tv_nsec / 1000000ull;
}
