#define _WIN32_WINNT 0x0600
#include <conio.h>
#include <windows.h>
#include <stdio.h>
#include "console.h"


/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */

/*------------------------------------------------------------------
 * term_initio -- Dummy function for terminal initialization.
 *  Unused on Windows.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: void
 * Author: Boldizsar Palotas
 */
void term_initio(void) { }

/*------------------------------------------------------------------
 * term_exitio -- Dummy function for terminal uninitialization.
 *  Unused on Windows.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: void
 * Author: Boldizsar Palotas
 */
void term_exitio(void) { }

void term_enable_canonical() { }
void term_disable_canonical() { }

/*------------------------------------------------------------------
 * term_getchar_nb -- reads a character from the terminal if
 * available.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: the read character or -1 is none is available
 * Author: copied from demo project (Boldizsar Palotas)
 */
int term_getchar_nb(void) {
    if (_kbhit()) 
        return _getch(); 
    return -1;
}

unsigned long long time_get_ms(void) {
    LARGE_INTEGER freq, cnt;
    unsigned long long f, c;
    if (QueryPerformanceFrequency(&freq) && QueryPerformanceCounter(&cnt)) {
        f = freq.QuadPart / 1000;
        c = cnt.QuadPart / f;
    } else {
        c = 0;
    }
    return c;
}
