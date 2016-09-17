#include "console.h"
#include <conio.h>
#include <windows.h>
#include <stdio.h>

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
