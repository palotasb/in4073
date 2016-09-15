#include "pc_terminal_common.h"

#include <conio.h>
#include <windows.h>
#include <stdio.h>
HANDLE hSerial;

// Most code taken or adapted from:
/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Sujay Narayana
 * Embedded Software Group
 *
 * 11-09-2016
 *------------------------------------------------------------
 */

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */

/*------------------------------------------------------------------
 * rs232_open -- Opens the serial line to the quadcopter
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: void
 * Author: copied from demo project (Boldizsar Palotas)
 */
void rs232_open(void)
{
    DCB dcbSerialParams = { 0 };
    
    //Open Serial port in blocking mode
    hSerial = CreateFile(
        PC_SERIAL_HANDLE, GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (hSerial == INVALID_HANDLE_VALUE)
    {
        printf("\r\nCOM port cannot be opened\r\n");
        exit(1);
    }

    PurgeComm(hSerial, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(hSerial, &dcbSerialParams) == 0)
    {
        printf("\r\nError reading COM properties\r\n");
        CloseHandle(hSerial);
        exit(1);
    }

    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (SetCommState(hSerial, &dcbSerialParams) == 0)
    {
        printf("\r\nError setting COM properties\r\n");
        CloseHandle(hSerial);
        exit(1);
    }
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
    CloseHandle(hSerial);
}

/*------------------------------------------------------------------
 * rs232_getchar_nb -- reads a character from the serial line if
 *  available.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: the read character or -1 is none is available
 * Author: copied from demo project (Boldizsar Palotas)
 */
int rs232_getchar_nb(void)
{
    unsigned char data[1];
    DWORD bytes_read;
    if (!ReadFile(hSerial, data, 1, &bytes_read, NULL) || bytes_read != 1)
        return -1;
    return (int) data[0];
}

/*------------------------------------------------------------------
 * rs232_putchar -- Transmits a character on the serial line
 *------------------------------------------------------------------
 * Parameters:
 *  - c: the byte to transmit
 * Returns: void
 * Author: copied from demo project (Boldizsar Palotas)
 */
void rs232_putchar(char c)
{
    char data[1];
    DWORD bytes_written = 0;

    data[0] = c;
    if (!WriteFile(hSerial, data, 1, &bytes_written, NULL))
    {
        printf("\r\nError writing data to COM port\r\n");
    }
}

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
