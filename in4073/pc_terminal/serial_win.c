#include "serial.h"
#include <conio.h>
#include <stdbool.h>
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
 * Parameters: device name
 * Returns: int, zero on success nonzero on failure
 * Author: copied from demo project (Boldizsar Palotas)
 */

int rs232_open(char* dev)
{
    DCB dcbSerialParams = { 0 };
    
    //Open Serial port in blocking mode
    hSerial = CreateFile(
        dev, GENERIC_READ | GENERIC_WRITE, 0, NULL,
        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (hSerial == INVALID_HANDLE_VALUE)
		return 1;

    PurgeComm(hSerial, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR);
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (GetCommState(hSerial, &dcbSerialParams) == 0)
    {
                CloseHandle(hSerial);
        return 2;
    }

    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (SetCommState(hSerial, &dcbSerialParams) == 0)
    {
                CloseHandle(hSerial);
        return 3;
    }
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout         = 1; // in milliseconds
    timeouts.ReadTotalTimeoutConstant    = 5; // in milliseconds
    timeouts.ReadTotalTimeoutMultiplier  = 1; // in milliseconds
    timeouts.WriteTotalTimeoutConstant   = 50; // in milliseconds
    timeouts.WriteTotalTimeoutMultiplier = 50; // in milliseconds
    if (!SetCommTimeouts(hSerial, &timeouts)) {
        CloseHandle(hSerial);
        return 4;
    }
	return 0;
}

/*------------------------------------------------------------------
 * rs232_close -- Closes the serial line to the quadcopter
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: zero on success
 * Author: copied from demo project (Boldizsar Palotas)
 */
int rs232_close(void)
{
    return(!CloseHandle(hSerial));
}

/*------------------------------------------------------------------
 * rs232_getchar_nb -- reads a character from the serial line if
 *  available.
 *------------------------------------------------------------------
 * Parameters: none
 * Returns: the read character -1 on error or -2 if no data is available
 * Author: copied from demo project (Boldizsar Palotas)
 */
int rs232_getchar_nb(void)
{
    unsigned char data[1];
    DWORD bytes_read;
	int result, error = 0;
    if ((!ReadFile(hSerial, data, 1, &bytes_read, NULL) && (error = GetLastError(), 1)) || bytes_read != 1){
		if(error != ERROR_IO_PENDING)
			result = -1;
		else
			result = -2;
	} else {
		result = (int) data[0];
    }
               
    return result;
}

/*------------------------------------------------------------------
 * rs232_putchar -- Transmits a character on the serial line
 *------------------------------------------------------------------
 * Parameters:
 *  - c: the byte to transmit
 * Returns: zero on success
 * Author: copied from demo project (Boldizsar Palotas)
 */
int rs232_putchar(char c)
{
    char data[1];
	 bool result;
    DWORD bytes_written = 0;

    data[0] = c;
	 do{
		result = WriteFile(hSerial, data, 1, &bytes_written, NULL);
		if(!result)
			result = (GetLastError() == ERROR_IO_PENDING);

	 } while (bytes_written == 0 && result);

	return result;		
}

int virt_open(char* dev_in, char* dev_out) {
    fprintf(stderr, "Virtual quadcopter in not supported on Windows.\n");
    return -1;
}
int virt_close(void) {return-1;}
int virt_getchar_nb(void) {return-1;}
int virt_putchar(char c) {return-1;}
