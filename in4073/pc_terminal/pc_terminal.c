/*------------------------------------------------------------
* Simple pc terminal in C
*
* Sujay Narayana
* Embedded Software Group
*
* 11-09-2016
*------------------------------------------------------------
*/

#define PC_TERMINAL	1

#include <stdio.h>
#include <conio.h>
#include <windows.h>
#include <limits.h>
#include "../common.h"

HANDLE hSerial;
void pc_rx_complete(message_t*);
void pc_tx_byte(uint8_t);
serialcomm_t sc;
frame_t rx_frame;

char term_getchar()
{
	if (_kbhit())
	{
		return _getch();
	}

	return -1;
}


/*------------------------------------------------------------
* Serial I/O
* 8 bits, 1 stopbit, no parity,
* 115,200 baud
*------------------------------------------------------------
*/

void rs232_open()
{

	DCB dcbSerialParams = { 0 };
	
	//Open Serial port in blocking mode
	hSerial = CreateFile(
		"\\\\.\\COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL,
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

void 	rs232_close(void)
{
	CloseHandle(hSerial);
}


int rs232_getchar()
{
	unsigned char data[1];
	DWORD bytes_read;
	while (!ReadFile(hSerial, data, 1, &bytes_read, NULL))
	{
	}
	if (bytes_read != 1)
		return -1;
	return data[0];
}

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

int main()
{
	int c;
	printf("\r\nTerminal program - Embedded Real-Time Systems\r\n");
	rs232_open();

    serialcomm_init(&sc);
    sc.rx_frame = &rx_frame;
    sc.rx_complete_callback = &pc_rx_complete;
    sc.tx_byte = (void (*)(unsigned char)) &rs232_putchar;
    serialcomm_send_start(&sc);
    serialcomm_send_restart_request(&sc);

	for (;;)
	{
		if ((c = term_getchar()) != -1) {
			serialcomm_quick_send(&sc, MESSAGE_SET_KEYCODE_ID, c, 0);
		}

		//rs232_getchar() is blocking. It is also possible to set serial rx to non-blocking mode by changing the flags in CreateFile
		while ((c = rs232_getchar()) != -1) {
			serialcomm_receive_char(&sc, (unsigned char) c);
			// printf("%3u = 0x%02hx %c\n", (unsigned char) c, (unsigned char) c, ('A' <= c && c <= 'Z') ? c : ' ');
		}
	
	}

	rs232_close();
	printf("\r\n<exit>\r\n");
}

void pc_rx_complete(message_t* message) {
    switch (message->ID) {
        case MESSAGE_TIME_MODE_VOLTAGE_ID:
            printf("T%10u M%2d V%5d | ",
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
        		MESSAGE_TEMP_VALUE(message), MESSAGE_PRESSURE_VALUE(message));
        	break;
        case MESSAGE_PHI_THETA_PSI_ID:
        	printf("P%6d T%6d P%6d | ",
        		MESSAGE_PHI_VALUE(message),
        		MESSAGE_THETA_VALUE(message),
        		MESSAGE_PSI_VALUE(message));
        	break;
        default:
        	break;
    }
}
