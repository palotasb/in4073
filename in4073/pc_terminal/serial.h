#ifndef __SERIAL_C
#define __SERIAL_C

int rs232_open(char* dev);
int 	rs232_close(void);
int	rs232_getchar_nb();
int 	rs232_getchar();
int 	rs232_putchar(char c);

#endif
