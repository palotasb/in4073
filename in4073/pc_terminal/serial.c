#include "serial.h"

int 	rs232_getchar()
{
	int 	c;

	while ((c = rs232_getchar_nb()) == -2) 
		;
	return c;
}


