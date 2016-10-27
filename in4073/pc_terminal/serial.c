#include "serial.h"

//Taken from the example file


int 	rs232_getchar()
{
	int 	c;

	while ((c = rs232_getchar_nb()) == -2) 
		;
	return c;
}


