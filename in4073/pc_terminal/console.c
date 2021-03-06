#include "console.h"
#include <stdio.h>


/*******************
terminal functions taken from the example file
*******************/


int	term_getchar() 
{ 
        int    c;

        while ((c = term_getchar_nb()) == -1)
                ;
        return c;
}

void    term_puts(char *s) 
{ 
    fprintf(stderr,"%s",s); 
}

void    term_putchar(char c) 
{ 
    putc(c,stderr); 
}
