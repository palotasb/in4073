#ifndef MODES_H
#define MODES_H

//mode definitions
//please note that a using enum is a bad idea, since its type is compiler dependent
//Since we are sending these values over the UART, it is safer to use constants
//


#define		SAFEMODE 	 0
#define		PANICMODE 	 1
#define		MANUALMODE   2
#define		CALMODE		 3
#define		YAWCTRLMODE  4
#define		FULLCRTLMODE 5
#define		YAWMODE		 6
#define		HEIGHTMODE	 7
#define		WIRELESSMODE 8
#define		INVALIDMODE	 255

#endif // MODES_H
