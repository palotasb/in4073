#ifndef __INTERRUPT_PRIO_H
#define __INTERRUPT_PRIO_H

#define ADC_INT_PRIO 3
#define GPIOTE_INT_PRIO 3 // either 1 or 3, 3 being low. (sd present)
#define TIMER1_INT_PRIO 3
#define TIMER2_INT_PRIO 3
#define SPI0_TWI0_INT_PRIO 3
#define UART0_INT_PRIO 3 // either 1 or 3, 3 being low. (sd present)
#define SWI0_INT_PRIO 5


#endif
