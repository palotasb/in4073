/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input = 1-2ms
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"

void update_motors(void)
{								
	NRF_TIMER1->CC[0] = 1000 + ae[0];			
	NRF_TIMER1->CC[1] = 1000 + ae[1];			
	NRF_TIMER1->CC[2] = 1000 + ae[2];			
	NRF_TIMER1->CC[3] = 1000 + ae[3];		
}

void run_filters_and_control()
{
	// fancy stuff here
	// control loops and/or filters
	update_motors();
}

