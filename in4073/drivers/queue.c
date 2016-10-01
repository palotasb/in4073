/*------------------------------------------------------------------
 *  queue.c -- some queue implementation stolen from the interwebs
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"

void init_queue(queue *q){
	
	q->first = 0;
	q->last = QUEUE_SIZE - 1;
	q->count = 0;
}

void enqueue(queue *q,char x){

	if (q->count == QUEUE_SIZE)
		return;

	CRITICALSECTION_FastEnter();
	q->last = (q->last + 1) & (QUEUE_SIZE - 1);
	q->Data[ q->last ] = x;
	q->count += 1;
	CRITICALSECTION_FastExit();
}

char dequeue(queue *q){

	char x = q->Data[ q->first ];
	CRITICALSECTION_FastEnter();
	q->first = (q->first + 1) & (QUEUE_SIZE - 1);
	q->count -= 1;
	CRITICALSECTION_FastExit();
	return x;
}
