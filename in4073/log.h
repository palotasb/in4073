#ifndef __LOG_H
#define __LOG_H

#include <inttypes.h>
#include <stdio.h>



struct  LOG_FORMAT {
	uint32_t time;
	uint16_t mode;
	uint16_t ae[4];
	int16_t phi;
	int16_t theta;
	int16_t psi;
	int16_t sp;
	int16_t sq;
	int16_t sr;
	int32_t pressure;
	int32_t temperature;
	uint16_t bat_volt;
};


bool log_init();
int log_getsize();
bool log_read(int index, struct LOG_FORMAT* item);
bool log_write(struct LOG_FORMAT* item);
bool log_close();

#endif
