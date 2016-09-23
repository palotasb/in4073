#ifndef __LOG_H
#define __LOG_H

#include "message.h"
#include <inttypes.h>

bool log_init(serialcomm_t* sc);
uint32_t log_getsize(void);
bool log_read(uint32_t index, message_t* item);
bool log_write(message_t* item);
void log_reset(void);

#endif
