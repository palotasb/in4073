#ifndef MODE_5_FULL_H
#define MODE_5_FULL_H

#include "qc_mode.h"

void mode_2_manual_init(qc_mode_table_t* mode_table);
void mode_4_yaw_init(qc_mode_table_t* mode_table);
void mode_5_full_init(qc_mode_table_t* mode_table);

void acc_filter(qc_state_t* state);

#endif // MODE_5_FULL_H
