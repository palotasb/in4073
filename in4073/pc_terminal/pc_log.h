#ifndef PC_LOG_H
#define PC_LOG_H

#include <stdio.h>
#include "../common.h"
#include "../qc_mode.h"
#include "../serialcomm.h"

typedef enum pc_log_item {
    PC_LOG_time,
    PC_LOG_mode,
    PC_LOG_lift,
    PC_LOG_roll,
    PC_LOG_pitch,
    PC_LOG_yaw,
    PC_LOG_ae1,
    PC_LOG_ae2,
    PC_LOG_ae3,
    PC_LOG_ae4,
    PC_LOG_sp,
    PC_LOG_sq,
    PC_LOG_sr,
    PC_LOG_sax,
    PC_LOG_say,
    PC_LOG_saz,
    PC_LOG_sphi,
    PC_LOG_stheta,
    PC_LOG_spsi,
    PC_LOG_temperature,
    PC_LOG_pressure,
    PC_LOG_voltage,
    PC_LOG_x,
    PC_LOG_y,
    PC_LOG_z,
    PC_LOG_phi,
    PC_LOG_theta,
    PC_LOG_psi,
    PC_LOG_X,
    PC_LOG_Y,
    PC_LOG_Z,
    PC_LOG_L,
    PC_LOG_M,
    PC_LOG_N,
    PC_LOG_u,
    PC_LOG_v,
    PC_LOG_w,
    PC_LOG_p,
    PC_LOG_q,
    PC_LOG_r,
    PC_LOG_yaw_p,
    PC_LOG_p1,
    PC_LOG_p2,
    PC_LOG_PR0_CURR,
    PC_LOG_PR1_CURR,
    PC_LOG_PR2_CURR,
    PC_LOG_PR3_CURR,
    PC_LOG_PR4_CURR,
    PC_LOG_PR0_MAX,
    PC_LOG_PR1_MAX,
    PC_LOG_PR2_MAX,
    PC_LOG_PR3_MAX,
    PC_LOG_PR4_MAX,
    _PC_LOG_LAST_ITEM_GUARD
} pc_log_item_t;

#define PC_LOG_ITEM_COUNT   _PC_LOG_LAST_ITEM_GUARD

typedef struct pc_log {
    FILE*       file;
    qc_state_t  state;
    uint32_t    time;
    qc_mode_t   mode;
    bool        initialised;
    bool        set[PC_LOG_ITEM_COUNT];
} pc_log_t;

bool pc_log_init(pc_log_t* log, FILE* file);

void pc_log_receive(pc_log_t* log, message_t*);

void pc_log_close(pc_log_t* log);

#endif // PC_LOG_H
