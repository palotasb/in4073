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


typedef struct pc_logfiles {
	 FILE*       setpoints; //lift roll pitch yaw
	 FILE*       motors; //ae0 to ae3
	 FILE*       gyro; //sp sq sr
	 FILE*       accelero; //sax say saz
	 FILE*       position; //x y z
	 FILE*       angle; //phi theta psi
	 FILE*       force; //X Y Z
	 FILE*       torque; //L M N
	 FILE*       spin; //p q r
	 FILE*       trim; //yaw_p, p1 p2
	 FILE*       state; //mode voltage 
	 FILE*		 baro; //pressure, temperature
	 FILE*          single;
	 bool           split_files;
} pc_logfiles_t;



#define PC_LOG_ITEM_COUNT   _PC_LOG_LAST_ITEM_GUARD

#define SETPOINTS_FILE    "log/setpoints.txt" //lift roll pitch yaw
#define MOTORS_FILE       "log/motors.txt" //ae0 to ae3
#define GYRO_FILE         "log/gyro.txt" //sp sq sr
#define ACCELERO_FILE     "log/accelero.txt" //sax say saz
#define POSITION_FILE     "log/position.txt" //x y z
#define ANGLE_FILE        "log/angle.txt" //phi theta psi
#define FORCE_FILE        "log/force.txt" //X Y Z
#define TORQUE_FILE       "log/torque.txt" //L M N
#define SPIN_FILE         "log/spin.txt" //p q r
#define TRIM_FILE         "log/trim.txt" //yaw_p, p1 p2
#define STATE_FILE        "log/state.txt" //mode, voltage
#define BARO_FILE         "log/baro.txt" // temp pressure

typedef struct pc_log {
    pc_logfiles_t*  files;
    qc_state_t     state;
    uint32_t       time;
    qc_mode_t      mode;
    bool           initialised;
    bool           set[PC_LOG_ITEM_COUNT];
} pc_log_t;

bool pc_logfiles_open_default(pc_logfiles_t*);
void pc_logfiles_set_single(pc_logfiles_t*, FILE*);
bool pc_logfiles_close(pc_logfiles_t*);


bool pc_log_init(pc_log_t* log, pc_logfiles_t*);

void pc_log_receive(pc_log_t* log, message_t*);


#endif // PC_LOG_H
