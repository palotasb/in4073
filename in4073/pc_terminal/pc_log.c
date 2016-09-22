#include "pc_log.h"
#include "../serialcomm.h"
#include "../fixedpoint.h"
#include "../qc_mode.h"
#include <string.h>
#include <stdarg.h>

// Value to print in placeof on unknown number
#define _NAN    "NaN"

// Value to print to separate values
#define _SEP    "\t"

// Value to print to separate entries
#define _END    "\n"

static void pc_log_flush(pc_log_t* log);
static void pc_log_clear(pc_log_t* log);
static void pc_log_print(pc_log_t* log, const char * fmt, pc_log_item_t item, ...);

bool pc_log_init(pc_log_t* log, const char* filename) {
    log->file = fopen(filename, "a");
    if (!log->file)
        return false;
    qc_state_init(&log->state);
    log->time = 0;
    log->mode = MODE_UNKNOWN;
    log->initialised = false;
    pc_log_clear(log);
    return true;
}

void pc_log_receive(pc_log_t* log, message_t* message) {
    switch (message->ID) {
        case MESSAGE_TIME_MODE_VOLTAGE_ID:
            if (log->initialised) {
                pc_log_flush(log);
                pc_log_clear(log);
            }
            log->initialised = true;
            log->time =                 MESSAGE_TIME_VALUE(message);
            log->state.sensor.voltage = MESSAGE_VOLTAGE_VALUE(message);
            log->mode = (qc_mode_t)     MESSAGE_MODE_VALUE(message);
            log->set[PC_LOG_time] = true;
            log->set[PC_LOG_mode] = true;
            log->set[PC_LOG_voltage] = true;
            break;
        case MESSAGE_SPQR_ID:
            log->state.sensor.sp = MESSAGE_SP_VALUE(message);
            log->state.sensor.sq = MESSAGE_SQ_VALUE(message);
            log->state.sensor.sr = MESSAGE_SR_VALUE(message);
            log->set[PC_LOG_sp] = true;
            log->set[PC_LOG_sq] = true;
            log->set[PC_LOG_sr] = true;
            break;
        case MESSAGE_SAXYZ_ID:
            log->state.sensor.sax = MESSAGE_SAX_VALUE(message);
            log->state.sensor.say = MESSAGE_SAY_VALUE(message);
            log->state.sensor.saz = MESSAGE_SAZ_VALUE(message);
            log->set[PC_LOG_sax] = true;
            log->set[PC_LOG_say] = true;
            log->set[PC_LOG_saz] = true;
            break;
        case MESSAGE_AE1234_ID:
            log->state.motor.ae1 = MESSAGE_AE1_VALUE(message);
            log->state.motor.ae2 = MESSAGE_AE2_VALUE(message);
            log->state.motor.ae3 = MESSAGE_AE3_VALUE(message);
            log->state.motor.ae4 = MESSAGE_AE4_VALUE(message);
            log->set[PC_LOG_ae1] = true;
            log->set[PC_LOG_ae2] = true;
            log->set[PC_LOG_ae3] = true;
            log->set[PC_LOG_ae4] = true;
            break;
        case MESSAGE_TEMP_PRESSURE_ID:
            log->state.sensor.temperature = MESSAGE_TEMP_VALUE(message);
            log->state.sensor.pressure =    MESSAGE_PRESSURE_VALUE(message);
            log->set[PC_LOG_temperature] =  true;
            log->set[PC_LOG_pressure] =     true;
            break;
        case MESSAGE_XYZPOS_ID:
            log->state.pos.x = MESSAGE_XPOS_VALUE(message);
            log->state.pos.y = MESSAGE_YPOS_VALUE(message);
            log->state.pos.z = MESSAGE_ZPOS_VALUE(message);
            log->set[PC_LOG_x] = true;
            log->set[PC_LOG_y] = true;
            log->set[PC_LOG_z] = true;
            break;
        case MESSAGE_PHI_THETA_PSI_ID:
            log->state.att.phi =    MESSAGE_PHI_VALUE(message);
            log->state.att.theta =  MESSAGE_THETA_VALUE(message);
            log->state.att.psi =    MESSAGE_PSI_VALUE(message);
            log->set[PC_LOG_phi] =      true;
            log->set[PC_LOG_theta] =    true;
            log->set[PC_LOG_psi] =      true;
            break;
        case MESSAGE_XYZFORCE_ID:
            log->state.force.X = MESSAGE_XFORCE_VALUE(message);
            log->state.force.Y = MESSAGE_YFORCE_VALUE(message);
            log->state.force.Z = MESSAGE_ZFORCE_VALUE(message);
            log->set[PC_LOG_X] = true;
            log->set[PC_LOG_Y] = true;
            log->set[PC_LOG_Z] = true;
            break;
        case MESSAGE_LMN_ID:
            log->state.torque.L = MESSAGE_L_VALUE(message);
            log->state.torque.M = MESSAGE_M_VALUE(message);
            log->state.torque.N = MESSAGE_N_VALUE(message);
            log->set[PC_LOG_L] = true;
            log->set[PC_LOG_M] = true;
            log->set[PC_LOG_N] = true;
            break;
        case MESSAGE_PQR_ID:
            log->state.spin.p = MESSAGE_P_VALUE(message);
            log->state.spin.q = MESSAGE_Q_VALUE(message);
            log->state.spin.r = MESSAGE_R_VALUE(message);
            log->set[PC_LOG_p] = true;
            log->set[PC_LOG_q] = true;
            log->set[PC_LOG_r] = true;
            break;
        case MESSAGE_P12_ID:
            log->state.trim.p1 = MESSAGE_P1_VALUE(message);
            log->state.trim.p2 = MESSAGE_P2_VALUE(message);
            log->set[PC_LOG_p1] = true;
            log->set[PC_LOG_p2] = true;
        default:
            break;
    }
}

void pc_log_close(pc_log_t* log) {
    fclose(log->file);
}

void pc_log_clear(pc_log_t* log) {
	int i;
    for (i = 0; i < PC_LOG_ITEM_COUNT; i++) {
        log->set[i] = false;
    }
}

void pc_log_flush(pc_log_t* log) {
    pc_log_print(log, "%u"  _SEP, PC_LOG_time, log->time);
    pc_log_print(log, "%hhu"_SEP, PC_LOG_mode, log->mode);
    pc_log_print(log, "%d"  _SEP, PC_LOG_lift, log->state.orient.lift);
    pc_log_print(log, "%d"  _SEP, PC_LOG_roll, log->state.orient.roll);
    pc_log_print(log, "%d"  _SEP, PC_LOG_pitch, log->state.orient.pitch);
    pc_log_print(log, "%d"  _SEP, PC_LOG_yaw, log->state.orient.yaw);
    pc_log_print(log, "%d"  _SEP, PC_LOG_ae1, log->state.motor.ae1);
    pc_log_print(log, "%d"  _SEP, PC_LOG_ae2, log->state.motor.ae2);
    pc_log_print(log, "%d"  _SEP, PC_LOG_ae3, log->state.motor.ae3);
    pc_log_print(log, "%d"  _SEP, PC_LOG_ae4, log->state.motor.ae3);
    pc_log_print(log, "%d"  _SEP, PC_LOG_sp, log->state.sensor.sp);
    pc_log_print(log, "%d"  _SEP, PC_LOG_sq, log->state.sensor.sq);
    pc_log_print(log, "%d"  _SEP, PC_LOG_sr, log->state.sensor.sr);
    pc_log_print(log, "%d"  _SEP, PC_LOG_sax, log->state.sensor.sax);
    pc_log_print(log, "%d"  _SEP, PC_LOG_say, log->state.sensor.say);
    pc_log_print(log, "%d"  _SEP, PC_LOG_saz, log->state.sensor.saz);
    pc_log_print(log, "%f"  _SEP, PC_LOG_temperature, FLOAT_FP(log->state.sensor.temperature, 8));
    pc_log_print(log, "%f"  _SEP, PC_LOG_pressure, FLOAT_FP(log->state.sensor.pressure, 8));
    pc_log_print(log, "%f"  _SEP, PC_LOG_voltage, FLOAT_FP(log->state.sensor.voltage, 8));
    pc_log_print(log, "%d"  _SEP, PC_LOG_x, log->state.pos.x);
    pc_log_print(log, "%d"  _SEP, PC_LOG_y, log->state.pos.y);
    pc_log_print(log, "%d"  _SEP, PC_LOG_z, log->state.pos.z);
    pc_log_print(log, "%f"  _SEP, PC_LOG_phi, FLOAT_FP(log->state.att.phi, 8));
    pc_log_print(log, "%f"  _SEP, PC_LOG_theta, FLOAT_FP(log->state.att.theta, 8));
    pc_log_print(log, "%f"  _SEP, PC_LOG_psi, FLOAT_FP(log->state.att.psi, 8));
    pc_log_print(log, "%d"  _SEP, PC_LOG_X, log->state.force.X);
    pc_log_print(log, "%d"  _SEP, PC_LOG_Y, log->state.force.Y);
    pc_log_print(log, "%d"  _SEP, PC_LOG_Z, log->state.force.Z);
    pc_log_print(log, "%d"  _SEP, PC_LOG_L, log->state.torque.L);
    pc_log_print(log, "%d"  _SEP, PC_LOG_M, log->state.torque.M);
    pc_log_print(log, "%d"  _SEP, PC_LOG_N, log->state.torque.N);
    pc_log_print(log, "%d"  _SEP, PC_LOG_u, log->state.velo.u);
    pc_log_print(log, "%d"  _SEP, PC_LOG_v, log->state.velo.v);
    pc_log_print(log, "%d"  _SEP, PC_LOG_w, log->state.velo.w);
    pc_log_print(log, "%d"  _SEP, PC_LOG_p, log->state.spin.p);
    pc_log_print(log, "%d"  _SEP, PC_LOG_q, log->state.spin.q);
    pc_log_print(log, "%d"  _SEP, PC_LOG_r, log->state.spin.r);
    pc_log_print(log, "%d"  _SEP, PC_LOG_yaw_p, log->state.trim.yaw_p);
    pc_log_print(log, "%d"  _SEP, PC_LOG_p1, log->state.trim.p1);
    pc_log_print(log, "%d"  _SEP, PC_LOG_p2, log->state.trim.p2);
    fprintf(log->file, _END);
}

void pc_log_print(pc_log_t* log, const char * fmt, pc_log_item_t item, ...) {
    if (log->set[item]) {
        // If the value exists we print it with the specified format string
        va_list argptr;
        va_start(argptr, item);
        vfprintf(log->file, fmt, argptr);
        va_end(argptr);
    } else {
        // If no value then print as many NANs as many %_ format arguments we got
        char* result;
        while ((result = strchr(fmt, '%')) != 0) {
            fprintf(log->file, _NAN _SEP);
            result = result + 2;
        }
    }
}
