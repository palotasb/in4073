#include "pc_log.h"
#include "../serialcomm.h"
#include "../fixedpoint.h"
#include "../qc_mode.h"
#include <string.h>
#include <stdarg.h>
#ifdef WINDOWS
    #include <windows.h>
#endif

// Value to print in placeof on unknown number
#define _NAN    "NaN"

// Value to print to separate values
#define _SEP    "\t"

// Value to print to separate entries
#define _END    "\n"

static void pc_log_flush(pc_log_t* log);
static void pc_log_clear(pc_log_t* log);
static void pc_log_print(pc_log_t* log, const char * fmt, pc_log_item_t item, ...);

bool pc_log_init(pc_log_t* log, FILE* file) {
    log->file = file;
    qc_state_init(&log->state);
    log->time = 0;
    log->mode = MODE_UNKNOWN;
    log->initialised = false;
    pc_log_clear(log);
    return true;
}

void pc_log_receive(pc_log_t* log, message_t* message) {
    switch (message->ID) {
        case MESSAGE_LOG_END_ID:
            if (log->initialised) {
                pc_log_flush(log);
            }
            break;
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
        case MESSAGE_SETPOINT_ID:
            if (log->initialised && log->set[PC_LOG_lift]) {
                pc_log_flush(log);
            }
            log->state.orient.lift  = MESSAGE_SETPOINT_LIFT_VALUE(message);
            log->state.orient.roll  = MESSAGE_SETPOINT_ROLL_VALUE(message);
            log->state.orient.pitch = MESSAGE_SETPOINT_PITCH_VALUE(message);
            log->state.orient.yaw   = MESSAGE_SETPOINT_YAW_VALUE(message);
            log->set[PC_LOG_lift] = true;
            log->set[PC_LOG_roll] = true;
            log->set[PC_LOG_pitch] = true;
            log->set[PC_LOG_yaw] = true;
            break;
        case MESSAGE_SPQR_ID:
            if (log->initialised && log->set[PC_LOG_sp]) {
                pc_log_flush(log);
            }
            log->state.sensor.sp = FP_EXTEND(MESSAGE_SP_VALUE(message), 16, 8);
            log->state.sensor.sq = FP_EXTEND(MESSAGE_SQ_VALUE(message), 16, 8);
            log->state.sensor.sr = FP_EXTEND(MESSAGE_SR_VALUE(message), 16, 8);
            log->set[PC_LOG_sp] = true;
            log->set[PC_LOG_sq] = true;
            log->set[PC_LOG_sr] = true;
            break;
        case MESSAGE_SAXYZ_ID:
            if (log->initialised && log->set[PC_LOG_sax]) {
                pc_log_flush(log);
            }
            log->state.sensor.sax = FP_EXTEND(MESSAGE_SAX_VALUE(message), 16, 8);
            log->state.sensor.say = FP_EXTEND(MESSAGE_SAY_VALUE(message), 16, 8);
            log->state.sensor.saz = FP_EXTEND(MESSAGE_SAZ_VALUE(message), 16, 8);
            log->set[PC_LOG_sax] = true;
            log->set[PC_LOG_say] = true;
            log->set[PC_LOG_saz] = true;
            break;
        case MESSAGE_S_ATT_ID:
            if (log->initialised && log->set[PC_LOG_sphi]) {
                pc_log_flush(log);
            }
            log->state.sensor.sphi =    FP_EXTEND(MESSAGE_S_PHI_VALUE(message), 16, 8);
            log->state.sensor.stheta =  FP_EXTEND(MESSAGE_S_THETA_VALUE(message), 16, 8);
            log->state.sensor.spsi =    FP_EXTEND(MESSAGE_S_PSI_VALUE(message), 16, 8);
            log->set[PC_LOG_sphi]   = true;
            log->set[PC_LOG_stheta] = true;
            log->set[PC_LOG_spsi]   = true;
            break;
        case MESSAGE_AE1234_ID:
            if (log->initialised && log->set[PC_LOG_ae1]) {
                pc_log_flush(log);
            }
            log->state.motor.ae1 = MESSAGE_AE1_VALUE(message);
            log->state.motor.ae2 = MESSAGE_AE2_VALUE(message);
            log->state.motor.ae3 = MESSAGE_AE3_VALUE(message);
            log->state.motor.ae4 = MESSAGE_AE4_VALUE(message);
            log->set[PC_LOG_ae1] = true;
            log->set[PC_LOG_ae2] = true;
            log->set[PC_LOG_ae3] = true;
            log->set[PC_LOG_ae4] = true;
            break;
        case MESSAGE_Z_Z_PRES_ID:
            if (log->initialised && log->set[PC_LOG_z]) {
                pc_log_flush(log);
            }
            log->state.pos.z            =   FP_EXTEND(MESSAGE_ZPOS_VALUE(message), 16, 8);
            log->state.force.Z          =   FP_EXTEND(MESSAGE_ZFORCE_VALUE(message), 16, 8);
            log->state.sensor.pressure  =   MESSAGE_PRES_VALUE(message);
            log->set[PC_LOG_z] =     true;
            log->set[PC_LOG_Z] =     true;
            log->set[PC_LOG_pressure] =     true;
            break;
        case MESSAGE_PHI_THETA_PSI_ID:
            if (log->initialised && log->set[PC_LOG_phi]) {
                pc_log_flush(log);
            }
            log->state.att.phi =    FP_EXTEND(MESSAGE_PHI_VALUE(message), 16, 8);
            log->state.att.theta =  FP_EXTEND(MESSAGE_THETA_VALUE(message), 16, 8);
            log->state.att.psi =    FP_EXTEND(MESSAGE_PSI_VALUE(message), 16, 8);
            log->set[PC_LOG_phi] =      true;
            log->set[PC_LOG_theta] =    true;
            log->set[PC_LOG_psi] =      true;
            break;
        case MESSAGE_LMN_ID:
            if (log->initialised && log->set[PC_LOG_L]) {
                pc_log_flush(log);
            }
            log->state.torque.L = FP_EXTEND(MESSAGE_L_VALUE(message), 16, 8);
            log->state.torque.M = FP_EXTEND(MESSAGE_M_VALUE(message), 16, 8);
            log->state.torque.N = FP_EXTEND(MESSAGE_N_VALUE(message), 16, 8);
            log->set[PC_LOG_L] = true;
            log->set[PC_LOG_M] = true;
            log->set[PC_LOG_N] = true;
            break;
        case MESSAGE_PQR_ID:
            if (log->initialised && log->set[PC_LOG_p]) {
                pc_log_flush(log);
            }
            log->state.spin.p = FP_EXTEND(MESSAGE_P_VALUE(message), 16, 8);
            log->state.spin.q = FP_EXTEND(MESSAGE_Q_VALUE(message), 16, 8);
            log->state.spin.r = FP_EXTEND(MESSAGE_R_VALUE(message), 16, 8);
            log->set[PC_LOG_p] = true;
            log->set[PC_LOG_q] = true;
            log->set[PC_LOG_r] = true;
            break;
        case MESSAGE_PROFILE_ID:
            if (log->initialised && log->set[PC_LOG_PR0_CURR]) {
                pc_log_flush(log);
            }
            log->state.prof.pr[0].last_delta = MESSAGE_PROFILE_0_VALUE(message);
            log->state.prof.pr[1].last_delta = MESSAGE_PROFILE_1_VALUE(message);
            log->state.prof.pr[2].last_delta = MESSAGE_PROFILE_2_VALUE(message);
            log->state.prof.pr[3].last_delta = MESSAGE_PROFILE_3_VALUE(message);
            log->set[PC_LOG_PR0_CURR] = true;
            log->set[PC_LOG_PR1_CURR] = true;
            log->set[PC_LOG_PR2_CURR] = true;
            log->set[PC_LOG_PR3_CURR] = true;
            break;
        case MESSAGE_PROFILE_4_ID:
            if (log->initialised && log->set[PC_LOG_PR4_CURR]) {
                pc_log_flush(log);
            }
            log->state.prof.pr[4].last_delta = MESSAGE_PROFILE_4_VALUE(message);
            log->set[PC_LOG_PR4_CURR] = true;
            break;
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

    /*  1 */    pc_log_print(log, "%u"  _SEP, PC_LOG_time, log->time);
    /*  2 */    pc_log_print(log, "%hhu"_SEP, PC_LOG_mode, log->mode);
    /*  3 */    pc_log_print(log, "%f"  _SEP, PC_LOG_lift,  FLOAT_FP(log->state.orient.lift, 8));
    /*  4 */    pc_log_print(log, "%f"  _SEP, PC_LOG_roll,  FLOAT_FP(log->state.orient.roll, 14));
    /*  5 */    pc_log_print(log, "%f"  _SEP, PC_LOG_pitch, FLOAT_FP(log->state.orient.pitch, 14));
    /*  6 */    pc_log_print(log, "%f"  _SEP, PC_LOG_yaw,   FLOAT_FP(log->state.orient.yaw, 10));
    /*  7 */    pc_log_print(log, "%d"  _SEP, PC_LOG_ae1, log->state.motor.ae1);
    /*  8 */    pc_log_print(log, "%d"  _SEP, PC_LOG_ae2, log->state.motor.ae2);
    /*  9 */    pc_log_print(log, "%d"  _SEP, PC_LOG_ae3, log->state.motor.ae3);
    /* 10 */    pc_log_print(log, "%d"  _SEP, PC_LOG_ae4, log->state.motor.ae4);
    /* 11 */    pc_log_print(log, "%f"  _SEP, PC_LOG_sp,    FLOAT_FP(log->state.sensor.sp, 16));
    /* 12 */    pc_log_print(log, "%f"  _SEP, PC_LOG_sq,    FLOAT_FP(log->state.sensor.sq, 16));
    /* 13 */    pc_log_print(log, "%f"  _SEP, PC_LOG_sr,    FLOAT_FP(log->state.sensor.sr, 16));
    /* 14 */    pc_log_print(log, "%f"  _SEP, PC_LOG_sax,   FLOAT_FP(log->state.sensor.sax, 16));
    /* 15 */    pc_log_print(log, "%f"  _SEP, PC_LOG_say,   FLOAT_FP(log->state.sensor.say, 16));
    /* 16 */    pc_log_print(log, "%f"  _SEP, PC_LOG_saz,   FLOAT_FP(log->state.sensor.saz, 16));
    /* 17 */    pc_log_print(log, "%f"  _SEP, PC_LOG_temperature, FLOAT_FP(log->state.sensor.temperature, 8));
    /* 18 */    pc_log_print(log, "%f"  _SEP, PC_LOG_pressure, FLOAT_FP(log->state.sensor.pressure, 16));
    /* 19 */    pc_log_print(log, "%f"  _SEP, PC_LOG_voltage, (float)(log->state.sensor.voltage) / 100.0f);
    /* 20 */    pc_log_print(log, "%f"  _SEP, PC_LOG_x, FLOAT_FP(log->state.pos.x, 16));
    /* 21 */    pc_log_print(log, "%f"  _SEP, PC_LOG_y, FLOAT_FP(log->state.pos.y, 16));
    /* 22 */    pc_log_print(log, "%f"  _SEP, PC_LOG_z, FLOAT_FP(log->state.pos.z, 16));
    /* 23 */    pc_log_print(log, "%f"  _SEP, PC_LOG_phi,   FLOAT_FP(log->state.att.phi, 16));
    /* 24 */    pc_log_print(log, "%f"  _SEP, PC_LOG_theta, FLOAT_FP(log->state.att.theta, 16));
    /* 25 */    pc_log_print(log, "%f"  _SEP, PC_LOG_psi,   FLOAT_FP(log->state.att.psi, 16));
    /* 26 */    pc_log_print(log, "%f"  _SEP, PC_LOG_X, FLOAT_FP(log->state.force.X, 16));
    /* 27 */    pc_log_print(log, "%f"  _SEP, PC_LOG_Y, FLOAT_FP(log->state.force.Y, 16));
    /* 28 */    pc_log_print(log, "%f"  _SEP, PC_LOG_Z, FLOAT_FP(log->state.force.Z, 16));
    /* 29 */    pc_log_print(log, "%f"  _SEP, PC_LOG_L, FLOAT_FP(log->state.torque.L, 16));
    /* 30 */    pc_log_print(log, "%f"  _SEP, PC_LOG_M, FLOAT_FP(log->state.torque.M, 16));
    /* 31 */    pc_log_print(log, "%f"  _SEP, PC_LOG_N, FLOAT_FP(log->state.torque.N, 16));
    /* 32 */    pc_log_print(log, "%f"  _SEP, PC_LOG_u, FLOAT_FP(log->state.velo.u, 16));
    /* 33 */    pc_log_print(log, "%f"  _SEP, PC_LOG_v, FLOAT_FP(log->state.velo.v, 16));
    /* 34 */    pc_log_print(log, "%f"  _SEP, PC_LOG_w, FLOAT_FP(log->state.velo.w, 16));
    /* 35 */    pc_log_print(log, "%f"  _SEP, PC_LOG_p, FLOAT_FP(log->state.spin.p, 16));
    /* 36 */    pc_log_print(log, "%f"  _SEP, PC_LOG_q, FLOAT_FP(log->state.spin.q, 16));
    /* 37 */    pc_log_print(log, "%f"  _SEP, PC_LOG_r, FLOAT_FP(log->state.spin.r, 16));
    /* 38 */    pc_log_print(log, "%d"  _SEP, PC_LOG_yaw_p, log->state.trim.yaw_p);
    /* 39 */    pc_log_print(log, "%d"  _SEP, PC_LOG_p1, log->state.trim.p1);
    /* 40 */    pc_log_print(log, "%d"  _SEP, PC_LOG_p2, log->state.trim.p2);
    /* n=0..4 */for (int i = 0; i < QC_STATE_PROF_CNT; i++) {
      /* 41+4*n, 42+4*n */  pc_log_print(log, "%u"_SEP"%u" _SEP, PC_LOG_PR0_CURR+i, log->state.prof.pr[i].last_delta, log->state.prof.pr[i].last_tag);
      /* 43+4*n, 44+4*n */  pc_log_print(log, "%u"_SEP"%u" _SEP, PC_LOG_PR0_MAX+i, log->state.prof.pr[i].max_delta, log->state.prof.pr[i].max_tag);
                }
    /* 61 */    pc_log_print(log, "%f"  _SEP, PC_LOG_sphi,      FLOAT_FP(log->state.sensor.sphi, 16));
    /* 62 */    pc_log_print(log, "%f"  _SEP, PC_LOG_stheta,    FLOAT_FP(log->state.sensor.stheta, 16));
    /* 63 */    pc_log_print(log, "%f"  _SEP, PC_LOG_spsi,      FLOAT_FP(log->state.sensor.spsi, 16));

    fprintf(log->file, _END);
    fflush(log->file);
    #ifdef WINDOWS
        FlushFileBuffers((HANDLE) _fileno(log->file));
    #endif

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
        const char * result = fmt;
        while (result && (result = strchr(result, '%')) != 0) {
            fprintf(log->file, _NAN _SEP);
            if (result[1] == '\0' || result[2] == '\0') {
                result = 0;
            } else {
                result = result + 2; // Step 2 to avoid %% escape sequence
            }
        }
    }
}
