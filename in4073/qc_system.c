#include "qc_system.h"
#include "qc_mode.h"
#include "mode_constants.h"
#include "printf.h"
#include "log.h"
#include <math.h>

#define SAFE_VOLTAGE 1050
extern bool is_test_device;
extern uint32_t iteration;

static void qc_system_log_data(qc_system_t* system);

/** =======================================================
 *  qc_system_init -- Initialise a model of the quadcopter
 *  =======================================================
 *  Initialises a qc_system_t struct in a way that the
 *  quadcopter model is ready to fly.
 *  Parameters:
 *  - system: The system to initialise.
 *  - mode: The mode to start the quadcopter in.
 *  - mode_tables: Pointer to the array containing the mode
 *      tables describing the various modes of control.
 *  - state: Pointer to the qc_state_t variable describing
 *      the internal state of the quadcopter.
 *  - command: Pointer to the qc_command_t struct doing the
 *      processing and dispatching of incoming commands.
 *  - serialcomm: Pointer to the serialcomm_t struct that
 *      controls the serial communication.
 *  - hal: Pointer to the qc_hal_t quadcopter hardware
 *      abstraction layer.
 *  Author: Boldizsar Palotas
**/
void qc_system_init(qc_system_t* system,
    qc_mode_t           mode,
    qc_mode_table_t*    mode_tables,
    qc_state_t*         state,
    qc_command_t*       command,
    serialcomm_t*       serialcomm,
    void (*rx_complete_fn)(message_t*),
    qc_hal_t*           hal
) {
    // Init system module
    system->mode                = mode;
    system->mode_tables         = mode_tables;
    system->current_mode_table  = &(system->mode_tables[(int) mode]);
    system->state               = state;
    system->command             = command;
    system->serialcomm          = serialcomm;
    system->hal                 = hal;
    system->do_logging          = false;
    system->log_mask            = 0;
    system->telemetry_mask      = 0;

    // Init command (and serialcomm within)
    qc_command_init(system->command,
        system->serialcomm, system->hal->tx_byte_fn, rx_complete_fn, system);

    // Init other members
    qc_state_init(system->state);
    if (!log_init(system->hal, system->serialcomm)) {
        qc_system_set_mode(system, MODE_1_PANIC);
        printf("> Log init error, starting in PANIC mode.\n");
    }
}

/** =======================================================
 *  qc_system_step -- Single control system step
 *  =======================================================
 *  Updates the internal state with the input sensor
 *  readings, runs the control loop and applies the
 *  calculated output to the quadcopter.
 *  Parameters:
 *  - system: The system on which to do the control loop.
 *  Author: Boldizsar Palotas
**/
void qc_system_step(qc_system_t* system) {
    system->hal->get_inputs_fn(system->state);
    qc_kalman_filter(system->state);
    if (!is_test_device && system->state->sensor.voltage_avg < SAFE_VOLTAGE) {
       if(system->mode != MODE_1_PANIC)
           printf("Low voltage (V = %"PRId32" centivolts)\n", system->state->sensor.voltage);

       qc_system_set_mode(system, MODE_1_PANIC);
    }

    qc_command_tick(system->command);

    profile_start_tag(&system->state->prof.pr[1], system->hal->get_time_us_fn(), iteration);
    system->current_mode_table->control_fn(system->state);
    profile_end(&system->state->prof.pr[1], system->hal->get_time_us_fn());

    system->hal->enable_motors_fn(
        system->current_mode_table->motor_on_fn(system->state)
        && system->state->option.enable_motors
        && ZERO_LIFT_THRESHOLD < system->state->orient.lift);

    system->hal->set_outputs_fn(system->state);

    qc_system_log_data(system);
}

void qc_kalman_filter(qc_state_t* state) {
    state->sensor.sphi = fp_angle_clip(
        FP_MUL1(FP_MUL1(T_CONST , state->sensor.sp, T_CONST_FRAC_BITS) + state->sensor.sphi,
                KALMAN_GYRO_WEIGHT, KALMAN_WEIGHT_FRAC_BITS)
        + FP_MUL1(fp_asin_t1(FP_MUL1( - state->sensor.say, KALMAN_M, KALMAN_M_FRAC_BITS)),
                KALMAN_ACC_WEIGHT, KALMAN_WEIGHT_FRAC_BITS));

    state->sensor.stheta = fp_angle_clip(
        FP_MUL1(FP_MUL1(T_CONST , state->sensor.sq, T_CONST_FRAC_BITS) + state->sensor.stheta,
                KALMAN_GYRO_WEIGHT, KALMAN_WEIGHT_FRAC_BITS)
      + FP_MUL1(fp_asin_t1(FP_MUL1(state->sensor.sax, KALMAN_M, KALMAN_M_FRAC_BITS)),
                KALMAN_ACC_WEIGHT, KALMAN_WEIGHT_FRAC_BITS));

    state->sensor.spsi = fp_angle_clip(state->sensor.spsi +
        FP_MUL1(T_CONST , state->sensor.sr, T_CONST_FRAC_BITS));
}

/** =======================================================
 *  qc_system_set_mode -- Set new operating mode.
 *  =======================================================
 *  Checks whether the requested mode can be reached from
 *  the current mode and switches to it if it is.
 *  Parameters:
 *  - system: The system on which to do the mode switch.
 *  - mode: The mode to switch to.
 *  Author: Boldizsar Palotas
**/
void qc_system_set_mode(qc_system_t* system, qc_mode_t mode) {
    if (!system->current_mode_table->trans_fn(system->state, mode))
        return;
    if (!IS_SAFE_OR_PANIC_MODE(mode) && ZERO_LIFT_THRESHOLD < system->state->orient.lift) {
        printf("Turn motor speed down first!\n");
        return;
    }
    if (!IS_SAFE_OR_PANIC_MODE(mode) && mode != MODE_3_CALIBRATE
        && mode != MODE_2_MANUAL && !system->state->offset.calibrated) {
        printf("Calibrate Quadcopter first!\n");
        return;
    }

    qc_mode_t old_mode = system->mode;
    system->mode = mode;
    system->current_mode_table = &system->mode_tables[(int) mode];
    system->current_mode_table->enter_fn(system->state, old_mode);

    serialcomm_quick_send(system->serialcomm, MESSAGE_TIME_MODE_VOLTAGE_ID,
        system->hal->get_time_us_fn(),
        system->mode | (system->state->sensor.voltage << 16) );
}

static void qc_system_log_data(qc_system_t* system) {
    int pr_id;
    uint32_t bit_mask, index;
    for (bit_mask = 0x01, index = 0; bit_mask; bit_mask = bit_mask << 1, index++) {
        message_t msg;
        msg.ID = index;
        switch (index) {
            case MESSAGE_TIME_MODE_VOLTAGE_ID:
                MESSAGE_TIME_VALUE(&msg) = system->hal->get_time_us_fn();
                MESSAGE_MODE_VALUE(&msg) = system->mode;
                MESSAGE_VOLTAGE_VALUE(&msg) = system->state->sensor.voltage;
                break;
            case MESSAGE_SETPOINT_ID:
                MESSAGE_SETPOINT_LIFT_VALUE(&msg)   = system->state->orient.lift >> LIFT_SHIFT;
                MESSAGE_SETPOINT_ROLL_VALUE(&msg)   = system->state->orient.roll >> ROLL_SHIFT;
                MESSAGE_SETPOINT_PITCH_VALUE(&msg)  = system->state->orient.pitch >> PITCH_SHIFT;
                MESSAGE_SETPOINT_YAW_VALUE(&msg)    = system->state->orient.yaw >> YAW_SHIFT;
                break;
            case MESSAGE_SPQR_ID:
                MESSAGE_SP_VALUE(&msg) = FP_CHUNK(system->state->sensor.sp, 8, 16);
                MESSAGE_SQ_VALUE(&msg) = FP_CHUNK(system->state->sensor.sq, 8, 16);
                MESSAGE_SR_VALUE(&msg) = FP_CHUNK(system->state->sensor.sr, 8, 16);
                break;
            case MESSAGE_SAXYZ_ID:
                MESSAGE_SAX_VALUE(&msg) = FP_CHUNK(system->state->sensor.sax, 8, 16);
                MESSAGE_SAY_VALUE(&msg) = FP_CHUNK(system->state->sensor.say, 8, 16);
                MESSAGE_SAZ_VALUE(&msg) = FP_CHUNK(system->state->sensor.saz, 8, 16);
                break;
            case MESSAGE_AE1234_ID:
                MESSAGE_AE1_VALUE(&msg) = system->state->motor.ae1;
                MESSAGE_AE2_VALUE(&msg) = system->state->motor.ae2;
                MESSAGE_AE3_VALUE(&msg) = system->state->motor.ae3;
                MESSAGE_AE4_VALUE(&msg) = system->state->motor.ae4;
                break;
            case MESSAGE_TEMP_PRESSURE_ID:
                MESSAGE_TEMP_VALUE(&msg) = system->state->sensor.temperature;
               // MESSAGE_PRESSURE_VALUE(&msg) = system->state->sensor.pressure;
                MESSAGE_PRESSURE_VALUE(&msg) = system->state->sensor.pressure_avg;
                break;
            case MESSAGE_XYZPOS_ID:
                MESSAGE_XPOS_VALUE(&msg) = system->state->pos.x;
                MESSAGE_YPOS_VALUE(&msg) = system->state->pos.y;
                MESSAGE_ZPOS_VALUE(&msg) = system->state->pos.z;
                break;
            case MESSAGE_PHI_THETA_PSI_ID:
                MESSAGE_PHI_VALUE(&msg)     = FP_CHUNK(system->state->att.phi, 8, 16);
                MESSAGE_THETA_VALUE(&msg)   = FP_CHUNK(system->state->att.theta, 8, 16);
                MESSAGE_PSI_VALUE(&msg)     = FP_CHUNK(system->state->att.psi, 8, 16);
                break;
            case MESSAGE_XYZFORCE_ID:
                MESSAGE_XFORCE_VALUE(&msg) = FP_CHUNK(system->state->force.X, 8, 16);
                MESSAGE_YFORCE_VALUE(&msg) = FP_CHUNK(system->state->force.Y, 8, 16);
                MESSAGE_ZFORCE_VALUE(&msg) = FP_CHUNK(system->state->force.Z, 8, 16);
                break;
            case MESSAGE_LMN_ID:
                MESSAGE_L_VALUE(&msg) = FP_CHUNK(system->state->torque.L, 8, 16);
                MESSAGE_M_VALUE(&msg) = FP_CHUNK(system->state->torque.M, 8, 16);
                MESSAGE_N_VALUE(&msg) = FP_CHUNK(system->state->torque.N, 8, 16);
                break;
            case MESSAGE_UVW_ID:
                MESSAGE_U_VALUE(&msg) = FP_CHUNK(system->state->velo.u, 8, 16);
                MESSAGE_V_VALUE(&msg) = FP_CHUNK(system->state->velo.v, 8, 16);
                MESSAGE_W_VALUE(&msg) = FP_CHUNK(system->state->velo.w, 8, 16);
                break;
            case MESSAGE_PQR_ID:
                MESSAGE_P_VALUE(&msg) = FP_CHUNK(system->state->spin.p, 8, 16);
                MESSAGE_Q_VALUE(&msg) = FP_CHUNK(system->state->spin.q, 8, 16);
                MESSAGE_R_VALUE(&msg) = FP_CHUNK(system->state->spin.r, 8, 16);
                break;
            case MESSAGE_P12_ID:
                MESSAGE_P1_VALUE(&msg) = system->state->trim.p1;
                MESSAGE_P2_VALUE(&msg) = system->state->trim.p2;
                MESSAGE_YAWP_VALUE(&msg) = system->state->trim.yaw_p;
                break;
            case MESSAGE_S_ATT_ID:
                MESSAGE_S_PHI_VALUE(&msg)   = FP_CHUNK(system->state->sensor.sphi, 8, 16);
                MESSAGE_S_THETA_VALUE(&msg) = FP_CHUNK(system->state->sensor.stheta, 8, 16);
                MESSAGE_S_PSI_VALUE(&msg)   = FP_CHUNK(system->state->sensor.spsi, 8, 16);
                break;
            case MESSAGE_PROFILE_0_CURR_ID:
            case MESSAGE_PROFILE_1_CURR_ID:
            case MESSAGE_PROFILE_2_CURR_ID:
            case MESSAGE_PROFILE_3_CURR_ID:
            case MESSAGE_PROFILE_4_CURR_ID:
                pr_id = index - MESSAGE_PROFILE_0_CURR_ID;
                MESSAGE_PROFILE_TIME_VALUE(&msg) = system->state->prof.pr[pr_id].last_delta;
                MESSAGE_PROFILE_TAG_VALUE(&msg) = system->state->prof.pr[pr_id].last_tag;
                break;
            case MESSAGE_PROFILE_0_MAX_ID:
            case MESSAGE_PROFILE_1_MAX_ID:
            case MESSAGE_PROFILE_2_MAX_ID:
            case MESSAGE_PROFILE_3_MAX_ID:
            case MESSAGE_PROFILE_4_MAX_ID:
                pr_id = index - MESSAGE_PROFILE_0_MAX_ID;
                MESSAGE_PROFILE_TIME_VALUE(&msg) = system->state->prof.pr[pr_id].max_delta;
                MESSAGE_PROFILE_TAG_VALUE(&msg) = system->state->prof.pr[pr_id].max_tag;
                break;
            default:
                continue;
                break;
        }

        if (system->do_logging && system->log_mask & bit_mask) {
            log_write(&msg);
        }

        if (system->telemetry_mask & bit_mask) {
            serialcomm_quick_send(system->serialcomm, msg.ID, msg.value.v32[0], msg.value.v32[1]);
        }
    }
}
