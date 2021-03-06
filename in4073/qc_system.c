#include "qc_system.h"
#include "qc_mode.h"
#include "mode_constants.h"
#include "printf.h"
#include "log.h"
#include <math.h>

#define SAFE_VOLTAGE 1050
extern bool is_test_device;
extern uint32_t iteration;

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
 *  - rx_complete_fn: The function to be called when a
 *      message is received succesfully.
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
    qc_system_set_raw(system, false);
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

    if (!is_test_device && system->state->sensor.voltage_avg < SAFE_VOLTAGE) {
       if(system->mode != MODE_1_PANIC)
           printf("Low voltage (V = %"PRId32" centivolts)\n", system->state->sensor.voltage);

       qc_system_set_mode(system, MODE_1_PANIC);
    }

    qc_command_tick(system->command);

    // Profile 1: Time needed to calculate everything in the control function.
    profile_start_tag(&system->state->prof.pr[1], system->hal->get_time_us_fn(), iteration);

    // The main control function of the current mode.
    system->current_mode_table->control_fn(system->state);

    // End profile 1.
    profile_end(&system->state->prof.pr[1], system->hal->get_time_us_fn());

    // Enable motors after various safety checks.
    system->hal->enable_motors_fn(
        system->current_mode_table->motor_on_fn(system->state)
        && system->state->option.enable_motors
        && ZERO_LIFT_THRESHOLD < system->state->orient.lift);

    // Set motor outputs.
    system->hal->set_outputs_fn(system->state);
}

/** =======================================================
 *  qc_kalman_filter -- Predict/filter attitude and spin bias
 *  =======================================================
 *  Predicts the values of phi, theta and psi and also
 *  continously updates the offset (bias) values of sp and
 *  sq based on the accelerometer readings.
 *
 *  Parameters:
 *  - state: The state in which to do the filtering.
 *  Author: Boldizsar Palotas
**/
void qc_kalman_filter(qc_state_t* state) {
    // Task 1: Estimate attitude angles phi and theta
    // ----------------------------------------------
    //
    // The estimate is a wighted average of sphi_prev + t * sp and
    // sphi_acc, where sphi_prev is the previous estimate, t is the
    // time constant (T_CONST_RAW) and sphi_acc is the estimated value
    // based on only the accelerometer reading. Same for stheta. The
    // weights are the KALMAN_GYRO_WEIGHT and KALMAN_ACC_WEIGHT.
    // 
    // Note that this isn't real Kalman filtering because we don't
    // estimate the state covariance and apply a constant gain intead
    // of a calculated optimal Kalman gain. The gain corresponds to
    // KALMAN_ACC_WEIGHT. This reduces the drift of sphi and stheta to
    // zero but the reduction might not be at an optimal level.
    //
    // Relation to the In4073 QR Controller Theory paper [1]
    // -----------------------------------------------------
    //
    // The state estimation technique presented there is the same as
    // basic technique as the one used here (which also isn't full
    // Kalman filtering). Our KALMAN_ACC_WEIGHT constant corresponds
    // to 1/C1. The bias term corresponds to our state.offset values
    // which are calculated at calibration and left as a constant.
    //
    // [1] In4073 QR Controller Theory (Arjan J.C. van Gemund, 2012)
    // http://www.st.ewi.tudelft.nl/~koen/in4073/Resources/kalman_control.pdf

    q32_t phi_state_est = state->sensor.sphi +
        FP_MUL1(T_CONST_RAW, state->sensor.sp, T_CONST_FRAC_BITS);
    q32_t phi_meas_est = fp_asin_t1(FP_MUL1( - state->sensor.say, KALMAN_M, KALMAN_M_FRAC_BITS));
    state->sensor.sphi = fp_angle_clip(
        FP_MUL1(phi_state_est, KALMAN_GYRO_WEIGHT, KALMAN_WEIGHT_FRAC_BITS) +
        FP_MUL1(phi_meas_est, KALMAN_ACC_WEIGHT, KALMAN_WEIGHT_FRAC_BITS));

    q32_t theta_state_est = state->sensor.stheta +
        FP_MUL1(T_CONST_RAW, state->sensor.sq, T_CONST_FRAC_BITS);
    q32_t theta_meas_est = fp_asin_t1(FP_MUL1(state->sensor.sax, KALMAN_M, KALMAN_M_FRAC_BITS));
    state->sensor.stheta = fp_angle_clip(
        FP_MUL1(theta_state_est, KALMAN_GYRO_WEIGHT, KALMAN_WEIGHT_FRAC_BITS) +
        FP_MUL1(theta_meas_est, KALMAN_ACC_WEIGHT, KALMAN_WEIGHT_FRAC_BITS));

    state->sensor.spsi = fp_angle_clip(state->sensor.spsi +
        FP_MUL1(T_CONST_RAW , state->sensor.sr, T_CONST_FRAC_BITS));

    // Task 2: Updating offset terms.
    state->offset.sp += FP_MUL1(KALMAN_OFFSET_WEIGHT, phi_state_est - phi_meas_est, KALMAN_OFFSET_FRAC_BITS);
    state->offset.sq += FP_MUL1(KALMAN_OFFSET_WEIGHT, theta_state_est - theta_meas_est, KALMAN_OFFSET_FRAC_BITS);

    qc_kalman_height(state);
}

/** =======================================================
 *  qc_kalman_height -- Predict/filter z coordinate and w.
 *  =======================================================
 *  Predicts the values of the vertical position in the QC
 *  frame (higher values towards the ground) based on mostly
 *  the pressure sensor readings (and the accelerometer a
 *  bit).
 *
 *  Parameters:
 *  - state: The state in which to do the filtering.
 *  Author: Boldizsar Palotas
**/
void qc_kalman_height(qc_state_t* state) {
    const int t = state->option.raw_control ? T_CONST_RAW : T_CONST;

    // Task 1: estimate w velocity (based on accelerometer integration + pressure sensor derivation)

    q32_t w_int_est = state->velo.w + FP_MUL1(t , 10 * state->sensor.saz, T_CONST_FRAC_BITS);
    q32_t w_deriv_est = FP_MUL1(_1_T_PRES,
        FP_MUL1(KALMAN_PRES , state->sensor.pressure_avg - state->sensor.prev_pressure_avg,
        KALMAN_PRES_FRAC_BITS), _1_T_PRES_FRAC_BITS);
    q32_t w_est = FP_MUL1(KALMAN_PRES_ACC_WEIGHT, w_int_est, KALMAN_PRES_WEIGHT_FRAC_BITS) +
        FP_MUL1(KALMAN_PRES_PRS_WEIGHT, w_deriv_est, KALMAN_PRES_WEIGHT_FRAC_BITS);

    static uint32_t counter = 0;
    counter++;
    if ((counter & 0xf) == 0)
        ;//printf("w_int:%8"PRId32" w_der:%8"PRId32" w_avg:%8"PRId32"\n", w_int_est, w_deriv_est, w_est);

    if (w_est < KALMAN_W_MIN) {
        w_est = KALMAN_W_MIN;
    } else if (KALMAN_W_MAX < w_est) {
        w_est = KALMAN_W_MAX;
    }
    state->velo.w = w_est;

    // Task 2: estimate z coordinate (based on speed + pressure sensor)

    q32_t z_state_est = state->pos.z + FP_MUL1(t , state->velo.w, T_CONST_FRAC_BITS);
    q32_t z_meas_est = FP_MUL3(KALMAN_PRES , state->sensor.pressure_avg, 0, KALMAN_PRES_FRAC_BITS, 0);
    q32_t z_est = FP_MUL1(KALMAN_PRES_ACC_WEIGHT, z_state_est, KALMAN_PRES_WEIGHT_FRAC_BITS) +
        FP_MUL1(KALMAN_PRES_PRS_WEIGHT, z_meas_est, KALMAN_PRES_WEIGHT_FRAC_BITS);

    if ((counter & 0xf) == 0){
        //printf("z_state:%8"PRId32" pravg:%"PRId32" z_meas:%9"PRId32" z_avg:%9"PRId32"\n", z_state_est, state->sensor.pressure_avg, z_meas_est, z_est);
        //printf("pres:%9"PRId32" offset:%"PRId32"\n", state->sensor.pressure, state->offset.pressure);
    }
    if (z_est < KALMAN_Z_MIN) {
        z_est = KALMAN_Z_MIN;
    } else if (KALMAN_Z_MAX < z_est) {
        z_est = KALMAN_Z_MAX;
    }
    state->pos.z = z_est;

}

/** =======================================================
 *  qc_system_set_raw -- Set the raw mode of the QC.
 *  =======================================================
 *  Tries setting the raw control option to the desired
 *  value. If this is not possible, return without change.
 *
 *  Parameters:
 *  - system: The system to apply the change to.
 *  - raw: The desired value.
 *  Author: Boldizsar Palotas
**/
void qc_system_set_raw(qc_system_t* system, bool raw) {
    if (system->mode != MODE_0_SAFE) {
        printf("Not in safe mode, not changing raw mode!\n");
        return;
    }

    system->state->option.raw_control = raw;
    if (raw) {
        system->hal->imu_init_fn(false, IMU_RAW_FREQ);
        printf("IMU reset. \nRAW MODE turned ON.\n");
    } else {
        system->hal->imu_init_fn(true, 0);
        printf("IMU and DMP reset. \nRAW MODE turned OFF.\n");
    }
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

/** =======================================================
 *  qc_system_log_data -- Do logging and telemetry collection.
 *  =======================================================
 *  Logs and/or sends the desired telemetry based on the bit
 *  mask set by the user.
 *
 *  Parameters:
 *  - system: The system from which to log the data.
 *  Author: Boldizsar Palotas
**/
void qc_system_log_data(qc_system_t* system) {
    int send_cnt = 0;
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
            case MESSAGE_Z_Z_PRES_ID:
                MESSAGE_ZPOS_VALUE(&msg) = FP_CHUNK(system->state->pos.z, 8, 16);
                MESSAGE_ZFORCE_VALUE(&msg) = FP_CHUNK(system->state->force.Z, 8, 16);
                MESSAGE_PRES_VALUE(&msg) = system->state->sensor.pressure;
                break;
            case MESSAGE_PHI_THETA_PSI_ID:
                MESSAGE_PHI_VALUE(&msg)     = FP_CHUNK(system->state->att.phi, 8, 16);
                MESSAGE_THETA_VALUE(&msg)   = FP_CHUNK(system->state->att.theta, 8, 16);
                MESSAGE_PSI_VALUE(&msg)     = FP_CHUNK(system->state->att.psi, 8, 16);
                break;
            case MESSAGE_LMN_ID:
                MESSAGE_L_VALUE(&msg) = FP_CHUNK(system->state->torque.L, 8, 16);
                MESSAGE_M_VALUE(&msg) = FP_CHUNK(system->state->torque.M, 8, 16);
                MESSAGE_N_VALUE(&msg) = FP_CHUNK(system->state->torque.N, 8, 16);
                break;
            case MESSAGE_PQR_ID:
                MESSAGE_P_VALUE(&msg) = FP_CHUNK(system->state->spin.p, 8, 16);
                MESSAGE_Q_VALUE(&msg) = FP_CHUNK(system->state->spin.q, 8, 16);
                MESSAGE_R_VALUE(&msg) = FP_CHUNK(system->state->spin.r, 8, 16);
                break;
            case MESSAGE_S_ATT_ID:
                MESSAGE_S_PHI_VALUE(&msg)   = FP_CHUNK(system->state->sensor.sphi, 8, 16);
                MESSAGE_S_THETA_VALUE(&msg) = FP_CHUNK(system->state->sensor.stheta, 8, 16);
                MESSAGE_S_PSI_VALUE(&msg)   = FP_CHUNK(system->state->sensor.spsi, 8, 16);
                break;
            case MESSAGE_PROFILE_ID:
                MESSAGE_PROFILE_0_VALUE(&msg) = system->state->prof.pr[0].last_delta;
                MESSAGE_PROFILE_1_VALUE(&msg) = system->state->prof.pr[1].last_delta;
                MESSAGE_PROFILE_2_VALUE(&msg) = system->state->prof.pr[2].last_delta;
                MESSAGE_PROFILE_3_VALUE(&msg) = system->state->prof.pr[3].last_delta;
                break;
            case MESSAGE_PROFILE_4_ID:
                MESSAGE_PROFILE_4_VALUE(&msg) = system->state->prof.pr[4].last_delta;
                break;
            default:
                continue;
                break;
        }

        if (system->do_logging && system->log_mask & bit_mask) {
            log_write(&msg);
        }

        if (system->telemetry_mask & bit_mask) {
            send_cnt++;
            if (10 < send_cnt) {
                system->telemetry_mask = system->telemetry_mask & (bit_mask - 1);
                printf("Too many messages, TELEMETRY MASK automatically reset to %#"PRIx32"!\n", system->telemetry_mask);
                break;
            }
            serialcomm_quick_send(system->serialcomm, msg.ID, msg.value.v32[0], msg.value.v32[1]);
        }
    }
}
