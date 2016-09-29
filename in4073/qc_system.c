#include "qc_system.h"
#include "in4073.h"
#include "log.h"

#define SAFE_VOLTAGE 1050

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
    if (!log_init(system->serialcomm)) {
        qc_system_set_mode(system, MODE_1_PANIC);
        printf("> Log init error\n");
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
    if (system->state->sensor.voltage < SAFE_VOLTAGE) {
       if(system->mode != MODE_1_PANIC)
           printf("low voltage\n");

       qc_system_set_mode(system, MODE_1_PANIC);
    }
    qc_command_tick(system->command);  
	system->current_mode_table->control_fn(system->state);
    system->hal->enable_motors_fn(
        system->current_mode_table->motor_on_fn(system->state)
        && system->state->option.enable_motors);
    system->hal->set_outputs_fn(system->state);

    qc_system_log_data(system);
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

    qc_mode_t old_mode = system->mode;
    system->mode = mode;
    system->current_mode_table = &system->mode_tables[(int) mode];
    system->current_mode_table->enter_fn(system->state, old_mode);

    serialcomm_quick_send(system->serialcomm, MESSAGE_TIME_MODE_VOLTAGE_ID,
        get_time_us(),
        system->mode | (system->state->sensor.voltage << 16) );
}

static void qc_system_log_data(qc_system_t* system) {
    uint32_t bit_mask, index;
    for (bit_mask = 0x01, index = 0; bit_mask; bit_mask = bit_mask << 1, index++) {
        message_t msg;
        msg.ID = index;
        switch (index) {
            case MESSAGE_TIME_MODE_VOLTAGE_ID:
                MESSAGE_TIME_VALUE(&msg) = get_time_us();
                MESSAGE_MODE_VALUE(&msg) = system->mode;
                MESSAGE_VOLTAGE_VALUE(&msg) = system->state->sensor.voltage;
                break;
            case MESSAGE_SPQR_ID:
                MESSAGE_SP_VALUE(&msg) = system->state->sensor.sp;
                MESSAGE_SQ_VALUE(&msg) = system->state->sensor.sq;
                MESSAGE_SR_VALUE(&msg) = system->state->sensor.sr;
                break;
            case MESSAGE_SAXYZ_ID:
                MESSAGE_SAX_VALUE(&msg) = system->state->sensor.sax;
                MESSAGE_SAY_VALUE(&msg) = system->state->sensor.say;
                MESSAGE_SAZ_VALUE(&msg) = system->state->sensor.saz;
                break;
            case MESSAGE_AE1234_ID:
                MESSAGE_AE1_VALUE(&msg) = system->state->motor.ae1;
                MESSAGE_AE2_VALUE(&msg) = system->state->motor.ae2;
                MESSAGE_AE3_VALUE(&msg) = system->state->motor.ae3;
                MESSAGE_AE4_VALUE(&msg) = system->state->motor.ae4;
                break;
            case MESSAGE_TEMP_PRESSURE_ID:
                MESSAGE_TEMP_VALUE(&msg) = system->state->sensor.temperature;
                MESSAGE_PRESSURE_VALUE(&msg) = system->state->sensor.pressure;
                break;
            case MESSAGE_XYZPOS_ID:
                MESSAGE_XPOS_VALUE(&msg) = system->state->pos.x;
                MESSAGE_YPOS_VALUE(&msg) = system->state->pos.y;
                MESSAGE_ZPOS_VALUE(&msg) = system->state->pos.z;
                break;
            case MESSAGE_PHI_THETA_PSI_ID:
                MESSAGE_PHI_VALUE(&msg) = system->state->att.phi;
                MESSAGE_THETA_VALUE(&msg) = system->state->att.theta;
                MESSAGE_PSI_VALUE(&msg) = system->state->att.psi;
                break;
            case MESSAGE_XYZFORCE_ID:
                MESSAGE_XFORCE_VALUE(&msg) = system->state->force.X;
                MESSAGE_YFORCE_VALUE(&msg) = system->state->force.Y;
                MESSAGE_ZFORCE_VALUE(&msg) = system->state->force.Z;
                break;
            case MESSAGE_LMN_ID:
                MESSAGE_L_VALUE(&msg) = system->state->torque.L;
                MESSAGE_M_VALUE(&msg) = system->state->torque.M;
                MESSAGE_N_VALUE(&msg) = system->state->torque.N;
                break;
            case MESSAGE_UVW_ID:
                MESSAGE_U_VALUE(&msg) = system->state->velo.u;
                MESSAGE_V_VALUE(&msg) = system->state->velo.v;
                MESSAGE_W_VALUE(&msg) = system->state->velo.w;
                break;
            case MESSAGE_PQR_ID:
                MESSAGE_P_VALUE(&msg) = system->state->spin.p;
                MESSAGE_Q_VALUE(&msg) = system->state->spin.q;
                MESSAGE_R_VALUE(&msg) = system->state->spin.r;
                break;
            case MESSAGE_P12_ID:
                MESSAGE_P1_VALUE(&msg) = system->state->trim.p1;
                MESSAGE_P2_VALUE(&msg) = system->state->trim.p2;
                MESSAGE_YAWP_VALUE(&msg) = system->state->trim.yaw_p;
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
