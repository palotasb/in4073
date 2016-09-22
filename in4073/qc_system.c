#include "qc_system.h"

#define SAFE_VOLTAGE 1050

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

    // Init command (and serialcomm within)
    qc_command_init(system->command,
        system->serialcomm, system->hal->tx_byte_fn, rx_complete_fn, system);

    // Init other members
    qc_state_init(system->state);
    // qc_logging_init(system->log, ...);
    // qc_telemetry_init(system->telemetry, ...);
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
    if (system->state->sensor.voltage < SAFE_VOLTAGE) {
        qc_system_set_mode(system, MODE_1_PANIC);
    }
    system->hal->get_inputs_fn(system->state);
    qc_command_tick(system->command);  
	system->current_mode_table->control_fn(system->state);
    system->hal->enable_motors_fn(
        system->current_mode_table->motor_on_fn(system->state)
        && system->state->option.enable_motors);
    system->hal->set_outputs_fn(system->state);
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
}
