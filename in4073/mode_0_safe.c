#include "mode_0_safe.h"
#include "printf.h"

static void control_fn(qc_state_t* state);
static bool trans_fn(qc_state_t* state, qc_mode_t new_mode);
static void enter_fn(qc_state_t* state, qc_mode_t old_mode);
static bool motor_on_fn(qc_state_t* state);

/** =======================================================
 *  mode_0_safe_init -- Initialise mode table for SAFE.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_0_safe_init(qc_mode_table_t* mode_table) {
    mode_table->control_fn  = &control_fn;
    mode_table->trans_fn    = &trans_fn;
    mode_table->enter_fn    = &enter_fn;
    mode_table->motor_on_fn = &motor_on_fn;
}

/** =======================================================
 *  control_fn -- The control function for this mode.
 *  =======================================================
 *  Sets internal state and output variables according to
 *  the control diagram described at the top of the file.
 *
 *  Parameters:
 *  - state: The state containing everything needed for the
 *      control: inputs, internal state variables and
 *      output.
 *  Author: Boldizsar Palotas
**/
void control_fn(qc_state_t* state) {
    // Maybe continue calculating the position.
    return;
}

/** =======================================================
 *  trans_fn -- Mode transition function.
 *  =======================================================
 *  Returns true if the desired mode should be reachable
 *  from the current state.
 * 
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - new_mode: The mode that is tested.
 *  Returns: true if new_mode could be entered.
 *  Author: Boldizsar Palotas
**/
bool trans_fn(qc_state_t* state, qc_mode_t new_mode) {
    return true;
}

/** =======================================================
 *  enter_fn -- Mode enter function.
 *  =======================================================
 *  This function is called upon entering this mode.
 *  It will reset all options, and some of the state values
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - old_mode: The previous mode.
 *  Author: Koos Eerden
**/
void enter_fn(qc_state_t* state, qc_mode_t old_mode) {
    state->option.enable_motors     = false;
    qc_state_clear_motor(state);
    qc_state_clear_pos(state);
    qc_state_clear_att(state);
    qc_state_clear_force(state);
    qc_state_clear_torque(state);
    qc_state_clear_velo(state);
    qc_state_clear_spin(state);
    printf("Safe mode: motors disabled.\n");
	return;
}

/** =======================================================
 *  motor_on_fn -- Motor enabling logic function.
 *  =======================================================
 *  This function is called to determine if motors should
 *  be enabled for flight. Note that this is completely
 *  separate from the state->option.enable_motors setting
 *  which should always be enabled manually by the user.
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  Author: Boldizsar Palotas
**/
bool motor_on_fn(qc_state_t* state) {
    return false;
}
