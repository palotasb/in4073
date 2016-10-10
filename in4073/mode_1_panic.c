#include "mode_1_panic.h"
#include "printf.h"

static void control_fn(qc_state_t* state);
static bool trans_fn(qc_state_t* state, qc_mode_t new_mode);
static void enter_fn(qc_state_t* state, qc_mode_t old_mode);
static bool motor_on_fn(qc_state_t* state);

static int timer = 0;

// 5s timer at 10 ms control loop frequency
#define TIMER               500
// TODO look into this
#define SAFE_MOTOR_SPEED    320

/** =======================================================
 *  mode_1_panic_init -- Initialise mode table for PANIC.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_1_panic_init(qc_mode_table_t* mode_table) {
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
    // TODO remove magic constants
    if (timer < TIMER)
        timer++;
    if (timer < TIMER) {
        // TODO set sensible defaults
        state->motor.ae1 = SAFE_MOTOR_SPEED;
        state->motor.ae2 = SAFE_MOTOR_SPEED;
        state->motor.ae3 = SAFE_MOTOR_SPEED;
        state->motor.ae4 = SAFE_MOTOR_SPEED;
    } else if (timer == TIMER) {
        state->motor.ae1 = 0;
        state->motor.ae2 = 0;
        state->motor.ae3 = 0;
        state->motor.ae4 = 0;
    }
}

/** =======================================================
 *  trans_fn -- Mode transition function.
 *  =======================================================
 *  Returns true if the desired mode should be reachable
 *  from the current state.
 *  Only SAFE mode is reachable from PANIC mode.
 * 
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - new_mode: The mode that is tested.
 *  Returns: true if new_mode could be entered.
 *  Author: Boldizsar Palotas
**/
bool trans_fn(qc_state_t* state, qc_mode_t new_mode) {
    return new_mode == MODE_0_SAFE;
}

/** =======================================================
 *  enter_fn -- Mode enter function.
 *  =======================================================
 *  This function is called upon entering this mode.
 *  If we are already in SAFE or PANIC mode or the rotors
 *  are running slowly we turn them off. 
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - old_mode: The previous mode.
 *  Author: Boldizsar Palotas
**/
void enter_fn(qc_state_t* state, qc_mode_t old_mode) {
    // TODO maybe not spin up motors if they are already at a low setting.
    if (old_mode == MODE_0_SAFE ||
        old_mode == MODE_1_PANIC || (
            state->motor.ae1 < SAFE_MOTOR_SPEED &&
            state->motor.ae2 < SAFE_MOTOR_SPEED &&
            state->motor.ae3 < SAFE_MOTOR_SPEED &&
            state->motor.ae4 < SAFE_MOTOR_SPEED
        )
    ) {
        timer = TIMER;
    } else {
        timer = 0;
    }
}

/** =======================================================
 *  motor_on_fn -- Motor enabling logic function.
 *  =======================================================
 *  This function is called to determine if motors should
 *  be enabled for flight. Note that this is completely
 *  separate from the state->option.enable_motors setting
 *  which should always be enabled manually by the user.
 *
 *  PANIC mode is the only mode which switches off the
 *  enable_motors option after a time delay.
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  Author: Boldizsar Palotas
**/
bool motor_on_fn(qc_state_t* state) {
    bool on = timer < TIMER;
    if (!on)
        state->option.enable_motors = false;
    return on;
}
