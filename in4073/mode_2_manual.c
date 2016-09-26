#include "mode_2_manual.h"
#include "in4073.h"

/** MODE 2 (MANUAL) description
 *  ===========================
 *  
 *          lift          Z          ae1
 *          roll          L          ae2
 *          pitch         M          ae3
 *  +----+  yaw    +---+  N   +---+  ae4  +----+
 *  | PC |-------->| 1 |----->| C |------>| QC |----+
 *  +----+         +---+      +---+       +----+    |
 *                   ^                              | force     (X Y Z)
 *                   |                   X ---------+----->
 *               change to                          V torque    (L M N)
 *               Body frame                    +--------+
 *                                             |1/m*I.dt|
 *             no feedback:                    +--------+
 *             assume same frames                   | velocity  (u v w)
 *             and zero movement         X ---------+----->
 *                                                  V spin      (p q r)
 *                                             +--------+
 *                                             |  I.dt  |<-- Change to Earth frame
 *                                             +--------+
 *                                                  | position  (x y z)
 *                                                  +----->
 *                                       X ---------+ attitude  (phi theta psi)
 *
**/

static void control_fn(qc_state_t* state);
static bool trans_fn(qc_state_t* state, qc_mode_t new_mode);
static void enter_fn(qc_state_t* state, qc_mode_t old_mode);
static bool motor_on_fn(qc_state_t* state);

/** =======================================================
 *  mode_2_manual_init -- Initialise mode table for MANUAL.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_2_manual_init(qc_mode_table_t* mode_table) {
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
    // See project_dir/control_ae.m MATLAB file for calculations.
    // ae_1^2 = -1/4b Z +     0 L +  1/2b M + -1/4d N
    // ae_2^2 = -1/4b Z + -1/2b L +     0 M +  1/4d N
    // ae_3^2 = -1/4b Z +     0 L + -1/2b M + -1/4d N
    // ae_4^2 = -1/4b Z +  1/2b L +     0 M +  1/4d N

    f24p8_t b = FP_INT(55, 8);
    f24p8_t d = FP_INT(15, 8);
    f24p8_t m1_4b = - b / 4;
    f24p8_t p1_2b =   b / 2;
    f24p8_t p1_4d =   d / 4;

    state->force.Z = -state->orient.lift;
    state->torque.L = state->orient.roll;
    state->torque.M = state->orient.pitch;
    state->torque.N = state->orient.yaw;

    // f24.8 * f16.16 == f8.24, we have to shift >> 8 to get f16.16 again.
    // TODO we should normalise the results in a sane way here
    int32_t ae1_sq = (m1_4b * state->force.Z + p1_2b * state->torque.M - p1_4d * state->torque.N);
    int32_t ae2_sq = (m1_4b * state->force.Z - p1_2b * state->torque.L + p1_4d * state->torque.N);
    int32_t ae3_sq = (m1_4b * state->force.Z - p1_2b * state->torque.M - p1_4d * state->torque.N);
    int32_t ae4_sq = (m1_4b * state->force.Z + p1_2b * state->torque.L + p1_4d * state->torque.N);

    state->motor.ae1 = ae1_sq < 0 ? 0 : fp_sqrt(ae1_sq);
    state->motor.ae2 = ae2_sq < 0 ? 0 : fp_sqrt(ae2_sq);
    state->motor.ae3 = ae3_sq < 0 ? 0 : fp_sqrt(ae3_sq);
    state->motor.ae4 = ae4_sq < 0 ? 0 : fp_sqrt(ae4_sq);

    static uint32_t counter = 0;
    counter++;

    if ((counter & 0x3F) == 0) {
        printf("M2: ZLMN : %ld %ld %ld %ld\n", state->force.Z, state->torque.L, state->torque.M,state->torque.N);
        printf("M2: ae_sq: %ld %ld %ld %ld\n", ae1_sq, ae2_sq, ae3_sq, ae4_sq);
        printf("M2: ae   : %u %u %u %u\n", state->motor.ae1, state->motor.ae2, state->motor.ae3, state->motor.ae4);
    }
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
    return IS_SAFE_OR_PANIC_MODE(new_mode);
}

/** =======================================================
 *  enter_fn -- Mode enter function.
 *  =======================================================
 *  This function is called upon entering this mode.
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - old_mode: The previous mode.
 *  Author: Boldizsar Palotas
**/
void enter_fn(qc_state_t* state, qc_mode_t old_mode) {
    state->force.X  = 0;
    state->force.Y  = 0;
    qc_state_clear_velo(state);
    qc_state_clear_spin(state);
    qc_state_clear_pos(state);
    qc_state_clear_att(state);
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
    return true;
}
