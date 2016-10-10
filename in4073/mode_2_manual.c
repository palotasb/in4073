#include "mode_2_manual.h"
#include "mode_constants.h"
#include "printf.h"

/** MANUAL MODE OPERATION (MODE 2)
 *  ==============================
 *
 *  Quadcopter system diagram
 *  -------------------------
 *
 *  +-----+
 *  | PC  |
 *  +-----+
 *   | | |
 *   | | +-------------------------------------------------+               : Plant, or
 *   | +------------------------+                          |               : Quadcopter
 *   | x_p=[roll,pitch,0]       | v_p=0    f_p=[0,0,-lift] |               :        /
 *   | φ_p=0                    | ω_p=[0,0,yaw]      t_p=0 |             u = ae^2  /
 *   V           +--+  +----+   V         +--+  +------+   V         +---+ :  +---+
 *   +---------->|C1|->|d/dt|-> O ------->|C2|->|k d/dt|-> O ------->| T |--->| P |
 *        x_s    +--+  +----+     v_s     +--+  +------+     f_s     +---+ :  +---+
 *        φ_s                     ω_s                        t_s           :    | f
 *                                                                         :    | t
 *                                                                  .......:    V
 *                                                                 :         +------+
 *                                                                 :   ω_n | | c∫dt |
 *                                                                 :       | +------+
 *                                                              +------+   V    | v
 *                                                            +-| Gyro |<- O <--+ ω
 *                                                          sp| +------+  ω     V
 *                                                       X <--+    :         +------+
 *                                                                 :   φ_n | | ∫dt  |
 *                                                       X <--+    :   Z_n | +------+
 *                                                          sa| +------+   V    | x
 *                                                            +-| Acc. |<- O <--+ φ
 *                                                              +------+ [φ;θ;Z]
 *                                                                 :
 *                                              Computed values <- : -> Physical values
 *
 *  In manual mode, the sensor data is not read, there is no feedback,
 *  the control loop is open. The exact configuration is as follows:
 *
 *  There are no estimated values (^a) because we have no feedback.
 *
 *  x_p = [0, 0, 0]
 *  φ_p = [roll, pitch, 0]
 *
 *  v_p = [0, 0, 0]
 *  ω_p = [0, 0, yaw]
 *
 *  f_p = [0, 0, -lift]
 *  t_p = [0, 0, 0]
 *
 *  Roll and pitch set the horizontal angles of the quadcopter. This
 *  gives us safety in that a maximum output from the PC will still
 *  only result in a fixed bank of the quadcopter.
 *
 *  Yaw is different because we want to be able to completely turn
 *  the quadcopter around with the twist of the joystick. This means
 *  that the joystick must control the rate of the turn and not the
 *  absolute angle.
 *
 *  The upwards (negative Z) force is controlled by the lift. As the
 *  joystick commands a constant rate of lift, we expect the
 *  quadcopter to maintain a steady altitude, with a steady force
 *  counteracting gravity.
 *
**/

static void control_fn(qc_state_t* state);
static bool trans_fn(qc_state_t* state, qc_mode_t new_mode);
static void enter_fn(qc_state_t* state, qc_mode_t old_mode);
static bool motor_on_fn(qc_state_t* state);

// Structs containing the relevant previous-iteration values.
static qc_state_att_t   prev_att;
static qc_state_spin_t  prev_spin;


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

    // Linear quantities
    // -----------------

    // Positions are zero.
    // Hence velocities are zero.
    // Hence forces are zero except for Z to which -lift is added.
    // Q16.16 <-- Q8.8
    state->force.Z      = - FP_EXTEND(state->orient.lift, 16, 8);

    // Attitude-related quantitites
    // ----------------------------

    // Roll and pitch set phi and theta but yaw is handled separately.
    // Q16.16 <-- Q2.14
    state->att.phi      = FP_EXTEND(state->orient.roll, 16, 14);
    state->att.theta    = FP_EXTEND(state->orient.pitch, 16, 14);

    // Q16.16 = Q24.8 * Q16.16 >> 8
    state->spin.p   = (T_INV * (state->att.phi - prev_att.phi)) >> 8;
    state->spin.q   = (T_INV * (state->att.theta - prev_att.theta)) >> 8;
    // Q16.16 <-- Q6.10
    state->spin.r   = FP_EXTEND(state->orient.yaw, 16, 10);

    // Q16.16 = Q24.8 * Q16.16 >> 8
    state->torque.L = (T_INV_I_L * (state->spin.p - prev_spin.p)) >> 8;
    state->torque.M = (T_INV_I_M * (state->spin.q - prev_spin.q)) >> 8;
    state->torque.N = (T_INV_I_N * (state->spin.r - prev_spin.r)) >> 8;

    // Override
    //state->torque.L = state->att.phi   * 2;
    //state->torque.M = state->att.theta * 2;
    //state->torque.N = state->spin.r;

    // See project_dir/control_ae.m MATLAB file for calculations.
    // ae_1^2 = -1/(4b') Z +        0 L +  1/(2b') M + -1/(4d') N
    // ae_2^2 = -1/(4b') Z + -1/(2b') L +        0 M +  1/(4d') N
    // ae_3^2 = -1/(4b') Z +        0 L + -1/(2b') M + -1/(4d') N
    // ae_4^2 = -1/(4b') Z +  1/(2b') L +        0 M +  1/(4d') N
    // b' and d' are system-specific constants

    // f24.8 * f16.16 == f8.24, we have to shift >> 8 to get f16.16 again.
    // Note that aex_sq can be interpreted as regular integers too.
    int32_t ae1_sq = (M1_4B * state->force.Z + _1_2B * state->torque.M - _1_4D * state->torque.N) >> 8;
    int32_t ae2_sq = (M1_4B * state->force.Z - _1_2B * state->torque.L + _1_4D * state->torque.N) >> 8;
    int32_t ae3_sq = (M1_4B * state->force.Z - _1_2B * state->torque.M - _1_4D * state->torque.N) >> 8;
    int32_t ae4_sq = (M1_4B * state->force.Z + _1_2B * state->torque.L + _1_4D * state->torque.N) >> 8;

    state->motor.ae1 = MAX_MOTOR_SPEED * MAX_MOTOR_SPEED < ae1_sq ? MAX_MOTOR_SPEED : ae1_sq < 0 ? 0 : fp_sqrt(ae1_sq);
    state->motor.ae2 = MAX_MOTOR_SPEED * MAX_MOTOR_SPEED < ae2_sq ? MAX_MOTOR_SPEED : ae2_sq < 0 ? 0 : fp_sqrt(ae2_sq);
    state->motor.ae3 = MAX_MOTOR_SPEED * MAX_MOTOR_SPEED < ae3_sq ? MAX_MOTOR_SPEED : ae3_sq < 0 ? 0 : fp_sqrt(ae3_sq);
    state->motor.ae4 = MAX_MOTOR_SPEED * MAX_MOTOR_SPEED < ae4_sq ? MAX_MOTOR_SPEED : ae4_sq < 0 ? 0 : fp_sqrt(ae4_sq);

    // Debug output, can be removed later
    // ----------------------------------
    static uint32_t counter = 0;
    counter++;

    if ((counter & 0x38) == 0x38) {
        //printf("T_INV %"PRId32", dphi %"PRId32" dtheta %"PRId32"\n", T_INV, (state->att.phi - prev_att.phi), (state->att.theta - prev_att.theta));
        printf("LRPY: %"PRId16" %"PRId16" %"PRId16" %"PRId16"\n", state->orient.lift, state->orient.roll, state->orient.pitch, state->orient.yaw);
        //printf("phi theta: %"PRId32" %"PRId32"\n", state->att.phi, state->att.theta);
        printf("pqr: %"PRId32" %"PRId32" %"PRId32"\n", state->spin.p, state->spin.q, state->spin.r);
        printf("ZLMN: %"PRId32" %"PRId32" %"PRId32" %"PRId32"\n", state->force.Z, state->torque.L, state->torque.M,state->torque.N);
        printf("ae_sq: %"PRId32" %"PRId32" %"PRId32" %"PRId32"\n", ae1_sq, ae2_sq, ae3_sq, ae4_sq);
        printf("ae: %"PRIu16" %"PRIu16" %"PRIu16" %"PRIu16"\n\n", state->motor.ae1, state->motor.ae2, state->motor.ae3, state->motor.ae4);
    }

    // Save values as prev_state for next iteration.
    // ---------------------------------------------
    prev_att.phi = state->att.phi;
    prev_att.theta = state->att.theta;
    // Saving psi is unneded

    prev_spin.p = state->spin.p;
    prev_spin.q = state->spin.q;
    prev_spin.r = state->spin.r;
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
    qc_state_clear_pos(state);
    qc_state_clear_velo(state);
    state->force.X  = 0;
    state->force.Y  = 0;
    state->att.psi = 0;

    prev_att.phi = state->att.phi;
    prev_att.theta = state->att.theta;
    prev_spin.p = state->spin.p;
    prev_spin.q = state->spin.q;
    prev_spin.r = state->spin.r;
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
