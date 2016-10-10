#include "mode_5_full.h"
#include "mode_constants.h"
#include "printf.h"

/** FULL CONTROL MODE OPERATION (MODE 5)
 *  ====================================
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
 *   | x_p=0              v_p=0 |          f_p=[0,0,-lift] |               :        /
 *   | φ_p   x_e            ω_p |     v_e            t_p=0 |     f_e     u = ae^2  /
 *   V       φ_e +--+  +----+   V     ω_e +--+  +------+   V     t_e +---+ :  +---+
 *   +--> O ---->|C1|->|d/dt|-> O -> O -->|C2|->|k d/dt|-> O -> O -->| T |--->| P |
 *   x_s  ^ -    +--+  +----+    v_s ^ -  +--+  +------+    f_s ^ -  +---+ :  +---+
 *   φ_s  | ^x=0                 ω_s | ^v                   t_s | ^f       :    | f
 *        | ^φ=0                     | ^ω                       | ^t       :    | t
 *        |              offsets --> O        +-----------------+   .......:    V
 *        |                        - ^ +      |                    :         +------+
 *        |                          |        |                    :   ω_n | | c∫dt |
 *        |                          |        |                    :       | +------+
 *        |                          |        |                 +------+   V    | v
 *        |                          |        |               +-| Gyro |<- O <--+ ω
 *        |                          |        |  +--------+sp | +------+  ω     V
 *        |                          +--------+--|--------|<--+    :         +------+
 *        |                                      | filter |        :   φ_n | | ∫dt  |
 *        +--------------------------------------|--------|<--+    :   Z_n | +------+
 *                                               +--------+sa | +------+   V    | x
 *                                                            +-| Acc. |<- O <--+ φ
 *                                                              +------+ [φ;θ;Z]
 *                                                                 :
 *                                              Computed values <- : -> Physical values
 *
 *  In full control mode, all sensor data is read and is fed into a
 *  Kalman-filter that provides estimates ^x, ^φ, ^v, ^ω, ^f and ^t.
 *  This is used to control the entire operation of the quadcopter.
 *
 *  x_p = [0, 0, 0]
 *  φ_p = [roll, pitch, 0]
 *  ^x  = [0, 0, 0]     // Could use Kalman-filter
 *  ^φ  = [0, 0, 0]     // Could use sax and say or Kalman-filter
 *
 *  v_p = [0, 0, 0]
 *  ω_p = [0, 0, yaw]
 *  ^v  = [0, 0, 0]     // Could use Kalman-filter
 *  ^ω  = [sp, sq, sr]  // Could use Kalman-filter
 *
 *  f_p = [0, 0, -lift]
 *  t_p = [0, 0, 0]
 *  ^f  = [0, 0, 0]     // Could use Kalman-filter
 *  ^t  = [0, 0, 0]     // Could use Kalman-filter
 *
 *  Roll and pitch set the horizontal angles of the quadcopter. This
 *  gives us safety in that a maximum output from the PC will still
 *  only result in a fixed bank of the quadcopter. The roll and pitch
 *  _rate_ (p and q within ω) are have the estimated p and q values
 *  subtracted from them to give an error which can be used for
 *  P-control.
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

// filtering
#define FILT_A_CNT  4
#define FILT_B_CNT  4
#define FILT_COEFF_FRAC_BITS    8
static const int32_t const filt_a[FILT_A_CNT] = {64, 64, 64, 64};
static int32_t filt_x[FILT_A_CNT];
static const int32_t const filt_b[FILT_B_CNT] = {0xDEAD, 0, 0, 0};
static int32_t filt_y[FILT_B_CNT];

static int32_t filter(int32_t val);

/** =======================================================
 *  mode_5_full_init -- Initialise mode table for FULL.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_5_full_init(qc_mode_table_t* mode_table) {
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
    state->spin.p   = (state->trim.p1 + 1) * (((T_INV * (state->att.phi - prev_att.phi)) >> 8) - (state->sensor.sp - state->offset.sp));
    state->spin.q   = (state->trim.p1 + 1) * (((T_INV * (state->att.theta - prev_att.theta)) >> 8) - (state->sensor.sq - state->offset.sq));
    // Q16.16 <-- Q6.10
    state->spin.r   = FP_EXTEND(state->orient.yaw, 16, 10) - (state->sensor.sr - state->offset.sr);

    // Q16.16 = Q24.8 * Q16.16 >> 8
    // Roll/Pitch 2nd P-value (P2) can be zero but we don't want 0 control over here.
    state->torque.L = ((state->trim.p2 + 1) * (T_INV_I_L * (state->spin.p)) >> 8) >> 5;
    state->torque.M = ((state->trim.p2 + 1) * (T_INV_I_M * (state->spin.q)) >> 8) >> 5;
    // YAW P-value can be zero but we don't want 0 control over here.
    state->torque.N = ((state->trim.yaw_p + 1) * ((T_INV_I_N * (state->spin.r)) >> 8)) >> 7;

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

    if ((counter & 0x038) == 0x038) {
        //printf("LRPY: %"PRId16" %"PRId16" %"PRId16" %"PRId16"\n", state->orient.lift, state->orient.roll, state->orient.pitch, state->orient.yaw);
        //printf("phi theta: %"PRId32" %"PRId32"\n", state->att.phi, state->att.theta);
        //printf("pqr: %"PRId32" %"PRId32" %"PRId32"\n", state->spin.p, state->spin.q, state->spin.r);
        //printf("sr ofsr dr yaw_p: %"PRId32" %"PRId32" %"PRId32" %"PRId32"\n", state->sensor.sr, state->offset.sr, (state->spin.r - prev_spin.r), state->trim.yaw_p);
        //printf("ZLMN: %"PRId32" %"PRId32" %"PRId32" %"PRId32"\n", state->force.Z, state->torque.L, state->torque.M,state->torque.N);
        //printf("ae_sq: %"PRId32" %"PRId32" %"PRId32" %"PRId32"\n", ae1_sq, ae2_sq, ae3_sq, ae4_sq);
        //printf("ae   : %"PRIu16" %"PRIu16" %"PRIu16" %"PRIu16"\n\n", state->motor.ae1, state->motor.ae2, state->motor.ae3, state->motor.ae4);
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

int32_t filter(int32_t val) {
    int32_t y = 0;
    for (int i = 0; i < FILT_A_CNT - 1; i++) {
        filt_x[i + 1] = filt_x[i];
    }
    filt_x[0] = val;
    // y = filt_a[0] * filt_x[0] + filt_a[1] * filt_x[1];
    for (int i = 0; i < FILT_A_CNT; i++) {
        y += (filt_a[i] * filt_x[i]) >> FILT_COEFF_FRAC_BITS;
    }
    // y -= (filt_b[1] * filt_y[1] + filt_b[2] * filt_x[2])
    for (int i = 1; i < FILT_B_CNT; i++) {
        y -= (filt_b[i] * filt_y[i]) >> FILT_COEFF_FRAC_BITS;
    }
    filt_y[0] = y;
    for (int i = 0; i < FILT_B_CNT - 1; i++) {
        filt_y[i + 1] = filt_y[i];
    }
    return y;
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
    volatile int i = 0;
    filter(i);
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

    for (int i = 0; i < FILT_A_CNT; i++) {
        filt_x[i] = 0;
    }
    for (int i = 0; i < FILT_B_CNT; i++) {
        filt_y[i] = 0;
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
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  Author: Boldizsar Palotas
**/
bool motor_on_fn(qc_state_t* state) {
    return true;
}
