#include "mode_5_full.h"
#include "mode_constants.h"
#include "printf.h"

/** FULL CONTROL MODE OPERATION (MODE 5)
 *  ====================================
 *
 *  Quadcopter system diagram
 *  -------------------------
 *
 *  WARNING: this diagram is a bit outdated compared to how the
 *  actual code evolved.
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
static void enter(qc_state_t* state, qc_mode_t old_mode);
static void enter_mode_2_manual_fn(qc_state_t* state, qc_mode_t old_mode);
static void enter_mode_4_yaw_fn(qc_state_t* state, qc_mode_t old_mode);
static void enter_mode_5_full_fn(qc_state_t* state, qc_mode_t old_mode);
static bool motor_on_fn(qc_state_t* state);
static void height_control(qc_state_t* state);

static qc_mode_t active_mode = MODE_0_SAFE;
static bool prev_height_control = false;

/** =======================================================
 *  mode_2_manual_init -- Initialise mode table for MANUAL mode.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_2_manual_init(qc_mode_table_t* mode_table) {
    mode_table->control_fn  = &control_fn;
    mode_table->trans_fn    = &trans_fn;
    mode_table->enter_fn    = &enter_mode_2_manual_fn;
    mode_table->motor_on_fn = &motor_on_fn;
}

/** =======================================================
 *  mode_4_yaw_init -- Initialise mode table for YAW mode.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_4_yaw_init(qc_mode_table_t* mode_table) {
    mode_table->control_fn  = &control_fn;
    mode_table->trans_fn    = &trans_fn;
    mode_table->enter_fn    = &enter_mode_4_yaw_fn;
    mode_table->motor_on_fn = &motor_on_fn;
}

/** =======================================================
 *  mode_5_full_init -- Initialise mode table for FULL mode.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_5_full_init(qc_mode_table_t* mode_table) {
    mode_table->control_fn  = &control_fn;
    mode_table->trans_fn    = &trans_fn;
    mode_table->enter_fn    = &enter_mode_5_full_fn;
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
    height_control(state);

    // Attitude-related quantitites
    // ----------------------------

    // Roll and pitch set phi and theta but yaw is handled separately.
    // Q16.16 <-- Q2.14
    state->att.phi      = FP_EXTEND(state->orient.roll, 16, 16);
    state->att.theta    = FP_EXTEND(state->orient.pitch, 16, 16);
    if (active_mode == MODE_5_FULL_CONTROL) {
        state->att.phi -=   state->sensor.sphi;
        state->att.theta -= state->sensor.stheta;
    }

    // Q16.16 = Q24.8 * Q16.16 >> 8
    state->spin.p   = FP_MUL3( (state->trim.p1 + P1_DEFAULT) , state->att.phi , 0, 0, P1_FRAC_BITS);
    state->spin.q   = FP_MUL3( (state->trim.p1 + P1_DEFAULT) , state->att.theta , 0, 0, P1_FRAC_BITS);
    // Q16.16 <-- Q6.10
    state->spin.r   = FP_EXTEND(state->orient.yaw, 16, 10);
    if (active_mode == MODE_4_YAW || active_mode == MODE_5_FULL_CONTROL) {
        state->spin.r -= state->sensor.sr;
        if (state->spin.r < SPIN_R_MIN)
            state->spin.r = SPIN_R_MIN;
        if (state->spin.r > SPIN_R_MAX)
            state->spin.r = SPIN_R_MAX;
    }

    // Q16.16 = Q24.8 * Q16.16 >> 8
    // Roll/Pitch 2nd P-value (P2) can be zero but we don't want 0 control over here.
    q32_t spin_p = state->spin.p;
    q32_t spin_q = state->spin.q;
    if (active_mode == MODE_5_FULL_CONTROL) {
        spin_p -= state->sensor.sp;
        spin_q -= state->sensor.sq;
    }
    state->torque.L = FP_MUL3(state->trim.p2 + P2_DEFAULT ,
                              FP_MUL3(I_L , spin_p, 0, 3, 5),
                              0, 2, P2_FRAC_BITS - 2);
    state->torque.M = FP_MUL3(state->trim.p2 + P2_DEFAULT ,
                              FP_MUL3(I_M , spin_q, 0, 3, 5),
                              0, 2, P2_FRAC_BITS - 2);
    // YAW P-value can be zero but we don't want 0 control over here.
    state->torque.N = FP_MUL3(state->trim.yaw_p + YAWP_DEFAULT ,
                              FP_MUL3(T_INV_I_N , state->spin.r, 4, 4, 0),
                              0, 0, YAWP_FRAC_BITS);

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
}

/** =======================================================
 *  height_control -- The control function that controls the Z force
 *  =======================================================
 *  If the height_control option is set, it will control the Z force
 *  such that the height is maintained.
 *  If the height_control option not set it will set the Z force
 *  to the value of the lift set-point.
 *  If the lift setpoint changes during height-control, the 
 *  height_control option will be disabled.
 *
 *  When controlling the height, thisfunction will use a PI controller
 *
 *  Parameters:
 *  - state: The state containing everything needed for the
 *      control: inputs, internal state variables and
 *      output.
 *  Author: Koos Eerden
**/
void height_control(qc_state_t* state) {
    static f8p8_t current_lift;
    static f16p16_t height_setpoint;
    static f16p16_t err_i;
    f16p16_t Z_noclip, err_p;
    const q32_t t = state->option.raw_control ? T_CONST_RAW : T_CONST;

    if(state->option.height_control == true) {
        if(prev_height_control == false) {   //check if height_control is just turned on
            if (state->force.Z < HC_Z_MIN || HC_Z_MAX < state->force.Z) {
                state->option.height_control = false;
                printf("Height control still off because lift is out of bounds.\n");
                return;
            }
            height_setpoint = state->pos.z;
            current_lift = state->orient.lift;
            err_i = state->force.Z;
            printf("Height control turned on.\n");
        } 
        
        if(current_lift == state->orient.lift){

            err_p       = height_setpoint - state->pos.z;
            err_i       = err_i + FP_MUL1(err_p, t * (P1_HEIGHT), P1_HEIGHT_FRAC_BITS + T_CONST_FRAC_BITS);
            Z_noclip    = FP_MUL1(err_p, P2_HEIGHT, P2_HEIGHT_FRAC_BITS) + err_i;
            state->force.Z = HC_Z_MAX < Z_noclip ? HC_Z_MAX : Z_noclip < HC_Z_MIN ? HC_Z_MIN : Z_noclip; 
            err_i       = err_i + state->force.Z - Z_noclip;

        } else {
            // Q16.16 <-- Q8.8
            state->force.Z      = - FP_EXTEND(state->orient.lift, 16, 8);
            state->option.height_control = false;
            printf("Height control turned off! (Throttle was touched.)\n");
        }

    } else {
        // Q16.16 <-- Q8.8
        state->force.Z      = - FP_EXTEND(state->orient.lift, 16, 8);
        if (prev_height_control == true)
            printf("Height control turned off.\n");
    }

    prev_height_control = state->option.height_control;
}

/** =======================================================
 *  acc_filter -- Filters the accellerometer data while in raw mode
 *  =======================================================
 *  An moving average is used as filter function.
 *
 *
 *  Parameters:
 *  - state: The state containing everything needed for the
 *      control: inputs, internal state variables and
 *      output.
 *  Author: Koos Eerden
**/

void acc_filter(qc_state_t* state) {

    #define ACC_FILTER_CONST_A  ((f16p16_t) FP_FLOAT(0.3858, 16))
    #define ACC_FILTER_CONST_B  1

    static f16p16_t    sax_p = 0;
    static f16p16_t    say_p = 0;
    static f16p16_t    saz_p = 0;
    f16p16_t    sax_n;
    f16p16_t    say_n;
    f16p16_t    saz_n;

    sax_n = state->sensor.sax;
    sax_p -= sax_p >> 3;
    sax_p += sax_n >> 3;
    state->sensor.sax = sax_p;

    say_n = state->sensor.say;
    say_p -= say_p >> 3;
    say_p += say_n >> 3;
    state->sensor.say = say_p;

    saz_n = state->sensor.saz;
    saz_p -= saz_p >> 3;
    saz_p += saz_n >> 3;
    state->sensor.saz = saz_p;


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
 *  enter -- Mode enter function.
 *  =======================================================
 *  This function is called upon entering this mode.
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - old_mode: The previous mode.
 *  Author: Boldizsar Palotas
**/
void enter(qc_state_t* state, qc_mode_t old_mode) {
    qc_state_clear_pos(state);
    qc_state_clear_velo(state);
    state->force.X  = 0;
    state->force.Y  = 0;
    state->att.psi = 0;
}

/** =======================================================
 *  enter_mode_2_manual_fn -- Mode enter function.
 *  =======================================================
 *  This function is called upon entering this mode.
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - old_mode: The previous mode.
 *  Author: Boldizsar Palotas
**/
void enter_mode_2_manual_fn(qc_state_t* state, qc_mode_t old_mode) {
    enter(state, old_mode);
    active_mode = MODE_2_MANUAL;
}

/** =======================================================
 *  enter_mode_4_yaw_fn -- Mode enter function.
 *  =======================================================
 *  This function is called upon entering this mode.
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - old_mode: The previous mode.
 *  Author: Boldizsar Palotas
**/
void enter_mode_4_yaw_fn(qc_state_t* state, qc_mode_t old_mode) {
    enter(state, old_mode);
    active_mode = MODE_4_YAW;
}

/** =======================================================
 *  enter_mode_5_full_fn -- Mode enter function.
 *  =======================================================
 *  This function is called upon entering this mode.
 *
 *  Parameters:
 *  - state: The current state of the quadcopter.
 *  - old_mode: The previous mode.
 *  Author: Boldizsar Palotas
**/
void enter_mode_5_full_fn(qc_state_t* state, qc_mode_t old_mode) {
    enter(state, old_mode);
    active_mode = MODE_5_FULL_CONTROL;
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
