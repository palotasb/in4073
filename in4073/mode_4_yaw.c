#include "mode_4_yaw.h"


/** MODE 2 (MANUAL) description
 *  ===========================
 *  
 *          lift           Z*          ae1
 *          roll           L*          ae2
 *          pitch          M*          ae3
 *  +----+  yaw +   +---+  N*   +---+  ae4  +----+
 *  | PC |-----> O->| C |------>| T |------>| QC |---+
 *  +----+     - ^  +---+    -  +---+       +----+   |
 *               |                                   | force     (X Y Z)
 *               |                        X ---------+----->
 *               |                                   V torque    (L M N)
 *               |                              +--------+
 *               |                              | 1/m∫dt |
 *               |                              +--------+
 *               |   ^ψ        +---------------+     | velocity  (u v w)
 *               +-------------| Kalman filter |<----+----->
 *   ^ψ = ψ prediction         +---------------+     V spin      (p q r)
 *                                              +--------+
 *                                              |   ∫dt  |
 *                                              +--------+
 *                                                   | position  (x y z)
 *                                                   +----->
 *                                        X ---------+ attitude  (φ θ ψ)
 *
 *
 *  System equations
 *  ================
 *
 *  Basic equations describing quadcopter dynamics
 *  ----------------------------------------------
 *
 *  +-  -+         +-     -+ +- -+     +-  -+
 *  | x' |         | t 0 0 | | u |     | x' |
 *  | y' |     =   | 0 t 0 | | v |   + | y' |
 *  | z' |         | 0 0 t | | w |     | z' |
 *  +-  -+[k+1]    +-     -+ +- -+[k]  +-  -+[k]
 *  Here x', y' and z' are in the body frame so they'd have
 *  to be transformed to yield meaningful x, y, z values.
 *  
 *  +- -+         +-     -+ +- -+     +- -+
 *  | φ |         | t 0 0 | | p |     | φ |
 *  | θ |     =   | 0 t 0 | | q |   + | θ |
 *  | ψ |         | 0 0 t | | r |     | ψ |
 *  +- -+[k+1]    +-     -+ +- -+[k]  +- -+[k]
 *  Here I think that the reference frame is the same for
 *  both Earth and Body and the equation is usable as is.
 *
 *  +- -+         +-     -+ +- -+     +- -+
 *  | u |       1 | t 0 0 | | X |     | u |
 *  | v |     = - | 0 t 0 | | Y |   + | v |
 *  | w |       m | 0 0 t | | Z |     | w |
 *  +- -+[k+1]    +-     -+ +- -+[k]  +- -+[k]
 *  (m is the weight of the quadcopter)
 *  
 *  +- -+         +-     -+ +- -+     +- -+
 *  | p |       1 | t 0 0 | | L |     | p |
 *  | q |     = - | 0 t 0 | | M |   + | q |
 *  | r |       I | 0 0 t | | N |     | r |
 *  +- -+[k+1]    +-     -+ +- -+[k]  +- -+[k]
 *  (I is the moment of inertia of the qc)
 *
 *                            +-      -+
 *  +- -+         +-       -+ | ae_1^2 |
 *  | X |         | 0 0 0 0 | | ae_2^2 |
 *  | Y |    = -b'| 0 0 0 0 | | ae_3^2 |
 *  | Z |         | 1 1 1 1 | | ae_4^2 |
 *  +- -+[k+1]    +-       -+ +-      -+[k]
 *  (b' is a conversion constant)
 *
 *                                   +-      -+
 *  +- -+         +-              -+ | ae_1^2 |
 *  | L |         |  0  -b'  0   b'| | ae_2^2 |
 *  | M |     =   |  b'  0  -b'  0 | | ae_3^2 |
 *  | N |         | -d'  d' -d'  d'| | ae_4^2 |
 *  +- -+[k+1]    +-              -+ +-      -+[k]
 *  (b' and d' are conversion constants)
 *
 *
 *  Relation to sensor readings (to be used in Kalman filter)
 *  ---------------------------------------------------------
 *  
 *  +- -+       +-     -+ +-  -+
 *  | p |       | 1 0 0 | | sp |
 *  | q |   =   | 0 1 0 | | sq |
 *  | r |       | 0 0 1 | | sr |
 *  +- -+[k]    +-     -+ +-  -+[k]
 *  
 *  +- -+       +-     -+ +-   -+
 *  | φ |       | 0 n 0 | | sax |
 *  | θ |   =   | n 0 0 | | say |
 *  | Z |       | 0 0 n | | saz |
 *  +- -+[k]    +-     -+ +-   -+[k]
 *  n = 1/m' where m' is a weight constant
 *  true at small angles where a ~ sin(a)
 *
**/

static void control_fn(qc_state_t* state);
static bool trans_fn(qc_state_t* state, qc_mode_t new_mode);
static void enter_fn(qc_state_t* state, qc_mode_t old_mode);
static bool motor_on_fn(qc_state_t* state);

/** =======================================================
 *  mode_4_yaw_init -- Initialise mode table for YAW.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_4_yaw_init(qc_mode_table_t* mode_table) {
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
