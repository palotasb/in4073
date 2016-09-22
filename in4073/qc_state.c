#include "qc_state.h"

/** =======================================================
 *  qc_state_init -- Initialise quadcopter state variable
 *  =======================================================
 *  Initialises all members of the quadcopter state
 *  variable to a zero value.
 *  Parameters:
 *  - state: The state variable to initialise.
 *  Author: Boldizsar Palotas
**/
void qc_state_init(qc_state_t* state) {
    qc_state_clear_orient(state);
    qc_state_clear_motor(state);
    qc_state_clear_sensor(state);
    qc_state_clear_pos(state);
    qc_state_clear_att(state);
    qc_state_clear_force(state);
    qc_state_clear_torque(state);
    qc_state_clear_velo(state);
    qc_state_clear_spin(state);
    qc_state_clear_trim(state);
    qc_state_clear_option(state);
}

/** =======================================================
 *  qc_state_clear_orient -- Clear orientation data
 *  =======================================================
 *  Clears all orientation data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_orient(qc_state_t* state) {
    state->orient.lift  = 0;
    state->orient.roll  = 0;
    state->orient.pitch = 0;
    state->orient.yaw   = 0;
}

/** =======================================================
 *  qc_state_clear_motor -- Clear motor data
 *  =======================================================
 *  Clears all motor data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_motor(qc_state_t* state) {
    state->motor.ae1    = 0;
    state->motor.ae2    = 0;
    state->motor.ae3    = 0;
    state->motor.ae4    = 0;
}

/** =======================================================
 *  qc_state_clear_sensor -- Clear sensor data
 *  =======================================================
 *  Clears all sensor data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_sensor(qc_state_t* state) {
    state->sensor.sp    = 0;
    state->sensor.sq    = 0;
    state->sensor.sr    = 0;
    state->sensor.sax   = 0;
    state->sensor.say   = 0;
    state->sensor.saz   = 0;
    state->sensor.temperature   = 0;
    state->sensor.pressure      = 0;
    state->sensor.voltage       = 0;
}

/** =======================================================
 *  qc_state_clear_pos -- Clear position data
 *  =======================================================
 *  Clears all position data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_pos(qc_state_t* state) {
    state->pos.x        = 0;
    state->pos.y        = 0;
    state->pos.z        = 0;
}

/** =======================================================
 *  qc_state_clear_att -- Clear attitude data
 *  =======================================================
 *  Clears all attitude data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_att(qc_state_t* state) {
    state->att.phi      = 0;
    state->att.theta    = 0;
    state->att.psi      = 0;
}

/** =======================================================
 *  qc_state_clear_force -- Clear force data
 *  =======================================================
 *  Clears all force data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_force(qc_state_t* state) {
    state->force.X      = 0;
    state->force.Y      = 0;
    state->force.Z      = 0;
}

/** =======================================================
 *  qc_state_clear_torque -- Clear torque data
 *  =======================================================
 *  Clears all torque data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_torque(qc_state_t* state) {
    state->torque.L     = 0;
    state->torque.M     = 0;
    state->torque.N     = 0;
}

/** =======================================================
 *  qc_state_clear_velo -- Clear velocity data
 *  =======================================================
 *  Clears all velocity data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_velo(qc_state_t* state) {
    state->velo.u       = 0;
    state->velo.v       = 0;
    state->velo.w       = 0;
}

/** =======================================================
 *  qc_state_clear_spin -- Clear angular velocity data
 *  =======================================================
 *  Clears all angular velocity data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_spin(qc_state_t* state) {
    state->spin.p       = 0;
    state->spin.q       = 0;
    state->spin.r       = 0;
}

/** =======================================================
 *  qc_state_clear_trim -- Clear control loop trimming data
 *  =======================================================
 *  Clears all control trimming data in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_trim(qc_state_t* state) {
    state->trim.yaw_p   = 0;
    state->trim.p1      = 0;
    state->trim.p2      = 0;
}

/** =======================================================
 *  qc_state_clear_option -- Clear operating options
 *  =======================================================
 *  Clears all operating options in the state variable.
 *  Parameters:
 *  - state: The state variable in which to clear the data.
 *  Author: Boldizsar Palotas
**/
void qc_state_clear_option(qc_state_t* state) {
    state->option.height_control    = false;
    state->option.raw_control       = false;
    state->option.wireless_control  = false;
    state->option.enable_motors     = false;
}
