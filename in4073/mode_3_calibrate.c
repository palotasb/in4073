#include "mode_3_calibrate.h"
#include "printf.h"

// 256 samples taken at 100 HZ means around 2.56 sec calibration time
// 256 means a shift amount of 8 bits
#define CALIBRATE_SAMPLES 256
#define CALIBRATE_SHIFT_AMOUNT 8		


static void control_fn(qc_state_t* state);
static bool trans_fn(qc_state_t* state, qc_mode_t new_mode);
static void enter_fn(qc_state_t* state, qc_mode_t old_mode);
static bool motor_on_fn(qc_state_t* state);


static mode_3_calibrate_state_t cal_state;

/** =======================================================
 *  mode_3_calibrate_init -- Initialise mode table for
 *  CALIBRATE.
 *  =======================================================
 *  Parameters:
 *  - mode_table: Pointer to the mode table to initialise.
 *  Author: Boldizsar Palotas
**/
void mode_3_calibrate_init(qc_mode_table_t* mode_table) {
    mode_table->control_fn  = &control_fn;
    mode_table->trans_fn    = &trans_fn;
    mode_table->enter_fn    = &enter_fn;
    mode_table->motor_on_fn = &motor_on_fn;
}

/** =======================================================
 *  control_fn -- The control function for this mode.
 *  =======================================================
 *  Takes the average value of the sensors in order to 
 *  determine the offsets
 *
 *  Parameters:
 *  - state: The state containing everything needed for the
 *      control: inputs, internal state variables and
 *      output.
 *  Author: Koos Eerden
**/
void control_fn(qc_state_t* state) {

	if(cal_state.busy){
		 cal_state.sp 		+= FP_CHUNK(state->sensor.sp, 16 - CALIBRATE_SHIFT_AMOUNT, 16);
		 cal_state.sq		+= FP_CHUNK(state->sensor.sq, 16 - CALIBRATE_SHIFT_AMOUNT, 16);
		 cal_state.sr		+= FP_CHUNK(state->sensor.sr, 16 - CALIBRATE_SHIFT_AMOUNT, 16);
		 cal_state.sax 		+= FP_CHUNK(state->sensor.sax, 16 - CALIBRATE_SHIFT_AMOUNT, 16);
		 cal_state.say 		+= FP_CHUNK(state->sensor.say, 16 - CALIBRATE_SHIFT_AMOUNT, 16);
		 cal_state.saz 		+= FP_CHUNK(state->sensor.saz, 16 - CALIBRATE_SHIFT_AMOUNT, 16);
		 cal_state.sphi 		+= FP_CHUNK(state->sensor.sphi, 16 - CALIBRATE_SHIFT_AMOUNT, 16);
		 cal_state.stheta 		+= FP_CHUNK(state->sensor.stheta, 16 - CALIBRATE_SHIFT_AMOUNT, 16);
		 cal_state.pressure 		+= FP_CHUNK(state->sensor.pressure, 16 - CALIBRATE_SHIFT_AMOUNT, 16);

		 //update offset
		 if(cal_state.counter++ == CALIBRATE_SAMPLES - 1){ 
			 cal_state.busy = false;
			 state->offset.sp  +=	cal_state.sp;
			 state->offset.sq  +=	cal_state.sq;
			 state->offset.sr  +=	cal_state.sr;
			 state->offset.sax +=	cal_state.sax;
			 state->offset.say +=	cal_state.say;
			 state->offset.saz +=	cal_state.saz;
			 state->offset.sphi +=	cal_state.sphi;
			 state->offset.stheta +=	cal_state.stheta;
			 state->offset.pressure +=	cal_state.pressure;
			 state->offset.calibrated = true;
			 state->pos.z = 0;
			 state->velo.w = 0;
			 printf("Calibration done\n");
		}		
	}
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
    return IS_SAFE_OR_PANIC_MODE(new_mode) || new_mode == MODE_3_CALIBRATE;
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
void enter_fn(qc_state_t* state, qc_mode_t old_mode) 
{
	 cal_state.sp 			= 0;
    cal_state.sq			= 0;
    cal_state.sr			= 0;
    cal_state.sax 		= 0;
    cal_state.say 		= 0;
    cal_state.saz 		= 0;
    cal_state.sphi 		= 0;
    cal_state.stheta	= 0;
	cal_state.pressure	= 0;
	 cal_state.counter	= 0;
	 cal_state.busy		= true;
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
