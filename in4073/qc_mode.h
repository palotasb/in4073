#ifndef QC_MODE_H
#define QC_MODE_H

#include "common.h"
#ifdef QUADCOPTER
    #include "qc_state.h"
    #include <stdbool.h>
#endif // QUADCOPTER

/** qc_mode_t: Flight mode
 *  Variants:
 *  - same as described in the assignment
 *  Author: Boldizsar Palotas
**/
typedef enum qc_mode {
    MODE_0_SAFE         = 0,
    MODE_1_PANIC        = 1,
    MODE_2_MANUAL       = 2,
    MODE_3_CALIBRATE    = 3,
    MODE_4_YAW          = 4,
    MODE_5_FULL_CONTROL = 5,
    MODE_UNKNOWN        = 0xFF
} qc_mode_t;

// The number of valid flight modes
#define MODE_COUNT  6

// Returns whether a number is a valid flight mode
#define IS_VALID_MODE(mode_id)  ((mode_id) == MODE_0_SAFE || \
                                (mode_id) == MODE_1_PANIC || \
                                (mode_id) == MODE_2_MANUAL || \
                                (mode_id) == MODE_3_CALIBRATE || \
                                (mode_id) == MODE_4_YAW || \
                                (mode_id) == MODE_5_FULL_CONTROL)

// Returns whether a flight mode is safe or panic (the ones which can be reached from any mode)
#define IS_SAFE_OR_PANIC_MODE(mode_id) ((mode_id) == MODE_0_SAFE || (mode_id) == MODE_1_PANIC)

#ifdef QUADCOPTER
    // Function for controlling the motors or any other state of the quadcopter
    typedef void (*qc_control_fn_t)     (qc_state_t* state);
    // Function called to determine if a requested mode can be reached from the current mode
    typedef bool (*qc_mode_trans_fn_t)  (qc_state_t* state, qc_mode_t new_mode);
    // Function called upon entering a mode
    typedef void (*qc_mode_enter_fn_t)  (qc_state_t* state, qc_mode_t old_mode);
    // Function called to determine whether the motors can be turned on or not
    typedef bool (*qc_motor_on_fn_t)    (qc_state_t* state);

    /** qc_mode_table_t: Mode table to specify the working of a mode
     *  This struct is used with different function pointers in each mode.
     *  The functions in the active mode table are called to control the
     *  quadcopter.
     *  ------------------
     *  Fields:
     *  - control_fn: Function called to act as a control
     *      system for the quadcopter.
     *  - trans_fn: Function called to determine if a new
     *      mode can be reached from the current one.
     *  - enter_fn: Function called upon entering a new mode.
     *  - motor_on_fn: Function called to determine if the motors
     *      can be turned on
     *  Author: Boldizsar Palotas
    **/
    typedef struct qc_mode_table {
        qc_control_fn_t         control_fn;
        qc_mode_trans_fn_t      trans_fn;
        qc_mode_enter_fn_t      enter_fn;
        qc_motor_on_fn_t        motor_on_fn;
int test; //DEBUG
    } qc_mode_table_t;

#endif // QUADCOPTER

#endif // QC_MODE_H
