#ifndef QC_HAL_H
#define QC_HAL_H

#include <inttypes.h>
#include <stdbool.h>
#include "qc_state.h"

/** qc_hal_t
 *  Quadcopter hardware abstraction layer
 *  -------------------
 *  Fields:
 *  - tx_byte_fn: Function for transferring a single byte of data to PC
 *  - get_inputs_fn: Function for reading sensor data and other inputs before control
 *  - set_outputs_fn: Function for setting motor speed and other outputs after control
 *  - enable_motors_fn: Function for enabling or disabling power on the motors
**/
typedef struct qc_hal {
    void (*tx_byte_fn)(uint8_t);
    void (*get_inputs_fn)(qc_state_t*);
    void (*set_outputs_fn)(qc_state_t*);
    void (*enable_motors_fn)(bool);
    bool (*flash_init_fn)(void);
    bool (*flash_write_fn)(uint32_t, uint8_t*, uint32_t);
    bool (*flash_read_fn)(uint32_t, uint8_t*, uint32_t);
    bool (*flash_erase_fn)(void);
    void (*imu_init_fn)(bool, uint16_t);
    void (*reset_fn)(void);
    uint32_t (*get_time_us_fn)(void);
} qc_hal_t;

void qc_hal_init(qc_hal_t* hal);

#endif // QC_HAL_H
