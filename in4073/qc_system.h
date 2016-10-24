#ifndef QC_SYSTEM_H
#define QC_SYSTEM_H

/*  Overview of the modules in the Quadcopter
 *  =========================================
 *
 *   +------------------------------------------------+
 *   | Quadcopter system                              |
 *   |                                                |  Mode tables
 *   |     +---------------+        +--------------+  |  +--------+
 *   |     | Command       |----+-->| Current      |<----| Mode 0 |
 *  to PC  +---------------+    |   | mode table   |  |  +--------+
 * <------>| Serial comm.  |<-+ |   +--------------+  |  | Mode 1 |
 *   |     +---------------+  | |       |             |  +--------+
 *   |     +---------------+  | |       |             |  | ...    |
 * to flash| Logging       |--+ |       |             |  +--------+
 * <------>| Telemetry     |<---+       |             |  | Mode N |
 *   |     +---------------+            |             |  +--------+
 *   |            ^ ^                   V             |
 *   |            | | +----------------------+        |
 *   |            | +-| System control       |  to sensors
 *   |            |   +----------------------+  & actuators 
 *   |            +---| State                |<---------->
 *   |                +----------------------+        |
 *   |                                                |
 *   +------------------------------------------------+
 *   |                                                |
 *   | Quadcopter hardware abstraction layer (QC HAL) |
 *   |                                                |
 *   +------------------------------------------------+
 *
 *  qc_system_init
 *      qc_command_init
 *          serialcomm_init
 *      qc_state_init
 *      qc_logging_init
 *      qc_telemetry_init
 *
**/

#include "qc_mode.h"
#include "qc_state.h"
#include "qc_command.h"
#include "serialcomm.h"
#include "qc_hal.h"
#include <inttypes.h>
#include <stdbool.h>

enum qc_mode;
struct qc_state;
struct qc_command;
struct serialcomm;

/** =======================================================
 *  qc_system_t -- store and reference all informcation
 *      related to flying and controlling the quadcoper
 *      including state, mode, log data and everything else
 *  =======================================================
 *  Fields:
 *      - mode: The current mode the quadcopter is in.
 *      - mode_tables: Pointer to the array containing
 *          tables that specify how each mode should work.
 *      - current_mode_table: Pointer to the mode_table of
 *          the currently active mode of the quadcopter.
 *      - state: Pointer to the struct containing all state
 *          variables of the quadcopter.
 *      - command: Pointer to the command module that
 *          processes all incoming messages (commands).
 *      - serialcomm: Pointer to the serial communication
 *          module for transmitting messages to the PC.
 *  Author: Boldizsar Palotas
**/
typedef struct qc_system {
    qc_mode_t           mode;
    qc_mode_table_t*    mode_tables;
    qc_mode_table_t*    current_mode_table;
    qc_state_t*         state;
    struct qc_command*  command;
    serialcomm_t*       serialcomm;
    qc_hal_t*           hal;
    uint32_t            do_logging;
    uint32_t            log_mask;
    uint32_t            telemetry_mask;
} qc_system_t;

void qc_system_init(qc_system_t* system,
    qc_mode_t           mode,
    qc_mode_table_t*    mode_tables,
    qc_state_t*         state,
    struct qc_command*  command,
    serialcomm_t*       serialcomm,
    void (*rx_complete_fn)(message_t*),
    qc_hal_t*           hal);

void qc_system_step(qc_system_t* system);

void qc_system_set_mode(qc_system_t* system, qc_mode_t mode);

void qc_kalman_filter(qc_state_t* state);

void qc_system_log_data(qc_system_t* system);

void qc_system_set_raw(qc_system_t* system, bool raw);

#endif // QC_SYSTEM_H