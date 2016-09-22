#ifndef QC_COMMAND_H
#define QC_COMMAND_H

#include "qc_system.h"
#include "serialcomm.h"

struct qc_system;

/** qc_command_t
 *  Command preprocessing and dispatching to the quadcopter system
 *  ------------------
 *  Fields:
 *  - serialcomm: The serial communication module that receives the messages.
 *  - rx_frame: buffer for receiving messages
 *  - system: pointer to the quadcopter system struct for command dispatching
 *  Author: Boldizsar Palotas
**/
typedef struct qc_command {
    serialcomm_t*           serialcomm;
    frame_t                 rx_frame;
    struct qc_system*       system;
    int                     timer;
} qc_command_t;

void qc_command_init(qc_command_t* command,
    serialcomm_t* serialcomm,
    void (*tx_byte_fn)(uint8_t),
    void (*rx_complete_fn)(message_t*),
    struct qc_system* system);

void qc_command_rx_message(qc_command_t* command, message_t* message);

#endif // QC_COMMAND_H
