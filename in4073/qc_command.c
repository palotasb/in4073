#include "qc_command.h"

static void qc_command_set_mode(qc_command_t* command, qc_mode_t mode);
static void qc_command_set_lift_roll_pitch_yaw(qc_command_t* command,
    f8p8_t lift, f8p8_t roll, f8p8_t pitch, f8p8_t yaw);

// 0.5s timeout
#define COMMAND_TIMEOUT 50

/** =======================================================
 *  qc_command_init -- Initialise quadcopter command module
 *  =======================================================
 *  Initialises the qc_command_t structure and the
 *  underlying serial communication module (serialcomm_t
 *  structure). 
 *  Parameters:
 *  - command: Pointer to the command struct to initialise.
 *  - serialcomm: Pointer to the underlying uninitialised
 *      serialcomm module.
 *  - tx_byte_fn: The function to transfer a single byte.
 *  - rx_complete_fn: The function to call after a
 *      received message. 
 *  Author: Boldizsar Palotas
**/
void qc_command_init(qc_command_t* command,
    serialcomm_t* serialcomm,
    void (*tx_byte_fn)(uint8_t),
    void (*rx_complete_fn)(message_t*),
    qc_system_t* system
) {
    command->serialcomm = serialcomm;
    command->timer = 0;

    serialcomm_init(serialcomm);
    serialcomm->rx_frame                = &command->rx_frame;
    serialcomm->rx_complete_callback    = rx_complete_fn;
    serialcomm->tx_byte                 = tx_byte_fn;
}

/** =======================================================
 *  qc_command_rx_message -- Receive and dispatch message
 *  =======================================================
 *  Receives, validates and dispatches command messages.
 *  Validation means syntactic validation. Modules that
 *  recieve the dispatched commands should still do
 *  semantic validation.
 *  Parameters:
 *  - command: Pointer to the command struct.
 *  - message: Pointer to the recived message.
 *  Author: Boldizsar Palotas
**/
void qc_command_rx_message(qc_command_t* command, message_t* message) {
    command->timer = 0;
    switch (message->ID) {
        case MESSAGE_SET_MODE_ID:
            qc_command_set_mode(command,
                (qc_mode_t) MESSAGE_SET_MODE_VALUE(message));
            break;
        case MESSAGE_SET_LIFT_ROLL_PITCH_YAW_ID:
            qc_command_set_lift_roll_pitch_yaw(command,
                MESSAGE_SET_LIFT_VALUE(message),
                MESSAGE_SET_ROLL_VALUE(message),
                MESSAGE_SET_PITCH_VALUE(message),
                MESSAGE_SET_YAW_VALUE(message));
            break;
        default:
            break;
    }
}

/** =======================================================
 *  qc_command_set_mode -- Dispatch SET MODE message
 *  =======================================================
 *  Dispatches the mode switching message
 *  Parameters:
 *  - command: Pointer to the command struct.
 *  - mode: The ID of the mode to switch to.
 *  Author: Boldizsar Palotas
**/
void qc_command_set_mode(qc_command_t* command, qc_mode_t mode) {
    if (!IS_VALID_MODE(mode))
        return;
    qc_system_set_mode(command->system, mode);
}

/** =======================================================
 *  qc_command_set_mode -- Dispatch SET LRPY. message
 *  =======================================================
 *  Dispatches the orientation (controller setpoint)
 *  message -- lift, roll, pitch and yaw.
 *  Parameters:
 *  - command: Pointer to the command struct.
 *  - lift: The lift value as defined in qc_state_t.
 *  - roll: The roll value as defined in qc_state_t.
 *  - pitch: The pitch value as defined in qc_state_t.
 *  - yaw: The yaw value as defined in qc_state_t.
 *  Author: Boldizsar Palotas
**/
void qc_command_set_lift_roll_pitch_yaw(qc_command_t* command,
    f8p8_t lift, f8p8_t roll, f8p8_t pitch, f8p8_t yaw
) {
    command->system->state->orient.lift  = (lift);
    command->system->state->orient.roll  = (roll);
    command->system->state->orient.pitch = (pitch);
    command->system->state->orient.yaw   = (yaw);
}

void qc_command_tick(qc_command_t* command) {
    if (command->timer < COMMAND_TIMEOUT) {
        command->timer++;
    } else {
        qc_command_set_mode(command, MODE_1_PANIC);
        command->timer = 0;
    }
}
