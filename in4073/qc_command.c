#include "qc_command.h"
#include <stdio.h>
#include "mode_constants.h"
#include "log.h"
#include "printf.h"

static void qc_command_set_mode(qc_command_t* command, qc_mode_t mode);
static void qc_command_set_lift_roll_pitch_yaw(qc_command_t* command,
    f8p8_t lift, f8p8_t roll, f8p8_t pitch, f8p8_t yaw);

// 0.5s timeout
#define COMMAND_TIMEOUT (500*1000)

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
 *  - system: The systom object to modify when specific
 *      commands arrive.
 *  Author: Boldizsar Palotas
**/
void qc_command_init(qc_command_t* command,
    serialcomm_t* serialcomm,
    void (*tx_byte_fn)(uint8_t),
    void (*rx_complete_fn)(message_t*),
    qc_system_t* system
) {
    command->serialcomm = serialcomm;
    command->system = system;
    command->timer = command->system->hal->get_time_us_fn();

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
    command->timer = command->system->hal->get_time_us_fn();
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
        case MESSAGE_SET_TELEMSK_ID:
            command->system->telemetry_mask = MESSAGE_SET_TELEMSK_VALUE(message);
            break;
        case MESSAGE_SET_LOGMSK_ID:
            command->system->log_mask = MESSAGE_SET_LOGMSK_VALUE(message);
            break;
        case MESSAGE_LOG_CTL_ID:
            switch (MESSAGE_LOG_CTL_VALUE(message)) {
                case MESSAGE_LOG_CTL_VALUE_START:
                    printf("> Start logging\n");
                    command->system->do_logging = true;
                    break;
                case MESSAGE_LOG_CTL_VALUE_STOP:
                    printf("> Stop logging\n");
                    command->system->do_logging = false;
                    break;
                case MESSAGE_LOG_CTL_VALUE_READ:
                    if (command->system->mode != MODE_0_SAFE) {
                        printf("> Not in SAFE mode!\n");
                        break;
                    }
                    log_readback();
                    command->system->do_logging = false;
                    break;
                case MESSAGE_LOG_CTL_VALUE_RESET:
                    log_reset();
                    break;
                default:
                    break;
            }
            break;
        case MESSAGE_SET_P12_ID:
            command->system->state->trim.p1 = MESSAGE_SET_P1_VALUE(message);
            command->system->state->trim.p2 = MESSAGE_SET_P2_VALUE(message);
            command->system->state->trim.yaw_p = MESSAGE_SET_YAWP_VALUE(message);
            break;
        case MESSAGE_SET_OPTION_ID:
            switch (MESSAGE_OPTNUM_VALUE(message)) {
                case 1: // Motor enable
                    if (MESSAGE_OPTMOD_VALUE(message) == 1) { // Set option
                        if (MESSAGE_OPTVAL_VALUE(message)) {
                            if (command->system->state->orient.lift < ZERO_LIFT_THRESHOLD) {
                                command->system->state->option.enable_motors = true;
                                printf("Motors enabled!\n");
                            } else
                                printf("Motors NOT enabled. Turn down throttle first!\n");
                        } else {
                            command->system->state->option.enable_motors = false;
                            printf("Motors disabled.\n");
                        }
                    }
                    break;
                case 6: // Raw
                    if (MESSAGE_OPTMOD_VALUE(message) == 2) // Toggle option
                        qc_system_set_raw(command->system, !command->system->state->option.raw_control);
                    break;
                case 7: // Height
                    if (MESSAGE_OPTMOD_VALUE(message) == 2) // Toggle option
                        command->system->state->option.height_control = !command->system->state->option.height_control;
                    break;
                case 8: // Wireless
                    if (MESSAGE_OPTMOD_VALUE(message) == 2) // Toggle option
                        command->system->state->option.wireless_control = !command->system->state->option.wireless_control;
                    break;
            }
            break;
        case MESSAGE_REBOOT_ID:
            command->system->hal->reset_fn();
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
    //printf("Set mode by command.\n");
    qc_system_set_mode(command->system, mode);
}

/** =======================================================
 *  qc_command_set_lift_roll_pitch_yaw --
 *  Dispatch SET_LIFT_ROLL_PITCH_YAW message
 *  =======================================================
 *  Dispatches the orientation (controller setpoint)
 *  message -- lift, roll, pitch and yaw.
 *
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
    command->system->state->orient.lift  = (lift) << LIFT_SHIFT;
    command->system->state->orient.roll  = (roll) << ROLL_SHIFT;
    command->system->state->orient.pitch = (pitch) << PITCH_SHIFT;
    command->system->state->orient.yaw   = (yaw) << YAW_SHIFT;
}

/** =======================================================
 *  qc_command_tick -- Check timeout of the comm channel
 *  =======================================================
 *  Send the QC into panic mode if no valid messages have
 *  been received for a specific amount of time.
 *
 *  Parameters:
 *  - command: Pointer to the command struct.
 *  Author: Boldizsar Palotas
**/
void qc_command_tick(qc_command_t* command) {
    if (COMMAND_TIMEOUT < command->system->hal->get_time_us_fn() - command->timer) {
        printf("Panic because of comm timeout.\n");
        qc_system_set_mode(command->system, MODE_1_PANIC);
        command->timer = command->system->hal->get_time_us_fn();
    }
}
