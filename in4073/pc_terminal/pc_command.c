#include "pc_command.h"

// mode_panic_status:
//  0: no panic
//  1: send SET_MODE (MODE_1_PANIC)
//  2: send START frame
//  3: send START frame

void pc_command_init(pc_command_t* command) {
    command->mode               = 0;
    command->mode_updated       = false;
    command->mode_panic_status  = 0;
    qc_command_clear_orient(&command->orient_js);
    qc_command_clear_orient(&command->orient_kb);
    command->orient_updated     = false;
    qc_command_clear_trim(&command->trim);
    command->trim_updated       = false;
    command->log_mask           = 0;
    command->log_mask_updated   = false;
    command->log_start          = false;
    command->log_stop           = false;
    command->log_read           = false;
    command->in_log_not_telemetry = false;
    command->telemetry_mask     = 0;
    command->telemetry_mask_updated = false;
    command->reboot             = false;
    command->option_number      = 0;
    command->option_set         = false;
    command->option_clear       = false;
    command->option_toggle      = false;
}

// Returns the highest-priority message that should be sent to the Quadcopter
bool pc_command_get_message(pc_command_t* command, message_t* message_out) {
    if (command->mode_updated && command->mode == MODE_1_PANIC) {
        switch (command->mode_panic_status) {
            case 1:
                message_out->ID = MESSAGE_SET_MODE_ID;
                MESSAGE_SET_MODE_VALUE(message_out) = MODE_1_PANIC;
                command->mode_panic_status = 2;
                break;
            case 2:
                message_out->ID = FRAME_START_ID;
                message_out->value.v32[0] = FRAME_START_VALUE32;
                message_out->value.v32[1] = FRAME_START_VALUE32;
                command->mode_panic_status = 3;
                break;
            case 3:
                message_out->ID = FRAME_START_ID;
                message_out->value.v32[0] = FRAME_START_VALUE32;
                message_out->value.v32[1] = FRAME_START_VALUE32;
                command->mode_panic_status = 1;
                break;
            default:
                command->mode_updated = false;
                break;
        }
        if (command->mode_updated)
            return true;
    }
    if (command->mode_updated) {
        message_out->ID = MESSAGE_SET_MODE_ID;
        MESSAGE_SET_MODE_VALUE(message_out) = command->mode;
        // Don't clear mode_updated flag until we get acknowledgement (done separately)
        return true;
    }
    if (command->orient_updated) {
        message_out->ID = MESSAGE_SET_LIFT_ROLL_PITCH_YAW_ID;
        MESSAGE_LIFT_VALUE(message_out)     = max(min(command->orient_kb.lift   + command->orient_js.lift, 127), -128);
        MESSAGE_ROLL_VALUE(message_out)     = max(min(command->orient_kb.roll   + command->orient_js.roll, 127), -128);
        MESSAGE_PITCH_VALUE(message_out)    = max(min(command->orient_kb.pitch  + command->orient_js.pitch, 127), -128);
        MESSAGE_YAW_VALUE(message_out)      = max(min(command->orient_kb.yaw    + command->orient_js.yaw, 127), -128);
        command->orient_updated = false;
        return true;
    }
    if (command->trim_updated) {
        message_out->ID = MESSAGE_SET_P12_ID;
        MESSAGE_SET_P1_VALUE(message_out)   = command->trim.p1;
        MESSAGE_SET_P2_VALUE(message_out)   = command->trim.p2;
        MESSAGE_SET_YAWP_VALUE(message_out) = command->trim.yaw_p;
        command->trim_updated = false;
        return true;
    }
    if (command->option_set || command->option_clear) {
        message_out->ID = MESSAGE_SET_OPTION_ID;
        MESSAGE_OPTNUM_VALUE(message_out) = command->option_number;
        MESSAGE_OPTMOD_VALUE(message_out) = 1;
        MESSAGE_OPTVAL_VALUE(message_out) = command->option_set ? 1 : 0;
        command->option_number  = 0;
        command->option_set     = false;
        command->option_clear   = false;
        return true;
    }
    if (command->option_toggle) {
        message_out->ID = MESSAGE_SET_OPTION_ID;
        MESSAGE_OPTNUM_VALUE(message_out) = command->option_number;
        MESSAGE_OPTMOD_VALUE(message_out) = 2;
        MESSAGE_OPTVAL_VALUE(message_out) = 0;
        command->option_number  = 0;
        command->option_toggle  = false;
        return true;
    }
    if (command->log_mask_updated) {
        message_out->ID = MESSAGE_SET_LOGMSK_ID;
        MESSAGE_SET_LOGMSK_VALUE(message_out) = command->log_mask;
        command->log_mask = 0;
        command->log_mask_updated = false;
        return true;
    }
    if (command->log_stop) {
        message_out->ID = MESSAGE_LOG_CTL_ID;
        MESSAGE_LOG_CTL_VALUE(message_out) = MESSAGE_LOG_CTL_VALUE_STOP;
        command->log_stop = false;
        return true;
    }
    if (command->log_start) {
        message_out->ID = MESSAGE_LOG_CTL_ID;
        MESSAGE_LOG_CTL_VALUE(message_out) = MESSAGE_LOG_CTL_VALUE_START;
        command->log_start = false;
        return true;
    }
    if (command->log_read) {
        message_out->ID = MESSAGE_LOG_CTL_ID;
        MESSAGE_LOG_CTL_VALUE(message_out) = MESSAGE_LOG_CTL_VALUE_READ;
        command->log_read = false;
        return true;
    }
    if (command->telemetry_mask_updated) {
        message_out->ID = MESSAGE_SET_TELEMSK_ID;
        MESSAGE_SET_TELEMSK_VALUE(message_out) = command->telemetry_mask;
        command->telemetry_mask = 0;
        command->telemetry_mask_updated = false;
        return true;
    }

    return false;
}