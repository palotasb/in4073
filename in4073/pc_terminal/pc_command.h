#ifndef PC_COMMAND_H
#define PC_COMMAND_H

#include <stdbool.h>
#include <inttypes.h>
#include "../serialcomm.h"
#include "../qc_mode.h"
#include "../qc_state.h"

typedef struct pc_command {
    qc_mode_t           mode;
    bool                mode_updated;
    uint8_t             mode_panic_status;
    qc_state_orient_t   orient_js;
    qc_state_orient_t   orient_kb;
    bool                orient_updated;
    qc_state_trim_t     trim;
    bool                trim_updated;
    uint32_t            log_mask;
    bool                log_mask_updated;
    bool                log_start;
    bool                log_stop;
    bool                log_read;
    bool                in_log_not_telemetry;
    uint32_t            telemetry_mask;
    bool                telemetry_mask_updated;
    bool                reboot;
    uint32_t            option_number;
    bool                option_set;
    bool                option_clear;
    bool                option_toggle;
} pc_command_t;

void pc_command_init(pc_command_t* command);

bool pc_command_get_message(pc_command_t* command, message_t* message_out);

#endif // PC_COMMAND_H
