#ifndef SERIALCOMM_H
#define SERIALCOMM_H

#include <inttypes.h>

#define MESSAGE_VALUE_SIZE  8

/*------------------------------------------------------------------
 * message_value_t -- message value type that can be reinterpreted
 *------------------------------------------------------------------
 * Fields:
 *  - v32: values interpreted as words
 *  - v16: values interpreted as halfwords
 *  - v8: values interpreted as bytes
 * Author: Boldizsar Palotas
 */
typedef union message_value {
    uint32_t v32[MESSAGE_VALUE_SIZE / 4];   // Interpret as array of 32 bit values
    uint16_t v16[MESSAGE_VALUE_SIZE / 2];   // Interpret as array of 16 bit values
    uint8_t   v8[MESSAGE_VALUE_SIZE];       // Interpret as array of  8 bit values
} message_value_t;

/*------------------------------------------------------------------
 * message_t -- Message format for quadcopter to PC communication
 *------------------------------------------------------------------
 * Fields:
 *  - value: the message value
 *  - ID: the message or frame ID
 * Author: Boldizsar Palotas
 */
typedef struct message {
    uint8_t           ID;
    message_value_t   value;
} message_t;

// MESSAGE_SIZE -- the size of the message messages from PC to quadcopter
#define MESSAGE_SIZE  (MESSAGE_VALUE_SIZE + 1)

/*------------------------------------------------------------------
 * MESSAGE_XYZ_ID -- Message ID containing the XYZ data.
 *------------------------------------------------------------------
 * Author
 *  - Boldizsar Palotas
 */

/*------------------------------------------------------------------
 * MESSAGE_XYZ_VALUE -- Reference to the element XYZ in the message
 *------------------------------------------------------------------
 * Parameters:
 *  - message_t* message: pointer to the message containing
 *    the data
 * Author:
 *  - Boldizsar Palotas
 */

// Messages in the Quadcopter -> PC direction

// MESSAGE 0
#define MESSAGE_TIME_MODE_VOLTAGE_ID    0

#define MESSAGE_TIME_ID                 MESSAGE_TIME_MODE_VOLTAGE_ID
#define MESSAGE_TIME_VALUE(message)     ((message)->value.v32[0])

#define MESSAGE_MODE_ID                 MESSAGE_TIME_MODE_VOLTAGE_ID
#define MESSAGE_MODE_VALUE(message)     ((message)->value.v16[2])

#define MESSAGE_VOLTAGE_ID              MESSAGE_TIME_MODE_VOLTAGE_ID
#define MESSAGE_VOLTAGE_VALUE(message)  ((message)->value.v16[3])

// MESSAGE 1
#define MESSAGE_SPQR_ID                 1

#define MESSAGE_SP_ID                   MESSAGE_SPQR_ID
#define MESSAGE_SP_VALUE(message)       ((message)->value.v16[0])

#define MESSAGE_SQ_ID                   MESSAGE_SPQR_ID
#define MESSAGE_SQ_VALUE(message)       ((message)->value.v16[1])

#define MESSAGE_SR_ID                   MESSAGE_SPQR_ID
#define MESSAGE_SR_VALUE(message)       ((message)->value.v16[2])

// MESSAGE 2
#define MESSAGE_SAXYZ_ID                2

#define MESSAGE_SAX_ID                  MESSAGE_SAXYZ_ID
#define MESSAGE_SAX_VALUE(message)      ((message)->value.v16[0])

#define MESSAGE_SAY_ID                  MESSAGE_SAXYZ_ID
#define MESSAGE_SAY_VALUE(message)      ((message)->value.v16[1])

#define MESSAGE_SAZ_ID                  MESSAGE_SAXYZ_ID
#define MESSAGE_SAZ_VALUE(message)      ((message)->value.v16[2])

// MESSAGE 3
#define MESSAGE_AE1234_ID               3

#define MESSAGE_AE1_ID                  MESSAGE_AE1234_ID
#define MESSAGE_AE1_VALUE(message)      ((message)->value.v16[0])

#define MESSAGE_AE2_ID                  MESSAGE_AE1234_ID
#define MESSAGE_AE2_VALUE(message)      ((message)->value.v16[1])

#define MESSAGE_AE3_ID                  MESSAGE_AE1234_ID
#define MESSAGE_AE3_VALUE(message)      ((message)->value.v16[2])

#define MESSAGE_AE4_ID                  MESSAGE_AE1234_ID
#define MESSAGE_AE4_VALUE(message)      ((message)->value.v16[3])

// MESSAGE 4
#define MESSAGE_TEMP_PRESSURE_ID        4

#define MESSAGE_TEMP_ID                 MESSAGE_TEMP_PRESSURE_ID
#define MESSAGE_TEMP_VALUE(message)     ((message)->value.v16[0])

#define MESSAGE_PRESSURE_ID             MESSAGE_TEMP_PRESSURE_ID
#define MESSAGE_PRESSURE_VALUE(message) ((message)->value.v32[1])

// MESSAGE 5
#define MESSAGE_XYZPOS_ID               5

#define MESSAGE_XPOS_ID                 MESSAGE_XYZPOS_ID
#define MESSAGE_XPOS_VALUE(message)     ((message)->value.v16[0])

#define MESSAGE_YPOS_ID                 MESSAGE_XYZPOS_ID
#define MESSAGE_YPOS_VALUE(message)     ((message)->value.v16[1])

#define MESSAGE_ZPOS_ID                 MESSAGE_XYZPOS_ID
#define MESSAGE_ZPOS_VALUE(message)     ((message)->value.v16[2])

// MESSAGE 6
#define MESSAGE_PHI_THETA_PSI_ID        6

#define MESSAGE_PHI_ID                  MESSAGE_PHI_THETA_PSI_ID
#define MESSAGE_PHI_VALUE(message)      ((message)->value.v16[0])

#define MESSAGE_THETA_ID                MESSAGE_PHI_THETA_PSI_ID
#define MESSAGE_THETA_VALUE(message)    ((message)->value.v16[1])

#define MESSAGE_PSI_ID                  MESSAGE_PHI_THETA_PSI_ID
#define MESSAGE_PSI_VALUE(message)      ((message)->value.v16[2])

// MESSAGE 7
#define MESSAGE_XYZFORCE_ID             7

#define MESSAGE_XFORCE_ID               MESSAGE_XYZFORCE_ID
#define MESSAGE_XFORCE_VALUE(message)   ((message)->value.v16[0])

#define MESSAGE_YFORCE_ID               MESSAGE_XYZFORCE_ID
#define MESSAGE_YFORCE_VALUE(message)   ((message)->value.v16[1])

#define MESSAGE_ZFORCE_ID               MESSAGE_XYZFORCE_ID
#define MESSAGE_ZFORCE_VALUE(message)   ((message)->value.v16[2])

// MESSAGE 8
#define MESSAGE_LMN_ID                  8

#define MESSAGE_L_ID                    MESSAGE_LMN_ID
#define MESSAGE_L_VALUE(message)        ((message)->value.v16[0])

#define MESSAGE_M_ID                    MESSAGE_LMN_ID
#define MESSAGE_M_VALUE(message)        ((message)->value.v16[1])

#define MESSAGE_N_ID                    MESSAGE_LMN_ID
#define MESSAGE_N_VALUE(message)        ((message)->value.v16[2])

// MESSAGE 9
#define MESSAGE_UVW_ID                  9

#define MESSAGE_U_ID                    MESSAGE_UVW_ID
#define MESSAGE_U_VALUE(message)        ((message)->value.v16[0])

#define MESSAGE_V_ID                    MESSAGE_UVW_ID
#define MESSAGE_V_VALUE(message)        ((message)->value.v16[1])

#define MESSAGE_W_ID                    MESSAGE_UVW_ID
#define MESSAGE_W_VALUE(message)        ((message)->value.v16[2])

// MESSAGE 10
#define MESSAGE_PQR_ID                  10

#define MESSAGE_P_ID                    MESSAGE_PQR_ID
#define MESSAGE_P_VALUE(message)        ((message)->value.v16[0])

#define MESSAGE_Q_ID                    MESSAGE_PQR_ID
#define MESSAGE_Q_VALUE(message)        ((message)->value.v16[1])

#define MESSAGE_R_ID                    MESSAGE_PQR_ID
#define MESSAGE_R_VALUE(message)        ((message)->value.v16[2])

// MESSAGE 11
#define MESSAGE_P12_ID                  11

#define MESSAGE_P1_ID                   MESSAGE_P12_ID
#define MESSAGE_P1_VALUE(message)       ((message)->value.v16[0])

#define MESSAGE_P2_ID                   MESSAGE_P12_ID
#define MESSAGE_P2_VALUE(message)       ((message)->value.v16[1])

#define MESSAGE_YAWP_ID                 MESSAGE_P12_ID
#define MESSAGE_YAWP_VALUE(message)     ((message)->value.v16[2])

// MESSAGE 12
#define MESSAGE_LOG_END_ID              12

// MESSAGE 16

#define MESSAGE_TEXT_ID                 16
#define MESSAGE_TEXT_VALUE(message)     ((message)->value.v8[0])

// Messages in PC -> Quadcopter direction

// MESSAGE 0
#define MESSAGE_SET_MODE_ID             0

#define MESSAGE_SET_MODE_VALUE(message) ((message)->value.v8[0])

// MESSAGE 1
#define MESSAGE_SET_LIFT_ROLL_PITCH_YAW_ID   1

#define MESSAGE_SET_LIFT_ID             MESSAGE_SET_LIFT_ROLL_PITCH_YAW_ID
#define MESSAGE_SET_LIFT_VALUE(message) ((message)->value.v16[0])

#define MESSAGE_SET_ROLL_ID             MESSAGE_SET_LIFT_ROLL_PITCH_YAW_ID
#define MESSAGE_SET_ROLL_VALUE(message) ((message)->value.v16[1])

#define MESSAGE_SET_PITCH_ID            MESSAGE_SET_LIFT_ROLL_PITCH_YAW_ID
#define MESSAGE_SET_PITCH_VALUE(message)    ((message)->value.v16[2])

#define MESSAGE_SET_YAW_ID              MESSAGE_SET_LIFT_ROLL_PITCH_YAW_ID
#define MESSAGE_SET_YAW_VALUE(message)  ((message)->value.v16[3])

// MESSAGE 2
#define MESSAGE_SET_P12_ID              2

#define MESSAGE_SET_P1_ID               MESSAGE_SET_P12_ID
#define MESSAGE_SET_P1_VALUE(message)   ((message)->value.v16[0])

#define MESSAGE_SET_P2_ID               MESSAGE_SET_P12_ID
#define MESSAGE_SET_P2_VALUE(message)   ((message)->value.v16[1])

#define MESSAGE_SET_YAWP_ID             MESSAGE_SET_P12_ID
#define MESSAGE_SET_YAWP_VALUE(message) ((message)->value.v16[2])

// MESSAGE 3
#define MESSAGE_SET_KEYCODE_ID          3

#define MESSAGE_SET_KEYCODE_VALUE(message)  ((message)->value.v8[0])

// MESSAGE 4
#define MESSAGE_SET_OPTION_ID           4

#define MESSAGE_OPTNUM_ID               MESSAGE_SET_OPTION_ID
#define MESSAGE_OPTNUM_VALUE(message)   ((message)->value.v16[0])

#define MESSAGE_OPTMOD_ID               MESSAGE_SET_OPTION_ID
#define MESSAGE_OPTMOD_VALUE(message)   ((message)->value.v16[1])

#define MESSAGE_OPTVAL_ID               MESSAGE_SET_OPTION_ID
#define MESSAGE_OPTVAL_VALUE(message)   ((message)->value.v32[1])

// MESSAGE 5
#define MESSAGE_SET_LOGMSK_ID           5

#define MESSAGE_SET_LOGMSK_VALUE(message)   ((message)->value.v32[0])

// MESSAGE 6
#define MESSAGE_LOG_CTL_ID              6

#define MESSAGE_LOG_CTL_VALUE(message)  ((message)->value.v32[0])
#define MESSAGE_LOG_CTL_VALUE_STOP      0
#define MESSAGE_LOG_CTL_VALUE_START     1
#define MESSAGE_LOG_CTL_VALUE_READ      2
#define MESSAGE_LOG_CTL_VALUE_RESET     2

// MESSAGE 7
#define MESSAGE_SET_TELEMSK_ID          7

#define MESSAGE_SET_TELEMSK_VALUE(message)  ((message)->value.v32[0])

// MESSAGE 8
#define MESSAGE_KEEP_ALIVE_ID           8

// Special frames

#define FRAME_START_ID                  0xFF
#define FRAME_START_VALUE               0xFF
#define FRAME_START_VALUE32             0xFFFFFFFF

#define FRAME_SPECIAL_ID                0xFE
#define FRAME_SPECIAL_NOP_VALUE         0x00000000
#define FRAME_SPECIAL_RESTART_VALUE     0xFEFEFEFE

#ifdef PC_TERMINAL
    const char * const message_id_to_qc_name(uint8_t);
    const char * const message_id_to_pc_name(uint8_t);
#endif // PC_TERMINAL

/*------------------------------------------------------------------
 * frame_t -- Frame format for quadcopter to PC communication
 *------------------------------------------------------------------
 * Fields:
 *  - message_buffer: The message within the frame
 *  - checksum: the checksum of the message
 * Author: Boldizsar Palotas
 */
typedef struct frame {
    message_t   message;
    uint8_t     checksum;
} frame_t;

// The size of a frame in bytes
#define FRAME_SIZE (MESSAGE_SIZE + 1)

/*------------------------------------------------------------------
 * serialcomm_status_t -- The status of the serial communication
 * channel
 *------------------------------------------------------------------
 * Fields:
 *  - message_buffer: The message within the frame
 *  - checksum: the checksum of the message
 * Author: Boldizsar Palotas
 */
typedef enum serialcomm_status {
    SERIALCOMM_STATUS_Prestart = 0,
    SERIALCOMM_STATUS_Start,
    SERIALCOMM_STATUS_OK,
    SERIALCOMM_STATUS_Off
} serialcomm_status_t;

/*------------------------------------------------------------------
 * serialcomm_t -- Structure used as handle for serial communication
 *------------------------------------------------------------------
 * Fields:
 *  - rx_frame:
 *  - tx_frame:
 *  - rx_ptr:
 *  - rx_complete_callback:
 *  - tx_byte:
 * Author:
 *  - Boldizsar Palotas
 */
typedef struct serialcomm {
    serialcomm_status_t status;
    frame_t* rx_frame;
    frame_t* tx_frame;
    int rx_cnt;
    int start_cnt;
    void (*rx_complete_callback)(message_t*);
    void (*tx_byte)(uint8_t);
} serialcomm_t;

void serialcomm_init(serialcomm_t* sc);

void serialcomm_receive_char(serialcomm_t* sc, uint8_t c);

void serialcomm_send(serialcomm_t* sc);

void serialcomm_quick_send(serialcomm_t* sc, uint8_t, uint32_t, uint32_t);

void serialcomm_send_start(serialcomm_t* sc);

void serialcomm_send_restart_request(serialcomm_t* sc);


#endif // SERIALCOMM_H
