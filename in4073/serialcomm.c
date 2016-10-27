#include "serialcomm.h"
#include "common.h"

static void serialcomm_rx_end(serialcomm_t* sc, uint8_t received_checksum);
static uint8_t frame_checksum(frame_t* frame);

/*----------------------------------------------------------------
 *  serialcomm_init -- Initialize a serial communication channel.
 *----------------------------------------------------------------
 *  Parameters:
 *      - sc: pointer to the channel state variable.
 *  Returns: void
 *  Author: Boldizsar Palotas
 */
void serialcomm_init(serialcomm_t* sc) {
    sc->status                  = SERIALCOMM_STATUS_Prestart;
    sc->rx_frame                = (frame_t*) 0;
    sc->tx_frame                = (frame_t*) 0;
    sc->rx_cnt                  = 0;
    sc->start_cnt               = 0;
    sc->rx_complete_callback    = (void (*)(message_t*)) 0;
    sc->tx_byte                 = (void (*)(uint8_t)) 0;
}

/*----------------------------------------------------------------
 *  serialcomm_send_restart_request -- Send a restart request
 *  frame.
 *----------------------------------------------------------------
 *  Parameters:
 *      - sc: pointer to the channel state variable.
 *  Returns: void
 *  Author: Boldizsar Palotas
 */
void serialcomm_send_restart_request(serialcomm_t* sc) {
    serialcomm_quick_send(sc, FRAME_SPECIAL_ID,
            FRAME_SPECIAL_RESTART_VALUE, FRAME_SPECIAL_RESTART_VALUE);
}

/*----------------------------------------------------------------
 *  serialcomm_send_start -- Send a start frame.
 *----------------------------------------------------------------
 *  Parameters:
 *      - sc: pointer to the channel state variable.
 *  Returns: void
 *  Author: Boldizsar Palotas
 */
void serialcomm_send_start(serialcomm_t* sc) {
    serialcomm_quick_send(sc, FRAME_START_ID,
            FRAME_START_VALUE32, FRAME_START_VALUE32);
}

/*----------------------------------------------------------------
 *  serialcomm_receive_char -- Handles receiving a byte from the
 *  underlying serial channel.
 *----------------------------------------------------------------
 *  Parameters:
 *      - sc: pointer to the channel state variable.
 *      - c: the received byte
 *  Returns: void
 *  Author: Boldizsar Palotas
 *
 *  The task after receiving a byte depends on the current
 *  status of the serial communication channel.
 *  
 *  In SERIALCOMM_STATUS_OK the bytes are saved in the rx_buffer
 *  until the checksum is recieved, then the frame is processed
 *  as a whole.
 *
 *  In SERIALCOMM_STATUS_Prestart we wait for at least FRAME_SIZE
 *  consecutive FRAME_START_VALUE bytes (a start frame).
 *
 *  In SERIALCOMM_STATUS_Start we wait unti the first
 *  non-FRAME_START_VALUE byte and continue in
 *  SERIALCOMM_STATUS_OK.
 *
 *  In SERIALCOMM_STATUS_Off the byte is disregarded.
 */
 void serialcomm_receive_char(serialcomm_t* sc, uint8_t c) {
    if (sc->status == SERIALCOMM_STATUS_OK) {
        // Normal operation: fill buffer
        if (sc->rx_cnt == MESSAGE_SIZE) {   // End of frame
            serialcomm_rx_end(sc, c);
            sc->rx_cnt = 0;
            return;
        }
        if (sc->rx_cnt == 0) {
            sc->rx_frame->message.ID = c;
            sc->rx_cnt++;
        } else {
            sc->rx_frame->message.value.v8[sc->rx_cnt++ - 1] = c;
        }
        if (c == FRAME_START_VALUE) {
            sc->start_cnt++;
            if (sc->start_cnt == FRAME_SIZE) {
                sc->status = SERIALCOMM_STATUS_Start;
                sc->start_cnt = 0;
            }
        } else {
            sc->start_cnt = 0;
        }
    } else if (sc->status == SERIALCOMM_STATUS_Prestart) {
        // Error: wait for at least a full START FRAME
        if (c == FRAME_START_VALUE) {
            sc->rx_cnt++;
            if (sc->rx_cnt == FRAME_SIZE) {
                sc->status = SERIALCOMM_STATUS_Start;
            }
        } else {
            sc->rx_cnt = 0;
        }
    } else if (sc->status == SERIALCOMM_STATUS_Start) {
        // After error: wait for start of first frame that is not a START FRAME
        if (c != FRAME_START_VALUE) {
            sc->status = SERIALCOMM_STATUS_OK;
            sc->rx_frame->message.ID = c;
            sc->rx_cnt = 1;
        }
    }
}

/*----------------------------------------------------------------
 *  serialcomm_rx_end -- Handles the receiving of a whole frame.
 *----------------------------------------------------------------
 *  Parameters:
 *      - sc: pointer to the channel state variable.
 *      - received_checksum: the checksum received as the last
 *      byte of the frame.
 *  Returns: void
 *  Author: Boldizsar Palotas
 *
 *  If the checksum is correct, the frame is handled by the code
 *  here if the message ID is FRAME_SPECIAL_ID or FRAME_START_ID
 *  and handled by the user code via rx_complete_callback
 *  otherwise.
 *
 *  On incorrect checksum, a currupted channel is assumed and the
 *  status is reverted to Prestart until a new start frame is
 *  received. This start frame is also requested here.
 */
void serialcomm_rx_end(serialcomm_t* sc, uint8_t received_checksum) {
    if (frame_checksum(sc->rx_frame) == received_checksum) {
        // Here we receive a frame with a correct checksum.
        // Frames with IDs FRAME_START_ID and FRAME_SPECIAL_ID are
        // handled separately.
        if (sc->rx_frame->message.ID != FRAME_SPECIAL_ID &&
            sc->rx_frame->message.ID != FRAME_START_ID) {
            if (sc->rx_complete_callback) {
                sc->rx_complete_callback(&sc->rx_frame->message);
            }
        } else {
            // Handle special frames
            if (sc->rx_frame->message.value.v32[0] == FRAME_SPECIAL_RESTART_VALUE) {
                serialcomm_send_start(sc);
            }
        }
    } else {
        // Here we have a checksum error. Go into prestart mode and request a start frame.
        sc->status = SERIALCOMM_STATUS_Prestart;
        // Send a start frame anticipating that the connection might have been lost
        // and the receiver could be in Prestart status.
        serialcomm_send_start(sc);
        serialcomm_send_restart_request(sc);
    }
}

/*----------------------------------------------------------------
 *  serialcomm_quick_send -- Sends a frame.
 *----------------------------------------------------------------
 *  Parameters:
 *      - sc: pointer to the channel state variable.
 *      - id: the message id
 *      - value_a: the lower 32 bits of the message value
 *      - value_b: the higher 32 bits of the message value
 *  Returns: void
 *  Author: Boldizsar Palotas
 */
void serialcomm_quick_send(serialcomm_t* sc, uint8_t id, uint32_t value_a, uint32_t value_b) {
    frame_t new_frame;
    new_frame.message.ID = id;
    new_frame.message.value.v32[0] = value_a;
    new_frame.message.value.v32[1] = value_b;
    frame_t* old_frame = sc->tx_frame;
    sc->tx_frame = &new_frame;
    serialcomm_send(sc);
    sc->tx_frame = old_frame;
}

/*----------------------------------------------------------------
 *  frame_checksum -- Calculates the checksum of a message.
 *----------------------------------------------------------------
 *  Parameters:
 *      - sc: pointer to the channel state variable.
 *      - frame: the frame whose checksum we want to calculate
 *  Returns: The checksum of the frame
 *  Author: Boldizsar Palotas
 *
 *  The checksum is the XOR-ed result of the message ID and all
 *  value bytes.
 *
 *  Note: if we change the checksum algorithm in a way that the checksum
 *  of a START frame becomes something else than 0xFF then we should
 *  handle receiving that byte separately in the serialcomm_receive_char
 *  function in the Prestart and Start branches.
 */
uint8_t frame_checksum(frame_t* frame) {
	 int i;
    uint8_t chkbuf = frame->message.ID;
    for (i = 0; i < MESSAGE_VALUE_SIZE; i++) {
        chkbuf ^= frame->message.value.v8[i];
    }
    return chkbuf;
}

/*----------------------------------------------------------------
 *  serialcomm_send -- Sends a frame.
 *----------------------------------------------------------------
 *  Parameters:
 *      - sc: pointer to the channel state variable.
 *  Returns: void
 *  Author: Boldizsar Palotas
 */
void serialcomm_send(serialcomm_t* sc) {
	int tx_ptr;
    if (sc->tx_byte == 0)
        return;
    sc->tx_byte(sc->tx_frame->message.ID);
    for (tx_ptr = 0; tx_ptr < MESSAGE_VALUE_SIZE; tx_ptr++) {
        uint8_t c = sc->tx_frame->message.value.v8[tx_ptr];
        sc->tx_byte(c);
    }
    sc->tx_frame->checksum = frame_checksum(sc->tx_frame);
    sc->tx_byte(sc->tx_frame->checksum);
}

#ifdef PC_TERMINAL
    static const char * const unknown = "(Unknown)";

    static const char * const message_id_names_to_qc[256] = {
        "SET_MODE",
        "SET_LIFT_ROLL_PITCH_YAW",
        "SET_P12",
        "SET_KEYCODE",
        "SET_OPTION",
        "SET_LOGMSK",
        "LOG_CTL",
        "SET_TELEMSK",
        "KEEP_ALIVE",
        "REBOOT",
        0
    };

    const char * const message_id_to_qc_name(uint8_t id) {
        if (message_id_names_to_qc[id])
            return message_id_names_to_qc[id];
        else
            return unknown;
    }

    static const char * const message_id_names_to_pc[256] = {
        "TIME MODE VOLTAGE", // 0
        "SP SQ SR",
        "SAX SAY SAZ",
        "SPHI STHETA SPSI",
        "AE1-4",
        "L M N", // 5
        "P Q R",
        "PHI THETA PSI",
        "SETPOINT",
        "Z FORCE POS PRESSURE",
        "PROFILE 0-3",
        "PROFILE 4",
        0
    };

    const char * const message_id_to_pc_name(uint8_t id) {
        if (message_id_names_to_pc[id])
            return message_id_names_to_pc[id];
        else
            return unknown;
    }
#endif // PC_TERMINAL
