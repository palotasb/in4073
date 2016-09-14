#include "serialcomm.h"
#include "common.h"

static void serialcomm_rx_end(serialcomm_t* sc, uint8_t received_checksum);
static uint8_t frame_checksum(frame_t* frame);

// zero all struct members.
void serialcomm_init(serialcomm_t* sc) {
    sc->status                  = SERIALCOMM_STATUS_Prestart;
    sc->rx_frame                = (frame_t*) 0;
    sc->tx_frame                = (frame_t*) 0;
    sc->rx_cnt                  = 0;
    sc->start_cnt               = 0;
    sc->rx_complete_callback    = (void (*)(message_t*)) 0;
    sc->tx_byte                 = (void (*)(uint8_t)) 0;
}

void serialcomm_send_restart_request(serialcomm_t* sc) {
    serialcomm_quick_send(sc, FRAME_SPECIAL_ID,
            FRAME_SPECIAL_RESTART_VALUE, FRAME_SPECIAL_RESTART_VALUE);
}

void serialcomm_send_start(serialcomm_t* sc) {
    serialcomm_quick_send(sc, FRAME_START_ID,
            FRAME_START_VALUE32, FRAME_START_VALUE32);
}

// called when a single character is received on the serial line.
void serialcomm_receive_char(serialcomm_t* sc, uint8_t c) {
    DEBUG_PRINT(">");
    if (sc->status == SERIALCOMM_STATUS_OK) {
        // Normal operation: fill buffer
        DEBUG_PRINT("SC_OK ");
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
        DEBUG_PRINT("SC_PS ");
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
        DEBUG_PRINT("SC_ST ");
        if (c != FRAME_START_VALUE) {
            sc->status = SERIALCOMM_STATUS_OK;
            sc->rx_frame->message.ID = c;
            sc->rx_cnt = 1;
        }
    }
}

void serialcomm_rx_end(serialcomm_t* sc, uint8_t received_checksum) {
    if (frame_checksum(sc->rx_frame) == received_checksum) {
        if (sc->rx_frame->message.ID != FRAME_SPECIAL_ID &&
            sc->rx_frame->message.ID != FRAME_START_ID) {
            if (sc->rx_complete_callback) {
                sc->rx_complete_callback(&sc->rx_frame->message);
            }
        } else {
            // Handle special frames
            if (sc->rx_frame->message.value.v32[0] == FRAME_SPECIAL_RESTART_VALUE) {
                serialcomm_send_start(sc); // Send multiple times
                serialcomm_send_start(sc);
            }
        }
    } else {
        sc->status = SERIALCOMM_STATUS_Prestart;
        serialcomm_send_start(sc);
        serialcomm_send_start(sc);
        serialcomm_send_restart_request(sc);
    }
}

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

// Calculate checksum of a buffer.
// Parameters:
//  - buf: the buffer to calculate the checksum on.
// Returns: the checksum value (4-bit XOR of all 4-bit nibbles in 'buf')
uint8_t frame_checksum(frame_t* frame) {
    uint8_t chkbuf = frame->message.ID;
    for (int i = 0; i < MESSAGE_VALUE_SIZE; i++) {
        chkbuf ^= frame->message.value.v8[i];
    }
    return chkbuf;
}

// Reset the receiving line. New data will be interpreted as a new frame.
void serialcomm_rx_reset(serialcomm_t* sc) {
    sc->rx_cnt = 0;
}

// Send the data loaded into the serialcomm_tx_buffer over the serial line.
// Calculates checksum and sends data byte-by-byte
void serialcomm_send(serialcomm_t* sc) {
    if (sc->tx_byte == 0)
        return;
    sc->tx_byte(sc->tx_frame->message.ID);
    DEBUG_PRINT("< SEND %3u = 0x%02hx ID\n", sc->tx_frame->message.ID, sc->tx_frame->message.ID);
    for (int tx_ptr = 0; tx_ptr < MESSAGE_VALUE_SIZE; tx_ptr++) {
        uint8_t c = sc->tx_frame->message.value.v8[tx_ptr];
        sc->tx_byte(c);
        DEBUG_PRINT("< SEND %3u = 0x%02hx #%d\n", c, c, tx_ptr);
    }
    sc->tx_frame->checksum = frame_checksum(sc->tx_frame);
    sc->tx_byte(sc->tx_frame->checksum);
    DEBUG_PRINT("< SEND %3u = 0x%02hx CHK\n", sc->tx_frame->checksum, sc->tx_frame->checksum);
}
