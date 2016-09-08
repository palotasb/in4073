#include "serialbuf.h"

static int rx_validate_checksum(serialbuf_t* sb);
static void rx_end(serialbuf_t* sb);
static void tx_add_chksum(serialbuf_t* sb);


void serialbuf_init(serialbuf_t* sb) {
    int i;
    sb->rx_ptr                  = 0;
    sb->rx_complete_callback    = (void (*)(void)) 0;
    sb->rx_error_callback       = (void (*)(void)) 0;
    sb->tx_callback             = (void (*)(unsigned char)) 0;
    for (i = 0; i < SERIALBUF_SIZE; i++) {
        sb->rx_buffer[i] = 0;
        sb->tx_buffer[i] = 0;
    }
}

// called when a single character is received on the serial line.
void serialbuf_receive_char(serialbuf_t* sb, unsigned char c) {
    int chk_result;
    sb->rx_buffer[rx_ptr++] = c; // fill buffer
    if (sb->rx_ptr == SERIALBUF_SIZE) {
        sb->rx_ptr = 0;
        rx_end(sb); // checksum check and callback
    }
}

// rx_end: Used to handle when all bytes are received and the buffer is full. Checks checksum and does callbacks.
void rx_end(serialbuf_t* sb) {
    if (rx_validate_checksum(sb)) {
        if (sb->rx_complete_callback)
            sb->rx_complete_callback();
    } else {
        if (sb->rx_error_callback)
            sb->rx_error_callback();
    }
}

// rx_validate_checksum: Used to check the checksum on the received data.
// returns 1 if the data and the checksum in the serialbuf->rx_buffer are consistent, otherwise 0
int rx_validate_chksum(serialbuf_t* sb) {
    unsigned char chkbuf = chksum(sb->rx_buffer);
    return (chkbuf == 0);
}

// Calculate checksum of a buffer.
// Parameters:
//  - buf: the buffer to calculate the checksum on.
// Returns: the checksum value (4-bit XOR of all 4-bit nibbles in 'buf')
unsigned char chksum(unsigned char* buf) {
    int i;
    unsigned char chkbuf = 0;
    for (i = 0; i < SERIALBUF_SIZE; i++) {
        chkbuf ^= buf[i] & 0x0F;
        chkbuf ^= buf[i] >> 4;
    }
    return chkbuf;
}

// Reset the receiving line. New data will be interpreted as a new frame.
void serialbuf_rx_reset(serialbuf_t* sb) {
    sb->rx_ptr = 0;
}

// Send the data loaded into the serialbuf_tx_buffer over the serial line.
// Calculates checksum and sends data byte-by-byte
void serialbuf_send(serialbuf_t* sb) {
    tx_add_chksum(sb);
    tx_transmit(sb);
}

// tx_add_chksum: Adds checksum to the 
void tx_add_chksum(serialbuf_t* sb) {
    unsigned char chkbuf = 0;
    int i;
    // Clear chksum placeholder first
    sb->tx_buffer[SERIALBUF_CHKSUM_INDEX] &= ~SERIALBUF_CHKSUM_MASK;
    // Calculate and add checksum
    chkbuf = chksum(sb->tx_buffer);
    sb->tx_buffer[SERIALBUF_CHKSUM_INDEX] |= (SERIALBUF_CHKSUM_MASK & (chkbuf << SERIALBUF_CHKSUM_POS));
}

// Calls the serialbuf->tx_callback function on all bytes of the tx buffer to send each byte
void tx_transmit(serialbuf_t* sb) {
    int tx_ptr;
    if (!sb->tx_callback)
        return;
    for (tx_ptr = 0; tx_ptr < SERIALBUF_SIZE; tx_ptr++) {
        sb->tx_callback(sb->tx_buffer[tx_ptr]);
    }
}
