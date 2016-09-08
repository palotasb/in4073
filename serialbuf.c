#include "serialbuf.h"

unsigned char serialbuf_rx_buffer[SERIALBUF_SIZE];
unsigned char serialbuf_tx_buffer[SERIALBUF_SIZE];

// Callback function when an entire frame is correctly received
void (*serialbuf_rx_complete_callback)(void) = 0;
// Callback function when an entire frame is received but there is a checksum error
void (*serialbuf_rx_error_callback)(void) = 0;
// The callback function to transmit a single character on the serial line
void (*serialbuf_tx_callback)(unsigned char) = 0;

static int rx_validate_checksum(void);
static void rx_end(void);
static void tx_add_chksum(void);

int rx_ptr = 0;

void serialbuf_receive_char(unsigned char c) {
    int chk_result;
    serialbuf_rx_buffer[rx_ptr++] = c;
    if (rx_ptr == SERIALBUF_SIZE) {
        rx_ptr = 0;
        rx_end();
    }
}

void rx_end(void) {
    if (rx_validate_checksum()) {
        if (serialbuf_rx_complete_callback)
            serialbuf_rx_complete_callback();
    } else {
        if (serialbuf_rx_error_callback)
            serialbuf_rx_error_callback();
    }
}

int rx_validate_chksum(void) {
    unsigned char chkbuf = chksum(serialbuf_rx_buffer);
    return (chkbuf == 0);
}

unsigned char chksum(unsigned char* buf) {
    int i;
    unsigned char chkbuf = 0;
    for (i = 0; i < SERIALBUF_SIZE; i++) {
        chkbuf ^= buf[i] & 0x0F;
        chkbuf ^= buf[i] >> 4;
    }
    return chkbuf;
}

void serialbuf_rx_reset(void) {
    rx_ptr = 0;
}

void serialbuf_send(void) {
    tx_add_chksum();
    tx_transmit();
}

void tx_add_chksum(void) {
    unsigned char chkbuf = 0;
    int i;
    // Clear chksum placeholder first
    serialbuf_tx_buffer[SERIALBUF_CHKSUM_INDEX] &= ~SERIALBUF_CHKSUM_MASK;
    // Calculate and add checksum
    chkbuf = chksum(serialbuf_tx_buffer);
    serialbuf_tx_buffer[SERIALBUF_CHKSUM_INDEX] |= (SERIALBUF_CHKSUM_MASK & (chkbuf << SERIALBUF_CHKSUM_POS));
}

void tx_transmit(void) {
    int tx_ptr;
    if (!serialbuf_tx_callback)
        return;
    for (tx_ptr = 0; tx_ptr < SERIALBUF_SIZE; tx_ptr++) {
        serialbuf_tx_callback(serialbuf_tx_buffer[tx_ptr]);
    }
}