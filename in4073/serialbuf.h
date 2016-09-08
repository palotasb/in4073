#ifndef SERIALBUF_H
#define SERIALBUF_H

#define SERIALBUF_SIZE          10
#define SERIALBUF_CHKSUM_INDEX  0
#define SERIALBUF_CHKSUM_POS    0
#define SERIALBUF_CHKSUM_MASK   (0x0F << (SERIALBUF_CHKSUM_POS))

typedef struct serialbuf {
    // The buffer for the received data
    unsigned char rx_buffer[SERIALBUF_SIZE];
    // The buffer for the data to be sent
    unsigned char tx_buffer[SERIALBUF_SIZE];
    // Pointer pointing into `rx_buffer` where the next received byte will go
    int rx_ptr;
    // Callback function when an entire frame is correctly received
    void (*rx_complete_callback)(void);
    // Callback function when an entire frame is received but there is a checksum error
    void (*rx_error_callback)(void);
    // The callback function to transmit a single character on the serial line
    //  parameter: unsigned char - the byte to be transmitted on the serial line
    void (*tx_callback)(unsigned char);
} serialbuf_t;

// Call to initialize the serialbuf structure.
void serialbuf_init(serialbuf_t* sb);

// Call when a single character is received on the serial line.
void serialbuf_receive_char(serialbuf_t* sb, unsigned char c);

// Call to reset the rx_ptr pointer.
void serialbuf_rx_reset(serialbuf_t* sb);

// Call to calculate checksum and send data loaded into the tx_buffer.
void serialbuf_send(serialbuf_t* sb);

#endif // #ifndef SERIALBUF_H
