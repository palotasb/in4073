#ifndef SERIALBUF_H
#define SERIALBUF_H

#define SERIALBUF_SIZE          10
#define SERIALBUF_CHKSUM_INDEX  0
#define SERIALBUF_CHKSUM_POS    0
#define SERIALBUF_CHKSUM_MASK   (0x0F << (SERIALBUF_CHKSUM_POS))

// Ignore this struct for now. It is for later refactoring.
typedef struct serialbuf {
    unsigned char* rx_buffer;
    unsigned char* tx_buffer;
} serialbuf_t;

extern unsigned char serialbuf_rx_buffer[SERIALBUF_SIZE];
extern unsigned char serialbuf_tx_buffer[SERIALBUF_SIZE];
extern void (*serialbuf_rx_complete_callback)(void);
extern void (*serialbuf_rx_error_callback)(void);
extern void (*serialbuf_tx_callback)(unsigned char);

void serialbuf_receive_char(unsigned char c);

void serialbuf_rx_reset(void);

void serialbuf_send(void);

#endif // #ifndef SERIALBUF_H
