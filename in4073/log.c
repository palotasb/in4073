#include <stdio.h>

#include "in4073.h"
#include "log.h"

// Flash is 1024 * 1024 bits.
// One item is 9 * 8 bits
// 1024 * 1024 / (8 * 9) = 14 563,56
#define LOG_MAX_ITEMS	14563

/** LOG FORMAT
 *
 *  | ...                                       |  Address
 *  +----------+----------+----------+----------+
 *  | ID_(n+0) | ID_(n+1) | ID_(n+2) | ID_(n+3) |  36 n
 *  +----------+----------+----------+----------+
 *  | ITEM_(n+0).v32[0]             ITEM_(n+0)  |  36 n + 4
 *  +----------------------------- - - - - - - -+
 *  | ITEM_(n+0).v32[1]                         | 
 *  +-------------------------------------------+
 *  | ITEM_(n+1).v32[0]             ITEM_(n+1)  |  36 n + 8
 *  +----------------------------- - - - - - - -+
 *  | ITEM_(n+1).v32[1]                         |
 *  +-------------------------------------------+
 *  | ITEM_(n+2).v32[0]             ITEM_(n+2)  |  36 n + 12
 *  +----------------------------- - - - - - - -+
 *  | ITEM_(n+2).v32[1]                         |
 *  +-------------------------------------------+
 *  | ITEM_(n+3).v32[0]             ITEM_(n+3)  |  36 n + 16
 *  +----------------------------- - - - - - - -+
 *  | ITEM_(n+3).v32[1]                         |
 *  +----------+----------+----------+----------+
 *  | ...      | ...      | ...      | ...      |
 *
 *  IDs contain the information needed to
 *  decode the values in the ITEMs.
 *  
 *  Address for ID of item #k is:
 *  addr(k) = floor(k, 4) * 36 + rem(k, 4)
 *  	floor(k, 4) is (k & ~0x03), round down modulo 4
 *		rem(k, 4)   is (k &  0x03), or (k % 4)
 *
 *  ITEMs have type message_value_t and can be accessed at byte,
 *  halword and word offsets via the .v8[], .v16[] or .v32[] union
 *  members.

 *  Address for ITEM #k is:
 *  addr(k) = floor(k, 4) * 36 + rem(k, 4) * 8 + 4
 *  	floor(k, 4) is (k & ~0x03)
 *		rem(k, 4)   is (k & 0x03)
 *
 *  The fill rate and status of the log is
 *	kept separately in static variables.
 *
**/

static uint32_t log_id(uint32_t index);
static uint32_t log_item(uint32_t index);

// Number of items in the log
uint32_t logsize;

// Serial communication ling
serialcomm_t* sc = 0;

// Return address of ID of item No. i
uint32_t log_id(uint32_t i) {
	return (i & ~0x03ul) * 36 + (i & 0x03ul);
}

// Return address of value of item No. i
uint32_t log_item(uint32_t i) {
	return (i & ~0x03ul) * 36 + (i & 0x03ul) * 8 + 4;
}

bool log_init(serialcomm_t* serialcomm) {	
	logsize = 0;
	bool result = spi_flash_init();
	if (result) {
		sc = serialcomm;
	}
	return result;
}

bool log_write(message_t* item) {
	if (LOG_MAX_ITEMS < logsize) {
		printf("> Log full!\n");
		return false;
	}

	bool result1 = flash_write_bytes(log_id(logsize), &item->ID, 1);
	bool result2 = flash_write_bytes(log_item(logsize), item->value.v8, 8);
	if (result1 && result2) {
		logsize++;
		return true;
	} else {
		printf("> Log wr err!\n");
		return false;
	}
}

bool log_read(uint32_t index, message_t* item) {
	if (LOG_MAX_ITEMS < index)
		return false;

	bool result1 = flash_read_bytes(log_id(index), &item->ID, 1);
	bool result2 = flash_read_bytes(log_item(index), item->value.v8, 8);
	if (result1 && result2) {
		return true;
	} else {
		return false;
	}
}

void log_readback(void) {
	message_t msg;
	printf("> Log read (sum %lu)\n", logsize);
	if (!sc)
		printf("> Log serial error\n");
	serialcomm_quick_send(sc, MESSAGE_LOG_START_ID, 0, 0);
	int i;
	for (i = 0; i < logsize; i++) {
		if (!log_read(i, &msg)) {
			break;
		}
		serialcomm_quick_send(sc, msg.ID, msg.value.v32[0], msg.value.v32[1]);
	}
	serialcomm_quick_send(sc, MESSAGE_LOG_END_ID, 0, 0);
	if (i != logsize)
		printf("> Log rd err at %d\n", i);
	log_reset();
}

void log_reset(void) {
	printf("> Log reset\n");
	logsize = 0;
	if (!flash_chip_erase())
		printf("> Chip erase failed!\n");;
}
