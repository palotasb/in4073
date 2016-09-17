#include "in4073.h"
#include "log.h"

uint32_t logsize;


bool log_init() {
	bool result;	
	result = spi_flash_init();
	if(result)
	{
		logsize = 0;
	}
	return result;
}

int log_getsize(){
	return logsize / sizeof(struct LOG_FORMAT);
}

bool log_write(struct LOG_FORMAT* item) {
	bool result;
	uint32_t address = logsize;
	result = flash_write_bytes(address, (uint8_t *) item, sizeof(struct LOG_FORMAT));
	
	if(result){
		logsize += sizeof(struct LOG_FORMAT);
	}

	return result;
}

bool log_read(int index, struct LOG_FORMAT* item) {
	bool result;
	uint32_t address = index * sizeof(struct LOG_FORMAT);

	result = (address + sizeof(struct LOG_FORMAT)) <= logsize;
	
	if(result)
		result = flash_read_bytes(address, (uint8_t *) item, sizeof(struct LOG_FORMAT));

	return result;
}
