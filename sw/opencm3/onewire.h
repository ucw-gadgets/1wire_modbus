#ifndef ONEWIRE_H
#define ONEWIRE_H

#include "stdint.h"
#include <stdbool.h>


#define OW_MAXEVENTS 256


enum oneWireEventType
{
	OW_FLOAT = 0,
	OW_HIGH = 1,
	OW_LOW = 2,
	OW_SAMPLE = 3,
	OW_END = 4,
};

	void ow_init(uint32_t port, uint16_t pin);
	void ow_reset_search(void);
	bool ow_search(uint8_t *newAddr, bool search_mode /* = true */);
	void ow_set_precision(uint8_t *addr, uint8_t prec);
	void ow_start_conversion(uint8_t *addr);
	int32_t ow_read_temperature(uint8_t *addr);
	
	void periodElapsedCallback(void);

	bool ow_reset_pulse(void);
	void ow_write_byte(uint8_t data);
	uint8_t ow_read_byte(void);
	uint8_t ow_read_address(uint8_t *addr);
	void ow_start_transaction(void);
	void ow_write_bit(bool bit);
	uint8_t ow_read_bit(void);

	uint32_t ow_reset_pulse_prepare(void);
	uint32_t ow_write_byte_prepare(uint8_t data);
	void ow_write_bit_prepare(bool bit);
	uint32_t ow_read_byte_prepare(void);
	void ow_read_bit_prepare(void);
	uint8_t crc_bits(uint8_t data);
	uint8_t dallas_crc8(const uint8_t *data, const unsigned int size);


	struct oneWireEvent_t
	{
		uint16_t next_arr;
		enum oneWireEventType event;
	};




// uint32_t ow_end(struct oneWireEvent_t events[]);

#endif