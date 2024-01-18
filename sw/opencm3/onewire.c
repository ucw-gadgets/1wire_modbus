#include "onewire.h"

#include <stdbool.h>

#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/nvic.h"
#include "config.h"

uint32_t port;
uint16_t pin;

uint8_t LastDiscrepancy = 0;
bool LastDeviceFlag = false;
uint8_t LastFamilyDiscrepancy = 0;
uint8_t ROM_NO[8];

volatile struct oneWireEvent_t events[OW_MAXEVENTS];
volatile uint32_t eventCount = 0;
uint8_t samples;

volatile int ow_wait;


void ow_init(void)
{
	timer_set_prescaler(TIM1, CPU_CLOCK_MHZ - 1);	// 1 tick = 1 μs
	//timer_set_prescaler(TIM1, 1);	// 1 tick = 1 μs
	timer_set_mode(TIM1, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_DOWN);
	//timer_update_on_overflow(TIM1);
	nvic_enable_irq(NVIC_TIM1_BRK_UP_TRG_COM_IRQ);	
	timer_disable_preload(TIM1);
}

void ow_init_pin(uint32_t aport, uint16_t apin){
	port = aport;
	pin = apin;

	gpio_mode_setup(port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, pin);
	gpio_set_output_options(port, GPIO_OTYPE_OD, GPIO_OSPEED_HIGH, pin);
	gpio_set(port, pin);
}

void ow_set_pin(uint32_t aport, uint16_t apin){
	port = aport;
	pin = apin;
}

uint8_t crc_bits(uint8_t data)
{
	uint8_t crc = 0;
	if (data & 1)
		crc ^= 0x5e;
	if (data & 2)
		crc ^= 0xbc;
	if (data & 4)
		crc ^= 0x61;
	if (data & 8)
		crc ^= 0xc2;
	if (data & 0x10)
		crc ^= 0x9d;
	if (data & 0x20)
		crc ^= 0x23;
	if (data & 0x40)
		crc ^= 0x46;
	if (data & 0x80)
		crc ^= 0x8c;
	return crc;
}

uint8_t dallas_crc8(const uint8_t *data, const unsigned int size)
{
	uint8_t crc = 0;
	for (unsigned int i = 0; i < size; ++i)
	{
		crc = crc_bits(data[i] ^ crc);
	}
	return crc;
}

uint32_t ow_reset_pulse_prepare()
{
	events[0].next_arr = 500;
	events[0].event = OW_FLOAT;
	events[1].next_arr = 68;
	events[1].event = OW_SAMPLE;
	events[2].next_arr = 420;
	events[2].event = OW_END;
	events[3].next_arr = 10;
	return 3;
}

uint32_t ow_write_byte_prepare(uint8_t data)
{
	for (int i = 0; i < 8; i++)
	{
		if (data & (1 << i))
		{
			events[2 * i].next_arr = 5;
			events[2 * i + 1].next_arr = 60;
		}
		else
		{
			events[2 * i].next_arr = 60;
			events[2 * i + 1].next_arr = 5;
		}
		events[2 * i].event = OW_HIGH;
		events[2 * i + 1].event = OW_LOW;
	}
	events[15].event = OW_END;
	events[16].next_arr = 10;
	return 16;
}

void ow_write_bit_prepare(bool bit)
{
	if (bit)
	{
		events[0].next_arr = 5;
		events[1].next_arr = 60;
	}
	else
	{
		events[0].next_arr = 60;
		events[1].next_arr = 5;
	}
	events[0].event = OW_HIGH;
	events[1].event = OW_END;
	events[2].next_arr = 10;
}

uint32_t ow_read_byte_prepare()
{
	for (int i = 0; i < 8; i++)
	{
		events[3 * i].next_arr = 1;
		events[3 * i].event = OW_FLOAT;
		events[3 * i + 1].next_arr = 14;
		events[3 * i + 1].event = OW_SAMPLE;
		events[3 * i + 2].next_arr = 60;
		events[3 * i + 2].event = OW_LOW;
	}
	events[23].event = OW_END;
	events[24].next_arr = 10;
	return 24;
}

void ow_read_bit_prepare()
{
	events[0].next_arr = 1;
	events[0].event = OW_FLOAT;
	events[1].next_arr = 14;
	events[1].event = OW_SAMPLE;
	events[2].next_arr = 60;
	events[2].event = OW_END;
	events[3].next_arr = 10;
}

void ow_start_transaction(void)
{
	eventCount = 0;
	samples = 0;
	while (ow_wait){};
	ow_wait = 1;
	// printf("GPIO: %u\n",HAL_GPIO_ReadPin(ONEWIRE_GPIO_Port, ONEWIRE_Pin));
	//HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	//timer_generate_event(TIM1, TIM_EGR_UG);
	timer_set_counter(TIM1, events[0].next_arr);
	timer_set_period(TIM1, events[1].next_arr);
	TIM_SR(TIM1) &= ~TIM_SR_UIF;
	timer_enable_irq(TIM1, TIM_DIER_UIE);
	gpio_clear(port, pin);
	timer_enable_counter(TIM1);
	/*__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
	__HAL_TIM_SetCounter(&htim8, events[0].next_arr);
	__HAL_TIM_SetAutoreload(&htim8, events[1].next_arr);
	if (HAL_TIM_Base_Start_IT(&htim8) != HAL_OK)
	{
		printf("HAL TIM fail\n");
	}*/
}

bool ow_reset_pulse(void)
{
	ow_reset_pulse_prepare();
	ow_start_transaction();
	bool result;
	volatile uint32_t cnt;
	while(ow_wait){
		cnt = timer_get_counter(TIM1);
	}
	result = !samples;
	ow_wait = 0;
	// printf("Samples: %x\n",oneWireSamples);
	return result;
}

void ow_write_byte(uint8_t data)
{
	ow_write_byte_prepare(data);
	ow_start_transaction();
	while (ow_wait){}
	// printf("Byte written\n", oneWireSamples);
}

void ow_write_bit(bool bit)
{
	ow_write_bit_prepare(bit);
	ow_start_transaction();
	while (ow_wait){}
}

uint8_t ow_read_byte(void)
{
	ow_read_byte_prepare();
	ow_start_transaction();
	uint8_t result;
	while (ow_wait){}
	ow_wait = 1;
	// printf("Data: %x\n", oneWireSamples);
	result = samples;
	ow_wait = 0;
	return result;
}

uint8_t ow_read_bit(void)
{
	ow_read_bit_prepare();
	ow_start_transaction();
	uint8_t result;
	while (ow_wait){}	
	ow_wait = 1;
	result = !!samples;
	ow_wait = 0;
	return result;
}

uint8_t ow_read_address(uint8_t *addrDst)
{
	uint8_t addr[8];
	ow_reset_pulse();
	ow_write_byte(0x33);
	for (int i = 0; i < 8; i++)
	{
		addr[i] = ow_read_byte();
	}
	for (int i = 0; i < 8; i++)
	{
		addrDst[i] = addr[i];
	}
	if (dallas_crc8(addr, 8) != 0x00)
	{
		//printf("CRC fail\n");
		return 1;
	}
	return 0;
}

void ow_start_conversion(uint8_t *addr)
{
	ow_reset_pulse();
	ow_write_byte(0x55);
	for (int i = 0; i < 8; i++)
	{
		ow_write_byte(addr[i]);
	}
	ow_write_byte(0x44);
}

int32_t ow_read_temperature(uint8_t *addr)
{
	ow_reset_pulse();
	ow_write_byte(0x55);
	for (int i = 0; i < 8; i++)
	{
		ow_write_byte(addr[i]);
	}
	ow_write_byte(0xBE);
	uint8_t result[9];
	for (int i = 0; i < 9; i++)
	{
		result[i] = ow_read_byte();
	}
	if (dallas_crc8(result, 9) != 0x00)
	{
		/*printf("CRC Error\n");
		printf("Got ");
		for (int i = 0; i < 9; i++)
		{
			printf("%x ", result[i]);
		}
		printf("\n");*/
		return -12800;
	}
	return ((int32_t) result[0] | (result[1]<<8))*100/16;
	/*printf("Scratchpad: ");
	for (int i=0;i<9;i++){
		printf("%x ",result[i]);
	}
	printf("\n");*/
}

void ow_set_precision(uint8_t *addr, uint8_t prec)
{
	ow_reset_pulse();
	ow_write_byte(0x55);
	for (int i = 0; i < 8; i++)
	{
		ow_write_byte(addr[i]);
	}
	ow_write_byte(0x4e);
	ow_write_byte(0x00); // Th
	ow_write_byte(0x00); // Tl
	ow_write_byte(prec);

	ow_reset_pulse();
	ow_write_byte(0x55);
	for (int i = 0; i < 8; i++)
	{
		ow_write_byte(addr[i]);
	}
	ow_write_byte(0x48);
}

//
// You need to use this function to start a search again from the beginning.
// You do not need to do it for the first search, though you could.
//
void ow_reset_search()
{
	// reset the search state
	LastDiscrepancy = 0;
	LastDeviceFlag = false;
	LastFamilyDiscrepancy = 0;
	for (int i = 7;; i--)
	{
		ROM_NO[i] = 0;
		if (i == 0)
			break;
	}
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
bool ow_search(uint8_t *newAddr, bool search_mode /* = true */)
{
	uint8_t id_bit_number;
	uint8_t last_zero, rom_byte_number;
	bool search_result;
	uint8_t id_bit, cmp_id_bit;

	unsigned char rom_byte_mask, search_direction;

	// initialize for search
	id_bit_number = 1;
	last_zero = 0;
	rom_byte_number = 0;
	rom_byte_mask = 1;
	search_result = false;

	// if the last call was not the last one
	if (!LastDeviceFlag)
	{
		// 1-Wire reset
		if (!ow_reset_pulse())
		{
			// reset the search
			LastDiscrepancy = 0;
			LastDeviceFlag = false;
			LastFamilyDiscrepancy = 0;
			return false;
		}

		// issue the search command
		if (search_mode == true)
		{
			ow_write_byte(0xF0); // NORMAL SEARCH
		}
		else
		{
			ow_write_byte(0xEC); // CONDITIONAL SEARCH
		}

		// loop to do the search
		do
		{
			// read a bit and its complement
			id_bit = ow_read_bit();
			cmp_id_bit = ow_read_bit();

			// check for no devices on 1-wire
			if ((id_bit == 1) && (cmp_id_bit == 1))
			{
				break;
			}
			else
			{
				// all devices coupled have 0 or 1
				if (id_bit != cmp_id_bit)
				{
					search_direction = id_bit; // bit write value for search
				}
				else
				{
					// if this discrepancy if before the Last Discrepancy
					// on a previous next then pick the same as last time
					if (id_bit_number < LastDiscrepancy)
					{
						search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
					}
					else
					{
						// if equal to last pick 1, if not then pick 0
						search_direction = (id_bit_number == LastDiscrepancy);
					}
					// if 0 was picked then record its position in LastZero
					if (search_direction == 0)
					{
						last_zero = id_bit_number;

						// check for Last discrepancy in family
						if (last_zero < 9)
							LastFamilyDiscrepancy = last_zero;
					}
				}

				// set or clear the bit in the ROM byte rom_byte_number
				// with mask rom_byte_mask
				if (search_direction == 1)
					ROM_NO[rom_byte_number] |= rom_byte_mask;
				else
					ROM_NO[rom_byte_number] &= ~rom_byte_mask;

				// serial number search direction write bit
				ow_write_bit(search_direction);

				// increment the byte counter id_bit_number
				// and shift the mask rom_byte_mask
				id_bit_number++;
				rom_byte_mask <<= 1;

				// if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
				if (rom_byte_mask == 0)
				{
					rom_byte_number++;
					rom_byte_mask = 1;
				}
			}
		} while (rom_byte_number < 8); // loop until through all ROM bytes 0-7

		// if the search was successful then
		if (!(id_bit_number < 65))
		{
			// search successful so set LastDiscrepancy,LastDeviceFlag,search_result
			LastDiscrepancy = last_zero;

			// check for last device
			if (LastDiscrepancy == 0)
			{
				LastDeviceFlag = true;
			}
			search_result = true;
		}
	}

	// if no device found then reset counters so next 'search' will be like a first
	if (!search_result || !ROM_NO[0])
	{
		LastDiscrepancy = 0;
		LastDeviceFlag = false;
		LastFamilyDiscrepancy = 0;
		search_result = false;
	}
	else
	{
		for (int i = 0; i < 8; i++)
			newAddr[i] = ROM_NO[i];
	}
	return search_result;
}

void tim1_brk_up_trg_com_isr(){
    if (TIM_SR(TIM1) & TIM_SR_UIF){
	TIM_SR(TIM1) &= ~TIM_SR_UIF;
    } else {
	return;
    }
    gpio_set(port,pin);
    volatile uint16_t cnt;
    cnt = timer_get_counter(TIM1);
    timer_set_period(TIM1, events[eventCount+2].next_arr);
    //__HAL_TIM_SetAutoreload(&htim8,events[eventCount+2].next_arr);
    struct oneWireEvent_t event;
    event = events[eventCount];
    switch (event.event){
      case OW_HIGH:
	GPIO_OTYPER(port) &= ~pin;
	gpio_set(port, pin);
        break;
      case OW_LOW:
	gpio_clear(port, pin);
        break;
      case OW_FLOAT:
	GPIO_OTYPER(port) |= pin;
	gpio_set(port, pin);
        break;
      case OW_SAMPLE:
        samples >>= 1;
        samples |= (!!gpio_get(port, pin))<<7;
        break;
      case OW_END:
	gpio_set(port, pin);
	timer_disable_counter(TIM1);
//        HAL_TIM_Base_Stop_IT(&htim8);
	ow_wait = 0;
        break;
    } 
    eventCount++;
    //HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin, GPIO_PIN_RESET);
}
