#include "sht30.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/i2c.h"

int16_t sht_temp;
uint16_t sht_hum;

static uint32_t last_read;
extern volatile uint32_t ms_ticks;

void sht_init(void){
	uint8_t i2c_buf[2] = {0x20, 0x32};
	i2c_transfer7(I2C2, 0x44, i2c_buf, 2, NULL, 0);
	last_read =  ms_ticks;
}

void sht_read(void){
	if (ms_ticks - last_read < 2500){
		return;
	}

	uint8_t i2c_buf[2] = {0xE0, 0x00};
	uint8_t i2c_res[6];
	i2c_transfer7(I2C2, 0x44, i2c_buf, 2, i2c_res, 6);
	
	int hum_raw = (i2c_res[3]<<8) + i2c_res[4];
	int temp_raw = (i2c_res[0]<<8) + i2c_res[1];

	sht_temp = -4500+17500*temp_raw/((1<<16)-1);
	sht_hum = 10000*hum_raw/((1<<16)-1);

	last_read = ms_ticks;

}