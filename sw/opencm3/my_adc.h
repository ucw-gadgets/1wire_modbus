#ifndef MY_ADC_H
#define MY_ADC_H

#include <stdint.h>

extern volatile uint16_t adc_vin_v;
extern volatile uint16_t adc_3V3_v;
extern volatile int16_t adc_temp_int;

void adc_init(void);
void adc_measure(void);

#endif
