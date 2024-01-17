#ifndef SHT30_H
#define SHT30_H

#include <stdint.h>

extern int16_t sht_temp;
extern uint16_t sht_hum;

void sht_init(void);
void sht_read(void);

#endif