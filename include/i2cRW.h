#ifndef _I2CRW_H_
#define _I2CRW_H_

#include <stdio.h>
#include "driver/i2c.h"

void write_register(uint8_t address, uint8_t *data);
uint8_t read_register(uint8_t address, uint8_t reg);
void write_register_2(uint8_t address, uint8_t *data);
// void read_register_stream();

#endif