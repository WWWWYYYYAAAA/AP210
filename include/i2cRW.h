#ifndef _I2CRW_H_
#define _I2CRW_H_

void write_register(uint8_t address, uint8_t *data);
uint8_t read_register(uint8_t address, uint8_t reg);
// void read_register_stream();

#endif