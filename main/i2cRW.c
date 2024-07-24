#include "i2cRW.h"

void write_register(uint8_t address, uint8_t *data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address<<1|0, true);
    i2c_master_write(cmd, data, sizeof(data), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// 读寄存器函数
uint8_t read_register(uint8_t address, uint8_t reg){
    uint8_t data;
    i2c_master_write_read_device(0,address,&reg,1,&data,1,1000/portTICK_PERIOD_MS);
    return data;
}

void write_register_2(uint8_t address, uint8_t *data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address<<1|0, true);
    i2c_master_write(cmd, data, sizeof(data), true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// void read_register_stream()
// {
//      uint8_t dataHMC[6] = {0};
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, 0X3C<<1|0, false);
//     i2c_master_write_byte(cmd, 0x03, false);
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, 0X3C<<1|1, false);
//     i2c_master_read_byte(cmd, dataHMC[0], true);
//     i2c_master_stop(cmd);
//     i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     for(int i=0; i<6; i++)
//     {
//         printf("%x ", dataHMC[i]);
//     }
//     printf("\n");
// }
