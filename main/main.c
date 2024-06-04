/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include <math.h>

#define PWM_CHANNEL_NUM 8
uint8_t RC_CHECK = 63;

//imu_param
#define MPU_ADDR 0x68
#define HMC5883_ADDR 0X3C
#define CALIB_TIMES 50

double G = 9.80665;
double PI = 3.1415926535;
float accel_scale=16384;
float gyro_scale=7509.872412338726;
int YAW_CAL = 0;

struct AccelGyroData_t
{
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
};

struct AccelGyroData_int32_t
{
    int accelX;
    int accelY;
    int accelZ;
    int gyroX;
    int gyroY;
    int gyroZ;
};

struct AccelGyroPHYSICSData
{
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float roll;
    float pitch;
    float yaw;
};

struct AccelGyroData_int32_t OFFSET_RAW = {0, 0, 0, 0, 0, 0};

//uint16_t out_cha[PWM_CHANNEL_NUM] = {0};

typedef struct motor{
    uint16_t motor_1;
    uint16_t motor_2;
    uint16_t motor_3;
    uint16_t motor_4;
}MOTOR;

typedef struct sbus_channel_16{
    uint16_t CH1;
    uint16_t CH2;
    uint16_t CH3;
    uint16_t CH4;
    uint16_t CH5;
    uint16_t CH6;
    uint16_t CH7;
    uint16_t CH8;
    uint16_t CH9;
    uint16_t CH10;
    uint16_t CH11;
    uint16_t CH12;
    uint16_t CH13;
    uint16_t CH14;
    uint16_t CH15;
    uint16_t CH16;
}SBUS;

SBUS RC_DATA = {0};

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

void HW_init()
{
    uart_config_t uart_config = {
        .baud_rate = 100000,			//波特率
        .data_bits = UART_DATA_8_BITS,	//数据位
        .parity = UART_PARITY_EVEN,	//奇偶校验
        .stop_bits = UART_STOP_BITS_2,	//停止位
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,	//流控
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, GPIO_NUM_45, GPIO_NUM_46, GPIO_NUM_1, GPIO_NUM_2));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, 1024 * 2, 0, 0, NULL, 0));
    //motor init
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 50,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_timer.timer_num = LEDC_TIMER_1;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 9,
        .duty           = 410, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = 10;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
     ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = 11;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_channel.channel = LEDC_CHANNEL_3;
    ledc_channel.gpio_num = 12;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    //imu_init
    i2c_config_t conf={
        .mode=I2C_MODE_MASTER,
        .sda_io_num=4,
        .scl_io_num=5,
        .sda_pullup_en=1,
        .scl_pullup_en=1,
        .master.clk_speed=400000
    };
    i2c_param_config(0,&conf);
    i2c_driver_install(0,I2C_MODE_MASTER,0,0,0);
    //init 6500
    uint8_t PWR_MGMT_1[] = {107,0x01};
    uint8_t PWR_MGMT_2[] = {108,0x00};
    uint8_t CONFIG[] = {26,0x0};
    uint8_t GYRO_CONFIG[] = {27,0b00000000};
    uint8_t ACCEL_CONFIG[] = {28,0b00000000};
    uint8_t MST_EN = {106, 0x00};
    uint8_t BYPASS_EN = {0x37, 0x02};
    write_register(MPU_ADDR, PWR_MGMT_1);
    write_register(MPU_ADDR, PWR_MGMT_2);
    write_register(MPU_ADDR, CONFIG);
    write_register(MPU_ADDR, GYRO_CONFIG);
    write_register(MPU_ADDR, ACCEL_CONFIG);
    // write_register(MPU_ADDR, MST_EN);
    // write_register(MPU_ADDR, BYPASS_EN);
    accel_scale=16384; //+-2G
    gyro_scale=7509.872412338726; //+-250 /rad
    //init HMC5883L
    uint8_t reg_A[] = {0, 0b01110000};
    uint8_t reg_B[] = {1, 0b00100000};
    uint8_t reg_mod[] = {2, 0b00000000};
    write_register(HMC5883_ADDR, reg_A);
    write_register(HMC5883_ADDR, reg_B);
    write_register(HMC5883_ADDR, reg_mod);
    // init BMP180
    //不想配了，反正不准

}

uint8_t list3[3]={0};

struct AccelGyroData_t get_raw_GY_87_data16(){
    struct AccelGyroData_t AccelGyroData;
    AccelGyroData.accelX = (read_register(MPU_ADDR, 59) << 8) + read_register(MPU_ADDR,60);
    AccelGyroData.accelY = (read_register(MPU_ADDR,61) << 8) + read_register(MPU_ADDR,62);
    AccelGyroData.accelZ = (read_register(MPU_ADDR,63) << 8) + read_register(MPU_ADDR,63);
    AccelGyroData.gyroX = (read_register(MPU_ADDR,67) << 8) + read_register(MPU_ADDR,68);
    AccelGyroData.gyroY = (read_register(MPU_ADDR,69) << 8) + read_register(MPU_ADDR,70);
    AccelGyroData.gyroZ = (read_register(MPU_ADDR,71) << 8) + read_register(MPU_ADDR,72);
    // AccelGyroData.roll = (read_register(HMC5883_ADDR,3) << 8) + read_register(HMC5883_ADDR,4);
    // AccelGyroData.yaw = (read_register(HMC5883_ADDR,5) << 8) + read_register(HMC5883_ADDR,6);
    // AccelGyroData.pitch = (read_register(HMC5883_ADDR,7) << 8) + read_register(HMC5883_ADDR,8);
    // list3[0] = read_register(HMC5883_ADDR, 0);
    // list3[1] = read_register(HMC5883_ADDR, 1); 
    // list3[2] = read_register(HMC5883_ADDR, 2);
    // uint8_t dataHMC[6] = {0};
    // read_register_stream(HMC5883_ADDR, 3, dataHMC, 6);
    // AccelGyroData.roll = (dataHMC[0]<<8) + dataHMC[1];
    // AccelGyroData.yaw = (dataHMC[2]<<8) + dataHMC[3];
    // AccelGyroData.pitch = (dataHMC[4]<<8) + dataHMC[5];
    return AccelGyroData;
}

struct AccelGyroData_int32_t get_raw_mpu6050_data32()
{
    struct AccelGyroData_int32_t AccelGyroData;
    AccelGyroData.accelX = (int)(int16_t)(read_register(MPU_ADDR, 59) << 8) + read_register(MPU_ADDR, 60);
    AccelGyroData.accelY = (int)(int16_t)(read_register(MPU_ADDR, 61) << 8) + read_register(MPU_ADDR, 62);
    AccelGyroData.accelZ = (int)(int16_t)(read_register(MPU_ADDR, 63) << 8) + read_register(MPU_ADDR, 63);
    AccelGyroData.gyroX = (int)(int16_t)(read_register(MPU_ADDR, 67) << 8) + read_register(MPU_ADDR, 68);
    AccelGyroData.gyroY = (int)(int16_t)(read_register(MPU_ADDR, 69) << 8) + read_register(MPU_ADDR, 70);
    AccelGyroData.gyroZ = (int)(int16_t)(read_register(MPU_ADDR, 71) << 8) + read_register(MPU_ADDR, 72);
   
    return AccelGyroData;
};

void MPU_OFFSET()
{
    for(int i=0; i<CALIB_TIMES; i++)
    {
        struct AccelGyroData_int32_t data = get_raw_mpu6050_data32();
        OFFSET_RAW.accelX += data.accelX;
        OFFSET_RAW.accelY += data.accelY;
        OFFSET_RAW.accelZ += data.accelZ;
        OFFSET_RAW.gyroX += data.gyroX;
        OFFSET_RAW.gyroY += data.gyroY;
        OFFSET_RAW.gyroZ += data.gyroZ;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    OFFSET_RAW.accelX /= CALIB_TIMES;
    OFFSET_RAW.accelY /= CALIB_TIMES;
    OFFSET_RAW.accelZ /= CALIB_TIMES;
    OFFSET_RAW.gyroX /= CALIB_TIMES;
    OFFSET_RAW.gyroY /= CALIB_TIMES;
    OFFSET_RAW.gyroZ /= CALIB_TIMES;
    OFFSET_RAW.accelZ -= accel_scale;
    printf("OFFSET COLLECTED\n");
}

struct AccelGyroData_int32_t get_calibration_data()
{
    struct AccelGyroData_int32_t AccelGyroData;
    AccelGyroData.accelX = (int)(int16_t)(read_register(MPU_ADDR, 59) << 8) + read_register(MPU_ADDR, 60) - OFFSET_RAW.accelX;
    AccelGyroData.accelY = (int)(int16_t)(read_register(MPU_ADDR, 61) << 8) + read_register(MPU_ADDR, 62) - OFFSET_RAW.accelY;
    AccelGyroData.accelZ = (int)(int16_t)(read_register(MPU_ADDR, 63) << 8) + read_register(MPU_ADDR, 63) - OFFSET_RAW.accelZ;
    AccelGyroData.gyroX = (int)(int16_t)(read_register(MPU_ADDR, 67) << 8) + read_register(MPU_ADDR, 68) - OFFSET_RAW.gyroX;
    AccelGyroData.gyroY = (int)(int16_t)(read_register(MPU_ADDR, 69) << 8) + read_register(MPU_ADDR, 70) - OFFSET_RAW.gyroY;
    AccelGyroData.gyroZ = (int)(int16_t)(read_register(MPU_ADDR, 71) << 8) + read_register(MPU_ADDR, 72) - OFFSET_RAW.gyroZ;
    return AccelGyroData;
}

struct AccelGyroPHYSICSData get_PHYSICS_Data()
{
    struct AccelGyroPHYSICSData phydata;
    struct AccelGyroData_int32_t rawData = get_calibration_data();
    phydata.accelX = 1.0 * rawData.accelX / accel_scale * G;
    phydata.accelY = 1.0 * rawData.accelY / accel_scale * G;
    phydata.accelZ = 1.0 * rawData.accelZ / accel_scale * G;
    phydata.gyroX = 1.0 * rawData.gyroX / gyro_scale;
    phydata.gyroY = 1.0 * rawData.gyroY / gyro_scale;
    phydata.gyroZ = 1.0 * rawData.gyroZ / gyro_scale;
    //phydata.roll = asin(phydata.accelY/G);
    //phydata.pitch = asin(phydata.accelX/G);
    // phydata.yaw = 1.0 * YAW_CAL / gyro_scale / 1000 * Tick;
    phydata.yaw = 0;
    if(phydata.accelY/G>=1)
        phydata.roll = asin(1);
    else if(phydata.accelY/G <= -1)
        phydata.roll = asin(-1);
    else if(phydata.accelZ >= 0)
        phydata.roll = asin(phydata.accelY/G);
    else if(phydata.accelZ < 0 && phydata.accelY < 0)
        phydata.roll = -asin(phydata.accelY/G) - PI;
    else
        phydata.roll = -asin(phydata.accelY/G) + PI;
    
    if(phydata.accelX/G>=1)
        phydata.pitch = asin(1);
    else if(phydata.accelX/G <= -1)
        phydata.pitch = asin(-1);
    else if(phydata.accelZ >= 0)
        phydata.pitch = asin(phydata.accelX/G);
    else if(phydata.accelZ < 0 && phydata.accelX < 0)
        phydata.pitch = -asin(phydata.accelX/G) - PI;
    else
        phydata.pitch = -asin(phydata.accelX/G) + PI;
    return phydata;
}

static void rx_task(void *arg)
{
    
    uint8_t data[64] = {0};
    uint16_t byte_list[24] = {0};
    int i;
    while (1)
    {
    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length));
    length = uart_read_bytes(UART_NUM_2, data, length, 5);
    for(i=0; i<128; i++)
    {   
        if(data[i]==0x0f)
        {
            break;
        }
    }
    for(int j=0; j<24; j++)
    {
        byte_list[j] = data[i+j+1];
    }

    // out_cha[0] = ((byte_list[0]>>0 | (byte_list[1]<<8)) & 2047);
    // out_cha[1] = ((byte_list[1]>>3 | (byte_list[2]<<5)) & 2047);
    // out_cha[2] = ((byte_list[2]>>6 | (byte_list[3]<<2) | (byte_list[4]<<10)) & 2047);
    // out_cha[3] = ((byte_list[4]>>1 | (byte_list[5]<<7)) & 2047);
    // out_cha[4] = ((byte_list[5]>>4 | (byte_list[6]<<4)) & 2047);
    // out_cha[5] = ((byte_list[6]>>7 | (byte_list[7]<<1) | (byte_list[8]<<9)) & 2047);
    // out_cha[6] = ((byte_list[8]>>2 | (byte_list[9]<<6)) & 2047);
    // out_cha[7] = ((byte_list[9]>>5 | (byte_list[10]<<3)) & 2047);
    // out_cha[8] = ((byte_list[11]>>0 | (byte_list[12]<<8)) & 2047);
    // out_cha[9] = ((byte_list[12]>>3 | (byte_list[13]<<5)) & 2047);
    // out_cha[10] = ((byte_list[13]>>6 | (byte_list[14]<<2) | (byte_list[15]<<10)) & 2047);
    // out_cha[11] = ((byte_list[15]>>1 | (byte_list[16]<<7)) & 2047);
    // out_cha[12] = ((byte_list[16]>>4 | (byte_list[17]<<4)) & 2047);
    // out_cha[13] = ((byte_list[17]>>7 | (byte_list[18]<<1) | (byte_list[19]<<9)) & 2047);
    // out_cha[14] = ((byte_list[19]>>2 | (byte_list[20]<<6)) & 2047);
    // out_cha[15] = ((byte_list[20]>>5 | (byte_list[21]<<3)) & 2047);
    // printf("|");
    // for(i=0; i<PWM_CHANNEL_NUM; i++)
    // {
    //     printf("CH%d %4d|", i+1, out_cha[i]);
    // }
    // printf("\n");

    RC_DATA.CH1 = ((byte_list[0]>>0 | (byte_list[1]<<8)) & 2047);
    RC_DATA.CH2 = ((byte_list[1]>>3 | (byte_list[2]<<5)) & 2047);
    RC_DATA.CH3 = ((byte_list[2]>>6 | (byte_list[3]<<2) | (byte_list[4]<<10)) & 2047);
    RC_DATA.CH4 = ((byte_list[4]>>1 | (byte_list[5]<<7)) & 2047);
    RC_DATA.CH5 = ((byte_list[5]>>4 | (byte_list[6]<<4)) & 2047);
    RC_DATA.CH6 = ((byte_list[6]>>7 | (byte_list[7]<<1) | (byte_list[8]<<9)) & 2047);
    RC_DATA.CH7 = ((byte_list[8]>>2 | (byte_list[9]<<6)) & 2047);
    RC_DATA.CH8 = ((byte_list[9]>>5 | (byte_list[10]<<3)) & 2047);
    RC_CHECK = byte_list[22];
    //printf("%d %d\n", (int)byte_list[22], (int)byte_list[23]);

    vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

static void motor_out_task(void *arg)
{
    MOTOR  MOUT= {410, 410, 410, 410};
    u_int8_t flag_start_check = 1;
    while (1)
    {
        if(RC_DATA.CH7 <= 400 && RC_CHECK == 0){
            if(flag_start_check == 1 && RC_DATA.CH3 < 240)
            {
                flag_start_check = 0;
            }
            else if(flag_start_check == 1 && RC_DATA.CH3 >= 240)
            {
                printf("UNABLE TO DISARM, CHECK THE Throttle Channel\n");
            }
            else
            {
                MOUT.motor_1 = 1.0 * ((int)(RC_DATA.CH3 - 200))*410*0.9/1600+410+41;
                MOUT.motor_2 = 1.0 * ((int)(RC_DATA.CH3 - 200))*410*0.9/1600+410+41;
                MOUT.motor_3 = 1.0 * ((int)(RC_DATA.CH3 - 200))*410*0.9/1600+410+41;
                MOUT.motor_4 = 1.0 * ((int)(RC_DATA.CH3 - 200))*410*0.9/1600+410+41;
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MOUT.motor_1);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MOUT.motor_2);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MOUT.motor_3);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MOUT.motor_4);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
            }
        }
        else
        {
            flag_start_check = 1;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 410);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 410);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 410);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 410);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
        }
        vTaskDelay(20/portTICK_PERIOD_MS);
    }
}

static void get_imu_task(void *arg)
{
    //struct AccelGyroPHYSICSData data;
    struct AccelGyroData_t imu_data_raw = {0};
    while (1)
    {
        imu_data_raw = get_raw_GY_87_data16();
        printf("%x %x %x ", list3[0], list3[1], list3[2]);
        printf("ROLL %3.5f, PITCH %3.5f, YAW %3.5f\n", 1.0 * (int16_t)imu_data_raw.roll/1090 , 1.0 * (int16_t)imu_data_raw.pitch/1090 , 1.0 * (int16_t)imu_data_raw.yaw/1090 );
        // for(int i=3; i<20; i++)
        // {
        //     printf("%d ", read_register(HMC5883_ADDR, i));
        // }
        // printf("\n");
        //read_register_stream();
        // data = get_PHYSICS_Data();
        // printf("##################\n");
        // printf("%f\n", data.accelX);
        // printf("%f\n", data.accelY);
        // printf("%f\n", data.accelZ);
        // printf("%f\n", data.roll*180.0/PI);
        // printf("%f\n", data.pitch*180.0/PI);
        // printf("%f\n", data.yaw*180.0/PI);
        // printf("##################\n");
        // vTaskDelay(300 / portTICK_PERIOD_MS);

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    printf("INITIALIZING...\n");
    HW_init();
    printf("INITIALIZATION COMPLETED\n");
    xTaskCreate(rx_task, "uart_rx_task", 1024*4, NULL, 1, NULL);
    xTaskCreate(motor_out_task, "motor_out_task", 1024*4, NULL, 1, NULL);
    xTaskCreate(get_imu_task, "get_imu_task", 1024*4, NULL, 1, NULL);
}
