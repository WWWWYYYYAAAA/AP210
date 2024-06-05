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
#include "common.h"
#include "i2cRW.h"
#include "imu.h"

SBUS RC_DATA = {0};
struct AccelGyroData_int32_t OFFSET_RAW = {410, 274, 1305, -309, -73, -3};
uint8_t RC_CHECK = 63;
float G = 9.80665;
float PI = 3.1415926535;
float accel_scale=16384;
float gyro_scale=7509.872412338726;
int YAW_CAL = 0;
struct AccelGyroPHYSICSData PYHdata;

Attitude Last_attitude = {0};
Attitude Now_attitude = {0};
Attitude Desire_attitude = {0};
PID_Param pidP_list[6] = {{5, 0, 1},  //roll
                          {5, 0, 2},  //gx
                          {5, 0, 1},  //pitch
                          {5, 0, 2},  //gy
                          {8, 0, 1},   //gyroZ
                          {0.5, 0, 1}};  //throttle
int integ_clr_flag[6] = {0};

PID_Element roll_pid = {0};
PID_Element gyroX_pid = {0};
PID_Element pitch_pid = {0};
PID_Element gyroY_pid = {0};
PID_Element gyroZ_pid = {0};
PID_Element throttle_pid = {0};
DMotor motor_delta = {0, 0, 0, 0, 1, 5, 5, 5};   //ROLL PITCH YAW

float hover_percentage = 0.3;

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
    
    //uint8_t MST_EN[] = {106, 0x00};
    // uint8_t BYPASS_EN[] = {0x37, 0x02};
    // uint8_t I2C_MST_CTRL[] = {36, 0b11011101};
    // uint8_t I2C_SLV0_ADDR[] = {37, 0xBC};
    // uint8_t I2C_SLV0_REG[] = {38, 0x03};
    // uint8_t I2C_SLV0_CTRL[] = {39, 0b10100110};
   
    write_register(MPU_ADDR, PWR_MGMT_1);
    write_register(MPU_ADDR, PWR_MGMT_2);
    write_register(MPU_ADDR, CONFIG);
    write_register(MPU_ADDR, GYRO_CONFIG);
    write_register(MPU_ADDR, ACCEL_CONFIG);
    // write_register(MPU_ADDR, MST_EN);
    // write_register(MPU_ADDR, BYPASS_EN);
    // write_register(MPU_ADDR, I2C_MST_CTRL);
    // write_register(MPU_ADDR, I2C_SLV0_ADDR);
    // write_register(MPU_ADDR, I2C_SLV0_REG);
    // write_register(MPU_ADDR, I2C_SLV0_CTRL);

    // uint8_t HMC_LIST_6050[][2] = {{0x6A, 0b00000000},
    //                         {0x37, 0x02},//hmcinit
    //                         {0x37, 0x00},//2
    //                         {0x6A, 0x20},
    //                         {0x25, HMC5883_ADDR|0x80},
    //                         {0x26, 0x03},
    //                         {0x27, 6|0x80},
    //                         {0x67, 1}
    //                         };
    // uint8_t HMC_LIST_5883[][2] = {{0x00, 0x18},
    //                             {0x01, 0x60},
    //                             {0x02, 0x00}
    //                             };
    // for(int i = 0; i<2; i++)
    // {
    //     write_register(MPU_ADDR, HMC_LIST_6050[i]);
    // }
    // for(int i = 0; i<3; i++)
    // {
    //     write_register(HMC5883_ADDR, HMC_LIST_5883[i]);
    // }
    // for(int i = 2; i<8; i++)
    // {
    //     write_register(MPU_ADDR, HMC_LIST_6050[i]);
    // }

    accel_scale=16384; //+-2G
    gyro_scale=7509.872412338726; //+-250 /rad

    //init HMC5883L
    // uint8_t reg_A[] = {0, 0b01110000};
    // uint8_t reg_B[] = {1, 0b00100000};
    // uint8_t reg_mod[] = {2, 0b00000000};
    // write_register(HMC5883_ADDR, reg_A);
    // write_register(HMC5883_ADDR, reg_B);
    // write_register(HMC5883_ADDR, reg_mod);
    // init BMP180
    //不想配了，反正不准
    //caliberate the mpu6050
    //MPU_OFFSET();
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
    // printf("CH1 %4d|", RC_DATA.CH1);
    // printf("CH2 %4d|", RC_DATA.CH2);
    // printf("CH3 %4d|", RC_DATA.CH3);
    // printf("CH4 %4d|", RC_DATA.CH4);
    // printf("\n");
    vTaskDelay(20/portTICK_PERIOD_MS);
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
                Desire_attitude.roll = 1.0*(RC_DATA.CH1-1000)*PI/4/800;
                Desire_attitude.pitch = 1.0*(RC_DATA.CH2-1000)*PI/4/800;
                Desire_attitude.gyroZ = 1.0*(RC_DATA.CH4-1000)*PI/4/800;
                Desire_attitude.throttle = (RC_DATA.CH3-200);
                //printf("Desire_attitude.roll %f\n", Desire_attitude.roll);
                motor_delta.throttle = Desire_attitude.throttle;
                // motor_delta.Droll = RC_DATA.CH1-1000;
                // motor_delta.Dpitch = RC_DATA.CH2-1000;
                // motor_delta.Dyaw = RC_DATA.CH4-1000;
                MOUT.motor_1 = (motor_delta.KT*motor_delta.throttle - motor_delta.KR*motor_delta.Droll + motor_delta.KP*motor_delta.Dpitch + motor_delta.KY*motor_delta.Dyaw)*410*0.9/1600+451;
                MOUT.motor_2 = (motor_delta.KT*motor_delta.throttle + motor_delta.KR*motor_delta.Droll - motor_delta.KP*motor_delta.Dpitch + motor_delta.KY*motor_delta.Dyaw)*410*0.9/1600+451;
                MOUT.motor_3 = (motor_delta.KT*motor_delta.throttle + motor_delta.KR*motor_delta.Droll + motor_delta.KP*motor_delta.Dpitch - motor_delta.KY*motor_delta.Dyaw)*410*0.9/1600+451;
                MOUT.motor_4 = (motor_delta.KT*motor_delta.throttle - motor_delta.KR*motor_delta.Droll - motor_delta.KP*motor_delta.Dpitch - motor_delta.KY*motor_delta.Dyaw)*410*0.9/1600+451;
                if(MOUT.motor_1>820){
                    MOUT.motor_1 = 820;
                }
                else if(MOUT.motor_1<451){
                    MOUT.motor_1 = 451;
                }
                 if(MOUT.motor_2>820){
                    MOUT.motor_2 = 820;
                }
                else if(MOUT.motor_2<451){
                    MOUT.motor_2 = 451;
                }
                 if(MOUT.motor_3>820){
                    MOUT.motor_3 = 820;
                }
                else if(MOUT.motor_3<451){
                    MOUT.motor_3 = 451;
                }
                 if(MOUT.motor_4>820){
                    MOUT.motor_4 = 820;
                }
                else if(MOUT.motor_4<451){
                    MOUT.motor_4 = 451;
                }
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MOUT.motor_1);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MOUT.motor_2);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MOUT.motor_3);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MOUT.motor_4);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
                printf("%d %d %d %d\n", MOUT.motor_1, MOUT.motor_2, MOUT.motor_3, MOUT.motor_4);
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
        vTaskDelay(33/portTICK_PERIOD_MS);
    }
}

static void get_imu_task(void *arg)
{
    //struct AccelGyroData_t imu_data_raw = {0};
    PYHdata = get_PHYSICS_Data();
    while (1)
    {
        //imu_data_raw = get_raw_GY_87_data16();
        //printf("%x %x %x ", list3[0], list3[1], list3[2]);
        //printf("ROLL %d, PITCH %d, YAW %d\n", (int16_t)imu_data_raw.roll , (int16_t)imu_data_raw.pitch , (int16_t)imu_data_raw.yaw );
        // for(int i=3; i<20; i++)
        // {
        //     printf("%d ", read_register(HMC5883_ADDR, i));
        // }
        // printf("\n");
        //read_register_stream();
        // Last_attitude.gyroX = PYHdata.gyroX;
        // Last_attitude.gyroY = PYHdata.gyroY;
        // Last_attitude.gyroZ = PYHdata.gyroZ;
        // Last_attitude.roll = PYHdata.roll;
        // Last_attitude.pitch = PYHdata.pitch;
        // Last_attitude.yaw = PYHdata.yaw;
        PYHdata = get_PHYSICS_Data();
        
        // printf("##################\n");
        // printf("%f\n", PYHdata.accelX);
        // printf("%f\n", PYHdata.accelY);
        // printf("%f\n", PYHdata.accelZ);
        // printf("%f\n", PYHdata.roll*180.0/PI);
        // printf("%f\n", PYHdata.pitch*180.0/PI);
        // printf("%f\n", PYHdata.yaw*180.0/PI);
        // printf("##################\n");
        vTaskDelay(25/portTICK_PERIOD_MS);
    }
}

void PID_task()
{
    while(1){
        
        // if(integ_clr_flag[0])
        // {
        //     integ_clr_flag[0] = 0;
        //     roll_pid.integ = 0;
        // }
        // if(integ_clr_flag[1])
        // {
        //     integ_clr_flag[1] = 0;
        //     gyroX_pid.integ = 0;
        // }
        // if(integ_clr_flag[2])
        // {
        //     integ_clr_flag[2] = 0;
        //     pitch_pid.integ = 0;
        // }
        // if(integ_clr_flag[3])
        // {
        //     integ_clr_flag[3] = 0;
        //     gyroY_pid.integ = 0;
        // }
        // if(integ_clr_flag[4])
        // {
        //     integ_clr_flag[4] = 0;
        //     gyroZ_pid.integ = 0;
        // }
        //roll
        if(roll_pid.integ > 3)
        {
            roll_pid.integ = 3;
        }
        else if(roll_pid.integ <-3)
        {
            roll_pid.integ = -3;
        }
        //gyroX
        if(gyroX_pid.integ > 3)
        {
            gyroX_pid.integ = 3;
        }
        else if(gyroX_pid.integ <-3)
        {
            gyroX_pid.integ = -3;
        }
        //pitch
        if(pitch_pid.integ > 3)
        {
            pitch_pid.integ = 3;
        }
        else if(pitch_pid.integ <-3)
        {
            pitch_pid.integ = -3;
        }
        //gyroY
        if(gyroY_pid.integ > 3)
        {
            gyroY_pid.integ = 3;
        }
        else if(gyroY_pid.integ <-3)
        {
            gyroY_pid.integ = -3;
        }
        //gyroZ
        if(gyroZ_pid.integ > 5)
        {
            gyroZ_pid.integ = 5;
        }
        else if(gyroZ_pid.integ <-5)
        {
            gyroZ_pid.integ = -5;
        }
        //roll
        roll_pid.last_error = roll_pid.error;
        roll_pid.error = Desire_attitude.roll - PYHdata.roll;
        roll_pid.integ += roll_pid.error;
        roll_pid.diff = (roll_pid.error - roll_pid.last_error);
        Desire_attitude.gyroX = pidP_list[0].kp*(roll_pid.error + pidP_list[0].ki*roll_pid.integ - pidP_list[0].kd*roll_pid.diff);
        //printf("Desire_attitude.roll: %3.5f PYHdata.roll: %3.5f roll_pid.error: %3.5f\n", Desire_attitude.roll, PYHdata.roll, roll_pid.error);
        //printf(" Desire_attitude.gyroX: %f\n",  Desire_attitude.gyroX);
        //gyroX
       
        gyroX_pid.last_error = gyroX_pid.error;
        gyroX_pid.error = Desire_attitude.gyroX - PYHdata.gyroX;
        gyroX_pid.integ +=  gyroX_pid.error;
        gyroX_pid.diff = gyroX_pid.error - gyroX_pid.last_error;
        motor_delta.Droll = pidP_list[1].kp*(gyroX_pid.error + pidP_list[1].ki*gyroX_pid.integ - pidP_list[1].kd*gyroX_pid.diff);
        //printf("motor_delta.Droll : %f ", motor_delta.Droll);
        //pitch
       
        pitch_pid.last_error = pitch_pid.error;
        pitch_pid.error = Desire_attitude.pitch - PYHdata.pitch;
        pitch_pid.integ += pitch_pid.error;
        pitch_pid.diff = (pitch_pid.error - pitch_pid.last_error);
        Desire_attitude.gyroY = pidP_list[2].kp*(pitch_pid.error + pidP_list[2].ki*pitch_pid.integ - pidP_list[2].kd*pitch_pid.diff);
        //gyroY
        gyroY_pid.last_error = gyroY_pid.error;
        gyroY_pid.error = Desire_attitude.gyroY - PYHdata.gyroY;
        gyroY_pid.integ +=  gyroY_pid.error;
        gyroY_pid.diff = gyroY_pid.error - gyroY_pid.last_error;
        motor_delta.Dpitch = pidP_list[3].kp*(gyroY_pid.error + pidP_list[3].ki*gyroY_pid.integ - pidP_list[3].kd*gyroY_pid.diff);
        //printf("motor_delta.Dpitch : %f\n", motor_delta.Dpitch);
        //gyroZ
    
        gyroZ_pid.last_error = gyroZ_pid.error;
        gyroZ_pid.error = Desire_attitude.gyroZ - PYHdata.gyroZ;
        gyroZ_pid.integ += gyroZ_pid.error;
        gyroZ_pid.diff = gyroZ_pid.error - gyroZ_pid.last_error;
        motor_delta.Dyaw = pidP_list[4].kp*(gyroZ_pid.error + pidP_list[4].ki*gyroZ_pid.integ - pidP_list[4].kd*gyroZ_pid.diff);
        //printf("motor_delta.Dyaw : %f\n", motor_delta.Dyaw);
        //throttle
        //throttle_pid.last_error = throttle_pid.error;
        //throttle_pid.error = Desire_attitude.throttle - PYHdata.accelZ;
        //throttle_pid.integ += throttle_pid.error;
        //throttle_pid.diff = throttle_pid.error - throttle_pid.last_error;
        //motor_delta.throttle = pidP_list[5].kp*(throttle_pid.error + pidP_list[5].ki*throttle_pid.integ + pidP_list[5].kd*throttle_pid.diff);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void clear_I(){
    while (1)
    {
        integ_clr_flag[0] = 1;
        integ_clr_flag[1] = 1;
        integ_clr_flag[2] = 1;
        integ_clr_flag[3] = 1;
        vTaskDelay(200/portTICK_PERIOD_MS);
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
    xTaskCreate(PID_task, "PID_task", 1024*4,  NULL, 1, NULL);
    //xTaskCreate(clear_I, "clear_I", 1024,  NULL, 1, NULL);
}
