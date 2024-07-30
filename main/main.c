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
#include <dirent.h>
#include "fatfs.h"
#include "jsmn.h"

// #include "esp_vfs.h"
// #include "esp_vfs_fat.h"
// #include "esp_system.h"


SBUS RC_DATA = {0};
struct AccelGyroData_int32_t OFFSET_RAW = {0, 0, 0,-499 ,139 ,90};//{-39, -167, 190,-499 ,139 ,90};
uint8_t RC_CHECK = 63;
double G = 9.80665;
double PI = 3.1415926535;
double accel_scale=16384;
double gyro_scale=7509.872412338726;
int YAW_CAL = 0;
struct AccelGyroPHYSICSData PYHdata;

static Attitude Last_attitude = {0};
static Attitude Now_attitude = {0};
static Attitude Desire_attitude = {0};
static PID_Param pidP_list[6] = {{0.2, 0.0, 1},  //roll
                          {0.02, 0.8, 0},  //gx
                          {0.2, 0.0, 1},  //pitch
                          {0.02, 0.8, 0},  //gy
                          {0.02, 0, 0},   //gyroZ
                          {05., 0, 0}};  //throttle  --x
int integ_clr_flag[6] = {0};
PID_Element roll_pid = {0};
PID_Element gyroX_pid = {0};
PID_Element pitch_pid = {0};
PID_Element gyroY_pid = {0};
PID_Element gyroZ_pid = {0};
PID_Element throttle_pid = {0};
DMotor motor_delta = {0, 0, 0, 0, 1, 1, 1, 1};   //ROLL PITCH YAW

double hover_percentage = 0.3;
uint8_t sw1=0, sw2=0;

char log_path[32] = {0};
//unsigned long run_time = 0;

void HW_init();
uint8_t mode_switch();


static void get_data_task(void *arg);
static void control_task(void *arg);
static void save_data_task(void *arg);

void get_mode_task(void *arg)
{
    
    while (1)
    {
        sw1 = gpio_get_level(2);
        sw2 = gpio_get_level(1);
        printf("sw1 %d sw2 %d\n", sw1, sw2);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    
    
}

int8_t creat_file_consv(char * path)
{
    if(!IsExist(path))
    {
        FILE* fp;
        printf("CREATING MAIN PARAM\n");
        fp = fopen(path, "w");
        if(fp != NULL)
        {
            fclose(fp);
        }
        else{
            printf("cannot open\n");
            return -1;
        }
    }
    return 0;
}



void init_file()
{
    char main_param[32] = "/data/main_param.TXT";
    int times = 0;
    FILE* fmain;
    char paramstr[100];
    if(!IsExist(main_param))
    {
        printf("CREATING MAIN PARAM\n");
        fmain = fopen(main_param, "w");
        if(fmain != NULL)
        {
            fprintf(fmain, "%5d\n", 0);
            fclose(fmain);
        }
        else{
            printf("cannot open\n");
        }
    }
    fmain = fopen(main_param, "r");
    if (fmain == NULL) {
        printf("FMAIN OPEN FAILED<READ>\n");
    }
    else{
        fgets(paramstr, sizeof(paramstr) ,fmain);
        char power_times[6] = {paramstr[0], paramstr[1], paramstr[2], paramstr[3], paramstr[4], 0};
        times = atoi(power_times) + 1;
        fclose(fmain);
        }

    fmain = fopen(main_param, "w");
    if (fmain == NULL) {
        printf("FMAIN OPEN FAILED<WRITE>\n");
    }
    else
    {
        fprintf(fmain,"%5d\n", times);
        printf("POWER TIMES:%5d\n", times);
        fclose(fmain);
    }

    
    sprintf(log_path, "/data/log_0000.csv");
    FILE * fplog = fopen(log_path, "w");
    fprintf(fplog, "time, roll, pitch, gyroZ,\n");
    fclose(fplog);
}

void clear_file()
{
    char main_param[50] = "/data/main_param.txt";
    int times = 0;
    FILE* fmain;
    char paramstr[100];
    printf("CLEAR MAIN PARAM\n");
    fmain = fopen(main_param, "w");
    fprintf(fmain, "%5d\n", 0);
    fclose(fmain);

    fmain = fopen(main_param, "r");
    if (fmain == NULL) {
        printf("FMAIN OPEN FAILED<READ>\n");
    }
    else{
        fgets(paramstr, sizeof(paramstr) ,fmain);
        char power_times[6] = {paramstr[0], paramstr[1], paramstr[2], paramstr[3], paramstr[4], 0};
        times = atoi(power_times) + 1;
        fclose(fmain);
        }

    fmain = fopen(main_param, "w");
    if (fmain == NULL) {
        printf("FMAIN OPEN FAILED<WRITE>\n");
    }
    else
    {
        fprintf(fmain,"%5d\n", times);
        printf("POWER TIMES:%5d\n", times);
        fclose(fmain);
    }
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start && 
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) 
    {
        return 0;
    }
  return -1;
}

void test_json()
{
    char json_contents[128] = {0};
    FILE* fjson = fopen("/data/testjson.json", "r");
    if(fjson != NULL)
    {
        fgets(json_contents, sizeof(json_contents),fjson);
        int r, i;
        jsmn_parser p;
        jsmntok_t t[128]; /* We expect no more than 128 tokens */
        jsmn_init(&p);
        r = jsmn_parse(&p, json_contents, strlen(json_contents), t, sizeof(t) / sizeof(t[0]));
        for (i = 1; i < r; i++) {
        if (jsoneq(json_contents, &t[i], "NAME") == 0) {
        /* We may use strndup() to fetch string value */
        printf("- NAME: %.*s\n", t[i + 1].end - t[i + 1].start,
                json_contents + t[i + 1].start);
        i++;
        } 
        else if (jsoneq(json_contents, &t[i], "TIMES") == 0) {
            /* We may additionally check if the value is either "true" or "false" */
            printf("- TIMES: %.*s\n", t[i + 1].end - t[i + 1].start,
                    json_contents + t[i + 1].start);
            i++;
        } 
        else if (jsoneq(json_contents, &t[i], "KD") == 0) {
            /* We may want to do strtol() here to get numeric value */
            printf("- KD: %.*s\n", t[i + 1].end - t[i + 1].start,
                    json_contents + t[i + 1].start);
            i++;
        } 
        else if (jsoneq(json_contents, &t[i], "PID") == 0) {
            int j;
            printf("- PID:\n");
            if (t[i + 1].type != JSMN_ARRAY) {
                continue; /* We expect groups to be an array of strings */
            }
            for (j = 0; j < t[i + 1].size; j++) {
                jsmntok_t *g = &t[i + j + 2];
                printf("  * %.*s\n", g->end - g->start, json_contents + g->start);
            }
            i += t[i + 1].size + 1;
        } 
        else {
            printf("Unexpected key: %.*s\n", t[i].end - t[i].start, json_contents + t[i].start);
        }
        }
    }
    else{
        printf("Read json failed!\n");
    }
}


static void save_data_task(void *arg)
{
    while (1)
    {
        int ms_clock = clock();
        FILE * fplog = fopen(log_path, "a");
        fprintf(fplog, "%d, %f, %f, %f,\n",ms_clock, PYHdata.roll, PYHdata.pitch, PYHdata.gyroZ);
        fclose(fplog);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    vTaskDelay(200/portTICK_PERIOD_MS);
    printf("INITIALIZING...\n");
    //user_partition_init();
    HW_init();
    mode_switch();
}

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
    
    uint8_t MST_EN1[] = {106, 0x00};
    uint8_t MST_EN2[] = {106, 0x20};
    uint8_t BYPASS_EN1[] = {0x37, 0x02};
    uint8_t BYPASS_EN2[] = {0x37, 0x00};
    uint8_t I2C_MST_CTRL[] = {36, 0b00001101};
    uint8_t I2C_SLV0_ADDR[] = {0x25, 0x80|0x1E};
    uint8_t I2C_SLV0_ADDR_W[] = {0x25, 0x1E};
    uint8_t I2C_SLV0_REG[] = {0x26, 0x03};
    uint8_t I2C_SLV0_CTRL[] = {0x27, 0b10000110};
    uint8_t DELAY_CTRL[] = {0x67, 0b00000001};
    //hmc5883l
    // write_register(MPU_ADDR, MST_EN1);
    // vTaskDelay(100/portTICK_PERIOD_MS);
    // write_register(MPU_ADDR, BYPASS_EN1);
    // printf("confa %d\n", read_register(0x1E, 0x00));
    // printf("confb %d\n", read_register(0x1E, 0x01));
    // printf("confc %d\n", read_register(0x1E, 0x04));
    // uint8_t a_config = {0x00, 0X58};
    // uint8_t b_config = {0x01, 0X40};
    // uint8_t mode_config = {0x02, 0X00};
    // write_register(0x1E, a_config);
    // write_register(0x1E, b_config);
    // write_register(0x1E, mode_config);
    uint8_t confa[] = {99, 0x00, 0X18};
    uint8_t confb[] = {99, 0x01, 0X60};
    uint8_t confc[] = {99, 0x02, 0X00};
    //mpu6050
    write_register(MPU_ADDR, MST_EN2);
    vTaskDelay(100/portTICK_PERIOD_MS);
    write_register(MPU_ADDR, BYPASS_EN2);
    write_register(MPU_ADDR, I2C_MST_CTRL);


    write_register(MPU_ADDR, I2C_SLV0_ADDR_W);
    vTaskDelay(5/portTICK_PERIOD_MS);
    // write_register(MPU_ADDR, confa);
    // vTaskDelay(10/portTICK_PERIOD_MS);
    // write_register(MPU_ADDR, confb);
    // vTaskDelay(10/portTICK_PERIOD_MS);
    // write_register(MPU_ADDR, confc);
    // vTaskDelay(10/portTICK_PERIOD_MS);
    
    write_register(MPU_ADDR, I2C_SLV0_ADDR);
    write_register(MPU_ADDR, I2C_SLV0_REG);
    write_register(MPU_ADDR, I2C_SLV0_CTRL);
    write_register(MPU_ADDR, DELAY_CTRL);

    write_register(MPU_ADDR, PWR_MGMT_1);
    write_register(MPU_ADDR, PWR_MGMT_2);
    write_register(MPU_ADDR, CONFIG);
    write_register(MPU_ADDR, GYRO_CONFIG);
    write_register(MPU_ADDR, ACCEL_CONFIG);
    //hmc5883
    // write_register(MPU_ADDR, MST_EN1);
    // write_register(MPU_ADDR, BYPASS_EN1);
    // printf("BYPASS_EN %d\n", read_register(MPU_ADDR, 0x37));
    // printf("MST_EN %d\n", read_register(MPU_ADDR, 106));



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
    // MPU_OFFSET();
    gpio_config_t ioConfig = {
		.pin_bit_mask = (1ull << 1)|(1ull << 2),
		.mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
	};
    gpio_config(&ioConfig);
    // MPU_OFFSET();
}

uint8_t mode_switch()
{
    sw1 = gpio_get_level(2);
    sw2 = gpio_get_level(1);
    if(!sw1 && !sw2)
    {
        FAT_regular_init();
        init_file();
        //test_json();
        printf("INITIALIZATION COMPLETED\n");
        vTaskDelay(500/portTICK_PERIOD_MS);
        // xTaskCreate(rx_task, "uart_rx_task", 1024*4, NULL, 1, NULL);
        // xTaskCreate(motor_out_task, "motor_out_task", 1024*4, NULL, 1, NULL);
        // xTaskCreate(get_imu_task, "get_imu_task", 1024*4, NULL, 8, NULL);
        // xTaskCreate(PID_task, "PID_task", 1024*4,  NULL, 1, NULL);
        // xTaskCreate(save_data_task, "save_data_task", 1024*4,  NULL, 4, NULL);
        // flight_task(NULL);
        xTaskCreate(control_task, "control_task", 1024*16,  NULL, 1, NULL);
        xTaskCreate(get_data_task, "get_data_task", 1024*8,  NULL, 1, NULL);
        
    }
    else if(sw1 && !sw2)
    {
        init_myfatfs();
    }
    else if(!sw1 && sw2)
    {
        init_myfatfs_sdmmc();
    }
    else if(sw1 && sw2)
    {
        FAT_format_init();
        FAT_unmount();
        vTaskDelay(1000/portTICK_PERIOD_MS);
        init_myfatfs();
    }
    return 0;
}

static void get_data_task(void *arg)
{
    //rx
    uint8_t data[64] = {0};
    uint16_t byte_list[24] = {0};
    int i;
    //imu
    
    while (1)
    {
        //rx
        // int length = 0;
        // ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_2, (size_t*)&length));
        uart_read_bytes(UART_NUM_2, data, 50, 5);
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
        // printf("\n");s
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    return;
}

double platform(int data_raw, int mid, int width, double kout)
{
    if(data_raw - mid + width < 0)
    {
        return kout*(data_raw - mid + width);
    }
    else if(data_raw - mid - width > 0)
    {
        return kout*(data_raw - mid - width);
    }
    else
    {
        return 0;
    }
    
}

static void control_task(void *arg)
{
    //motor_out
    MOTOR  MOUT= {410, 410, 410, 410};
    u_int8_t flag_start_check = 1;
    PYHdata = get_PHYSICS_Data();
    double motor_percent[4];
    while (1)
    {
        PYHdata = get_PHYSICS_Data();

        //motor_out
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
                Desire_attitude.roll = platform(RC_DATA.CH1, 1000, 40, PI/3200);
                Desire_attitude.pitch = -platform(RC_DATA.CH2, 1000, 40, PI/3200);
                Desire_attitude.gyroZ = -platform(RC_DATA.CH4, 1000, 40, PI/1600);
                Desire_attitude.throttle = 0.9 * (RC_DATA.CH3-200)/1600 + 0.1;
                //rolls
                roll_pid.last_error = roll_pid.error;
                roll_pid.error = Desire_attitude.roll - PYHdata.roll;
                roll_pid.integ += roll_pid.error;
                if(roll_pid.error < 0.2 && roll_pid.error > -0.2)
                {
                    roll_pid.integ = 0;
                }
                if(pidP_list[0].kp*pidP_list[0].ki*roll_pid.integ>0.5)
                {
                    gyroZ_pid.integ = 0.5/(pidP_list[0].kp*pidP_list[0].ki);
                }
                roll_pid.diff = (roll_pid.error - roll_pid.last_error);
                //Desire_attitude.gyroX 
                motor_delta.Droll = pidP_list[0].kp*(roll_pid.error + pidP_list[0].ki*roll_pid.integ + pidP_list[0].kd*roll_pid.diff);
                //printf("Desire_attitude.roll: %3.5f PYHdata.roll: %3.5f roll_pid.error: %3.5f\n", Desire_attitude.roll, PYHdata.roll, roll_pid.error);
                //printf(" Desire_attitude.gyroX: %f\n",  Desire_attitude.gyroX);
                //gyroX
                
                // gyroX_pid.last_error = gyroX_pid.error;
                // gyroX_pid.error = Desire_attitude.gyroX - PYHdata.gyroX;
                // gyroX_pid.integ = roll_pid.error;  
                // gyroX_pid.diff = gyroX_pid.error - gyroX_pid.last_error;
                // motor_delta.Droll = pidP_list[1].kp*(gyroX_pid.error + pidP_list[1].ki*gyroX_pid.integ + pidP_list[1].kd*gyroX_pid.diff);
                //printf("motor_delta.Droll : %f ", motor_delta.Droll);
                //pitch
                
                pitch_pid.last_error = pitch_pid.error;
                pitch_pid.error = Desire_attitude.pitch - PYHdata.pitch;
                pitch_pid.integ += pitch_pid.error;
                if(pitch_pid.error < 0.2 && pitch_pid.error > -0.2)
                {
                    pitch_pid.integ = 0;
                }
                if(pidP_list[2].kp*pidP_list[2].ki*pitch_pid.integ>0.5)
                {
                    pitch_pid.integ = 0.5/(pidP_list[2].kp*pidP_list[2].ki);
                }
                pitch_pid.diff = (pitch_pid.error - pitch_pid.last_error);
                //Desire_attitude.gyroY 
                motor_delta.Dpitch = pidP_list[2].kp*(pitch_pid.error + pidP_list[2].ki*pitch_pid.integ + pidP_list[2].kd*pitch_pid.diff);
                //gyroY
                // gyroY_pid.last_error = gyroY_pid.error;
                // gyroY_pid.error = Desire_attitude.gyroY - PYHdata.gyroY;
                // gyroY_pid.integ = pitch_pid.error;
                // gyroY_pid.diff = gyroY_pid.error - gyroY_pid.last_error;
                // motor_delta.Dpitch = pidP_list[3].kp*(gyroY_pid.error + pidP_list[3].ki*gyroY_pid.integ + pidP_list[3].kd*gyroY_pid.diff);
                //printf("motor_delta.Dpitch : %f\n", motor_delta.Dpitch);
                //gyroZ

                gyroZ_pid.last_error = gyroZ_pid.error;
                gyroZ_pid.error = Desire_attitude.gyroZ - PYHdata.gyroZ;
                gyroZ_pid.integ += gyroZ_pid.error;
                if(gyroZ_pid.error > 0.8 || gyroZ_pid.error < -0.8)
                {
                    gyroZ_pid.integ = 0;
                }
                if(gyroZ_pid.integ>5)
                {
                    gyroZ_pid.integ = 5;
                }
                gyroZ_pid.diff = gyroZ_pid.error - gyroZ_pid.last_error;
                motor_delta.Dyaw = pidP_list[4].kp*(gyroZ_pid.error + pidP_list[4].ki*gyroZ_pid.integ + pidP_list[4].kd*gyroZ_pid.diff);
                //printf("motor_delta.Dyaw : %f\n", motor_delta.Dyaw);
                //throttle
                // throttle_pid.last_error = throttle_pid.error;
                // throttle_pid.error = Desire_attitude.throttle - PYHdata.accelZ;
                // throttle_pid.integ += throttle_pid.error;
                // if(throttle_pid.error > 0.5 || throttle_pid.error < -0.5)
                // {
                //     throttle_pid.integ = 0;
                // }
                // throttle_pid.diff = throttle_pid.error - throttle_pid.last_error;
                // motor_delta.throttle = pidP_list[5].kp*(throttle_pid.error + pidP_list[5].ki*throttle_pid.integ + pidP_list[5].kd*throttle_pid.diff);
                motor_delta.throttle = Desire_attitude.throttle;
                ///---------------------///
                motor_percent[0] = motor_delta.KT*motor_delta.throttle - motor_delta.KR*motor_delta.Droll + motor_delta.KP*motor_delta.Dpitch - motor_delta.KY*motor_delta.Dyaw;
                motor_percent[1] = motor_delta.KT*motor_delta.throttle + motor_delta.KR*motor_delta.Droll - motor_delta.KP*motor_delta.Dpitch - motor_delta.KY*motor_delta.Dyaw;
                motor_percent[2] = motor_delta.KT*motor_delta.throttle + motor_delta.KR*motor_delta.Droll + motor_delta.KP*motor_delta.Dpitch + motor_delta.KY*motor_delta.Dyaw;
                motor_percent[3] = motor_delta.KT*motor_delta.throttle - motor_delta.KR*motor_delta.Droll - motor_delta.KP*motor_delta.Dpitch + motor_delta.KY*motor_delta.Dyaw;
                MOUT.motor_1 = (motor_delta.KT*motor_delta.throttle - motor_delta.KR*motor_delta.Droll + motor_delta.KP*motor_delta.Dpitch - motor_delta.KY*motor_delta.Dyaw)*410+410;
                MOUT.motor_2 = (motor_delta.KT*motor_delta.throttle + motor_delta.KR*motor_delta.Droll - motor_delta.KP*motor_delta.Dpitch - motor_delta.KY*motor_delta.Dyaw)*410+410;
                MOUT.motor_3 = (motor_delta.KT*motor_delta.throttle + motor_delta.KR*motor_delta.Droll + motor_delta.KP*motor_delta.Dpitch + motor_delta.KY*motor_delta.Dyaw)*410+410;
                MOUT.motor_4 = (motor_delta.KT*motor_delta.throttle - motor_delta.KR*motor_delta.Droll - motor_delta.KP*motor_delta.Dpitch + motor_delta.KY*motor_delta.Dyaw)*410+410;
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
                //printf("%d %d %d %d\n", MOUT.motor_1, MOUT.motor_2, MOUT.motor_3, MOUT.motor_4);
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
        int ms_clock = clock();
        // printf("%d, %f--%f, %f--%f, %f--%f\n",ms_clock, PYHdata.gyroX, Desire_attitude.roll, PYHdata.gyroY, Desire_attitude.pitch, PYHdata.gyroZ, Desire_attitude.gyroZ);
        // printf("%d, %f, %f, %f\n",ms_clock, PYHdata.roll, PYHdata.pitch, PYHdata.gyroZ);
        printf("%d, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n",ms_clock, motor_percent[0], motor_percent[1], motor_percent[2], motor_percent[3], motor_delta.Droll, motor_delta.Dpitch, motor_delta.Dyaw);
        // printf("%8d, %8f, %8f, %8f, %8f, %8f, %8f\n",ms_clock, Desire_attitude.gyroX, Desire_attitude.gyroY, Desire_attitude.gyroZ, motor_delta.Droll, motor_delta.Dpitch, motor_delta.Dyaw);
        // printf("%d, %f, %f, %f, %f\n", ms_clock, motor_percent[0], motor_percent[1], motor_percent[2], motor_percent[3]);
        // printf("%d, %d, %d, %d, %d\n",ms_clock, MOUT.motor_1, MOUT.motor_2, MOUT.motor_3, MOUT.motor_4);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}
