#ifndef _COMMMON_H_
#define _COMMMON_H_

#include <stdio.h>
#include "esp_log.h"

#define PWM_CHANNEL_NUM 8
//imu_param
#define MPU_ADDR 0x68
#define HMC5883_ADDR 0X3C
#define CALIB_TIMES 100

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

typedef struct attitude{
    float gyroX;
    float gyroY;
    float gyroZ;
    float roll;
    float pitch;
    float yaw; 
    float throttle;
}Attitude;

typedef struct motor_delta{
    float throttle;
    float Droll;
    float Dpitch;
    float Dyaw;
    float KT, KR, KP, KY;
}DMotor;

/*
|M1|        |+1 -1 +1 +1| |throttle |       M3      M1
|M2|    __  |+1 +1 -1 +1| |Droll    |          \--/
|M3|    --  |+1 +1 +1 -1| |Dpitch   |          /--\
|M4|        |+1 -1 -1 -1| |Dyaw     |       M2      M4
*/

typedef struct pidparam
{
    float kp;
    float ki;
    float kd;
}PID_Param;

typedef struct pid_element
{
    double error;
    double integ;
    double diff;
    double last_error;
}PID_Element;

#endif