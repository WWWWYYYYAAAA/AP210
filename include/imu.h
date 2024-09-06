#ifndef _IMU_H_
#define _IMU_H_

#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2cRW.h"
#include <math.h>
#include "common.h"


struct AccelGyroData_int32_t get_raw_mpu6050_data32();
void MPU_OFFSET();
struct AccelGyroData_int32_t get_calibration_data();
struct AccelGyroPHYSICSData get_PHYSICS_Data();
struct AccelGyroPHYSICSData get_PHYSICS_Data_with_Kalman(float dt);
#endif