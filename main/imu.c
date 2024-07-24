#include <imu.h>

extern struct AccelGyroData_int32_t OFFSET_RAW;
extern uint8_t RC_CHECK;
extern float G;
extern float PI;
extern float accel_scale;
extern float gyro_scale;
extern int YAW_CAL;

//uint8_t list3[3]={0};

// struct AccelGyroData_t get_raw_GY_87_data16(){
//     struct AccelGyroData_t AccelGyroData;
//     AccelGyroData.accelX = (read_register(MPU_ADDR, 59) << 8) + read_register(MPU_ADDR,60);
//     AccelGyroData.accelY = (read_register(MPU_ADDR,61) << 8) + read_register(MPU_ADDR,62);
//     AccelGyroData.accelZ = (read_register(MPU_ADDR,63) << 8) + read_register(MPU_ADDR,63);
//     AccelGyroData.gyroX = (read_register(MPU_ADDR,67) << 8) + read_register(MPU_ADDR,68);
//     AccelGyroData.gyroY = (read_register(MPU_ADDR,69) << 8) + read_register(MPU_ADDR,70);
//     AccelGyroData.gyroZ = (read_register(MPU_ADDR,71) << 8) + read_register(MPU_ADDR,72);
    
//     AccelGyroData.roll = (read_register(MPU_ADDR, 0x49) << 8) + read_register(MPU_ADDR,0x4A);
//     AccelGyroData.yaw = (read_register(MPU_ADDR,0x4B) << 8) + read_register(MPU_ADDR,0x4C);
//     AccelGyroData.pitch = (read_register(MPU_ADDR,0x4D) << 8) + read_register(MPU_ADDR,0x4E);

//     // list3[0] = read_register(HMC5883_ADDR, 0);
//     // list3[1] = read_register(HMC5883_ADDR, 1); 
//     // list3[2] = read_register(HMC5883_ADDR, 2);
//     // uint8_t dataHMC[6] = {0};
//     // read_register_stream(HMC5883_ADDR, 3, dataHMC, 6);
//     // AccelGyroData.roll = (dataHMC[0]<<8) + dataHMC[1];
//     // AccelGyroData.yaw = (dataHMC[2]<<8) + dataHMC[3];
//     // AccelGyroData.pitch = (dataHMC[4]<<8) + dataHMC[5];
//     return AccelGyroData;
// }

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
    printf("Begin to Caliberate the IMU in 2000ms\n");
    vTaskDelay(2000/portTICK_PERIOD_MS);
    printf("Begin\n");
    for(int i=0; i<CALIB_TIMES; i++)
    {
        struct AccelGyroData_int32_t data = get_raw_mpu6050_data32();
        OFFSET_RAW.accelX += data.accelX;
        OFFSET_RAW.accelY += data.accelY;
        OFFSET_RAW.accelZ += data.accelZ;
        OFFSET_RAW.gyroX += data.gyroX;
        OFFSET_RAW.gyroY += data.gyroY;
        OFFSET_RAW.gyroZ += data.gyroZ;
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    OFFSET_RAW.accelX /= CALIB_TIMES;
    OFFSET_RAW.accelY /= CALIB_TIMES;
    OFFSET_RAW.accelZ /= CALIB_TIMES;
    OFFSET_RAW.gyroX /= CALIB_TIMES;
    OFFSET_RAW.gyroY /= CALIB_TIMES;
    OFFSET_RAW.gyroZ /= CALIB_TIMES;
    printf("%d\n", OFFSET_RAW.accelZ);
    OFFSET_RAW.accelZ = OFFSET_RAW.accelZ - 16384;
    printf("%d %d %d\n%d %d% d\n", OFFSET_RAW.accelX, OFFSET_RAW.accelY, OFFSET_RAW.accelZ, OFFSET_RAW.gyroX, OFFSET_RAW.gyroY, OFFSET_RAW.gyroZ);
    printf("OFFSET COLLECTED\n");
}
// 410 274 1305
// -309 -73-3

// -39 -151 182
// -497 139 90

// -39 -167 190
// -499 139 90


struct AccelGyroData_int32_t get_calibration_data()
{
    struct AccelGyroData_int32_t AccelGyroData;
    AccelGyroData.accelX = (int)(int16_t)(read_register(MPU_ADDR, 59) << 8) + read_register(MPU_ADDR, 60) - OFFSET_RAW.accelX;
    AccelGyroData.accelY = (int)(int16_t)(read_register(MPU_ADDR, 61) << 8) + read_register(MPU_ADDR, 62) - OFFSET_RAW.accelY;
    AccelGyroData.accelZ = (int)(int16_t)(read_register(MPU_ADDR, 63) << 8) + read_register(MPU_ADDR, 63) - OFFSET_RAW.accelZ;
    AccelGyroData.gyroX = (int)(int16_t)(read_register(MPU_ADDR, 67) << 8) + read_register(MPU_ADDR, 68) - OFFSET_RAW.gyroX;
    AccelGyroData.gyroY = (int)(int16_t)(read_register(MPU_ADDR, 69) << 8) + read_register(MPU_ADDR, 70) - OFFSET_RAW.gyroY;
    AccelGyroData.gyroZ = (int)(int16_t)(read_register(MPU_ADDR, 71) << 8) + read_register(MPU_ADDR, 72) - OFFSET_RAW.gyroZ;
    
    // printf("accelX %d, accelY %d, accelZ %d ", AccelGyroData.accelX, AccelGyroData.accelY, AccelGyroData.accelZ);
    // printf("8553 %d %d\n", read_register(0x3d, 0x3), read_register(0x3d, 0x4));
    return AccelGyroData;
}

struct AccelGyroPHYSICSData get_PHYSICS_Data()
{
    static int filter_init_flag = 1;
    static double kfL = 0.98;
    static struct AccelGyroPHYSICSData lastdata;
    struct AccelGyroPHYSICSData phydataIN1,phydata;
    struct AccelGyroData_int32_t rawData = get_calibration_data();
    phydataIN1.accelX = 1.0 * rawData.accelX / accel_scale * G;
    phydataIN1.accelY = 1.0 * rawData.accelY / accel_scale * G;
    phydataIN1.accelZ = 1.0 * rawData.accelZ / accel_scale * G;
    phydataIN1.gyroX = 1.0 * rawData.gyroX / gyro_scale;
    phydataIN1.gyroY = 1.0 * rawData.gyroY / gyro_scale;
    phydataIN1.gyroZ = 1.0 * rawData.gyroZ / gyro_scale;
    if(filter_init_flag)
    {
        filter_init_flag = 0;
        lastdata.accelX = phydataIN1.accelX;
        lastdata.accelY = phydataIN1.accelY;
        lastdata.accelZ = phydataIN1.accelZ;
        lastdata.gyroX = phydataIN1.gyroX;
        lastdata.gyroY = phydataIN1.gyroY;
        lastdata.gyroZ = phydataIN1.gyroZ;
    }
    phydata.accelX = lastdata.accelX + kfL*(phydataIN1.accelX - lastdata.accelX);
    phydata.accelY = lastdata.accelY + kfL*(phydataIN1.accelY - lastdata.accelY);
    phydata.accelZ = lastdata.accelZ + kfL*(phydataIN1.accelZ - lastdata.accelZ);
    phydata.gyroX = lastdata.gyroX + kfL*(phydataIN1.gyroX - lastdata.gyroX);
    phydata.gyroY = lastdata.gyroY + kfL*(phydataIN1.gyroY - lastdata.gyroY);
    phydata.gyroZ = lastdata.gyroZ + kfL*(phydataIN1.gyroZ - lastdata.gyroZ);

    lastdata.accelX = phydata.accelX;
    lastdata.accelY = phydata.accelY;
    lastdata.accelZ = phydata.accelZ;
    lastdata.gyroX = phydata.gyroX;
    lastdata.gyroY = phydata.gyroY;
    lastdata.gyroZ = phydata.gyroZ;

    
    //phydata.roll = asin(phydata.accelY/G);
    //phydata.pitch = asin(phydata.accelX/G);
    // phydata.yaw = 1.0 * YAW_CAL / gyro_scale / 1000 * Tick;
    //printf("accelX %f, accelY %f, accelZ %f\n", phydata.accelX, phydata.accelY, phydata.accelZ);
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