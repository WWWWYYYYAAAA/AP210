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
    // printf("HMC8553l %d\n", (int)(int16_t)((read_register(MPU_ADDR, 0x49)<<8)+read_register(MPU_ADDR, 0x4A)));
    return AccelGyroData;
}

struct AccelGyroPHYSICSData get_PHYSICS_Data()
{
    static int filter_init_flag = 1;
    static float kfL = 0.98;
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

struct AccelGyroPHYSICSData get_PHYSICS_Data_with_Kalman(float dt)
{
    static int filter_init_flag = 1;
    struct AccelGyroData_int32_t AccelGyroData;
    AccelGyroData.accelX = (int)(int16_t)(read_register(MPU_ADDR, 59) << 8) + read_register(MPU_ADDR, 60) - OFFSET_RAW.accelX;
    AccelGyroData.accelY = (int)(int16_t)(read_register(MPU_ADDR, 61) << 8) + read_register(MPU_ADDR, 62) - OFFSET_RAW.accelY;
    AccelGyroData.accelZ = (int)(int16_t)(read_register(MPU_ADDR, 63) << 8) + read_register(MPU_ADDR, 63) - OFFSET_RAW.accelZ;
    AccelGyroData.gyroX = (int)(int16_t)(read_register(MPU_ADDR, 67) << 8) + read_register(MPU_ADDR, 68) - OFFSET_RAW.gyroX;
    AccelGyroData.gyroY = (int)(int16_t)(read_register(MPU_ADDR, 69) << 8) + read_register(MPU_ADDR, 70) - OFFSET_RAW.gyroY;
    AccelGyroData.gyroZ = (int)(int16_t)(read_register(MPU_ADDR, 71) << 8) + read_register(MPU_ADDR, 72) - OFFSET_RAW.gyroZ;


    struct AccelGyroPHYSICSData phydataIN;
    static struct AccelGyroPHYSICSData phydataP;
    phydataIN.accelX = 1.0 * AccelGyroData.accelX / accel_scale * G;
    phydataIN.accelY = 1.0 * AccelGyroData.accelY / accel_scale * G;
    phydataIN.accelZ = 1.0 * AccelGyroData.accelZ / accel_scale * G;
    phydataIN.gyroX = 1.0 * AccelGyroData.gyroX / gyro_scale;
    phydataIN.gyroY = 1.0 * AccelGyroData.gyroY / gyro_scale;
    phydataIN.gyroZ = 1.0 * AccelGyroData.gyroZ / gyro_scale;

    
    if(phydataIN.accelY/G>=1)
        phydataIN.roll = asin(1);
    else if(phydataIN.accelY/G <= -1)
        phydataIN.roll = asin(-1);
    else if(phydataIN.accelZ >= 0)
        phydataIN.roll = asin(phydataIN.accelY/G);
    else if(phydataIN.accelZ < 0 && phydataIN.accelY < 0)
        phydataIN.roll = -asin(phydataIN.accelY/G) - PI;
    else
        phydataIN.roll = -asin(phydataIN.accelY/G) + PI;
    
    if(phydataIN.accelX/G>=1)
        phydataIN.pitch = asin(1);
    else if(phydataIN.accelX/G <= -1)
        phydataIN.pitch = asin(-1);
    else if(phydataIN.accelZ >= 0)
        phydataIN.pitch = asin(phydataIN.accelX/G);
    else if(phydataIN.accelZ < 0 && phydataIN.accelX < 0)
        phydataIN.pitch = -asin(phydataIN.accelX/G) - PI;
    else
        phydataIN.pitch = -asin(phydataIN.accelX/G) + PI;

    if(filter_init_flag)
    {
        filter_init_flag = 0;
        phydataP.roll = phydataIN.roll;
        phydataP.pitch = phydataIN.roll;
        phydataP.yaw = 0;
    }
    
    phydataP.accelX = phydataIN.accelX;
    phydataP.accelY = phydataIN.accelY;
    phydataP.accelZ = phydataIN.accelZ;
    phydataP.gyroX = phydataIN.gyroX;
    phydataP.gyroY = phydataIN.gyroY;
    phydataP.gyroZ = phydataIN.gyroZ;
    phydataP.yaw = 0;

    //roll
    static float Q_roll = 0.001;		//角度数据置信度，角度噪声的协方差
    static float Q_gyroX  = 0.003;		//角速度数据置信度，角速度噪声的协方差  
    static float R_roll = 0.5;			//加速度计测量噪声的协方差
    static float Q_gyroX_bias = 0;				//Q_bias:陀螺仪的偏差
    static float K_0_roll, K_1_roll;				//卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差
    static float PP_roll[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P，初始值为单位阵
    phydataP.roll += (phydataIN.gyroX - Q_gyroX_bias) * dt;
    PP_roll[0][0] = PP_roll[0][0] + Q_roll - (PP_roll[0][1] + PP_roll[1][0])*dt;
	PP_roll[0][1] = PP_roll[0][1] - PP_roll[1][1]*dt;
	PP_roll[1][0] = PP_roll[1][0] - PP_roll[1][1]*dt;
	PP_roll[1][1] = PP_roll[1][1] + Q_gyroX;
    K_0_roll = PP_roll[0][0] / (PP_roll[0][0] + R_roll);
	K_1_roll = PP_roll[1][0] / (PP_roll[0][0] + R_roll);
    //out
	phydataP.roll = phydataP.roll + K_0_roll * (phydataIN.roll - phydataP.roll);
    //update
    Q_gyroX_bias = Q_gyroX_bias + K_1_roll * (phydataIN.roll - phydataP.roll);
	PP_roll[0][0] = PP_roll[0][0] - K_0_roll * PP_roll[0][0];
	PP_roll[0][1] = PP_roll[0][1] - K_0_roll * PP_roll[0][1];
	PP_roll[1][0] = PP_roll[1][0] - K_1_roll * PP_roll[0][0];
	PP_roll[1][1] = PP_roll[1][1] - K_1_roll * PP_roll[0][1];

    //pitch
    static float Q_pitch = 0.001;		//角度数据置信度，角度噪声的协方差
    static float Q_gyroY  = 0.003;		//角速度数据置信度，角速度噪声的协方差  
    static float R_pitch = 0.5;			//加速度计测量噪声的协方差
    static float Q_gyroY_bias = 0;				//Q_bias:陀螺仪的偏差
    static float K_0_pitch, K_1_pitch;				//卡尔曼增益  K_0:用于计算最优估计值  K_1:用于计算最优估计值的偏差
    static float PP_pitch[2][2] = { { 1, 0 },{ 0, 1 } };//过程协方差矩阵P，初始值为单位阵
    phydataP.pitch += (phydataIN.gyroX - Q_gyroY_bias) * dt;
    PP_pitch[0][0] = PP_pitch[0][0] + Q_pitch - (PP_pitch[0][1] + PP_pitch[1][0])*dt;
	PP_pitch[0][1] = PP_pitch[0][1] - PP_pitch[1][1]*dt;
	PP_pitch[1][0] = PP_pitch[1][0] - PP_pitch[1][1]*dt;
	PP_pitch[1][1] = PP_pitch[1][1] + Q_gyroY;
    K_0_pitch = PP_pitch[0][0] / (PP_pitch[0][0] + R_pitch);
	K_1_pitch = PP_pitch[1][0] / (PP_pitch[0][0] + R_pitch);
    //out
	phydataP.pitch = phydataP.pitch + K_0_pitch * (phydataIN.pitch - phydataP.pitch);
    //update
    Q_gyroY_bias = Q_gyroY_bias + K_1_pitch * (phydataIN.pitch - phydataP.pitch);
	PP_pitch[0][0] = PP_pitch[0][0] - K_0_pitch * PP_pitch[0][0];
	PP_pitch[0][1] = PP_pitch[0][1] - K_0_pitch * PP_pitch[0][1];
	PP_pitch[1][0] = PP_pitch[1][0] - K_1_pitch * PP_pitch[0][0];
	PP_pitch[1][1] = PP_pitch[1][1] - K_1_pitch * PP_pitch[0][1];

    return phydataP;
}