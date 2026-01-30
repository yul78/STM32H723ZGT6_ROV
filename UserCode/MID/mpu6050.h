#ifndef __MPU6050_H
#define __MPU6050_H

#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75

/* MPU6050 原始数据结构 */
typedef struct 
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    float roll;
    float pitch;
    float yaw;

}mpu6050_raw_t;

/* MPU6050 物理数据结构 */
typedef struct
{
    float ax;     // X轴加速度（单位：g）
    float ay;     // Y轴加速度（单位：g）
    float az;     // Z轴加速度（单位：g）

    float gx;     // X轴角速度（单位：度每秒）
    float gy;     // Y轴角速度（单位：度每秒）
    float gz;     // Z轴角速度（单位：度每秒）

    float roll;   // 欧拉角（单位：度）
    float pitch;  // 欧拉角（单位：度）
    float yaw;    // 欧拉角（单位：度）

} mpu6050_data_t;

extern mpu6050_raw_t mpu6050_raw;
extern mpu6050_data_t mpu6050_data;
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(void);
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
int MPU6050_dmp_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int MPU6050_dmp_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
void DMP_Init(void);
uint8_t MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw);

#endif
