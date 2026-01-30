#include "gpio.h"
#include "MyIIC.h"
#include "mpu6050.h"
#include "main.h"

#define MPU6050_ADDRESS		0xD0		//MPU6050的I2C从机地址
#define ACCEL_SENSITIVITY 2048.0f   // 对于 ±16g 量程
#define GYRO_SENSITIVITY 16.4f      // 对于 ±2000°/s 量程

#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"

#define q30  1073741824.0f //用于归一化四元数
#define DEFAULT_MPU_HZ  (200) //DMP采样频率200Hz

mpu6050_raw_t mpu6050_raw;
mpu6050_raw_t *p_mpu6050_raw = &mpu6050_raw;
mpu6050_data_t mpu6050_data;
mpu6050_data_t *p_mpu6050_data = &mpu6050_data;

static void MPU6050_Data_Convert(const mpu6050_raw_t *raw, mpu6050_data_t *out);

short gyro[3], accel[3], sensors; //用于读取DMP_FIFO

static signed char gyro_orientation[9] = { 1, 0, 0,  //方向矩阵
                                           0, 1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row) //用于初始化DMP
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;

}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx) //用于初始化DMP
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;

}

static void run_self_test(void) //用于初始化DMP
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        //printf("setting bias succesfully ......\r\n");
    }

}

// DMP（数字运动处理器）初始化函数
void DMP_Init(void)
{ 
    uint8_t temp[1]={0};
    temp[0] = MPU6050_GetID();          // 读取MPU6050设备ID
    if(temp[0]!=0x68) NVIC_SystemReset(); // 校验设备ID是否为0x68（正常值），错误则系统复位

    // 主初始化流程
    if(!mpu_init()) // 成功返回0，初始化MPU6050底层,与步骤7相呼应
    {
        // 配置传感器使能状态
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {} // 启用陀螺仪和加速度计的三轴数据
        
        // FIFO配置
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {} // 配置FIFO存储陀螺仪和加速度计数据
        
        // 采样率设置
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ)) {} // 设置采样率为默认值（典型值如100Hz）
        
        // DMP固件加载
        if(!dmp_load_motion_driver_firmware()) {} // 加载DMP运动驱动固件
        
        // 方向矩阵配置
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {} // 设置传感器方向校准矩阵
        
        // 启用DMP特性
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |  // 启用6轴低功耗四元数 | 敲击检测
                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |   // Android方向识别 | 原始加速度数据
                DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {}       // 校准后的陀螺仪数据 | 陀螺仪校准
        
        // FIFO速率设置
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ)) {} // 设置DMP输出速率与采样率同步
        
        run_self_test(); // 运行自检程序（校准传感器）
        
        // 启用DMP
        if(!mpu_set_dmp_state(1)) {} // 激活DMP功能（1=启用，0=关闭）
    }

}

uint8_t MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw)
{
    unsigned long sensor_timestamp;
    unsigned char more;
    long quat[4];
    
    // 从DMP FIFO读取四元数数据
    if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
        return 1;

    // 四元数数据解析（q30格式转浮点）
    if (sensors & INV_WXYZ_QUAT) 
    {
        float q0 = quat[0] / q30;
        float q1 = quat[1] / q30;
        float q2 = quat[2] / q30;
        float q3 = quat[3] / q30;

        // 欧拉角计算（单位：度）
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;  // 俯仰角
        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, 
                      -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;  // 横滚角
        *yaw   = atan2(2 * (q1 * q2 + q0 * q3),
                       q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;  // 偏航角
    }
    else {
        return 2;
    }
    
    return 0;
}



/* MPU6050 引脚配置层 */

void MPU6050_W_SCL(GPIO_PinState BitValue)
{
    MyI2C_W_SCL(MPU_IIC_SCL_GPIO_Port, MPU_IIC_SCL_Pin, (uint8_t)BitValue);
}

void MPU6050_W_SDA(GPIO_PinState BitValue)
{
    MyI2C_W_SDA(MPU_IIC_SDA_GPIO_Port, MPU_IIC_SDA_Pin, (uint8_t)BitValue);
}

uint8_t MPU6050_R_SDA(void)
{
    return MyI2C_R_SDA(MPU_IIC_SDA_GPIO_Port, MPU_IIC_SDA_Pin);
}

/* MPU6050 I2C 协议层 */

void MPU6050_I2C_Start(void)
{
    MyI2C_Start(MPU_IIC_SCL_GPIO_Port, MPU_IIC_SCL_Pin,
                MPU_IIC_SDA_GPIO_Port, MPU_IIC_SDA_Pin);
}

void MPU6050_I2C_Stop(void)
{
    MyI2C_Stop(MPU_IIC_SCL_GPIO_Port, MPU_IIC_SCL_Pin,
               MPU_IIC_SDA_GPIO_Port, MPU_IIC_SDA_Pin);
}

void MPU6050_I2C_SendByte(uint8_t Byte)
{
    MyI2C_SendByte(MPU_IIC_SCL_GPIO_Port, MPU_IIC_SCL_Pin,
                   MPU_IIC_SDA_GPIO_Port, MPU_IIC_SDA_Pin,
                   Byte);
}

uint8_t MPU6050_I2C_ReceiveByte(void)
{
    return MyI2C_ReceiveByte(MPU_IIC_SCL_GPIO_Port, MPU_IIC_SCL_Pin,
                             MPU_IIC_SDA_GPIO_Port, MPU_IIC_SDA_Pin);
}

uint8_t MPU6050_I2C_ReceiveAck(void)
{
    return MyI2C_ReceiveAck(MPU_IIC_SCL_GPIO_Port, MPU_IIC_SCL_Pin,
                            MPU_IIC_SDA_GPIO_Port, MPU_IIC_SDA_Pin);
}

void MPU6050_I2C_SendAck(uint8_t AckBit)
{
    MyI2C_SendAck(MPU_IIC_SCL_GPIO_Port, MPU_IIC_SCL_Pin,
                  MPU_IIC_SDA_GPIO_Port, MPU_IIC_SDA_Pin,
                  AckBit);
}

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(MPU6050_ADDRESS);   // 写地址
    MPU6050_I2C_ReceiveAck();

    MPU6050_I2C_SendByte(RegAddress);
    MPU6050_I2C_ReceiveAck();

    MPU6050_I2C_SendByte(Data);
    MPU6050_I2C_ReceiveAck();

    MPU6050_I2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data;

    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(MPU6050_ADDRESS);   // 写地址
    MPU6050_I2C_ReceiveAck();

    MPU6050_I2C_SendByte(RegAddress);
    MPU6050_I2C_ReceiveAck();

    MPU6050_I2C_Start();                     // 重复起始
    MPU6050_I2C_SendByte(MPU6050_ADDRESS | 0x01); // 读地址
    MPU6050_I2C_ReceiveAck();

    Data = MPU6050_I2C_ReceiveByte();
    MPU6050_I2C_SendAck(1);                  // NACK
    MPU6050_I2C_Stop();

    return Data;
}

void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    MPU6050_WriteReg(MPU6050_CONFIG, 0x06);
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}


/**
  * 函    数：MPU6050获取数据
  * 参    数：AccX AccY AccZ 加速度计X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 参    数：GyroX GyroY GyroZ 陀螺仪X、Y、Z轴的数据，使用输出参数的形式返回，范围：-32768~32767
  * 返 回 值：无
  */
void MPU6050_GetData(void)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);		//读取加速度计X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);		//读取加速度计X轴的低8位数据
	p_mpu6050_raw->ax = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);		//读取加速度计Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);		//读取加速度计Y轴的低8位数据
	p_mpu6050_raw->ay = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);		//读取加速度计Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);		//读取加速度计Z轴的低8位数据
	p_mpu6050_raw->az = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);		//读取陀螺仪X轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);		//读取陀螺仪X轴的低8位数据
	p_mpu6050_raw->gx = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);		//读取陀螺仪Y轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);		//读取陀螺仪Y轴的低8位数据
	p_mpu6050_raw->gy = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	
	DataH = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);		//读取陀螺仪Z轴的高8位数据
	DataL = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);		//读取陀螺仪Z轴的低8位数据
	p_mpu6050_raw->gz = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回

    MPU6050_DMP_Get_Data(&(p_mpu6050_raw->pitch), &(p_mpu6050_raw->roll), &(p_mpu6050_raw->yaw));

    MPU6050_Data_Convert(p_mpu6050_raw, p_mpu6050_data);
}

static void MPU6050_Data_Convert(const mpu6050_raw_t *raw, mpu6050_data_t *out)
{
    out->ax = (float)raw->ax / ACCEL_SENSITIVITY;
    out->ay = (float)raw->ay / ACCEL_SENSITIVITY;
    out->az = (float)raw->az / ACCEL_SENSITIVITY;

    out->gx = (float)raw->gx / GYRO_SENSITIVITY;
    out->gy = (float)raw->gy / GYRO_SENSITIVITY;
    out->gz = (float)raw->gz / GYRO_SENSITIVITY;

    out->pitch = raw->pitch;
    out->roll = raw->roll;
    out->yaw = raw->yaw;
}

int MPU6050_dmp_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) //向指定地址写len个字节,用于dmp
{
	int i;
    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(addr << 1);
    MPU6050_I2C_ReceiveAck();
    MPU6050_I2C_SendByte(reg);
    MPU6050_I2C_ReceiveAck();
	for (i = 0; i < len; i++) 
	{
        MPU6050_I2C_SendByte(data[i]);
        MPU6050_I2C_ReceiveAck();
    }
    MPU6050_I2C_Stop();
	return 0;
}

int MPU6050_dmp_Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)// 向指定从机指定地址读len个字节，用于dmp
{
    MPU6050_I2C_Start();
    MPU6050_I2C_SendByte(addr << 1);
    MPU6050_I2C_ReceiveAck();
    MPU6050_I2C_SendByte(reg);
    MPU6050_I2C_ReceiveAck();
    MPU6050_I2C_Start();  
    MPU6050_I2C_SendByte((addr << 1)+1);
    MPU6050_I2C_ReceiveAck();
    while (len) {
        if (len == 1)
		{
            *buf = MPU6050_I2C_ReceiveByte();
			MPU6050_I2C_SendAck(1); 
		}
        else
		{
            *buf = MPU6050_I2C_ReceiveByte();
			MPU6050_I2C_SendAck(0);
		}
        buf++;
        len--;
    }
	MPU6050_I2C_Stop();
	return 0;
}

    

    

