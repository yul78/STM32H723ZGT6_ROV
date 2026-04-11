#ifndef __JY901_H
#define __JY901_H

#include "main.h"
#include "bsp_jy901.h"

void JY901_Task(void);

/* JY901 原始数据结构 */
typedef struct 
{
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t roll;
    int16_t pitch;
    int16_t yaw;

}jy901_raw_t;

/* JY901 物理数据结构 */
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

} jy901_data_t;


extern jy901_raw_t jy901_raw;           // JY901 原始数据
extern jy901_data_t jy901_data;         // JY901 物理量数据


#endif