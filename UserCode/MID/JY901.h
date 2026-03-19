#ifndef __JY901_H
#define __JY901_H

/*****************JY901S驱动板级设置部分*****************/

#define huart_jy901 huart1                   //JY901S连接的USART句柄
#define hdma_jy901_rx (huart1.hdmarx)        //JY901S连接的USART的DMA接收句柄
/*******************************************************/

#include "main.h"

#define JY_Buffer_Size  256

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

typedef struct
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;

    uint8_t received_byte;
    uint8_t data_ready;
} jy901_uart_t;


extern jy901_uart_t jy901_uart;
extern jy901_raw_t jy901_raw;           // JY901 原始数据
extern jy901_data_t jy901_data;         // JY901 物理量数据
extern uint8_t JY_RxBuffer[JY_Buffer_Size];

void JY901_Init(void);

#endif