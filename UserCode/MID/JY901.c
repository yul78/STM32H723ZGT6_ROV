#include "JY901.h"
#include <string.h>
#include "usart.h"
#include "dma.h"

jy901_uart_t jy901_uart;

jy901_raw_t jy901_raw;             //使用jy901_raw_t类型定义jy901_raw结构体变量，存放JY901的原始数据
jy901_raw_t* raw = &jy901_raw;     //定义指向jy901_raw的指针变量
jy901_data_t jy901_data;           //使用jy901_data_t类型定义jy901_data结构体变量，存放JY901的物理量数据
jy901_data_t* data = &jy901_data;  //定义指向jy901_data的指针变量

uint8_t JY_RxBuffer[JY_Buffer_Size];

static void JY901_Feed(uint8_t byte);
static void JY901_Convert(const jy901_raw_t *raw, jy901_data_t *out);

/**
 * @brief JY901模块 绑定 单片机USART和DMA句柄
 */
void JY901_Init(void)
{
    jy901_uart.huart   = &huart_jy901;
    jy901_uart.hdma_rx = huart_jy901.hdmarx;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == jy901_uart.huart->Instance)
    {
        /* Size = 本次 DMA 实际收到的字节数 */
        for (uint16_t i = 0; i < Size; i++)
        {
            uint8_t byte = JY_RxBuffer[i];
            /* 喂给 JY901 协议解析 */
            JY901_Feed(byte);
        }

        /* 重新启动 DMA + IDLE 接收 */
        HAL_UARTEx_ReceiveToIdle_DMA(jy901_uart.huart,JY_RxBuffer, JY_Buffer_Size);
    }
}


// /**
//  * @brief 通过 USART4 发送一个完整的数据帧
//  * @param buf 数据帧缓冲区指针
//  * @param len 数据帧长度
//  */
// void uart4_send_bytes(uint8_t *buf, uint16_t len)
// {
//     for (uint16_t i = 0; i < len; i++)
//     {
//         HAL_UART_Transmit(&huart4, &buf[i], 1, HAL_MAX_DELAY);
//     }
// }

/**
 * @brief JY901 数据帧解析
 * @param buf 数据帧数组
 */
static void JY901_Process(uint8_t *buf)
{

    switch(buf[1])
    {
        case 0x51: // 加速度
            raw->ax = (int16_t)(buf[3] << 8 | buf[2]);
            raw->ay = (int16_t)(buf[5] << 8 | buf[4]);
            raw->az = (int16_t)(buf[7] << 8 | buf[6]);
            break;

        case 0x52: // 角速度
            raw->gx = (int16_t)(buf[3] << 8 | buf[2]);
            raw->gy = (int16_t)(buf[5] << 8 | buf[4]);
            raw->gz = (int16_t)(buf[7] << 8 | buf[6]);
            break;

        case 0x53: // 角度
            raw->roll  = (int16_t)(buf[3] << 8 | buf[2]);
            raw->pitch = (int16_t)(buf[5] << 8 | buf[4]);
            raw->yaw   = (int16_t)(buf[7] << 8 | buf[6]);
            break;
        default:
            break;
    }

    /* 实时转换为物理量 */
    JY901_Convert(raw, data);
}

/**
 * @brief JY901 数据帧校验与转发
 * @param byte 输入的单个字节
 */
static void JY901_Feed(uint8_t byte)
{
    static uint8_t state = 0;
    static uint8_t frame[11];
    static uint8_t index = 0;

    if (state == 0)
    {
        if (byte == 0x55)
        {
            frame[0] = byte;
            index = 1;
            state = 1;
        }
    }
    else if(state == 1)
    {
        if(byte == 0x51 || byte == 0x52 || byte == 0x53 || byte == 0x54)
        {
            frame[index++] = byte;
            state = 2;
        }
        else
        {
            state = 0;
            index = 0;
            frame[0] = 0;
        }
    }
    else if(state == 2)
    {
        frame[index++] = byte;

        if (index == 11)
        {
            /* 对一帧数据进行校验和 */
            uint8_t sum = 0;
            for (int i = 0; i < 10; i++) sum += frame[i];
            if (sum == frame[10])
            {
                /* 合法帧，才转发给解析函数 */
                JY901_Process(frame);
            }

            state = 0;
            index = 0;
            memset(frame, 0, sizeof(frame));
        }
    }
}

/**
 * @brief 将 JY901 原始数据转换为物理量
 * @param raw 输入的原始数据结构体指针
 * @param out 输出的物理量数据结构体指针
 */
static void JY901_Convert(const jy901_raw_t *raw, jy901_data_t *out)
{
    /* 加速度 ±16g */
    out->ax = raw->ax / 2048.0f;
    out->ay = raw->ay / 2048.0f;
    out->az = raw->az / 2048.0f;

    /* 角速度 ±2000 dps */
    out->gx = raw->gx / 16.4f;
    out->gy = raw->gy / 16.4f;
    out->gz = raw->gz / 16.4f;

    /* 欧拉角 ±180 deg */
    out->roll  = raw->roll  / 32768.0f * 180.0f;
    out->pitch = raw->pitch / 32768.0f * 180.0f;
    out->yaw   = raw->yaw   / 32768.0f * 180.0f;
}
