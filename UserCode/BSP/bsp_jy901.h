#ifndef __BSP_JY901_H
#define __BSP_JY901_H

#include "main.h"

#define JY_Buffer_Size  256

/*****************JY901S驱动板级设置部分*****************/

#define huart_jy901 huart1                   //JY901S连接的USART句柄
#define hdma_jy901_rx (huart1.hdmarx)        //JY901S连接的USART的DMA接收句柄
/*******************************************************/

typedef struct
{
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;

    uint8_t received_byte;
    uint8_t data_ready;
} jy901_uart_t;

extern jy901_uart_t jy901_uart;
extern uint8_t JY_RxBuffer[JY_Buffer_Size];

void BSP_JY901_Init(void);

#endif