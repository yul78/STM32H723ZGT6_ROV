#ifndef __BSP_GPS_H
#define __BSP_GPS_H

#include "main.h"
#include "usart.h"

/* BSP层只负责串口接收、缓存和完整NMEA语句提取。 */

#define BSP_GPS_UART                 huart3
#define BSP_GPS_NMEA_SENTENCE_SIZE   128U

typedef struct
{
    UART_HandleTypeDef *huart;       /* GPS使用的串口句柄。 */
    volatile uint16_t received_byte; /* 最近一次接收到的字节数。 */
    volatile uint8_t overflow;       /* 软件FIFO是否发生过溢出。 */
} bsp_gps_state_t;

extern bsp_gps_state_t bsp_gps_state;
extern uint8_t bsp_gps_rx_byte;


void BSP_GPS_Init(void);
void BSP_GPS_Task(void);
uint8_t BSP_GPS_HasPendingByte(void);
uint8_t BSP_GPS_GetLatestRmcSentence(uint8_t *buf, uint16_t buf_size);
void BSP_GPS_WriteFifo(uint8_t byte);

#endif