#include "bsp_jy901.h"

jy901_uart_t jy901_uart;
uint8_t JY_RxBuffer[JY_Buffer_Size];

/**
 * @brief JY901模块 绑定 单片机USART和DMA句柄
 */
void BSP_JY901_Init(void)
{
    jy901_uart.huart   = &huart_jy901;
    jy901_uart.hdma_rx = huart_jy901.hdmarx;

    // 清除 USART1 可能的 Overrun/Noise/Framing/Parity 错误标志
    __HAL_UART_CLEAR_FLAG(jy901_uart.huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF  | UART_CLEAR_PEF);
    HAL_UART_DMAStop(jy901_uart.huart); // 确保 DMA 处于空闲状态
    HAL_UARTEx_ReceiveToIdle_DMA(jy901_uart.huart, JY_RxBuffer, JY_Buffer_Size);   //开启UART空闲中断+DMA接收
    __HAL_DMA_DISABLE_IT(jy901_uart.hdma_rx, DMA_IT_HT);                           //关闭DMA的半传输中断
}