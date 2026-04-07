/*
获取GPS数据流程：
1. GPS模块通过串口发送NMEA语句，BSP层在串口中断中接收字节，并将其压入软件FIFO。
2. APP层定期调用BSP_GPS_ReadFifo函数，BSP层从FIFO中按字节拼接NMEA语句，并提取完整的RMC语句。
3. APP层调用BSP_GPS_GetLatestRmcSentence函数获取最近的RMC语句，并使用MID层的解析函数将其转换为经纬度等业务数据。
*/


#include "bsp_gps.h"
#include <string.h>

#define BSP_GPS_RX_FIFO_SIZE  2048U

static uint8_t bsp_gps_rx_fifo[BSP_GPS_RX_FIFO_SIZE];                    // 原始接收字节FIFO                                    // 串口中断当前接收到的单个字节
static uint8_t bsp_gps_sentence_buffer[BSP_GPS_NMEA_SENTENCE_SIZE];      // 当前正在拼接的NMEA语句
static uint8_t bsp_gps_latest_rmc[BSP_GPS_NMEA_SENTENCE_SIZE];   
uint8_t bsp_gps_rx_byte = 0;          // 最近一次收到的完整RMC语句

bsp_gps_state_t bsp_gps_state;                                           // BSP层GPS运行状态
static volatile uint16_t bsp_gps_fifo_write = 0;                           // 接收FIFO写指针
static volatile uint16_t bsp_gps_fifo_read = 0;                            // 接收FIFO读指针
static uint16_t bsp_gps_sentence_length = 0;                             // 当前语句缓存中的有效长度
static volatile uint8_t bsp_gps_latest_rmc_ready = 0;                    // 最近RMC缓存是否有效

static void BSP_GPS_ProcessSentenceByte(uint8_t byte);

/**
 * @brief 初始化GPS接收硬件和软件状态。
 */
void BSP_GPS_Init(void)
{
    bsp_gps_state.huart = &BSP_GPS_UART;
    bsp_gps_state.received_byte = 0;
    bsp_gps_state.overflow = 0;
    bsp_gps_fifo_write = 0;
    bsp_gps_fifo_read = 0;
    bsp_gps_sentence_length = 0;
    bsp_gps_latest_rmc_ready = 0;

    __HAL_UART_CLEAR_FLAG(bsp_gps_state.huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_FE | UART_FLAG_PE);
    //HAL_UART_DMAStop(bsp_gps_state.huart);
    HAL_UART_AbortReceive(bsp_gps_state.huart);
    HAL_UART_Receive_IT(bsp_gps_state.huart, &bsp_gps_rx_byte, 1U);
}

/**
 * @brief 计算缓冲区在给定最大长度内的字符串长度。
 * @param buf 待计算的字符串缓冲区。
 * @param max_len 最大允许扫描的长度。
 * @retval 实际长度，不包含字符串结束符。
 */
static uint16_t BSP_GPS_StringLength(const uint8_t *buf, uint16_t max_len)
{
    uint16_t len = 0;

    while ((len < max_len) && (buf[len] != '\0'))
    {
        len++;
    }

    return len;
}


/**
 * @brief 将接收到的单个字节写入软件FIFO。
 * @param byte 串口接收到的原始字节。
 */
void BSP_GPS_WriteFifo(uint8_t byte)
{
    uint16_t next_write = (uint16_t)((bsp_gps_fifo_write + 1U) % BSP_GPS_RX_FIFO_SIZE);

    if (next_write == bsp_gps_fifo_read)
    {
        bsp_gps_state.overflow = 1U;
        return;
    }

    bsp_gps_rx_fifo[bsp_gps_fifo_write] = byte;
    bsp_gps_fifo_write = next_write;
}

/**
 * @brief 读取软件FIFO中的字节，并调用处理函数进行语句拼接和提取,作为GPS数据流的驱动
 */
void BSP_GPS_Task(void)
{
    while (bsp_gps_fifo_read != bsp_gps_fifo_write)
    {
        BSP_GPS_ProcessSentenceByte(bsp_gps_rx_fifo[bsp_gps_fifo_read]);
        bsp_gps_fifo_read = (uint16_t)((bsp_gps_fifo_read + 1U) % BSP_GPS_RX_FIFO_SIZE); //读指针后移
    }
}


/**
 * @brief 在任务态按字节拼接NMEA语句，并提取完整RMC语句。
 * @param byte 从接收FIFO取出的单个字节。
 */
static void BSP_GPS_ProcessSentenceByte(uint8_t byte)
{
    uint16_t copy_len;

    if (byte == '$')
    {
        bsp_gps_sentence_length = 0;
    }

    if (bsp_gps_sentence_length >= (BSP_GPS_NMEA_SENTENCE_SIZE - 1U))
    {
        bsp_gps_sentence_length = 0;
        return;
    }

    bsp_gps_sentence_buffer[bsp_gps_sentence_length++] = byte;

    if (byte == '\n')
    {
        bsp_gps_sentence_buffer[bsp_gps_sentence_length] = '\0';

        if (strstr((const char *)bsp_gps_sentence_buffer, "RMC") != NULL)
        {
            copy_len = bsp_gps_sentence_length + 1U; //下标从0开始，长度 = 下标 + 1
            memcpy(bsp_gps_latest_rmc, bsp_gps_sentence_buffer, copy_len);
            bsp_gps_latest_rmc_ready = 1U;
        }

        bsp_gps_sentence_length = 0;
    }
}


/**
 * @brief 判断接收FIFO中是否还有待处理字节。
 * @retval 1表示FIFO非空，0表示FIFO为空。
 */
uint8_t BSP_GPS_HasPendingByte(void)
{
    return (bsp_gps_fifo_read != bsp_gps_fifo_write) ? 1U : 0U;
}

/**
 * @brief 获取最近一次收到的完整RMC语句快照。
 * @param buf 用于接收RMC语句的目标缓冲区。
 * @param buf_size 目标缓冲区大小。
 * @retval 1表示获取成功，0表示当前没有可用RMC语句。
 */
uint8_t BSP_GPS_GetLatestRmcSentence(uint8_t *buf, uint16_t buf_size)
{
    uint16_t copy_len;

    if ((buf == NULL) || (buf_size == 0U) || (bsp_gps_latest_rmc_ready == 0U))
    {
        return 0U;
    }

    __disable_irq();
    copy_len = BSP_GPS_StringLength(bsp_gps_latest_rmc, BSP_GPS_NMEA_SENTENCE_SIZE);
    if (copy_len >= buf_size)
    {
        copy_len = (uint16_t)(buf_size - 1U);
    }
    memcpy(buf, bsp_gps_latest_rmc, copy_len);
    buf[copy_len] = '\0';
    __enable_irq();

    return 1U;
}



