#include "bsp_interrupt.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"

extern osSemaphoreId_t FocBinarySemHandle;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim3) //10ms定时器中断
  {
    
    // 每10ms触发一次FOC控制任务
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(FocBinarySemHandle, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);  // 中断结束后立刻触发任务调度

    static uint16_t cnt = 0;
    cnt++;
    if(cnt >= 10)
    {
      Scheduler_Set_DroneBalanceControlFlag(); // 每100ms触发一次平衡控制标志位
      cnt = 0;
    }
  }
  else if(htim == motor_tim[1] || htim == motor_tim[2])
  {
    for (int i = 1; i <= 2; i++)
    {
        if (htim == motor_tim[i])
        {
            if (Motor[i].mode == Constant_step && Motor[i].target_step)
            {
                Motor[i].current_step++;
                if (Motor[i].current_step >= Motor[i].target_step)
                {
                    // Motor[i].current_step = 0;
                    Motor[i].target_step = 0;
                    // HAL_GPIO_WritePin(en_ports[i], en_pins[i], GPIO_PIN_RESET);
                    HAL_TIM_Base_Stop_IT(motor_tim[i]);
                    HAL_TIM_PWM_Stop(motor_tim[i], motor_channel[i]);
                }
                // else
                // {
                //     Motor[i].hz = get_step_speed(Motor[i].current_step, Motor[i].steps, Motor[i].velocity);
                //     uint16_t arr = (TIMER_CLK_HZ / Motor[i].hz) - 1;
                //     __HAL_TIM_SET_AUTORELOAD(motor_tim[i], arr);
                //     __HAL_TIM_SET_COMPARE(motor_tim[i], motor_channel[i], arr / 2);
                //     __HAL_TIM_SET_COUNTER(motor_tim[i], 0);
                // }
            }
            break;
        }
    }
  }

  if(htim->Instance == TIM1)
  {
      Foc_Loop(2);
      // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
  }
  if(htim->Instance == TIM8)
  {
      Foc_Loop(1);
      // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == jy901_uart.huart->Instance)
    {
        jy901_uart.data_ready = 1; // 标志数据已准备好
        jy901_uart.received_byte = Size; // 接收到的字节数

        HAL_UARTEx_ReceiveToIdle_DMA(jy901_uart.huart, JY_RxBuffer, JY_Buffer_Size);
        
    }
}

/**
 * @brief 串口指令接收对应的回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &gamepad_huart)
    {
        #if 0
        if(rx_data == 0x02)
            Water_Tank_Draining(&tank_front, 1.0f);
        else if(rx_data == 0x01)
            Water_Tank_Filling(&tank_front, 1.0f);
        else if(rx_data == 0x03)
            Water_Tank_Filling(&tank_rear, 1.0f);
        else if(rx_data == 0x04)
            Water_Tank_Draining(&tank_rear, 1.0f);
        //OLED_ShowNum(64, 32, 0, 1, OLED_8X16);
        HAL_UART_Receive_IT(&huart3, &rx_data, 1); //开启UART�??�??接收
        #endif
        // 将收到的字节交给手柄解析
        Gamepad_RxCallback(gamepad_rxByte);

        // 重新开启接收
        HAL_UART_Receive_IT(huart, &gamepad_rxByte, 1);
    }

    if (huart->Instance == bsp_gps_state.huart->Instance)
    {
        /* 中断里仅做单字节入队和重新挂接收，避免拉长中断时间 */
        bsp_gps_state.received_byte = 1U;
        BSP_GPS_WriteFifo(bsp_gps_rx_byte);
        HAL_UART_Receive_IT(bsp_gps_state.huart, &bsp_gps_rx_byte, 1U);
    }
}

/**
 * @brief 串口错误回调，用于清错并重新开启接收
 * @param huart 触发错误的串口句柄
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == bsp_gps_state.huart->Instance)
    {
        BSP_GPS_UART_RxCallback();
    }
}
