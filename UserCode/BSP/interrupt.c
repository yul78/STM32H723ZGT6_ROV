#include "interrupt.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim3) //10ms定时器中断
  {
    static uint16_t cnt = 0;
    cnt++;
    if(cnt >= 10)
    {
      Scheduler_Set_DroneBalanceControlFlag(); //每100ms置一次平衡控制标志位
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
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == jy901_uart.huart->Instance)
    {
        jy901_uart.data_ready = 1; // 标记数据已准备好
        jy901_uart.received_byte = Size; // 接收到的字节数

        HAL_UARTEx_ReceiveToIdle_DMA(jy901_uart.huart, JY_RxBuffer, JY_Buffer_Size);
        
    }
}

/**
 * @brief 串口指令接收对应的回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3)
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
        HAL_UART_Receive_IT(&huart3, &rx_data, 1); //开启UART中断接收
        #endif
        // 将收到的字节交给手柄解析
        Gamepad_RxCallback(gamepad_rxByte);

        // 重新开启接收
        HAL_UART_Receive_IT(huart, &gamepad_rxByte, 1);
    }
}