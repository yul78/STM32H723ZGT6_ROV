#include "control.h"
#include "H_Tmc2209.h"
#include "WaterTank.h"

static volatile uint8_t drone_balance_control_flag = 0; // 1代表需要调用平衡控制函数，0代表无需调用平衡控制函数


void Drone_Balance_Control(void)
{
  
}

void Scheduler_Set_DroneBalanceControlFlag(void)
{
  drone_balance_control_flag = 1;
}

uint8_t Scheduler_Get_DroneBalanceControlFlag(void)
{
  if(drone_balance_control_flag)
  {
      drone_balance_control_flag = 0;
      return 1;
  }
  return 0;
}

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
