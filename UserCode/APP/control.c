#include "control.h"
#include "H_Tmc2209.h"
#include "WaterTank.h"

static volatile uint8_t drone_balance_control_flag = 0; // 1代表需要调用平衡控制函数，0代表无需调用平衡控制函数

/**
 * @brief 游戏手柄控制函数
 */
void Gamepad_Control(void)
{
    GamepadData_t *Gamepad_raw_data = Gamepad_GetData();
    Analysis_GamepadData_t Gamepad_analysis_data;
    GamepadData_Analysis(Gamepad_raw_data, &Gamepad_analysis_data);

    // 根据分析后的数据进行速度控制
    FOC_Set_Speed(1, Gamepad_analysis_data.leftX);
    FOC_Set_Speed(2, Gamepad_analysis_data.leftY);
}

/**
 * @brief 设置FOC速度
 * @param motor_num 电机编号,1电机是左右方向推进器，2电机是前后垂直推进器
 * @param speed 速度值，范围-1500~1500，正数代表正转，负数代表反转，绝对值越大速度越快
 */
void FOC_Set_Speed(uint8_t motor_num, int16_t speed)
{
  
}

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


