#include "control.h"

#define ELEVATION_ANGLE_THRESHOLD 10.0f   //仰角阈值，单位：度，超过这个值就认为需要进行平衡控制
#define DEPRESSION_ANGLE_THRESHOLD -10.0f //俯角阈值，单位：度，超过这个值就认为需要进行平衡控制

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
    FOC_Set_Speed(2, Gamepad_analysis_data.rightY);
}

/**
 * @brief 设置FOC速度
 * @param motor_num 电机编号,1电机是左右方向推进器，2电机是前后垂直推进器
 * @param speed 速度值，范围-1500~1500，正数代表正转，负数代表反转，绝对值越大速度越快
 */
void FOC_Set_Speed(uint8_t motor_num, int16_t speed)
{
  Foc_Set_Speed(motor_num, (float)speed);
}

void Drone_Balance_Control(void)
{
  if(Scheduler_Get_DroneBalanceControlFlag())
  {
    if(jy901_data.roll > ELEVATION_ANGLE_THRESHOLD) //仰角过大
    {
      Water_Tank_Filling(&tank_front, 0.5f); //前水舱进水
      Water_Tank_Draining(&tank_rear, 0.5f); //后水舱排水
    }
    if(jy901_data.roll < DEPRESSION_ANGLE_THRESHOLD) //俯角过大
    {
      Water_Tank_Filling(&tank_rear, 0.5f); //后水舱进水
      Water_Tank_Draining(&tank_front, 0.5f); //前水舱排水
    }
  }
  else
  {
    //不需要调用平衡控制函数，直接返回
    return;
  }
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


