/**
 * @file user_task.c
 * @author Lanlan
 * @brief FreeRTOS任务实现文件，包含系统任务、FOC控制任务、导航任务、浮力控制任务和通信任务的实现
 */
#include "user_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "bsp_jy901.h"
#include "control.h"
#include "WaterADC.h"
#include "WaterTank.h"
#include "Gamepad.h"
#include "app_gps.h"
#include "app_thrusters.h"
#include "semphr.h"
#include "foc.h"
#include "stdio.h"

extern osMessageQueueId_t WaterTankQueueHandle;
extern osThreadId_t FocTaskHandle;
GamepadData_t *pad;

/**
 * @brief 系统任务，包括游戏手柄控制无刷电机、ADC采样值、GPS数据、水舱状态机更新
 * @note 任务优先级：osPriorityLow，周期500ms
 */
void System_Task(void)
{
    /***********游戏手柄控制无刷电机************/
    pad = Gamepad_GetData();
    if (pad->isUpdated)
		{
			pad->isUpdated = 0;  // 清除标志

      // 将按键状态发送到BuoyancyTask中
      xQueueSend(WaterTankQueueHandle, &pad->buttons, 0);
    }

    /**************** ADC采样值 ****************/
    // char  water_adc_info[128];
    // sprintf(water_adc_info, "Voltage: %.2fV, %.2fV\n", voltage_value[0], voltage_value[1]);
    // Debug_USART_Show(water_adc_info);
    if(Water_Check()) 
    {
      //Debug_USART_Show("Water detected! Start draining...\r\n"); //调试信息
      Water_Tank_Draining_To_Empty(&tank_front);
      //Debug_USART_Show("Draing finished...\r\n"); //调试信息
      while(1);
    };

    /************* 获取GPS数据 *************/
    APP_GPS_Task();

    //水舱状态机更新
    Water_Tank_Update_Handler(&tank_front); 
    Water_Tank_Update_Handler(&tank_rear);
}

/**
 * @brief FOC控制任务，负责执行FOC算法控制无刷电机
 * @note 任务优先级：osPriorityHigh，周期10ms
 */
void Foc_Task(void)
{
    Gamepad_Control();  // 游戏手柄控制无刷电机
}

/**
 * @brief 导航任务，负责处理导航相关的传感器数据，如JY901S物理量数据
 * @note 任务优先级：osPriorityNormal，周期100ms
 */
void Navigation_Task(void)
{
    /************* JY901S物理数据 *************/
    JY901_Task();
}

/**
 * @brief 浮力控制任务，负责处理水舱相关的控制逻辑
 * @note 任务优先级：osPriorityBelowNormal，周期100ms
 */
void Buoyancy_Task(void)
{
    uint8_t buttons = 0;
    //A键：前舱吸水1ml B键：前舱排水1ml X键：后舱吸水1ml Y键：后舱排水1ml
    xQueueReceive(WaterTankQueueHandle, &buttons, portMAX_DELAY);
    if(buttons == 1)
    {
      Water_Tank_Filling(&tank_front, 1.0f);
    }
    if(buttons == 2)
    {
      Water_Tank_Draining(&tank_front, 1.0f);
    }
    if(buttons == 4)
    {
      Water_Tank_Filling(&tank_rear, 1.0f);
    }
    if(buttons == 8)
    {
      Water_Tank_Draining(&tank_rear, 1.0f);
    }
}

/**
 * @brief 通信任务，负责处理与上位机的通信，如打印GPS数据、JY901S数据、系统状态等调试信息
 * @note 任务优先级：osPriorityBelowNormal，周期100ms
 */
void Communication_Task(void)
{
    char Buffer[128];  // 调试信息缓冲区
    // gps数据通过串口中断接收并存储在gps_data结构体中，定期打印到调试串口
    sprintf(Buffer, "lat:%.6f,lng:%.6f\n", gps_data.latitude, gps_data.longitude);
    Debug_USART_Show(Buffer);

    // 显示剩余堆内存，返回值单位是字节
    sprintf(Buffer, "Free Heap: %u\r\n", xPortGetFreeHeapSize());
    Debug_USART_Show(Buffer);

    // 显示栈剩余空间，返回值代为是字
    sprintf(Buffer, "FOC stack left: %u\r\n", uxTaskGetStackHighWaterMark(FocTaskHandle));
    Debug_USART_Show(Buffer);

    // JY901S数据
    sprintf(Buffer, "pitch:%.2f,roll:%.2f,yaw:%.2f\n", jy901_data.pitch, jy901_data.roll, jy901_data.yaw);
    Debug_USART_Show(Buffer);

    sprintf(Buffer, "pitch:%.2f,roll:%.2f,yaw:%.2f\n", jy901_data.pitch, jy901_data.roll, jy901_data.yaw);
    Debug_USART_Show(Buffer);
}

