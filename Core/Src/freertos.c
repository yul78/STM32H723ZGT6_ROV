/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
GamepadData_t *pad;
/* USER CODE END Variables */
/* Definitions for SystemTask */
osThreadId_t SystemTaskHandle;
const osThreadAttr_t SystemTask_attributes = {
  .name = "SystemTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FocTask */
osThreadId_t FocTaskHandle;
const osThreadAttr_t FocTask_attributes = {
  .name = "FocTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for NavigationTask */
osThreadId_t NavigationTaskHandle;
const osThreadAttr_t NavigationTask_attributes = {
  .name = "NavigationTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BuoyancyTask */
osThreadId_t BuoyancyTaskHandle;
const osThreadAttr_t BuoyancyTask_attributes = {
  .name = "BuoyancyTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for GamepadQueue */
osMessageQueueId_t GamepadQueueHandle;
const osMessageQueueAttr_t GamepadQueue_attributes = {
  .name = "GamepadQueue"
};
/* Definitions for WaterTankQueue */
osMessageQueueId_t WaterTankQueueHandle;
const osMessageQueueAttr_t WaterTankQueue_attributes = {
  .name = "WaterTankQueue"
};
/* Definitions for DebugUsartMutex */
osMutexId_t DebugUsartMutexHandle;
const osMutexAttr_t DebugUsartMutex_attributes = {
  .name = "DebugUsartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void vSystemTask(void *argument);
void vFocTask(void *argument);
void vNavigationTask(void *argument);
void vBuoyancyTask(void *argument);
void vCommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of DebugUsartMutex */
  DebugUsartMutexHandle = osMutexNew(&DebugUsartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of GamepadQueue */
  GamepadQueueHandle = osMessageQueueNew (1, sizeof(uint8_t), &GamepadQueue_attributes);

  /* creation of WaterTankQueue */
  WaterTankQueueHandle = osMessageQueueNew (10, sizeof(uint16_t), &WaterTankQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SystemTask */
  SystemTaskHandle = osThreadNew(vSystemTask, NULL, &SystemTask_attributes);

  /* creation of FocTask */
  FocTaskHandle = osThreadNew(vFocTask, NULL, &FocTask_attributes);

  /* creation of NavigationTask */
  NavigationTaskHandle = osThreadNew(vNavigationTask, NULL, &NavigationTask_attributes);

  /* creation of BuoyancyTask */
  BuoyancyTaskHandle = osThreadNew(vBuoyancyTask, NULL, &BuoyancyTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(vCommTask, NULL, &CommTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_vSystemTask */
/**
* @brief Function implementing the SystemTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vSystemTask */
void vSystemTask(void *argument)
{
  /* USER CODE BEGIN vSystemTask */
  /* Infinite loop */
  // 优先级：SystemTask < CommTask < NavigationTask < BuoyancyTask < FocTask
  // 任务调度：FocTask（10ms周期）> BuoyancyTask（50ms周期）> NavigationTask（100ms周期）> CommTask（500ms周期）> SystemTask（20周期）
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
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

    // 每20ms检查一次
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20));
  }
  /* USER CODE END vSystemTask */
}

/* USER CODE BEGIN Header_vFocTask */
/**
* @brief Function implementing the FocTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vFocTask */
void vFocTask(void *argument)
{
  /* USER CODE BEGIN vFocTask */
  /* Infinite loop */
  HAL_TIM_Base_Start_IT(&htim3);
  for(;;)
  {
    // TIM3中断中每10ms释放一次
    // 等待中断通知
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Gamepad_Control();  // 游戏手柄控制无刷电机
  }
  /* USER CODE END vFocTask */
}

/* USER CODE BEGIN Header_vNavigationTask */
/**
* @brief Function implementing the NavigationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vNavigationTask */
void vNavigationTask(void *argument)
{
  /* USER CODE BEGIN vNavigationTask */
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前系统时间作为基准时间
  for(;;)
  {
    /************* JY901S物理数据 *************/
    JY901_Task();

    // 每100ms执行一次
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
  /* USER CODE END vNavigationTask */
}

/* USER CODE BEGIN Header_vBuoyancyTask */
/**
* @brief Function implementing the BuoyancyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vBuoyancyTask */
void vBuoyancyTask(void *argument)
{
  /* USER CODE BEGIN vBuoyancyTask */
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
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

    // 每50ms检查一次
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
  /* USER CODE END vBuoyancyTask */
}

/* USER CODE BEGIN Header_vCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vCommTask */
void vCommTask(void *argument)
{
  /* USER CODE BEGIN vCommTask */
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char Buffer[128];  // 调试信息缓冲区
  for(;;)
  {
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

    // 每500ms打印一次
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
  }
  /* USER CODE END vCommTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

