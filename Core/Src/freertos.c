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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FocTask */
osThreadId_t FocTaskHandle;
const osThreadAttr_t FocTask_attributes = {
  .name = "FocTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FocBinarySem */
osSemaphoreId_t FocBinarySemHandle;
const osSemaphoreAttr_t FocBinarySem_attributes = {
  .name = "FocBinarySem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void vFocTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of FocBinarySem */
  FocBinarySemHandle = osSemaphoreNew(1, 1, &FocBinarySem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of FocTask */
  FocTaskHandle = osThreadNew(vFocTask, NULL, &FocTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  Gamepad_Init(&gamepad_huart);  // 手柄初始化
  // OLED_Init();
  // MPU6050_Init();
  BSP_JY901_Init();
  Water_Tank_Init(); // 初始化排水，会卡死
  WaterADC_Init();
  APP_GPS_Init();
  APP_Thrusters_Init();
  HAL_TIM_Base_Start_IT(&htim3);

  // 无刷电机初始化
  Foc_Init(1, &foc_hal);
  Foc_Init(2, &foc_hal);
  
  vTaskDelay(50);

  HAL_GPIO_WritePin(USART10_485_GPIO_Port, USART10_485_Pin, GPIO_PIN_RESET); // 485接收使能

  /* Infinite loop */
  for(;;)
  {
    /***********485测试************/
    // HAL_GPIO_WritePin(USART10_485_GPIO_Port, USART10_485_Pin, GPIO_PIN_SET); // 485发送使能
    // char *test_str = "Hello, 485!\n";
    // HAL_UART_Transmit(&huart10, (uint8_t *)test_str, strlen(test_str), 100);
    // while(__HAL_UART_GET_FLAG(&huart10, UART_FLAG_TC) == RESET);
    
    // HAL_GPIO_WritePin(USART10_485_GPIO_Port, USART10_485_Pin, GPIO_PIN_RESET); // 485接收使能

    /***********游戏手柄控制无刷电机************/

    Gamepad_Control();  // 游戏手柄控制无刷电机
    pad = Gamepad_GetData();
    if (pad->isUpdated)
		{
			pad->isUpdated = 0;  // 清除标志
			// ========= 使用手柄数据 =========
      //Debug_USART_Show("Pad Upadate");
      //A键：前舱吸水1ml B键：前舱排水1ml X键：后舱吸水1ml Y键：后舱排水1ml
      if(pad->buttons == 1)
      {
        Water_Tank_Filling(&tank_front, 1.0f);
      }
      if(pad->buttons == 2)
      {
        Water_Tank_Draining(&tank_front, 1.0f);
      }
      if(pad->buttons == 4)
      {
        Water_Tank_Filling(&tank_rear, 1.0f);
      }
      if(pad->buttons == 8)
      {
        Water_Tank_Draining(&tank_rear, 1.0f);
      }
    }

    /************* OLED显示手柄数据 *************/
    // OLED_ShowString(0, 0, "LX:", OLED_8X16);
    // OLED_ShowString(0, 16, "LY:", OLED_8X16);
    // OLED_ShowString(56, 0, "RX:", OLED_8X16);
    // OLED_ShowString(56, 16, "RY:", OLED_8X16);
    // OLED_ShowString(0, 32, "btn:", OLED_8X16);
    // OLED_ShowString(0, 48, "hatX:", OLED_8X16);
    // OLED_ShowString(56, 48, "hatY:", OLED_8X16);
    // OLED_ShowNum(24, 0, pad->leftX, 3, OLED_8X16);
    // OLED_ShowNum(24, 16, pad->leftY, 3, OLED_8X16);

    // OLED_ShowNum(80, 0, pad->rightX, 3, OLED_8X16);
    // OLED_ShowNum(80, 16, pad->rightY, 3, OLED_8X16);

    // OLED_ShowNum(32, 32, pad->buttons, 4, OLED_8X16);
    // OLED_ShowNum(40, 48, pad->hatX, 1, OLED_8X16);
    // OLED_ShowNum(96, 48, pad->hatY, 1, OLED_8X16);

    // OLED_ShowNum(72, 32, pad->lt, 2, OLED_8X16);
    // OLED_ShowNum(96, 32, pad->rt, 2, OLED_8X16);
    //OLED_ShowString(0,0,"hello,723!",OLED_8X16);
    
    /************* OLED显示JY901S物理数据 *************/
    JY901_Task();
    char jy_info[128];
    sprintf(jy_info, "pitch:%.2f,roll:%.2f,yaw:%.2f\n", jy901_data.pitch, jy901_data.roll, jy901_data.yaw);
    // Debug_USART_Show(jy_info);
    // OLED_ShowFloatNum(0, 0, jy901_data.ax,2, 2, OLED_8X16);       // X轴加速度（单位：g）
    // OLED_ShowFloatNum(0, 16, jy901_data.ay,2, 2, OLED_8X16);      // Y轴加速度（单位：g）
    // OLED_ShowFloatNum(0, 32, jy901_data.az, 2, 2, OLED_8X16);     // Z轴加速度（单位：g）

    // OLED_ShowFloatNum(0, 0, jy901_data.gx, 2, 2, OLED_8X16);      // X轴角速度（单位：度每秒）
    // OLED_ShowFloatNum(0, 16, jy901_data.gy, 2, 2, OLED_8X16);     // Y轴角速度（单位：度每秒）
    // OLED_ShowFloatNum(0, 32, jy901_data.gz, 2, 2, OLED_8X16);     // Z轴角速度（单位：度每秒）

    // OLED_ShowFloatNum(0, 0, jy901_data.roll,2, 2, OLED_8X16);        // 欧拉角（单位：度）
    // OLED_ShowFloatNum(0, 16, jy901_data.pitch, 2, 2, OLED_8X16);     // 欧拉角（单位：度）
    // OLED_ShowFloatNum(0, 48, jy901_data.yaw, 2, 2, OLED_8X16);       // 欧拉角（单位：度）

    /************* 获取GPS数据 *************/
    APP_GPS_Task();
    char gps_info[128];
    sprintf(gps_info, "lat:%.6f,lng:%.6f\n", gps_data.latitude, gps_data.longitude);
    Debug_USART_Show(gps_info);

    /**************** OLED显示ADC采样值 ****************/
    // OLED_ShowNum(64, 0, adc_value[0], 5, OLED_8X16);
    // OLED_ShowNum(64, 16, adc_value[1], 5, OLED_8X16);
    char  water_adc_info[128];
    sprintf(water_adc_info, "Voltage: %.2fV, %.2fV\n", voltage_value[0], voltage_value[1]);
    //Debug_USART_Show(water_adc_info);
    if(Water_Check()) 
    {
      Debug_USART_Show("Water detected! Start draining...\r\n"); //调试信息
      Water_Tank_Draining_To_Empty(&tank_front);
      //Water_Tank_Draning_To_Empty(&tank_rear);
      Debug_USART_Show("Draing finished...\r\n"); //调试信息
      while(1);
    };
    // OLED_ShowFloatNum(0, 48, voltage_value[0], 1, 1, OLED_8X16);    
    // OLED_ShowFloatNum(64, 48,voltage_value[1], 1, 1, OLED_8X16);      

    /*********************步进电机**********************/
    // if(jy901_data.roll > 10)
    // {
    //     Motor_Set(1, 2, 1, 800,400,2400,800);
    //     Motor_Set(2, 2, 1, 800,400,2400,800);
    // }
    // else if(jy901_data.roll < -10)
    // {
    //     Motor_Set(1, 2, 0, 800,400,2400,800);
    //     Motor_Set(2, 2, 0, 800,400,2400,800);
    // }
    //OLED_ShowNum(64, 32, Motor_GetStep(1), 5, OLED_8X16);
    
    /*********************水舱**********************/
    //OLED_ShowNum(0,16, tank_front.state, 1, OLED_8X16);
    //OLED_ShowFloatNum(0, 32, tank_front.now_water_volume, 2, 2, OLED_8X16);
    
    //OLED_ShowNum(64, 16, tank_rear.state, 1, OLED_8X16);
    //Water_Tank_Front_Get_Volume();
    //OLED_ShowNum(64, 16, tank_rear.state, 1, OLED_8X16);
    //OLED_ShowFloatNum(64, 32, tank_rear.now_water_volume, 2, 2, OLED_8X16);
    //OLED_ShowFloatNum(64, 32, tank_front.target_water_volume, 2, 2, OLED_8X16);
    
    //OLED_Update();

    //水舱状态机更新
    Water_Tank_Update_Handler(&tank_front); 
    Water_Tank_Update_Handler(&tank_rear);

    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
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
  for(;;)
  {
    if (xSemaphoreTake(FocBinarySemHandle, portMAX_DELAY))
    {
      Debug_USART_Show("FocTask...\r\n"); //调试信息
      Gamepad_Control();  // 游戏手柄控制无刷电机
    }
  }
  /* USER CODE END vFocTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

