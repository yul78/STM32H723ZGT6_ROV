/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "H_Tmc2209.h"
#include "bsp_delay.h"
// #include "OLED.h"
// #include "mpu6050.h"
#include "bsp_jy901.h"
#include "control.h"
#include "WaterADC.h"
#include "WaterTank.h"
#include "Gamepad.h"
#include "app_gps.h"
#include "app_thrusters.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Print1_Motor_To_VOFA(float data, uint8_t length)
{
    char uart_buf[20];
    sprintf(uart_buf, "%.6f\n", data);
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, length, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)"\n", 1, 100);
}

void Print2_Motor_To_VOFA(float data1, float data2)
{

    char uart_buf[256]; // 足够长以容纳三个数据
    // 使用逗号分隔，结尾加换行，VOFA 的 FireWater 协议才能正确识别成一帧
    int len = sprintf(uart_buf, "%.3f,%.3f\n", data1, data2);
    
    // 一次性发送，不要分段发送逗号和换行
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, 10);
}

void Print3_Motor_To_VOFA(float data1, float data2, float data3)
{
    char uart_buf[256]; // 足够长以容纳三个数据
    // 使用逗号分隔，结尾加换行，VOFA 的 FireWater 协议才能正确识别成一帧
    int len = sprintf(uart_buf, "%.3f,%.3f,%.3f\n", data1, data2, data3);
    
    // 一次性发送，不要分段发送逗号和换行
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, 10);
}

void Print4_Motor_To_VOFA(float data1, float data2, float data3, float data4)
{
    char uart_buf[256]; // 足够长以容纳三个数据
    // 使用逗号分隔，结尾加换行，VOFA 的 FireWater 协议才能正确识别成一帧
    int len = sprintf(uart_buf, "%.3f,%.3f,%.3f,%.3f\n", data1, data2, data3, data4);
    
    // 一次性发送，不要分段发送逗号和换行
    HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, len, 10);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  //SCB_DisableDCache();
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_Delay(500);  //加个延时等其他设备启动

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_ADC3_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_USART6_UART_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */

  Gamepad_Init(&gamepad_huart);  // 手柄初始化
  // OLED_Init();
  // MPU6050_Init();
  BSP_JY901_Init();
  Water_Tank_Init();
  WaterADC_Init();
  APP_GPS_Init();
  APP_Thrusters_Init();
  HAL_TIM_Base_Start_IT(&htim3);

  // 无刷电机初始化
  Foc_Init(1, &foc_hal);
  Foc_Init(2, &foc_hal);

  HAL_Delay(50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GamepadData_t *pad;

  HAL_GPIO_WritePin(USART10_485_GPIO_Port, USART10_485_Pin, GPIO_PIN_RESET); // 485接收使能
  
  while (1)
  {

    /***********485测试************/
    // HAL_GPIO_WritePin(USART10_485_GPIO_Port, USART10_485_Pin, GPIO_PIN_SET); // 485发送使能
    // char *test_str = "Hello, 485!\n";
    // HAL_UART_Transmit(&huart10, (uint8_t *)test_str, strlen(test_str), 100);
    // while(__HAL_UART_GET_FLAG(&huart10, UART_FLAG_TC) == RESET);
    
    // HAL_GPIO_WritePin(USART10_485_GPIO_Port, USART10_485_Pin, GPIO_PIN_RESET); // 485接收使能

    /***********游戏手柄控制无刷电机************/
    // HAL_GPIO_WritePin(USART2_485_GPIO_Port, USART2_485_Pin, GPIO_PIN_RESET); // 485接收使能
    // Foc_Set_Speed(2, -1000);
    // Foc_Set_Speed(1, 1000);
    // Print3_Motor_To_VOFA(FOC_Motor[2].i_uvw.u, FOC_Motor[2].i_uvw.v, FOC_Motor[2].i_uvw.w);
    // HAL_UART_Transmit(&huart2, "1", 2, 10);

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
    //Debug_USART_Show(jy_info);
    // OLED_ShowFloatNum(0, 0, jy901_data.ax,2, 2, OLED_8X16);       // X轴加速度（单位：g）
    // OLED_ShowFloatNum(0, 16, jy901_data.ay,2, 2, OLED_8X16);      // Y轴加速度（单位：g）
    // OLED_ShowFloatNum(0, 32, jy901_data.az, 2, 2, OLED_8X16);     // Z轴加速度（单位：g）

    // OLED_ShowFloatNum(0, 0, jy901_data.gx, 2, 2, OLED_8X16);      // X轴角速度（单位：度每秒）
    // OLED_ShowFloatNum(0, 16, jy901_data.gy, 2, 2, OLED_8X16);     // Y轴角速度（单位：度每秒）
    // OLED_ShowFloatNum(0, 32, jy901_data.gz, 2, 2, OLED_8X16);     // Z轴角速度（单位：度每秒）

    //OLED_ShowFloatNum(0, 0, jy901_data.roll,2, 2, OLED_8X16);        // 欧拉角（单位：度）
    // OLED_ShowFloatNum(0, 16, jy901_data.pitch, 2, 2, OLED_8X16);     // 欧拉角（单位：度）
    //OLED_ShowFloatNum(0, 48, jy901_data.yaw, 2, 2, OLED_8X16);       // 欧拉角（单位：度）

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
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //OLED_Update();

    //水舱状态机更新
    Water_Tank_Update_Handler(&tank_front); 
    Water_Tank_Update_Handler(&tank_rear);

    HAL_Delay(20);
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 34;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 32;
  PeriphClkInitStruct.PLL2.PLL2N = 129;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Debug_USART_Show(const char* str)
{
    HAL_UART_Transmit(&DEBUG_huart, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
