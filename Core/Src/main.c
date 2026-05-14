/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
extern uint16_t adc2_buf[2];
extern uint16_t adc1_buf[2];
// foc_handle_t motor0;

void Print1_Motor_To_VOFA(float data, uint8_t length)
{
    char uart_buf[20];
    sprintf(uart_buf, "%.6f\n", data);
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, length, 100);
    HAL_UART_Transmit(&huart6, (uint8_t *)"\n", 1, 100);
}

void Print2_Motor_To_VOFA(float data1, float data2)
{

    char uart_buf[256]; // 足够长以容纳三个数据
    // 使用逗号分隔，结尾加换行，VOFA 的 FireWater 协议才能正确识别成一帧
    int len = sprintf(uart_buf, "%.3f,%.3f\n", data1, data2);
    
    // 一次性发送，不要分段发送逗号和换行
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, len, 10);
}

void Print3_Motor_To_VOFA(float data1, float data2, float data3)
{
    char uart_buf[256]; // 足够长以容纳三个数据
    // 使用逗号分隔，结尾加换行，VOFA 的 FireWater 协议才能正确识别成一帧
    int len = sprintf(uart_buf, "%.3f,%.3f,%.3f\n", data1, data2, data3);
    
    // 一次性发送，不要分段发送逗号和换行
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, len, 10);
}

void Print4_Motor_To_VOFA(float data1, float data2, float data3, float data4)
{
    char uart_buf[256]; // 足够长以容纳三个数据
    // 使用逗号分隔，结尾加换行，VOFA 的 FireWater 协议才能正确识别成一帧
    int len = sprintf(uart_buf, "%.3f,%.3f,%.3f,%.3f\n", data1, data2, data3, data4);
    
    // 一次性发送，不要分段发送逗号和换行
    HAL_UART_Transmit(&huart6, (uint8_t *)uart_buf, len, 10);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_USART10_UART_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  
  Foc_Init(1, &foc_hal); // 左边电机
  Foc_Init(2, &foc_hal); // 右边电机

  HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET); // 485接收使能
  while (1)
  {
    // Print3_Motor_To_VOFA(FOC_Motor[2].i_uvw.u, FOC_Motor[2].i_uvw.v, FOC_Motor[2].i_uvw.w);
    // Print2_Motor_To_VOFA(FOC_Motor[2].i_ab.alpha, FOC_Motor[2].i_ab.beta);
    // Print2_Motor_To_VOFA(FOC_Motor[2].i_ab.alpha, FOC_Motor[2].i_ab_hat.alpha);
    // Print2_Motor_To_VOFA(FOC_Motor[2].u_dq.q, FOC_Motor[2].u_dq.d);
    // Print2_Motor_To_VOFA(FOC_Motor[2].i_dq.q, FOC_Motor[2].i_dq.d);
    // Print2_Motor_To_VOFA(FOC_Motor[2].e_ab.alpha, FOC_Motor[2].e_ab.beta);
    // Print3_Motor_To_VOFA(FOC_Motor[2].e_ab.alpha, FOC_Motor[2].e_ab.beta, sqrt(FOC_Motor[2].e_ab.alpha * FOC_Motor[2].e_ab.alpha + FOC_Motor[2].e_ab.beta * FOC_Motor[2].e_ab.beta));
    // Print3_Motor_To_VOFA(FOC_Motor[2].theta, FOC_Motor[2].theta_Observer, angle_error);
    // Print4_Motor_To_VOFA(FOC_Motor[2].theta, FOC_Motor[2].theta_Observer, FOC_Motor[2].speed_observer, angle_error);
    // Print4_Motor_To_VOFA(FOC_Motor[2].i_dq.d, FOC_Motor[2].i_dq.q, FOC_Motor[2].pi_q.output, FOC_Motor[2].speed);
    // Print3_Motor_To_VOFA(FOC_Motor[2].speed, FOC_Motor[2].speed_observer, angle_error);

    // Print3_Motor_To_VOFA(FOC_Motor[2].speed, FOC_Motor[2].speed_ramp_target, FOC_Motor[2].pi_q.output);
    // Print3_Motor_To_VOFA(FOC_Motor[2].theta_Observer, FOC_Motor[2].theta, FOC_Motor[2].pi_q.target);
    // Print3_Motor_To_VOFA(FOC_Motor[2].i_dq.q, FOC_Motor[2].pi_q.target, FOC_Motor[2].pi_q.output);
    Print3_Motor_To_VOFA(FOC_Motor[2].i_dq.d, FOC_Motor[2].pi_d.target, FOC_Motor[2].pi_d.output);
    // Print3_Motor_To_VOFA(FOC_Motor[2].i_adc_u, FOC_Motor[2].i_uvw.u, FOC_Motor[2].pi_q.target);
    // Print3_Motor_To_VOFA(FOC_Motor[2].i_ab_hat.alpha, FOC_Motor[2].i_ab_hat.beta, angle_error);
    // Print3_Motor_To_VOFA(FOC_Motor[2].u_dq.q, FOC_Motor[2].i_dq.q, FOC_Motor[2].speed);
    // Print4_Motor_To_VOFA(FOC_Motor[2].theta, FOC_Motor[2].theta_Observer, FOC_Motor[2].speed, angle_error);
    // Print4_Motor_To_VOFA(
    //     FOC_Motor[2].i_dq.d,          // 期望≈0
    //     FOC_Motor[2].i_dq.q,          // 期望≈正7值稳定
    //     FOC_Motor[2].speed,            // 期望≈target_speed
    //     FOC_Motor[2].theta_Observer    // 观察是否平滑
    // );
    // Print4_Motor_To_VOFA(
    //     FOC_Motor[2].speed,              // 实际转速
    //     FOC_Motor[2].pi_speed.target,    // 速度目标（斜坡后）
    //     FOC_Motor[2].i_dq.q,             // Iq（力矩电流）
    //     FOC_Motor[2].i_dq.d              // Id（应接近0）
    // );
    // 在 Foc_Close_Loop 中，速度环计算完后添加
    // Print4_Motor_To_VOFA(FOC_Motor[2].speed,           // 观测器估计的 RPM
    //                  FOC_Motor[2].speed_ramp_target,// 爬坡目标 RPM
    //                  FOC_Motor[2].pi_q.target,     // 速度环输出的 Iq 目标 (A)
    //                  FOC_Motor[2].i_dq.q);         // 实测 q 轴电流 (A)
    // Print2_Motor_To_VOFA(FOC_Motor[2].i_dq.q, FOC_Motor[2].i_dq.d);

    // Print4_Motor_To_VOFA(FOC_Motor[2].id_fw,        // 弱磁电流（应为负值）
    //                  FOC_Motor[2].i_dq.q,        // 实际 iq
    //                  FOC_Motor[2].speed,          // 转速
    //                  FOC_Motor[2].fw_active);     // 弱磁是否激活

    Foc_Set_Speed(2, 2000);
    // Foc_Set_Speed(1, 50);
    // Print2_Motor_To_VOFA((float)adc2_buf[0], (float)adc1_buf[1]);
    // 速度环调试需要看4个量
    // 改为打印这4个量（用Print3打前3个，observer_speed最重要）
    // Print3_Motor_To_VOFA(motor0.speed, motor0.speed_ramp_target, motor0.theta);
    // Print3_Motor_To_VOFA(motor0.speed, motor0.speed_ramp_target, angle_error);
    // Print3_Motor_To_VOFA(motor0.theta, theta_Observer, angle_error);
    // Print2_Motor_To_VOFA(motor0.e_alpha, motor0.e_beta);
    // Print2_Motor_To_VOFA(motor0.i_alpha, motor0.i_beta);
    // Print2_Motor_To_VOFA(motor0.adc_iu, motor0.adc_iw);
    // Print2_Motor_To_VOFA(motor0.i_q, motor0.i_d);
    // Print2_Motor_To_VOFA(motor0.i_q, angle_error);
    // Print2_Motor_To_VOFA(motor0.theta, theta_Observer);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
        Foc_Loop(2);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    if(htim->Instance == TIM8)
    {
        Foc_Loop(1);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    }
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
