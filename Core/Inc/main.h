/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "dma.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Debug_USART_Show(const char* str);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SDA_Pin GPIO_PIN_0
#define OLED_SDA_GPIO_Port GPIOF
#define OLED_SCL_Pin GPIO_PIN_1
#define OLED_SCL_GPIO_Port GPIOF
#define EN2OUT_Pin GPIO_PIN_1
#define EN2OUT_GPIO_Port GPIOA
#define DIR2OUT_Pin GPIO_PIN_7
#define DIR2OUT_GPIO_Port GPIOA
#define DIR1OUT_Pin GPIO_PIN_4
#define DIR1OUT_GPIO_Port GPIOC
#define BM_ENB_Pin GPIO_PIN_5
#define BM_ENB_GPIO_Port GPIOC
#define BM_ENA_Pin GPIO_PIN_0
#define BM_ENA_GPIO_Port GPIOB
#define MPU_IIC_SDA_Pin GPIO_PIN_2
#define MPU_IIC_SDA_GPIO_Port GPIOB
#define MPU_IIC_SCL_Pin GPIO_PIN_1
#define MPU_IIC_SCL_GPIO_Port GPIOG
#define tank_front_full_Pin GPIO_PIN_2
#define tank_front_full_GPIO_Port GPIOG
#define tank_front_full_EXTI_IRQn EXTI2_IRQn
#define tank_front_empty_Pin GPIO_PIN_3
#define tank_front_empty_GPIO_Port GPIOG
#define tank_front_empty_EXTI_IRQn EXTI3_IRQn
#define tank_rear_full_Pin GPIO_PIN_4
#define tank_rear_full_GPIO_Port GPIOG
#define tank_rear_full_EXTI_IRQn EXTI4_IRQn
#define tank_rear_empty_Pin GPIO_PIN_5
#define tank_rear_empty_GPIO_Port GPIOG
#define tank_rear_empty_EXTI_IRQn EXTI9_5_IRQn
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOG
#define EN1OUT_Pin GPIO_PIN_10
#define EN1OUT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define DEBUG_huart huart6
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
