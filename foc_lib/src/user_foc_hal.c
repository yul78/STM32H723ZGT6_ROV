/**
 * @file user_foc_hal.c
 * @author MING
 * @brief 用户foc_hal.h接口实现文件
 * @version 0.1
 * @date 2026-03-22
 * 
 * @copyright Copyright (c) 2026
 * 
 */


/*--------------------硬件照应-----------------------*/
/*
-------------------------------------------------
|右电机--|--数组2--|--htim1--|--ADC2--|--adc2_buf|
|左电机--|--数组1--|--htim8--|--ADC1--|--adc1_buf|
-------------------------------------------------
*/
/*--------------------------------------------------*/


#include "foc_hal.h"

// #include "stm32h7xx_hal.h"
#include "main.h"
#include "tim.h"
#include "adc.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
uint16_t adc1_buf[2] __attribute__((aligned(32)));
uint16_t adc2_buf[2] __attribute__((aligned(32)));

static void foc_hal_init(uint8_t num)
{
    switch(num)
    {
        case 1:
            HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc1_buf, sizeof(adc1_buf) / sizeof(adc1_buf[0])); // 启动ADC DMA采样
            HAL_TIM_Base_Start_IT(&htim8);// 启动定时器（FOC计算）
            break;
        case 2:
            HAL_ADCEx_Calibration_Start(&hadc2, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
            HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2_buf, sizeof(adc2_buf) / sizeof(adc2_buf[0])); // 启动ADC DMA采样
            HAL_TIM_Base_Start_IT(&htim1);// 启动定时器（FOC计算）
            break;
    }
}

static void foc_hal_set_duty(uint8_t num, float du, float dv, float dw)
{
    switch(num)
    {
        case 1:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, du);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, dv);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, dw);
            break;
        case 2:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, du);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dv);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dw);
            break;
    }
}

static void foc_hal_tim_start(uint8_t num)
{
    switch(num)
    {
        case 1:
            HAL_TIM_Base_MspInit(&htim8);
            HAL_TIM_MspPostInit(&htim8);

            HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 5495);
            break;
        case 2:
            HAL_TIM_Base_MspInit(&htim1);
            HAL_TIM_MspPostInit(&htim1);

            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 5495);
            break;
    }
}

static void foc_hal_pwm_enable(uint8_t num)
{
    switch(num)
    {
        case 1: __HAL_TIM_MOE_ENABLE(&htim8); break; // 启动PWM输出
        case 2: __HAL_TIM_MOE_ENABLE(&htim1); break; // 启动PWM输出
    }
}

static void foc_hal_drive_init(uint8_t num)
{
    switch(num)
    {
        case 1: HAL_GPIO_WritePin(BM_ENA_GPIO_Port, BM_ENA_Pin, GPIO_PIN_SET); break; // 启动M1左电机
        case 2: HAL_GPIO_WritePin(BM_ENB_GPIO_Port, BM_ENB_Pin, GPIO_PIN_SET); break; // 启动M2右电机
    }
}

static void foc_hal_drive_deinit(uint8_t num)
{
    switch(num)
    {
        case 1: HAL_GPIO_WritePin(BM_ENA_GPIO_Port, BM_ENA_Pin, GPIO_PIN_RESET); break; // 启动M1左电机
        case 2: HAL_GPIO_WritePin(BM_ENB_GPIO_Port, BM_ENB_Pin, GPIO_PIN_RESET); break; // 启动M2右电机
    }
}

static void foc_hal_adc_get_value(uint8_t num, uint16_t *adc_u, uint16_t *adc_v, uint16_t *adc_w)
{
    switch(num)
    {
        case 1:
            SCB_InvalidateDCache_by_Addr((uint32_t*)adc1_buf, sizeof(adc1_buf));
            *adc_u = adc1_buf[1];
            *adc_v = 0;
            *adc_w = adc1_buf[0];
            break;
        case 2:
            SCB_InvalidateDCache_by_Addr((uint32_t*)adc2_buf, sizeof(adc2_buf));
            *adc_u = adc2_buf[1];
            *adc_v = 0;
            *adc_w = adc2_buf[0];
            break;
    }
}

const foc_hal_t foc_hal =
{
    .pwm_enable = foc_hal_pwm_enable,
    .pwm_start = foc_hal_tim_start,
    .pwm_set_duty = foc_hal_set_duty,
    .drv_enable = foc_hal_drive_init,
    .drv_disable = foc_hal_drive_deinit,
    .adc_get_value = foc_hal_adc_get_value,
    .init = foc_hal_init,
};



