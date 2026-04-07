/**
 * @file foc_utils.h
 * @author MING
 * @brief 存放派生参数，调用者非必要无需更改
 * @version 0.1
 * @date 2026-03-21
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef __FOC_UTILS_H__
#define __FOC_UTILS_H__ 

#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif


/*-------------------------------------派生参数 不需要更改-----------------------------------*/
#define OPEN_MECH_SPEED     (OPEN_LOOP_SPEED_RPM * _2_PI / 60.0f)
#define OPEN_ELEC_SPEED     (OPEN_MECH_SPEED * POLE_PAIRS)
#define CURRENT_SCALE       (PWM_SCALE / (INA240_GAIN * SAMPLE_RESISTOR * ADC_RESOLUTION))  // 从ad数值转换到实际电流值的转换系数
#define TS_MOTORL           (TS/MOTOR_L)
#define DEAD_COMP_V         (PWM_VBUS * BTN7960_DEAD_TIME_S / TS)  
#define PI_LIMIT            (PWM_VBUS * 0.95f / SQRT_3)                                     // 最大不失真电压


#ifdef __cplusplus
}
#endif

#endif

