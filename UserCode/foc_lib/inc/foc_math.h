#ifndef __FOC_MATH_H
#define __FOC_MATH_H

#include "foc_types.h"
#include "foc_config.h"

#ifdef __cplusplus
extern "C" {
#endif



// 数学符号定义
#define PI                      3.1415926535f
#define _2_PI                   6.283185307f
#define _PI_2                   1.570796326f
#define SQRT_3                  1.732050807f
#define SQRT_3_2                0.866025403f
#define MECHANICAL_SPEED        (TARGET_SPEED * _2_PI / 60.0f)
#define ELEC_SPEED              (MECHANICAL_SPEED * POLE_PAIRS)
#define _2_PI_POLE_PAIRS        (_2_PI * POLE_PAIRS)

float FOC_sat(float x, float boundary);
float FOC_calc_dynamic_lpf(float speed_rpm);
float calc_compensation_angle(float omega_e_est);
void FOC_Motor_Cali_Offset(foc_handle_t *motor);
void FOC_Clark_Transform(foc_handle_t *motor);
void FOC_Park_Transform(foc_handle_t *motor);
void FOC_InvPark_Transform(foc_handle_t *motor);
void FOC_PI_Regulator(foc_pid_t *pi, float dt);
void FOC_SVPWM_Generate(foc_handle_t *motor);



#ifdef __cplusplus
}
#endif


#endif // !__FOC_MATH_H 

