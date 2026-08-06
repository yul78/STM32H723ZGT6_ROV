/**
 * @file smo.c
 * @author MING
 * @brief 滑膜观测器的算法主要在这个文件之中
 * @version 1.0
 * @date 2026-04-23
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "smo.h"

float angle_error;

/**
 * @brief BEMF观测器的算法
 * 
 * @param motor 电机结构体
 * @param dt 单位时间
 * @param mode 当前旋转模式是开环还是闭环
 */
void BEMF_Observer(foc_handle_t *motor, float dt, foc_mode_t mode)
{
    float e_alpha_raw = motor->u_ab.alpha - MOTOR_R * motor->i_ab.alpha ;
    float e_beta_raw  = motor->u_ab.beta  - MOTOR_R * motor->i_ab.beta  ;
    
    motor->e_ab.alpha += (e_alpha_raw - motor->e_ab.alpha) * BEMF_LPF;
    motor->e_ab.beta  += (e_beta_raw  - motor->e_ab.beta)  * BEMF_LPF;
    
    float cos_obs = cosf(motor->theta_Observer);
    float sin_obs = sinf(motor->theta_Observer);
    angle_error = -motor->e_ab.alpha * cos_obs - motor->e_ab.beta * sin_obs; // 临时变为临时变量以便调试
    
    // 归一化：消除转速对增益的影响
    float e_amp = sqrtf(motor->e_ab.alpha * motor->e_ab.alpha 
                + motor->e_ab.beta  * motor->e_ab.beta);
    
    // 软限幅
    float e_amp_min = 0.5f;
    if (e_amp > e_amp_min)
    {
        angle_error /= e_amp;
    }
    else if (e_amp > 0.1f)
    {
        angle_error = angle_error / e_amp_min * (e_amp / e_amp_min); // 线性衰减到0
    }
    else if(e_amp < -e_amp_min)
    {
        angle_error /= e_amp;
    }
    else if (e_amp < -0.1f)
    {
        angle_error = angle_error / e_amp_min * (e_amp / e_amp_min);
    }
    else
    {
        angle_error = 0.0f;
    }
    
    // pll锁相环
    motor->pi_pll.integral += angle_error * PLL_KI * dt;
    
    // 积分限幅
    if(motor->pi_pll.integral >  OB_SPEED_LIMIT) motor->pi_pll.integral =  OB_SPEED_LIMIT;
    if(motor->pi_pll.integral < -OB_SPEED_LIMIT) motor->pi_pll.integral = -OB_SPEED_LIMIT;
    
    // 速度低通滤波
    float speed_raw = PLL_KP * angle_error + motor->pi_pll.integral;
    motor->speed_observer += (speed_raw - motor->speed_observer) * 0.30f;
    
    // 观测器估测速度限幅
    if(motor->speed_observer >  OB_SPEED_LIMIT) motor->speed_observer =  OB_SPEED_LIMIT;
    if(motor->speed_observer < -OB_SPEED_LIMIT) motor->speed_observer = -OB_SPEED_LIMIT;
    
    motor->theta_Observer += motor->speed_observer * dt;
    motor->theta_Observer = fmodf(motor->theta_Observer, _2_PI);
    if(motor->theta_Observer < 0) motor->theta_Observer += _2_PI;
    
    if(mode == MOTOR_STATE_CLOSE)
        motor->theta = motor->theta_Observer;
    
    motor->speed = motor->speed_observer * 60.0f / _2_PI_POLE_PAIRS;
}

/**
 * @brief SMO观测器的算法
 * 
 * @param motor 电机结构体
 * @param dt 单位时间
 * @param mode 当前旋转模式是开环还是闭环
 */
void SMO_Observer(foc_handle_t *motor, float dt, foc_mode_t mode)
{
    // ===== 预计算常数（可在初始化时计算一次）=====
    float a = 1.0f - MOTOR_R * dt / MOTOR_L;  // 0.8545
    float b = dt / MOTOR_L;                    // 0.6550
    
    // ===== 第1步：电流误差 =====
    float err_alpha = motor->i_ab_hat.alpha - motor->i_ab.alpha;
    float err_beta  = motor->i_ab_hat.beta  - motor->i_ab.beta;
    
    // ===== 第2步：滑模切换项（使用 SMO_K 和 SAT_BOUNDARY）=====
    float z_alpha = SMO_K * FOC_sat(err_alpha, SAT_BOUNDARY);
    float z_beta  = SMO_K * FOC_sat(err_beta,  SAT_BOUNDARY);
    
    // ===== 第3步：电流观测器迭代 =====
    // Î[k+1] = a·Î[k] + b·(U[k] - Z[k])
    motor->i_ab_hat.alpha = a * motor->i_ab_hat.alpha 
                       + b * (motor->u_ab.alpha - z_alpha);
    motor->i_ab_hat.beta  = a * motor->i_ab_hat.beta  
                       + b * (motor->u_ab.beta  - z_beta);
    
    // ===== 第4步：BEMF = 切换项的低通滤波 =====
    #ifdef FOC_PLL_ENABLE
    float speed_rpm = fabsf(motor->speed_observer) * 60.0f / _2_PI_POLE_PAIRS;
    #else
    float speed_rpm = fabsf(motor->speed) * 60.0f / _2_PI_POLE_PAIRS;
    #endif // FOC_PLL_ENABLE
    
    motor->e_ab.alpha += (z_alpha - motor->e_ab.alpha) * FOC_calc_dynamic_lpf(speed_rpm);
    motor->e_ab.beta  += (z_beta  - motor->e_ab.beta)  * FOC_calc_dynamic_lpf(speed_rpm);

    // ===== 第5步：PLL=====
    #ifdef FOC_PLL_ENABLE

    float theta_comp = motor->theta_Observer - calc_compensation_angle(motor->speed_observer);
    // float theta_comp = motor->theta_Observer;

    float cos_obs = cosf(theta_comp);
    float sin_obs = sinf(theta_comp);

    float speed_sign = (motor->speed_observer >= 0.0f) ? 1.0f : -1.0f;
    angle_error = -motor->e_ab.alpha * cos_obs - motor->e_ab.beta  * sin_obs;
    angle_error *= speed_sign;
    
    // 归一化：消除转速对增益的影响
    float e_amp = sqrtf(motor->e_ab.alpha * motor->e_ab.alpha 
                + motor->e_ab.beta  * motor->e_ab.beta);
    
    // 软限幅
    float e_amp_min = 0.5f;
    if (e_amp > e_amp_min) {
        angle_error /= e_amp;
    } else if (e_amp > 0.1f) {
        angle_error = angle_error / e_amp_min * (e_amp / e_amp_min); // 线性衰减到0
    } else {
        angle_error = 0.0f;
    }
    
    // pll锁相环
    motor->pi_pll.integral += angle_error * PLL_KI * dt;
    
    // 积分限幅
    if(motor->pi_pll.integral >  OB_SPEED_LIMIT) motor->pi_pll.integral =  OB_SPEED_LIMIT;
    if(motor->pi_pll.integral < -OB_SPEED_LIMIT) motor->pi_pll.integral = -OB_SPEED_LIMIT;
    
    // 速度低通滤波
    float speed_raw = PLL_KP * angle_error + motor->pi_pll.integral;
    motor->speed_observer += (speed_raw - motor->speed_observer) * SPEED_OBSERBER_LPF;
    
    // 观测器估测速度限幅
    if(motor->speed_observer >  OB_SPEED_LIMIT) motor->speed_observer =  OB_SPEED_LIMIT;
    if(motor->speed_observer < -OB_SPEED_LIMIT) motor->speed_observer = -OB_SPEED_LIMIT;
    
    motor->theta_Observer += motor->speed_observer * dt;
    motor->theta_Observer = fmodf(motor->theta_Observer, _2_PI);
    if(motor->theta_Observer < 0) motor->theta_Observer += _2_PI;
    
    if(mode == MOTOR_STATE_CLOSE)
        motor->theta = motor->theta_Observer;
    
    motor->speed = motor->speed_observer * 60.0f / _2_PI_POLE_PAIRS;

    #endif // FOC_PLL_ENABLE
}

