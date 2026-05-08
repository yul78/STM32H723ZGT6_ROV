/**
 * @file foc_math.c
 * @author MING
 * @brief foc有关计算函数，最新版本完善了动态相位补偿
 * @version 0.2
 * @date 2026-05-03
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "foc_math.h"

/**
 * @brief sat函数
 * 
 * @param x 
 * @param boundary 
 * @return float 
 */
float FOC_sat(float x, float boundary)
{
    if (x > boundary) return 1.0f;
    else if (x < -boundary) return -1.0f;
    else return x / boundary;
}

/**
 * @brief 动态反电动势低通滤波系数
 * 
 * @param speed_rpm 转速
 * @return float 滤波系数
 */
float FOC_calc_dynamic_lpf(float speed_rpm)
{
    float fe = fabsf(speed_rpm) * POLE_PAIRS / 60.0f;
    
    // 目标：截止频率 = 3~5 倍电频率
    float fc_target = 2.0f * fe;  
    
    // 限制范围
    if (fc_target < 80.0f)  fc_target = 80.0f;   // 最低100Hz
    if (fc_target > 800.0f) fc_target = 800.0f;   // 最高2000Hz 
    
    // 反算α。
    // fc = lfp × fs / (2π × (1-lfp))
    // lfp = 2π × fc × TS / (1 + 2π × fc × TS)
    float wc_ts = _2_PI * fc_target * TS;
    float lfp = wc_ts / (1.0f + wc_ts);
    
    return lfp;
}

/**
 * @brief 动态相位补偿
 * 
 * @param omega_e_est pll的电角度
 * @return float 补偿的角度
 */
float calc_compensation_angle(float omega_e_est)
{
    // omega_e_est: 电角速度 (rad/s)，来自PLL
    float fe = fabsf(omega_e_est) / _2_PI;
    
    // LPF截止频率
    // float fc = BEMF_LPF / (_2_PI * TS * (1.0f - BEMF_LPF));
    float speed_rpm = fe * 60.0f / POLE_PAIRS;
    float actual_lfp = FOC_calc_dynamic_lpf(speed_rpm);
    float fc = actual_lfp / (_2_PI * TS * (1.0f - actual_lfp));
    
    // LPF相位延迟（主要部分）
    float comp = atanf(fe / fc);
    
    // 加上数字延迟（可选，通常较小）
    // comp += 1.5f * _2_PI * fe * TS; // 1.5是指1.5个周期的延迟
    
    // 方向：正转加，反转减
    if (omega_e_est < 0) comp = -comp;
    
    return comp;
}

/**
 * @brief 计算电流的零点偏移
 * 
 * @param motor motor结构体
 */
void FOC_Motor_Cali_Offset(foc_handle_t *motor)
{
    uint32_t sum_iu = 0, sum_iw = 0;
    uint16_t cali_cnt = 1000;
    
    // 读取ADC平均值作为零点偏移
    for(uint16_t i=0; i<cali_cnt; i++)
    {
        uint16_t adc_temp_u, adc_temp_v, adc_temp_w;
        motor->hal.adc_get_value(motor->num, &adc_temp_u, &adc_temp_v, &adc_temp_w); // 读取ADC数据

        sum_iu += adc_temp_u;
        sum_iw += adc_temp_w;

        HAL_Delay(1);
    }
    motor->i_cali_uvw.u = sum_iu / cali_cnt;
    motor->i_cali_uvw.w = sum_iw / cali_cnt;
}

/**
 * @brief clack变换
 * 
 * @param motor motor结构体
 */
void FOC_Clark_Transform(foc_handle_t *motor)
{
    // 纯电流：仅用IU/iw，Iu=-IU-iw (IU + iw + Iv = 0)
    motor->i_uvw.w = -motor->i_uvw.v - motor->i_uvw.u;
    // Clark变换公式（等功率变换）
    motor->i_ab.alpha = motor->i_uvw.u;
    motor->i_ab.beta = (motor->i_uvw.u + 2.0f * motor->i_uvw.v) / SQRT_3;
}

/**
 * @brief park变换
 * 
 * @param motor motor结构体
 */
void FOC_Park_Transform(foc_handle_t *motor)
{
    float cos_theta = cosf(motor->theta);
    float sin_theta = sinf(motor->theta);
    // Park变换公式
    motor->i_dq.d = motor->i_ab.alpha * cos_theta + motor->i_ab.beta * sin_theta;
    motor->i_dq.q = -motor->i_ab.alpha * sin_theta + motor->i_ab.beta * cos_theta;
}

/**
 * @brief 反park变换
 * 
 * @param motor motor结构体
 */
void FOC_InvPark_Transform(foc_handle_t *motor)
{
    float cos_theta = cosf(motor->theta);
    float sin_theta = sinf(motor->theta);
    // 反Park变换公式
    motor->u_ab.alpha = motor->u_dq.d * cos_theta - motor->u_dq.q * sin_theta;
    motor->u_ab.beta = motor->u_dq.d * sin_theta + motor->u_dq.q * cos_theta;
}

/**
 * @brief pid控制器
 * 
 * @param pi foc中的pid结构体
 * @param dt 单位时间
 */
void FOC_PI_Regulator(foc_pid_t *pi, float dt)
{
    float error = pi->target - pi->feedback;
    // 比例项
    pi->output = pi->kp * error;
    // 积分项（限幅防饱和）
    pi->integral += pi->ki * error * dt;
    pi->integral = (pi->integral > pi->limit) ? pi->limit : (pi->integral < -pi->limit) ? -pi->limit : pi->integral; // 三元运算符进行积分限幅
    pi->output += pi->integral;
    // 输出限幅
    pi->output = (pi->output > pi->limit) ? pi->limit : (pi->output < -pi->limit) ? -pi->limit : pi->output;// 三元运算符进行输出限幅
}

/**
 * @brief SVPWM正弦波生成
 * 
 * @param motor motor结构体
 */
void FOC_SVPWM_Generate(foc_handle_t *motor)
{
    float u_alpha = motor->u_ab.alpha;// α // β
    float u_beta = motor->u_ab.beta;
    float Ta, Tb, Tc;
    float Tx = 0.0f, Ty = 0.0f;
    uint8_t sector;

    float k = PWM_ARR / PWM_VBUS;
    float U1 = u_beta;
    float U2 = -SQRT_3_2 * u_alpha - u_beta / 2.0f;
    float U3 = SQRT_3_2 * u_alpha - u_beta / 2.0f;

    // 扇区判断
    uint8_t A, B, C;
    A = U1 > 0 ? 1 : 0;
    B = U2 > 0 ? 1 : 0;
    C = U3 > 0 ? 1 : 0;
    uint8_t N = A + B * 2 + C * 4;

    // 判断扇区
    switch (N)
    {
        case 1: sector = 2; break;
        case 2: sector = 6; break;
        case 3: sector = 1; break;
        case 4: sector = 4; break;
        case 5: sector = 3; break;
        case 6: sector = 5; break;
        default: break;
    }
    
      // 计算切换间隔
    switch (sector)
    {
    case 1:
        Tx = U2 * k;
        Ty = U1 * k;
        break;
    case 2:
        Tx = -U2 * k;
        Ty = -U3 * k;
        break;
    case 3:
        Tx = U1 * k;
        Ty = U3 * k;
        break;
    case 4:
        Tx = -U1 * k;
        Ty = -U2 * k;
        break;
    case 5:
        Tx = U3 * k;
        Ty = U2 * k;
        break;
    case 6:
        Tx = -U3 * k;
        Ty = -U1 * k;
        break;
    default:
        break;
    }

    // 过调制处理
    if(Tx + Ty > PWM_ARR)
    {
        float T_sum = Tx + Ty;
        Tx = Tx / T_sum * PWM_ARR;
        Ty = Ty / T_sum * PWM_ARR;
    }

    // 计算占空比
    Ta = (PWM_ARR + Tx + Ty) / 2;
    Tb = Ta - Tx;
    Tc = Tb - Ty;
    
    switch(sector)
    {
        case 1: motor->pwm.duty_u = Ta; motor->pwm.duty_v = Tb; motor->pwm.duty_w = Tc; break;
        case 2: motor->pwm.duty_u = Tb; motor->pwm.duty_v = Ta; motor->pwm.duty_w = Tc; break;
        case 3: motor->pwm.duty_u = Tc; motor->pwm.duty_v = Ta; motor->pwm.duty_w = Tb; break;
        case 4: motor->pwm.duty_u = Tc; motor->pwm.duty_v = Tb; motor->pwm.duty_w = Ta; break;
        case 5: motor->pwm.duty_u = Tb; motor->pwm.duty_v = Tc; motor->pwm.duty_w = Ta; break;
        case 6: motor->pwm.duty_u = Ta; motor->pwm.duty_v = Tc; motor->pwm.duty_w = Tb; break;
        default: break;
    }
    
    // 占空比限幅（防止超量程）
    motor->pwm.duty_u = (motor->pwm.duty_u > PWM_ARR) ? PWM_ARR : (motor->pwm.duty_u < 0) ? 0 : motor->pwm.duty_u;
    motor->pwm.duty_v = (motor->pwm.duty_v > PWM_ARR) ? PWM_ARR : (motor->pwm.duty_v < 0) ? 0 : motor->pwm.duty_v;
    motor->pwm.duty_w = (motor->pwm.duty_w > PWM_ARR) ? PWM_ARR : (motor->pwm.duty_w < 0) ? 0 : motor->pwm.duty_w;

    // 设置占空比
    motor->hal.pwm_set_duty(motor->num, motor->pwm.duty_u, motor->pwm.duty_v, motor->pwm.duty_w);
}



