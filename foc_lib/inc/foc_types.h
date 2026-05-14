/**
 * @file foc_types.h
 * @author MING
 * @brief 结构体定义文件
 * @version 0.1
 * @date 2026-03-24
 * 
 * @copyright Copyright (c) 2026
 * 
 */


#ifndef __FOC_TYPEDEF_H
#define __FOC_TYPEDEF_H

#include "stdint.h"
#include "foc_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************* 数据结构定义 *************************/
typedef enum {
    MOTOR_STATE_IDLE = 0,  // ← 新增：空闲/停机等待
    MOTOR_STATE_ALIGN, // 转子定位
    MOTOR_STATE_OPEN,      // 开环强拉
    MOTOR_STATE_CLOSE      // 闭环运行
} foc_mode_t;

typedef enum
{
    FOC_OK = 0,                 // 正常
    FOC_ERR_LOOP,               // 循环中断
    FOC_ERR_NOT_INIT,           // 尚未初始化
    FOC_ERR_OVERCURRENT,        // 过流
    FOC_ERR_OVERVOLTAGE,        // 过压
    FOC_ERR_UNDERVOLTAGE,       // 过压
    FOC_ERR_ENCODER,            // 编码器错误
    FOC_ERR_HAL_NULL,           // HAL模块为空
    FOC_ERR_INVALID_PARAM,      // 无效参数
}foc_state_t;

typedef struct
{
    float u;
    float v;
    float w;
}foc_uvw_t; 

typedef struct
{
    float alpha;
    float beta;
}foc_ab_t;

typedef struct
{
    float d;
    float q;
}foc_dq_t;

typedef struct
{
    float k;            // 滑膜增益
    float bemf_lfp;     // 反电动势低通滤波系数
    float comp;         // 偏移量
}foc_smo_t;

typedef struct
{
    float pairs;        // 极对数
    float rs;           // 相电阻
    float ls;           // 相电感
    float a_limit;      // 相电流限幅
}foc_motor_params_t; 

// PI调节器结构体
typedef struct {
    float kp;            // 比例系数
    float ki;            // 积分系数
    float target;        // 目标值
    float feedback;      // 反馈值
    float output;        // 输出值
    float integral;      // 积分项
    float limit;         // 输出限幅
} foc_pid_t;

typedef struct
{
    float duty_u;
    float duty_v;
    float duty_w;
}foc_pwm_t;

typedef struct
{
    foc_hal_t               hal;
    foc_state_t             state;              // 运行状态
    uint8_t                 num;                // 电机编号

    foc_mode_t              mode;               // 运行模式
    foc_motor_params_t      motor;              // 电机参数
    foc_smo_t               smo;                // 滑膜观测器
    foc_pwm_t               pwm;                // PWM输出

    foc_uvw_t               i_uvw;              // 三相电流
    foc_uvw_t               i_cali_uvw;         // 零偏电流
    foc_ab_t                i_ab;               // clack变换的结果
    foc_ab_t                i_ab_pre;           // 上一拍电流
    foc_dq_t                i_dq;               // park变换的结果
    foc_ab_t                u_ab;               // park逆变换的结果
    foc_ab_t                e_ab;               // 在观测器中滤波之后的反电动势
    foc_ab_t                i_ab_hat;           // 估计的电流
    foc_dq_t                u_dq;               // q轴和d轴电压

    /*pid参数*/     
    foc_pid_t               pi_d;               // d轴电流PI
    foc_pid_t               pi_q;               // q轴电流PI
    foc_pid_t               pi_speed;           // 速度PI
    foc_pid_t               pi_pll;             // 锁相环PI

    uint16_t                i_adc_u;            // adc得到的三相电流
    uint16_t                i_adc_v;            // adc得到的三相电流
    uint16_t                i_adc_w;            // adc得到的三相电流
    float                   theta;              // 转子电角度(rad)
    float                   speed;              // 电机转速(rpm)
    float                   theta_Observer;     // 观测器得到的转子电角度(rad)
    float                   theta_obs_prev;     // 上一拍观测角度，用于微分估速
    float                   speed_observer;     // 观测器得到的电机转速(rpm)
    float                   speed_sign;         // 电机转子正转还是反转

    float   id_fw;                              // 弱磁注入的负 id（弱磁控制器输出，≤0）
    float   fw_active;                          // 弱磁激活标志（调试用）

    /*目标值*/      
    float                   target_iq;          // q轴电流目标值
    float                   target_id;          // d轴电流目标值
    float                   target_speed;       // 电机目标速度
    float                   speed_ramp_target;  // 电机爬坡目标速度

    /*运行计数*/
    uint32_t                state_timer;         // 运行状态定时器
    uint8_t                 PI_Speed_cnt;        // 速度PI计数
    uint32_t                close_cnt;           // 闭环计数

    uint8_t                 init_done;            // 初始化标志位
}foc_handle_t;

#ifdef __cplusplus
}
#endif



#endif // !__FOC_TYPEDEF_H


