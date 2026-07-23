#include "PID.h"

// 水舱平衡PID控制器参数初始化
pid_control_t balance_pid = {
    .kp = 0.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral = 0.0f,
    .integral_limit = 0.0f,
    .last_error = 0.0f,
    .output_limit = 0.0f,
    .a = 0.0f,
    .filtered_error = 0.0f
};


/**
 * @brief PID控制器计算函数
 * @param pid: PID控制器结构体指针
 * @param expect: 期望值
 * @param actual: 真实值
 * @return PID输出值
 */
float pid_calculate(pid_control_t* pid, float expect, float actual) {
    float error = expect - actual;

    /* 一阶滤波 */
    //pid->filtered_error = pid->a * error + (1 - pid->a) * pid->last_error;
    pid->filtered_error = error;

    pid->integral += pid->filtered_error;
    
    // 积分限幅
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    /* PID计算 */
    float output = pid->kp * pid->filtered_error + pid->ki * pid->integral + pid->kd * (pid->filtered_error - pid->last_error);

    /* 误差更新 */
    pid->last_error = pid->filtered_error;
    
    // 输出限幅
    if (output > pid->output_limit) 
        output = pid->output_limit;
    else if (output < -pid->output_limit) 
        output = -pid->output_limit;
    
    return output;
}

