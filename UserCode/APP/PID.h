#ifndef __PID_H
#define __PID_H


// PID参数结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;         // 积分项
    float last_error;       // 上次误差
    float output_limit;     // 输出限幅
    float integral_limit;   // 积分限幅
    float a;                // 一阶低通滤波系数
    float filtered_error;   // 滤波后的误差
} pid_control_t;

extern pid_control_t balance_pid; // 水舱平衡PID控制器参数结构体
float pid_calculate(pid_control_t* pid, float expect, float actual);
#endif