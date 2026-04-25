#ifndef __FOC_CONFIG_H
#define __FOC_CONFIG_H

#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif

// PWM参数
#define PWM_ARR                 8088.0f         // 自动重载值
#define PWM_SCALE               3.3f            // ADC参考电压
#define PWM_VBUS                13.5f           // VBUS母线电压
#define ADC_RESOLUTION          65535.0f        // 16位ADC分辨率
#define TS                      0.0001176f     // 采样时间间隔

// INA240参数   
#define INA240_GAIN             50.0f           // INA240A2增益50V/V
#define SAMPLE_RESISTOR         0.00162f        // 采样电阻A1mΩ

// 观测器参数   
#define SMO_K                   3.2f            // 滑模增益 (根据实际效果调试)
#define BEMF_LPF                0.1f           // 反电动势低通滤波系数
#define COMP                    0.0f            // 偏移量
#define SINGLE_STEP_LIMIT       1.0f            // 单步限幅
#define OB_SPEED_LIMIT          4500.0f         // 观测速度限幅
#define PLL_INIT_LIMIT          1500.0f         // PLL积分限幅4,673.521850899743
#define SAT_BOUNDARY            0.6f            // sat函数饱和边界   

// 电机通用参数（根据电机修改）
#define POLE_PAIRS              7.0f            // 电机极对数（示例：7对极）
#define CURRENT_LIMIT           20.0f           // 最大相电流(A)
#define MOTOR_R                 0.222261666f      // 相电阻 (Ohm)
#define MOTOR_L                 0.0000897929f   // 相电感 (Henry)
#define MAX_MOTOR_NUM           2               // 最大电机数量

// PLL参数
#define PLL_KP                  500.0f      
#define PLL_KI                  20000.0f    
#define BTN7960_DEAD_TIME_S     0.0000005f 

// 电流环参数
#define PI_KP_D                 0.8591f 
#define PI_KI_D                 2374.069f   
#define PI_KP_Q                 0.8591f 
#define PI_KI_Q                 2374.069f
#define PI_KP_SPEED             0.0015f         // 速度PI比例系数
#define PI_KI_SPEED             0.002f          // 速度PI积分系数
#define PI_LIMIT_SPEED          15.0f           // 速度PI输出限幅

// 速度定义（分离开环和闭环）
// #define TARGET_SPEED            2000.0f         // 闭环最终目标(RPM)
#define OPEN_LOOP_SPEED_RPM     300.0f          // 开环启动速度(RPM)，必须是电机能跟上的
#define SPEED_RAMP_RATE         400.0f          // 闭环加速斜率(RPM/s)
#define SPEED_START_THRESHOLD   50.0f   // RPM，低于此值视为停机指令

#ifdef __cplusplus
}
#endif

#endif 

