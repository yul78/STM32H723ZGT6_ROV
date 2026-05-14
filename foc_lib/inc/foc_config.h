#ifndef __FOC_CONFIG_H
#define __FOC_CONFIG_H

#include "math.h"

#ifdef __cplusplus
extern "C" {
#endif


// #define ONLY_OPEN_LOOP
#define FOC_SPEED_CONTROL
// #define FOC_CLOSE_I_DEBUG
#define FOC_PLL_ENABLE                      // 使能锁相环
// #define FW_ENABLE                           // 使能弱磁


// PWM参数
#define PWM_ARR                 5500.0f         // 自动重载值
#define PWM_SCALE               3.3f            // ADC参考电压
#define PWM_VBUS                12.4f           // VBUS母线电压
#define ADC_RESOLUTION          65535.0f        // 16位ADC分辨率
// #define TS                      0.0001176f     // 采样时间间隔
#define TS                      0.00008f     // 采样时间间隔

// INA240参数   
#define INA240_GAIN             50.0f           // INA240A2增益50V/V
#define SAMPLE_RESISTOR         0.00148f        // 采样电阻A1mΩ

// 观测器参数   
#define SMO_K                   4.0f            // 滑模增益 (根据实际效果调试)
#define BEMF_LPF                0.1f           // 反电动势低通滤波系数
#define SPEED_OBSERBER_LPF      0.1f           // 观测器求得的速度的低通滤波系数
#define COMP                    0.0f            // 偏移量
#define OB_SPEED_LIMIT          10000.0f         // 观测速度限幅
#define PLL_INIT_LIMIT          1500.0f         // PLL积分限幅4,673.521850899743
#define SAT_BOUNDARY            0.8f            // sat函数饱和边界   

// ===== 弱磁控制参数 =====
#define CURRENT_PI_LIMIT        6.801f      // 电流环电压输出限幅（V），与PI limit一致
#define FW_VOLTAGE_THRESHOLD    0.92f       // 触发弱磁的电压利用率（建议0.93~0.97）
#define FW_KI                   1.0f       // 弱磁积分增益（越大响应越快，但可能振荡）
#define FW_EXIT_RATE            0.3f        // 退出弱磁时id恢复速率倍数（相对FW_KI）
#define FW_ID_MAX               8.0f        // 最大弱磁电流限幅（A），不超过 CURRENT_LIMIT/2

// 电机通用参数（根据电机修改）
#define POLE_PAIRS              7.0f            // 电机极对数（示例：7对极）
#define CURRENT_LIMIT           20.0f           // 最大相电流(A)
#define MOTOR_R                 0.222261666f      // 相电阻 (Ohm)
#define MOTOR_L                 0.0000897929f   // 相电感 (Henry)
#define MAX_MOTOR_NUM           2               // 最大电机数量

// PLL参数
#define PLL_KP                  170.0f      
#define PLL_KI                  5000.0f    
#define BTN7960_DEAD_TIME_S     0.0000005f 

// 电流环参数
#define PI_KP_D                 0.56418f 
#define PI_KI_D                 1396.511f   
#define PI_KP_Q                 0.56418f 
#define PI_KI_Q                 1396.511f
// #define PI_KP_D                 0.11224f 
// #define PI_KI_D                 277.827f   
// #define PI_KP_Q                 0.11224f 
// #define PI_KI_Q                 277.827f
// #define PI_KP_D                 0.564f    // 2π*带宽*L = 2*3.14*1000*0.00008979
// #define PI_KI_D                 2472.0f   // R/L = 0.222/0.00008979
// #define PI_KP_Q                 0.564f 
// #define PI_KI_Q                 2472.0f
#define PI_KP_SPEED             0.005f         // 速度PI比例系数
#define PI_KI_SPEED             0.2f          // 速度PI积分系数
#define PI_LIMIT_SPEED          14.0f           // 速度PI输出限幅

// 速度定义（分离开环和闭环）
// #define TARGET_SPEED            2000.0f         // 闭环最终目标(RPM)
#define OPEN_LOOP_SPEED_RPM     300.0f          // 开环启动速度(RPM)，必须是电机能跟上的
#define SPEED_RAMP_RATE         1000.0f          // 闭环加速斜率(RPM/s)
#define SPEED_START_THRESHOLD   50.0f   // RPM，低于此值视为停机指令

#ifdef __cplusplus
}
#endif

#endif 

