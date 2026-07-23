#ifndef _USER_MATH_H
#define _USER_MATH_H

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
typedef struct
{
    uint16_t v_start;   // 起始速度（步/秒）
    uint16_t v_max;     // 最大速度（步/秒）
    uint16_t acc;       // 加速度（步/秒^2）
    uint16_t total_steps; // 总步数
} TrapezoidVelocity;

typedef struct
{
    uint16_t accel_steps;  // 加速段步数
    uint16_t decel_steps;  // 减速段步数
    uint16_t cruise_steps; // 匀速段步数
} MotionStep;

MotionStep calc_trapezoid_profile(TrapezoidVelocity profile);
// 根据当前步数计算对应速度（可转为对应的延时）
uint16_t get_step_speed(int step_idx, MotionStep phase, TrapezoidVelocity profile);

#endif /* _USER_MATH_H */