#include "user_math.h"

MotionStep calc_trapezoid_profile(TrapezoidVelocity profile) // 计算梯形运动规划
{
    MotionStep phase = {0};

    // 加速段步数 = (v_max^2 - v_start^2) / (2a) 9
    phase.accel_steps = (int)((profile.v_max * profile.v_max - profile.v_start * profile.v_start) / (2 * profile.acc));
    phase.decel_steps = phase.accel_steps; // 对称减速
    int sum_steps = phase.accel_steps + phase.decel_steps;

    if (sum_steps > profile.total_steps)
    {
        // 达不到最大速度，只能部分加速再减速（即三角形加减速）
        phase.accel_steps = profile.total_steps / 2;
        phase.decel_steps = profile.total_steps - phase.accel_steps;
        phase.cruise_steps = 0;
    }
    else
    {
        phase.cruise_steps = profile.total_steps - sum_steps;
    }

    return phase;
}

uint16_t get_step_speed(int step_idx, MotionStep phase, TrapezoidVelocity profile) // 根据当前步数计算对应速度
{
    if (step_idx <= phase.accel_steps)
    {
        // 加速段
        return (uint16_t)sqrtf(profile.v_start * profile.v_start + 2 * profile.acc * step_idx);
    }
    else if (step_idx <= phase.accel_steps + phase.cruise_steps)
    {
        // 匀速段
        return profile.v_max;
    }
    else if (step_idx <= phase.accel_steps + phase.cruise_steps + phase.decel_steps)
    {
        // 减速段
        int s = step_idx - (phase.accel_steps + phase.cruise_steps);
        return (uint16_t)sqrtf(profile.v_max * profile.v_max - 2 * profile.acc * s);
    }
    else
    {
        // 超出范围，默认停止
        return 0;
    }
    
}
