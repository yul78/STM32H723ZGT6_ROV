/* field_weakening.c */
#include "field_weakening.h"
#include "foc_math.h"

void FOC_FieldWeakening(foc_handle_t *motor, float dt)
{
#ifndef FW_ENABLE
    motor->target_id = 0.0f;
    motor->id_fw = 0.0f;
    return;
#endif

    // ===== Step1：计算当前电压矢量幅值（使用上一拍PI输出）=====
    float vd = motor->u_dq.d;
    float vq = motor->u_dq.q;
    float vs = sqrtf(vd * vd + vq * vq);

    float vlim       = CURRENT_PI_LIMIT;
    float vlim_enter = vlim * FW_VOLTAGE_THRESHOLD;      // 进入弱磁阈值

    // ===== Step2：弱磁积分器 =====
    if (vs > vlim_enter)
    {
        // 进入弱磁：积分注入负 id
        float id_fw_new = motor->id_fw - FW_KI * (vs - vlim_enter) * dt; // 原来的计算
        float delta = id_fw_new - motor->id_fw;
        float delta_max = 0.05f; // 每拍最大允许变化 50mA（根据采样频率 12.5kHz 算，0.05A/80µs ≈ 625 A/s）
        if (delta < -delta_max) delta = -delta_max;
        if (delta >  delta_max) delta = delta_max;
        motor->id_fw += delta;
        motor->fw_active = 1.0f;
    }
    else
    {
        // 退出弱磁：只有 id_fw < 0 时才逐渐恢复
        if (motor->id_fw < 0.0f)
        {
            motor->id_fw += FW_KI * FW_EXIT_RATE * dt;
            if (motor->id_fw > 0.0f)
                motor->id_fw = 0.0f;
        }
        if (motor->id_fw >= 0.0f)
            motor->fw_active = 0.0f;
    }

    // ===== Step3：双重限幅保护 =====
    // 限幅1：最大弱磁电流
    if (motor->id_fw < -FW_ID_MAX)
        motor->id_fw = -FW_ID_MAX;

    // 限幅2：总电流圆限制 sqrt(id^2 + iq^2) < CURRENT_LIMIT
    // 保证 iq 优先，id 被圆限制
    float iq_ref     = motor->pi_d.target;
    float id_max_fw  = -sqrtf(fmaxf(0.0f,
                         CURRENT_LIMIT * CURRENT_LIMIT - iq_ref * iq_ref));
    if (motor->id_fw < id_max_fw)
        motor->id_fw = id_max_fw;

    // ===== Step4：写入 id 目标值 =====
    motor->pi_d.target = motor->id_fw;
}
