/**
 * @file foc.c
 * @author MING
 * @brief foc调用库
 * @version 0.6
 * @date 2026-05-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "foc.h"

foc_handle_t FOC_Motor[MAX_MOTOR_NUM + 1] = {0};
uint32_t vofa_cnt = 0;

/**
 * @brief foc初始化
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_Init(uint8_t motor_num, const foc_hal_t *hal_interface)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];
    motor->num = motor_num;
    Foc_ParamInit(motor, hal_interface); // FOC初始化

    motor->hal.pwm_start(motor_num);
    FOC_Motor_Cali_Offset(motor);        // 电机零点校准（上电静止时执行）
    motor->hal.pwm_enable(motor_num);            // PWM使能
    motor->hal.drv_enable(motor_num);            // 驱动使能
    motor->mode = MOTOR_STATE_IDLE;
    motor->state_timer = 0;
    motor->init_done = 1;

    return FOC_OK;
}


/**
 * @brief foc参数初始化
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_ParamInit(foc_handle_t *motor, const foc_hal_t *hal_interface)
{
    // 绑定定时器
    // motor->htim = htim;
    motor->hal = *hal_interface; // 绑定HAL
    motor->hal.init(motor->num);
    // d轴电流环参数初始化
    motor->pi_d = (foc_pid_t){
        .kp = PI_KP_D,
        .ki = PI_KI_D,
        .limit = PI_LIMIT,
        .target = 0.0f};
        
    // q轴电流环参数初始化
    motor->pi_q = (foc_pid_t){
        .kp = PI_KP_Q,
        .ki = PI_KI_Q,
        .limit = PI_LIMIT,
        .target = 0.0f};

    // 速度PI参数初始化
    motor->pi_speed = (foc_pid_t){
        .kp = PI_KP_SPEED,
        .ki = PI_KI_SPEED,
        .limit = PI_LIMIT_SPEED,
        .target = 0,
        .integral = 0.0f};

    // 初始占空比
    motor->pwm = (foc_pwm_t){
        .duty_u = PWM_ARR / 2,
        .duty_v = PWM_ARR / 2,
        .duty_w = PWM_ARR / 2,
    };

    motor->i_ab_pre = (foc_ab_t){
        .alpha = 0.0f,
        .beta = 0.0f};

    motor->e_ab = (foc_ab_t){
        .alpha = 0.0f,
        .beta = 0.0f};

    motor->i_cali_uvw = (foc_uvw_t){
        .u = 0.0f,
        .v = 0.0f,
        .w = 0.0f,
    };

    motor->pi_pll.integral = 0.0f;
    motor->speed_observer = 0.0f;
    motor->target_speed = 0.0f;
    motor->theta = 0.0f; // 确保起始角度从0开始
    motor->speed_ramp_target = 0.0f;
    motor->speed_sign = 1.0;
    motor->state_timer = 0;
    motor->state = FOC_OK;
    return FOC_OK;
}

foc_state_t Foc_Deinit(foc_handle_t *motor)
{
    return FOC_OK;
}

// float speed_diff;
/**
 * @brief foc主循环，用于在定时器中调用，采用了分段式处理，开环强拉到闭环状态
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_Loop(uint8_t motor_num)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];

    if (motor->init_done != 1)
        return FOC_ERR_NOT_INIT; // 初始化未完成，直接返回

    motor->hal.adc_get_value(motor_num, &motor->i_adc_u, &motor->i_adc_v, &motor->i_adc_w);

    switch (motor->mode)
    {
    case MOTOR_STATE_IDLE: // 空状态
        // ✅ 每个控制周期都刷新为零电压，防止PWM寄存器残留
        motor->hal.pwm_set_duty(motor->num,
            PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);

        if (fabsf(motor->target_speed) > SPEED_START_THRESHOLD)
        {
            motor->theta             = 0.0f;
            motor->pi_pll.integral   = 0.0f;
            motor->pi_d.integral     = 0.0f;
            motor->pi_q.integral     = 0.0f;
            motor->pi_speed.integral = 0.0f;
            motor->state_timer       = 0;
            motor->hal.drv_enable(motor_num);
            motor->mode = MOTOR_STATE_ALIGN;
        }
        break;

    case MOTOR_STATE_ALIGN:
        // 1. 定位阶段：给 D 轴施加固定电压，Q 轴为 0，强制转子对齐到 0 度
        motor->u_dq.d = 2.0f; // 2V 左右，根据电机阻抗微调
        motor->u_dq.q = 0.0f;
        motor->theta = 0.0f;
        motor->pi_pll.integral = 0;
        motor->theta_Observer = 0.0f;
        FOC_InvPark_Transform(motor);
        FOC_SVPWM_Generate(motor);

        // ALIGN 期间若遥控器归零 → 回 IDLE
        if (fabsf(motor->target_speed) < SPEED_START_THRESHOLD)
        {
            motor->state_timer = 0;
            motor->mode = MOTOR_STATE_IDLE;
            break;
        }

        motor->state_timer++;

        if (motor->state_timer > 4000)
        { // 持续约 100ms (假设频率 18kHz)
            motor->state_timer = 0;
            motor->pi_pll.integral = motor->target_speed > 0 ? OPEN_ELEC_SPEED : -OPEN_ELEC_SPEED;
            motor->mode = MOTOR_STATE_OPEN;
        }
        break;

    case MOTOR_STATE_OPEN:
        // 2. 开环启动：按设定转速匀速旋转
        Foc_Open_Loop(motor, TS);

        // ALIGN 期间若遥控器归零 → 回 IDLE
        if (fabsf(motor->target_speed) < SPEED_START_THRESHOLD)
        {
            motor->hal.pwm_set_duty(motor->num,
                PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);
            motor->e_ab.alpha     = 0.0f;
            motor->e_ab.beta      = 0.0f;
            motor->i_ab_hat.alpha = 0.0f;
            motor->i_ab_hat.beta  = 0.0f;
            motor->speed_observer = 0.0f;
            motor->speed_sign     = 1.0f;
            motor->state_timer    = 0;
            motor->mode = MOTOR_STATE_IDLE;
            break;
        }

        motor->state_timer++;

        #ifndef ONLY_OPEN_LOOP
        // 观测速度与开环速度接近才切换
        float speed_rpm = motor->speed_observer * 60.0f / _2_PI_POLE_PAIRS; // 把电角速度转换为圈每秒
        float speed_diff = fabsf(fabsf(speed_rpm) - fabsf(OPEN_LOOP_SPEED_RPM));
        if (motor->state_timer > 8500 && speed_diff < OPEN_LOOP_SPEED_RPM * 0.1f && fabs(angle_error) < 0.1f)
        {
            motor->pi_pll.integral = motor->target_speed > 0 ? fabsf(motor->speed_observer) : -fabsf(motor->speed_observer);
            motor->pi_d.integral = 0.0f;
            motor->pi_d.output = 0.0f;

            motor->pi_speed.integral = 0.0f; // 初始驱动力
            motor->pi_speed.output = 6.7f;

            motor->pi_q.integral = PWM_VBUS * 0.2f; // 给个初始积分，约2.4V
            motor->pi_q.output = PWM_VBUS * 0.25f;

            if(motor->target_speed > 0)
            {
                motor->speed_ramp_target = fabsf(motor->speed_observer) * 60.0f / (_2_PI * POLE_PAIRS) + 50.0f;
                // motor->pi_pll.integral = OPEN_ELEC_SPEED;
            } 
            else
            {
                motor->speed_ramp_target = fabsf(motor->speed_observer) * 60.0f / (_2_PI * POLE_PAIRS) + 50.0f;
                motor->speed_ramp_target = -motor->speed_ramp_target;
                // motor->pi_pll.integral = -OPEN_ELEC_SPEED;
            }
                
            motor->theta_Observer = motor->theta;
            motor->PI_Speed_cnt = 0;
            motor->close_cnt = 0;

            motor->mode = MOTOR_STATE_CLOSE;
        }
        #endif

        break;

    case MOTOR_STATE_CLOSE:
        // 3. 闭环运行
        // 闭环期间若遥控器归零 → 回 IDLE
        if (fabsf(motor->target_speed) < SPEED_START_THRESHOLD)
        {
            // 先归零PWM再切状态
            motor->hal.pwm_set_duty(motor->num,
                PWM_ARR / 2, PWM_ARR / 2, PWM_ARR / 2);
            // 同时清空SMO残留状态，防止下次重启时观测器从错误状态收敛
            motor->e_ab.alpha     = 0.0f;
            motor->e_ab.beta      = 0.0f;
            motor->i_ab_hat.alpha = 0.0f;
            motor->i_ab_hat.beta  = 0.0f;
            motor->speed_observer = 0.0f;
            motor->speed_sign     = 1.0f;
            motor->mode = MOTOR_STATE_IDLE;
            break;
        }

        {
            float run_sign    = (motor->speed_ramp_target >= 0.0f) ? 1.0f : -1.0f;
            float target_sign = (motor->target_speed      >= 0.0f) ? 1.0f : -1.0f;
            if (run_sign != target_sign)
            {
                // 关闭PWM输出，等惯性停下
                motor->hal.pwm_set_duty(motor->num, PWM_ARR/2, PWM_ARR/2, PWM_ARR/2);
                motor->mode = MOTOR_STATE_IDLE;
                break;
            }
        }
        Foc_Close_Loop(motor, TS);
        motor->mode = MOTOR_STATE_CLOSE;
        break;
    }
    return FOC_OK;
}

/**
 * @brief 开环控制
 * 
 * @param motor 
 * @param dt 
 * @return foc_state_t 
 */
foc_state_t Foc_Open_Loop(foc_handle_t *motor, float dt)
{
    foc_state_t foc_state = FOC_OK;
    // 1. 电流校准（减去零点偏移）
    motor->i_uvw.v = -(float)((int32_t)motor->i_adc_u - motor->i_cali_uvw.u) * CURRENT_SCALE;
    motor->i_uvw.u = (float)((int32_t)motor->i_adc_w - motor->i_cali_uvw.w) * CURRENT_SCALE;
    // 2. 电流限幅（保护电机）
    // motor->i_uvw.v = (motor->i_uvw.v > CURRENT_LIMIT) ? CURRENT_LIMIT : (motor->i_uvw.v < -CURRENT_LIMIT) ? -CURRENT_LIMIT
    //                                                                                                       : motor->i_uvw.v;
    // motor->i_uvw.u = (motor->i_uvw.u > CURRENT_LIMIT) ? CURRENT_LIMIT : (motor->i_uvw.u < -CURRENT_LIMIT) ? -CURRENT_LIMIT
    //                                                                                                       : motor->i_uvw.u;

    if(fabs(motor->i_uvw.v) >= CURRENT_LIMIT)
    {
        motor->i_uvw.v = motor->i_uvw.v > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    if(fabs(motor->i_uvw.u) >= CURRENT_LIMIT)
    {
        motor->i_uvw.u = motor->i_uvw.u > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    if(fabs(motor->i_uvw.v) >= CURRENT_LIMIT)
    {
        motor->i_uvw.v = motor->i_uvw.v > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    // 3. Clark变换，
    FOC_Clark_Transform(motor);
    // 4. smo观测器推算转子位置，得到电角度和转速
    SMO_Observer(motor, dt, MOTOR_STATE_OPEN);
    FOC_Park_Transform(motor);

    motor->u_dq.d = 0.0f;             // 通常d轴电流设为0以获得最大转矩效率
    motor->u_dq.q = PWM_VBUS * 0.45f; // q轴电压与期望转矩相关, 电压范围为母线电压的30%~50%

    if(motor->target_speed > 0)
        motor->theta += OPEN_ELEC_SPEED * dt;     // 电角度递增
    else if (motor->target_speed < 0)
        motor->theta -= OPEN_ELEC_SPEED * dt;

    motor->theta = fmod(motor->theta, _2_PI); // 限制在0~2π范围内，保持归一化
    if (motor->theta < 0)
        motor->theta += _2_PI; // 处理负数情况

    // 3. 反Park变换
    FOC_InvPark_Transform(motor);
    // 4. SVPWM生成并输出
    FOC_SVPWM_Generate(motor);
    return foc_state;
}

/**
 * @brief 开环控制测试代码
 * 
 * @param motor 
 * @param dt 
 * @return foc_state_t 
 */
foc_state_t Foc_Open_Loop_Test(foc_handle_t *motor, float dt)
{
    // 1. 电流校准（减去零点偏移）
    motor->i_uvw.v = -(float)((int32_t)motor->i_adc_u - motor->i_cali_uvw.u) * CURRENT_SCALE;
    motor->i_uvw.u = (float)((int32_t)motor->i_adc_w - motor->i_cali_uvw.w) * CURRENT_SCALE;
    // 2. 电流限幅（保护电机）
    motor->i_uvw.v = (motor->i_uvw.v > CURRENT_LIMIT) ? CURRENT_LIMIT : (motor->i_uvw.v < -CURRENT_LIMIT) ? -CURRENT_LIMIT
                                                                                                          : motor->i_uvw.v;
    motor->i_uvw.u = (motor->i_uvw.u > CURRENT_LIMIT) ? CURRENT_LIMIT : (motor->i_uvw.u < -CURRENT_LIMIT) ? -CURRENT_LIMIT
                                                                                                          : motor->i_uvw.u;
    // 3. Clark变换，
    FOC_Clark_Transform(motor);
    // 4. smo观测器推算转子位置，得到电角度和转速
    SMO_Observer(motor, dt, MOTOR_STATE_OPEN);

    // 固定角度为0，不依赖观测器
    float control_theta = 0.0f;
    float cos_th = cosf(control_theta);
    float sin_th = sinf(control_theta);

    // Park变换
    motor->i_dq.d = motor->i_ab.alpha * cos_th + motor->i_ab.beta * sin_th;
    motor->i_dq.q = -motor->i_ab.alpha * sin_th + motor->i_ab.beta * cos_th;

    // 阶跃信号
    vofa_cnt++;
    if(vofa_cnt < 8500)
        motor->pi_q.target = 2.0f;
    else if(vofa_cnt < 17000)
        motor->pi_q.target = 4.0f;
    else
        vofa_cnt = 0;
    motor->pi_d.target = 0.0f;

    motor->pi_d.feedback = motor->i_dq.d;
    motor->pi_q.feedback = motor->i_dq.q;
    motor->pi_d.limit = PI_LIMIT;
    motor->pi_q.limit = PI_LIMIT;

    FOC_PI_Regulator(&motor->pi_d, dt); // pid计算
    FOC_PI_Regulator(&motor->pi_q, dt); // pid计算
    motor->u_dq.d = motor->pi_d.output;
    motor->u_dq.q = motor->pi_q.output;

    motor->theta = 0;

    // 3. 反Park变换
    FOC_InvPark_Transform(motor);
    // 4. SVPWM生成并输出
    FOC_SVPWM_Generate(motor);
    return FOC_OK;
}

/**
 * @brief 闭环控制
 * 
 * @param motor 
 * @param dt 
 * @return foc_state_t 
 */
foc_state_t Foc_Close_Loop(foc_handle_t *motor, float dt)
{
    foc_state_t foc_state = FOC_OK;
    // pi输出限幅缓启动
    float pi_limit;
    if(motor->close_cnt < 500)           // 缩短到500拍（0.04秒）
    {
        motor->close_cnt++;
        pi_limit = 3.0f + motor->close_cnt * 0.007f;  // 3V→6.5V，0.04秒到位
    }
    else
        pi_limit = PI_LIMIT;

    // motor->close_cnt++;

    // float blend = motor->close_cnt / 1000.0f;

    // if(blend > 1.0f) blend = 1.0f;

    // motor->theta = (1.0f - blend) * motor->theta + blend * motor->theta_Observer;

    // 1. 电流校准（减去零点偏移）
    motor->i_uvw.v = -(float)((int32_t)motor->i_adc_u - motor->i_cali_uvw.u) * CURRENT_SCALE;
    motor->i_uvw.u = (float)((int32_t)motor->i_adc_w - motor->i_cali_uvw.w) * CURRENT_SCALE;
    // 2. 电流限幅（保护电机）
    // motor->i_uvw.v = (motor->i_uvw.v > CURRENT_LIMIT) ? CURRENT_LIMIT : (motor->i_uvw.v < -CURRENT_LIMIT) ? -CURRENT_LIMIT
    //                                                                                                       : motor->i_uvw.v;
    // motor->i_uvw.u = (motor->i_uvw.u > CURRENT_LIMIT) ? CURRENT_LIMIT : (motor->i_uvw.u < -CURRENT_LIMIT) ? -CURRENT_LIMIT
    //                                                                                                       : motor->i_uvw.u;

    if(fabs(motor->i_uvw.v) >= CURRENT_LIMIT)
    {
        motor->i_uvw.v = motor->i_uvw.v > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    if(fabs(motor->i_uvw.u) >= CURRENT_LIMIT)
    {
        motor->i_uvw.u = motor->i_uvw.u > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    if(fabs(motor->i_uvw.v) >= CURRENT_LIMIT)
    {
        motor->i_uvw.v = motor->i_uvw.v > 0 ? CURRENT_LIMIT : -CURRENT_LIMIT;
        foc_state = FOC_ERR_OVERCURRENT;
    }

    // 3. Clark变换，
    FOC_Clark_Transform(motor);
    // 4. MRAS观测器推算转子位置，得到电角度和转速
    SMO_Observer(motor, dt, MOTOR_STATE_CLOSE);

    float control_theta = motor->theta;
    control_theta = fmodf(control_theta, _2_PI);
    if (control_theta < 0)
        control_theta += _2_PI;

    // 保存当前角度备份，确保 Park 和 InvPark 使用完全一致的角度
    float cos_th = cosf(control_theta);
    float sin_th = sinf(control_theta);
    // 5. Park变换
    motor->i_dq.d = motor->i_ab.alpha * cos_th + motor->i_ab.beta * sin_th;
    motor->i_dq.q = -motor->i_ab.alpha * sin_th + motor->i_ab.beta * cos_th;

    // 6. 速度环PI调节
    #ifdef FOC_SPEED_CONTROL
    
    motor->PI_Speed_cnt++;
    if (motor->PI_Speed_cnt >= 10)
    {
        motor->PI_Speed_cnt = 0;

        if(motor->target_speed > motor->speed_ramp_target)
        {
            motor->speed_ramp_target += SPEED_RAMP_RATE * dt * 10.0f;
            if (motor->speed_ramp_target > motor->pi_speed.target)
                motor->speed_ramp_target = motor->pi_speed.target;
        }
        else if(motor->target_speed < motor->speed_ramp_target)
        {
            motor->speed_ramp_target -= SPEED_RAMP_RATE * dt * 10.0f;
            if (motor->speed_ramp_target < motor->pi_speed.target)
                motor->speed_ramp_target = motor->pi_speed.target;
        }

        float spd_err = motor->speed_ramp_target - motor->speed;

        motor->pi_speed.integral += PI_KI_SPEED * spd_err * (dt * 10.0f); // 速度积分

        motor->pi_speed.integral = fmaxf(-PI_LIMIT_SPEED, fminf(PI_LIMIT_SPEED, motor->pi_speed.integral)); // 速度积分限幅

        float iq_p = PI_KP_SPEED * spd_err; // 速度环比例
        float iq_cmd = iq_p + motor->pi_speed.integral;

        iq_cmd = fmaxf(-PI_LIMIT_SPEED, fminf(PI_LIMIT_SPEED, iq_cmd));

        motor->pi_speed.output = iq_cmd;
        motor->pi_q.target = iq_cmd;
    }
    motor->pi_d.target = 0.0f; // id始终为0

    FOC_FieldWeakening(motor, TS);    

    #endif

    #ifdef FOC_CLOSE_I_DEBUG
    vofa_cnt++;
    if(vofa_cnt >= 17000 && vofa_cnt < 34000)
    {
        motor->pi_q.target = 4.0f; // 速度环输出→iq目标
    }
    else if(vofa_cnt > 0 && vofa_cnt < 17000) 
        motor->pi_q.target = 2.0f; // 速度环输出→iq目标
    else 
        vofa_cnt = 0;
    motor->pi_d.target = 0.0f;                   // id始终为0
    #endif

    // 6. 电流环PI调节
    motor->pi_d.feedback = motor->i_dq.d;
    motor->pi_q.feedback = motor->i_dq.q;
    motor->pi_d.limit = pi_limit;
    motor->pi_q.limit = pi_limit;

    FOC_PI_Regulator(&motor->pi_d, dt); // pid计算
    FOC_PI_Regulator(&motor->pi_q, dt); // pid计算
    motor->u_dq.d = motor->pi_d.output;
    motor->u_dq.q = motor->pi_q.output;

    // ------------------------------建议：在调试阶段将电压限幅设得极低（如 1.5V），保护驱动板----------------------------
    // float voltage_limit = 6.0f;
    // if (motor->u_dq.d > voltage_limit)
    //     motor->u_dq.d = voltage_limit;
    // if (motor->u_dq.d < -voltage_limit)
    //     motor->u_dq.d = -voltage_limit;
    // if (motor->u_dq.q > voltage_limit)
    //     motor->u_dq.q = voltage_limit;
    // if (motor->u_dq.q < -voltage_limit)
    //     motor->u_dq.q = -voltage_limit;
    //----------------------------------------------------------------------------------------------------------------

    // 7. 反Park变换
    motor->u_ab.alpha = motor->u_dq.d * cos_th - motor->u_dq.q * sin_th;
    motor->u_ab.beta = motor->u_dq.d * sin_th + motor->u_dq.q * cos_th;

    // 8. SVPWM生成并输出
    FOC_SVPWM_Generate(motor);
    return foc_state;
}

/**
 * @brief foc失能
 * 
 * @param motor 
 * @return foc_state_t 
 */
foc_state_t Foc_Stop(uint8_t motor_num)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];
    motor->target_speed = 0;
    motor->hal.drv_disable(motor_num); // 驱动失能
    return FOC_OK;
}

/**
 * @brief 设置目标速度
 * 
 * @param motor 
 * @param speed 
 * @return foc_state_t 
 */
foc_state_t Foc_Set_Speed(uint8_t motor_num, float speed)
{
    foc_handle_t *motor = &FOC_Motor[motor_num];
    motor->target_speed = speed;
    motor->pi_speed.target = speed;
    return FOC_OK;
}




