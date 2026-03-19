#define MOTOR_COUNT 2

#include "H_Tmc2209.h"
/******************步进电机驱动板级设置部分******************/


// GPIO 端口和引脚定义数组（下标从 1 开始，0 占位）
GPIO_TypeDef *en_ports[MOTOR_COUNT + 1] = {
    NULL,
    EN1OUT_GPIO_Port, EN2OUT_GPIO_Port
};

uint16_t en_pins[MOTOR_COUNT + 1] = {
    0,
    EN1OUT_Pin, EN2OUT_Pin
};

GPIO_TypeDef *dir_port[MOTOR_COUNT + 1] = {
    NULL,
    DIR1OUT_GPIO_Port, DIR2OUT_GPIO_Port
};

uint16_t dir_pins[MOTOR_COUNT + 1] = {
    0,
    DIR1OUT_Pin, DIR2OUT_Pin
};

// 定时器和通道定义数组（根据你当前使用的定时器顺序）
TIM_HandleTypeDef *motor_tim[MOTOR_COUNT + 1] = {
    NULL,
    &htim2, &htim5
};

uint32_t motor_channel[MOTOR_COUNT + 1] = {
    0,
    TIM_CHANNEL_1, TIM_CHANNEL_1
};
/*******************************************************/

uint16_t pulse_percircle = 3200;  //转一圈走的步数，产生的脉冲数

MotorStruct Motor[10] = {0};

void stepper_init(MotorStruct *motor, uint16_t v_start, uint16_t v_max, uint16_t acc, uint16_t steps)
{
    motor->velocity.v_start = v_start;
    motor->velocity.v_max = v_max;
    motor->velocity.acc = acc;
    motor->velocity.total_steps = motor->target_step = steps;
    motor->steps = calc_trapezoid_profile(motor->velocity);
    motor->current_step = 0;
}

void Motor_Set(uint8_t num, uint8_t mode, GPIO_PinState dir, uint16_t pulse_num, uint16_t pulse_hz, uint16_t vstart, uint16_t vmax, uint16_t vacc) // 模式1定速 模式2定步 模式3停止
{
    Motor[num].en = ENABLE;
    switch (mode)
    {
    case Constant_speed: // 定速模式
        Motor[num].mode = Constant_speed;
        Motor[num].hz = pulse_hz;
        break;
    case Constant_step: // 定步模式
        Motor[num].mode = Constant_step;
        stepper_init(&Motor[num], vstart, vmax, vacc, pulse_num);
        Motor[num].hz = pulse_hz;
        // Motor[num].hz = get_step_speed(Motor[num].current_step, Motor[num].steps, Motor[num].velocity);
        break;
    case STOP_mode: // 停止模式
        Motor[num].mode = STOP_mode;
        //Motor[num].en = 0;
        HAL_TIM_Base_Stop_IT(motor_tim[num]);
        HAL_TIM_PWM_Stop(motor_tim[num], motor_channel[num]);
        break;
    default:
        break;
    }
    Motor[num].dir = dir;
    Motor[num].arr = (TIMER_CLK_HZ / Motor[num].hz) - 1;
    Motor_SetSpeed(num);
}

void Motor_SetSpeed(uint8_t num)
{
    if (num == 0 || num > MOTOR_COUNT)
        return;

    // 设置使能和方向
    HAL_GPIO_WritePin(en_ports[num], en_pins[num], (Motor[num].en == ENABLE) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dir_port[num], dir_pins[num], Motor[num].dir);

    // 设置定时器参数
    __HAL_TIM_SET_AUTORELOAD(motor_tim[num], Motor[num].arr);
    __HAL_TIM_SET_COMPARE(motor_tim[num], motor_channel[num], Motor[num].arr / 2);
    __HAL_TIM_SET_COUNTER(motor_tim[num], 0);

    // 启动 PWM 和中断
    HAL_TIM_PWM_Start(motor_tim[num], motor_channel[num]);
    HAL_TIM_Base_Start_IT(motor_tim[num]);
}

uint32_t Motor_GetStep(uint8_t num) // 细分步数为1  脉冲大概为180度
{                                   // 电机脉冲值获取
    switch (num)
    {
    case 1:
        return Motor[1].current_step;
        
    case 2:
        return Motor[2].current_step;
        
    case 3:
        return Motor[3].current_step;
        
    case 4:
        return Motor[4].current_step;
        
    case 5:
        return Motor[5].current_step;
        
    case 6:
        return Motor[6].current_step;
        
    default:
        return 0;
    }
}

void Motor_Stop(uint8_t num)
{
    HAL_TIM_Base_Stop_IT(motor_tim[num]);
    HAL_TIM_PWM_Stop(motor_tim[num], motor_channel[num]);
}

