#ifndef __H_TMC2209_H
#define __H_TMC2209_H

#include "gpio.h"
#include "math.h"
#include "tim.h"

typedef enum {
    Constant_speed = 1, // 恒速模式
    Constant_step = 2,  // 定步模式
    STOP_mode = 3       // 停止模式
} MotorMode;

typedef struct {
    uint8_t mode;
    uint8_t en;
    GPIO_PinState dir;
    uint16_t hz;
    volatile uint32_t current_step;
    uint32_t target_step;
    uint32_t arr;
    MotionStep steps;
    TrapezoidVelocity velocity;
} MotorStruct;

// 外部声明全局数组
#define MOTOR_NUM 3  // 电机编号从 1~6，索引0不使用
/* 定时器句柄数组，由 H_Tmc2209.c 定义为 motor_tim，
   头文件以前误写为 motor_htim，导致链接错误。 */
extern TIM_HandleTypeDef *motor_tim[];
extern uint32_t motor_channel[MOTOR_NUM];
extern GPIO_TypeDef* motor_en_port[MOTOR_NUM];
extern uint16_t motor_en_pin[MOTOR_NUM];
extern uint16_t pulse_percircle;
extern MotorStruct Motor[10];

// 如果未定义 MOTOR_COUNT，则定义为 6
#ifndef MOTOR_COUNT
#define MOTOR_COUNT 6
#endif

// 定时器输入时钟：72 MHz，预分频 72 → 定时器计数频率 = 10 kHz
#define TIMER_CLK_HZ 275000000u

void stepper_init(MotorStruct *Motor, uint16_t v_start, uint16_t v_max, uint16_t acc, uint16_t steps);

void Motor_Set(uint8_t num, uint8_t mode, GPIO_PinState dir, uint16_t pulse_num, uint16_t pulse_hz, uint16_t vstart, uint16_t vmax, uint16_t vacc); // 模式1定速 模式2定步 模式3停止
void Motor_SetSpeed(uint8_t num);
void Motor_Stop(uint8_t num);
uint32_t Motor_GetStep(uint8_t num);

#endif /* __H_TMC2209_H__ */