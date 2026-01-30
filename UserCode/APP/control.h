#ifndef __CONTROL_H
#define __CONTROL_H

#include "H_Tmc2209.h"
#include "gpio.h"
#include "main.h"
#include "OLED.h"

typedef enum 
{
    TANK_EMPTY = 0,     //已空，空端微动开关触发
    TANK_DRAINING,
    TANK_MID,           //半满不空
    TANK_FILLING,
    TANK_FULL,          //已满，满端微动开关触发
}water_tank_state_t;

typedef struct 
{
    //水舱连接的步进电机的编号
    uint8_t connect_motor_num;

    //水舱的状态
    water_tank_state_t state;

    //初始水量，单位：ml
    float start_water_volume;
    //当前水量，单位：ml
    float now_water_volume;
    //目标水量，单位：ml
    float target_water_volume;
    //吞吐水速度,单位：ml/s
    float water_velocity;

    //对应的步进电机在运动时已完成的步数
    uint16_t last_finished_steps; 

    //满端限位
    volatile uint8_t event_full;  
    //空端限位
    volatile uint8_t event_empty; 


}water_tank_t;

extern water_tank_t tank_front;  
extern water_tank_t tank_rear;  

void Drone_Rising(void);
void Water_Tank_Init(void);
void Water_Tank_Filling(water_tank_t *tank, float delta_xML);
void Water_Tank_Draining(water_tank_t *tank, float delta_xML);
void Water_Tank_Fill_To(water_tank_t *tank, float target_xML);
void Water_Tank_Drain_TO(water_tank_t *tank, float target_xML);
void Water_Tank_State_Update(water_tank_t *tank);
//void Water_Tank_Front_Get_Volume(void);
void Water_Tank_Update_Handler(water_tank_t *tank);


#endif