#include "control.h"

#define Water_Volume_Percircle 1.0f  //步进电机每转一圈对应的水量变化值，单位：ml

water_tank_t tank_front;  
water_tank_t tank_rear;   

void Load_Water_Volume(water_tank_t *tank);
void Water_Tank_Filling(water_tank_t *tank, float delta_xML);
void Water_Tank_Draining(water_tank_t *tank, float delta_xML);

void Water_Tank_Init(void)
{
    // 链接步进电机
    tank_front.connect_motor_num = 1;   
    tank_rear.connect_motor_num = 2;

    // 初始水量先设置为0，后面可以写成从flash中读
    tank_front.start_water_volume = 0;  
    tank_rear.start_water_volume = 0;
    
    // 把当前水量设置为初始水量
    tank_front.now_water_volume = tank_front.start_water_volume; 
    tank_rear.now_water_volume = tank_rear.start_water_volume;

    // 设置吞吐水速度
    tank_front.water_velocity = 1.0;
    tank_rear.water_velocity = 1.0;
}

// /**
//  * @brief 水舱注水
//  * @param tank 指定水舱
//  * @param delta_xML 注水量>=0，单位：ml
//  */
// void Water_Tank_Filling(water_tank_t *tank, float delta_xML)
// {
//     //传入负值时，调用排水函数
//     if(delta_xML < 0.0f)
//     {
//         Water_Tank_Draining(tank, -delta_xML);
//         return;
//     }

//     //满状态时禁止进水动作
//     if (tank->state == TANK_FULL)
//     {
//         return;
//     }
//     //进水状态时禁止叠加进水
//     if(tank->state == TANK_FILLING)
//     {
//         return;
//     }

//     //开始进水动作时，如果是空状态，先回到半满状态
//     if(tank->state == TANK_EMPTY)
//     {
//         tank->state = TANK_MID;
//     }

//     tank->target_water_volume = tank->now_water_volume + delta_xML; //根据 Δ水量 计算 目标水量
//     Load_Water_Volume(tank);
// }

// /**
//  * @brief 水舱排水水
//  * @param tank 指定水舱
//  * @param delta_xML 排水量>=0，单位：ml
//  */
// void Water_Tank_Draining(water_tank_t *tank, float delta_xML)
// {
//     //传入负值时，调用注水函数
//     if(delta_xML < 0.0f)
//     {
//         Water_Tank_Draining(tank, -delta_xML);
//         return;
//     }

//     //空状态时禁止排水水动作
//     if (tank->state == TANK_EMPTY)
//     {
//         return;
//     }
//     //排水状态时禁止叠加排水
//     if(tank->state == TANK_DRAINING)
//     {
//         return;
//     }

//     //开始排水动作时，如果是满状态，先回到半满状态
//     if(tank->state == TANK_FULL)
//     {
//         tank->state = TANK_MID;
//     }

//     tank->target_water_volume = tank->now_water_volume + delta_xML; //根据 Δ水量 计算 目标水量
//     tank->state = TANK_FILLING;
//     Load_Water_Volume(tank);
    
// }
void Water_Tank_Filling(water_tank_t *tank, float delta_xML)
{
    if (delta_xML <= 0.0f) return;
    if (tank->state == TANK_FULL) return;

    tank->target_water_volume = tank->now_water_volume + delta_xML;
    
    // 设置状态，Load_Water_Volume 内部会启动电机
    tank->state = TANK_FILLING; 
    Load_Water_Volume(tank);
}

void Water_Tank_Draining(water_tank_t *tank, float delta_xML)
{
    if (delta_xML <= 0.0f) return;
    if (tank->state == TANK_EMPTY) return;

    tank->target_water_volume = tank->now_water_volume - delta_xML;
    
    // 设置状态
    tank->state = TANK_DRAINING;
    Load_Water_Volume(tank);
}

void Water_Tank_Fill_To(water_tank_t *tank, float target_xML)
{
    if (tank->state != TANK_FULL)
    {
        tank->target_water_volume = target_xML;
        Load_Water_Volume(tank);
        
    }
}

void Water_Tank_Drain_TO(water_tank_t *tank, float target_xML)
{
    if (tank->state != TANK_EMPTY)
    {
        tank->target_water_volume = target_xML;
        Load_Water_Volume(tank);
        
    }
}

// void Water_Tank_Front_Get_Volume(void)
// {
    
//     static uint32_t last_motor_step;
//     static uint32_t now_motor_step;
//     static uint32_t delta_step = 0;
//     static float delta_volume = 0.0f;

//     now_motor_step = Motor_GetStep(tank_front.connect_motor_num);
    

//     if(now_motor_step == 0)   //步进电机走完步数后会重置为0，此后步进电机不再动，水量不在变化
//     {
//         last_motor_step = 0;
//         return;
//     }
//     delta_step = now_motor_step - last_motor_step;
//     last_motor_step = now_motor_step;

//     delta_volume = (float)delta_step / (float)pulse_percircle * Water_Volume_Percircle; //把步数转化为水量

//     if(tank_front.state == TANK_DRAINING)
//     {
//         tank_front.now_water_volume = tank_front.now_water_volume - delta_volume;
//     }
//     if(tank_front.state == TANK_FILLING)
//     {
//         tank_front.now_water_volume = tank_front.now_water_volume + delta_volume;
//     }
//     tank_front.now_water_volume = tank_front.now_water_volume + delta_volume;
    
// }

// void Water_Tank_Rear_Get_Volume(void)
// {
    
//     static uint32_t last_motor_step;
//     static uint32_t now_motor_step;
//     static uint32_t delta_step = 0;
//     static float delta_volume = 0.0f;

//     now_motor_step = Motor_GetStep(tank_rear.connect_motor_num);
    

//     if(now_motor_step == 0)   //步进电机走完步数后会重置为0，此后步进电机不再动，水量不在变化
//     {
//         last_motor_step = 0;
//         return;
//     }
//     delta_step = now_motor_step - last_motor_step;
//     last_motor_step = now_motor_step;

//     delta_volume = (float)delta_step / (float)pulse_percircle * Water_Volume_Percircle; //把步数转化为水量

//     if(tank_rear.state == TANK_DRAINING)
//     {
//         tank_rear.now_water_volume = tank_rear.now_water_volume - delta_volume;
//     }
//     if(tank_rear.state == TANK_FILLING)
//     {
//         tank_rear.now_water_volume = tank_rear.now_water_volume + delta_volume;
//     }
//     tank_rear.now_water_volume = tank_rear.now_water_volume + delta_volume;
// }

// void Water_Tank_State_Update(water_tank_t *tank)
// {
//     if (tank->event_full)
//     {
//         tank->event_full = 0;
//         tank->state = TANK_FULL;

//         // 立即停止吸水
//         Motor_Stop(tank->connect_motor_num);
        
//     }

//     if (tank->event_empty)
//     {
//         tank->event_empty = 0;
//         tank->state = TANK_EMPTY;

//         // 立即停止排水
//         Motor_Stop(tank->connect_motor_num);
//     }

//     // 如果既不空也不满 -> MID 
//     if (tank->state != TANK_FULL && tank->state != TANK_EMPTY)
//     {
//         tank->state = TANK_MID;
//     }
// }

/**
 * @brief 监控水舱实时变化
 * @param tank 指定水舱
 */
void Water_Tank_Update_Handler(water_tank_t *tank)
{
    /*满/空事件触发处理（最高优先级状态切换*/
    if (tank->event_full)
    {
        tank->event_full = 0;                // 清空满水事件
        tank->state = TANK_FULL;             // 设置满水状态
        Motor_Stop(tank->connect_motor_num); // 立即停止电机
        tank->last_finished_steps = 0;       // 水舱对应的步进电机实时完成的步数清零
        return; 
    }
    if (tank->event_empty)
    {
        tank->event_empty = 0;               // 清空空水事件
        tank->state = TANK_EMPTY;            // 设置空水状态
        tank->now_water_volume = 0;          // 物理校准：碰到空限位，水量强制归零
        Motor_Stop(tank->connect_motor_num); // 立即停止电机
        tank->last_finished_steps = 0;       // 水舱对应的步进电机实时完成的步数清零
        return;
    }

    /*实时水量更新*/
    uint32_t current_finished_steps = 0;  //获取水舱对应的步进电机的实时完成的步数
    if(tank->connect_motor_num == 1)      current_finished_steps = Motor_GetStep(1);
    else if(tank->connect_motor_num == 2) current_finished_steps = Motor_GetStep(2); 

    if (current_finished_steps > 0)       //当步进电机有已完成的步数时，就更新水舱水量
    {
        uint32_t delta_step = 0;          //Δ步数
        float delta_volume = 0.0f;        //Δ水量

        delta_step = current_finished_steps - tank->last_finished_steps; //Δ步数 = 当前已完成的步数 - 上次已完成的步数
        delta_volume = (float)delta_step / (float)pulse_percircle * Water_Volume_Percircle; //Δ步数 ==> Δ水量

        if(tank_front.state == TANK_DRAINING) //排水状态，就减去Δ水量
        {
            tank_front.now_water_volume = tank_front.now_water_volume - delta_volume;
        }
        else if(tank_front.state == TANK_FILLING) //进水状态，就加上Δ水量
        {
            tank_front.now_water_volume = tank_front.now_water_volume + delta_volume;
        }
        tank->last_finished_steps = current_finished_steps; //为下次做准备
    }
    else
    {
        // 动作完成后的状态切换
        // 如果电机停了，且目前是注/排状态，切换回 MID，同时清零上次完成的步数
        if (tank->state == TANK_FILLING || tank->state == TANK_DRAINING)
        {
            tank->state = TANK_MID;
        }
        tank->last_finished_steps = 0;
    }
}
void Load_Water_Volume(water_tank_t *tank)
{
    //目标水量 - 当前水量 = Δ水量
    float delta_water_volume = 0.0;
    delta_water_volume = tank->target_water_volume - tank->now_water_volume;

    //Δ水量 ==> Δstep
    int32_t delta_step = 0;
    delta_step = delta_water_volume  * pulse_percircle / Water_Volume_Percircle;

    //水速 ==> 步进电机脉冲频率
    uint32_t step_velocity = 0;
    step_velocity = 3200 * tank->water_velocity / Water_Volume_Percircle;

    //设置步进电机
    if(delta_step > 0)
        Motor_Set(tank->connect_motor_num, 2, 1, delta_step, step_velocity, 400, 2400, 800);
    else 
        Motor_Set(tank->connect_motor_num, 2, 0, delta_step, step_velocity, 400, 2400, 800);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

    if(GPIO_Pin == tank_front_full_Pin ) 
    {
        tank_front.event_full = 1;
    }
    if(GPIO_Pin == tank_front_empty_Pin )
    {
        tank_front.event_empty = 1;
    }
    if(GPIO_Pin == tank_rear_full_Pin)
    {
        tank_rear.event_full = 1;
    }
    if(GPIO_Pin == tank_rear_empty_Pin)
    {
        tank_rear.event_empty = 1;
    }
}