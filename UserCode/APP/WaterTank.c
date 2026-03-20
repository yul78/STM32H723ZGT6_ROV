#include "WaterTank.h"

#define Water_Volume_Percircle 1.0f  //步进电机每转一圈对应的水量变化值，单位：ml/圈

water_tank_t tank_front;  
water_tank_t tank_rear;   

void Draining_Test(void);

/**
 * @brief 前后两个水舱初始化
 */
void Water_Tank_Init(void)
{
    // 链接步进电机
    tank_front.connect_motor_num = 1;   
    tank_rear.connect_motor_num = 2;

    // 初始水量先设置为0
    tank_front.start_water_volume = 0;  
    tank_rear.start_water_volume = 0;
    
    // 把当前水量设置为初始水量
    tank_front.now_water_volume = tank_front.start_water_volume; 
    tank_rear.now_water_volume = tank_rear.start_water_volume;

    // 设置吞吐水速度，单位：ml/s
    tank_front.water_velocity = 1.0;
    tank_rear.water_velocity = 1.0;

    tank_front.stepping_motor_polarity = 0;
    tank_rear.stepping_motor_polarity = 1;

    if(HAL_GPIO_ReadPin(tank_front_empty_GPIO_Port, tank_front_empty_Pin) == GPIO_PIN_RESET)
    {
        tank_front.state = TANK_EMPTY;
    }
    if(HAL_GPIO_ReadPin(tank_rear_empty_GPIO_Port, tank_rear_empty_Pin) == GPIO_PIN_RESET)
    {
        tank_rear.state = TANK_EMPTY;
    }
    
    // Water_Tank_Draning_To_Empty(&tank_front);
    // Water_Tank_Draning_To_Empty(&tank_rear);
    Draining_Test();
}

/**
 * @brief 水舱进水控制函数
 * @param tank 指定水舱的结构体指针
 * @param delta_xML 进水量
 */
void Water_Tank_Filling(water_tank_t *tank, float delta_xML)
{
    if (delta_xML <= 0.0f) return;
    if (tank->state == TANK_FULL) return;
    if(tank->state == TANK_FILLING || tank->state == TANK_DRAINING) return;

    tank->target_water_volume = tank->now_water_volume + delta_xML;
    
    // 设置状态，Load_Water_Volume 内部会启动电机
    tank->state = TANK_FILLING; 
    Load_Water_Volume(tank);
}

/**
 * @brief 水舱排水控制函数
 * @param tank 指定水舱的结构体指针
 * @param delta_xML 排水量
 */
void Water_Tank_Draining(water_tank_t *tank, float delta_xML)
{
    if (delta_xML <= 0.0f) return;
    if (tank->state == TANK_EMPTY) return;
    if(tank->state == TANK_FILLING || tank->state == TANK_DRAINING) return;

    tank->target_water_volume = tank->now_water_volume - delta_xML;
    
    // 设置状态
    tank->state = TANK_DRAINING;
    Load_Water_Volume(tank);
}

void Water_Tank_Draining_To_Empty(water_tank_t *tank)
{
    if(tank->state == TANK_EMPTY) return;

    while(1)
    {
        if(tank->event_empty)
        {
            tank->event_empty = 0;
            tank->state = TANK_EMPTY;
            tank->now_water_volume = 0;
            Motor_Stop(tank->connect_motor_num);
            tank->last_finished_steps = 0;
            Motor[tank->connect_motor_num].current_step = 0;
            break;
        }
        tank->target_water_volume = tank->now_water_volume - 0.1f; //每次排0.1ml水
        tank->state = TANK_DRAINING;
        Load_Water_Volume(tank);
        uint16_t delay_time_ms = 0.1f / tank->water_velocity * 1000.0f; //根据水速计算每0.1ml水的排空时间
        delay_ms(delay_time_ms);
        
    }
}

void Draining_Test(void)
{
    while(1)
    {
        if(tank_front.state == TANK_EMPTY && tank_rear.state == TANK_EMPTY) return;
        else
        {
            if(tank_front.event_empty)
            {
                tank_front.event_empty = 0;
                tank_front.state = TANK_EMPTY;
                tank_front.now_water_volume = 0;
                Motor_Stop(tank_front.connect_motor_num);
                tank_front.last_finished_steps = 0;
                Motor[tank_front.connect_motor_num].current_step = 0;
            }
            if(tank_rear.event_empty)
            {
                tank_rear.event_empty = 0;
                tank_rear.state = TANK_EMPTY;
                tank_rear.now_water_volume = 0;
                Motor_Stop(tank_rear.connect_motor_num);
                tank_rear.last_finished_steps = 0;
                Motor[tank_rear.connect_motor_num].current_step = 0;
            }

            if(tank_front.state != TANK_EMPTY)
            {
                tank_front.target_water_volume = tank_front.now_water_volume - 0.1f; //每次排0.1ml水
                tank_front.state = TANK_DRAINING;
                Load_Water_Volume(&tank_front);
            }
            if(tank_rear.state != TANK_EMPTY)
            {
                tank_rear.target_water_volume = tank_rear.now_water_volume - 0.1f; //每次排0.1ml水
                tank_rear.state = TANK_DRAINING;
                Load_Water_Volume(&tank_rear);
            }

            double min_velocity = tank_front.water_velocity < tank_rear.water_velocity ? tank_front.water_velocity : tank_rear.water_velocity;
            uint16_t delay_time_ms = 0.1f / min_velocity * 1000.0f; //根据水速计算每0.1ml水的排空时间
            delay_ms(delay_time_ms);
        }
    }
}
/**
 * @brief 监控水舱实时变化
 * @param tank 指定水舱的结构体指针
 */
void Water_Tank_Update_Handler(water_tank_t *tank)
{
    if(tank->state == TANK_DRAINING && tank->now_water_volume <= 0.01f)  //排水至空，则产生空水事件
    {
        tank->event_empty = 1;
    }
    /*满/空事件触发处理（最高优先级状态切换*/
    if (tank->event_full)
    {
        tank->event_full = 0;                // 清空满水事件
        tank->state = TANK_FULL;             // 设置满水状态
        Motor_Stop(tank->connect_motor_num); // 立即停止电机
        tank->last_finished_steps = 0;       // 水舱对应的步进电机实时完成的步数清零
        Motor[tank->connect_motor_num].current_step = 0; // 电机当前步数清零
        return; 
    }
    if (tank->event_empty)
    {
        tank->event_empty = 0;               // 清空空水事件
        tank->state = TANK_EMPTY;            // 设置空水状态
        tank->now_water_volume = 0;          // 物理校准：碰到空限位，水量强制归零
        Motor_Stop(tank->connect_motor_num); // 立即停止电机
        tank->last_finished_steps = 0;       // 水舱对应的步进电机实时完成的步数清零
        Motor[tank->connect_motor_num].current_step = 0; // 电机当前步数清零
        return;
    }

    /*实时水量更新*/
    uint32_t current_finished_steps = 0;  //获取水舱对应的步进电机的实时完成的步数
    if(tank->connect_motor_num == 1)      current_finished_steps = Motor_GetStep(1);
    else if(tank->connect_motor_num == 2) current_finished_steps = Motor_GetStep(2); 

    if (current_finished_steps > tank->last_finished_steps)       //当步进电机有已完成的步数时，就更新水舱水量
    {   
        uint32_t delta_step = 0;          //Δ步数
        double delta_volume = 0.0f;        //Δ水量

        delta_step = current_finished_steps - tank->last_finished_steps; //Δ步数 = 当前已完成的步数 - 上次已完成的步数
        delta_volume = (double)delta_step / (double)pulse_percircle * Water_Volume_Percircle; //Δ步数 ==> Δ水量

        if(tank->state == TANK_DRAINING) //排水状态，目标水量 = 当前水量 - Δ水量
        {
            tank->now_water_volume = tank->now_water_volume - delta_volume;
        }
        else if(tank->state == TANK_FILLING) //进水状态，目标水量 = 当前水量 + Δ水量
        {
            tank->now_water_volume = tank->now_water_volume + delta_volume;
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
        
    }
}

/**
 * @brief 将目标水量装载至水舱
 * @param tank 指定的水舱结构体指针
 */
void Load_Water_Volume(water_tank_t *tank)
{
    tank->last_finished_steps = 0;

    //目标水量 - 当前水量 = Δ水量
    float delta_water_volume = 0.0;
    delta_water_volume = tank->target_water_volume - tank->now_water_volume;

    //Δ水量 ==> Δstep
    int32_t delta_step = 0;
    delta_step = (int32_t)(delta_water_volume  * (float)pulse_percircle / Water_Volume_Percircle);

    //水速 ==> 步进电机脉冲频率
    uint32_t step_velocity = 0;
    step_velocity = 3200 * tank->water_velocity / Water_Volume_Percircle;

    //设置步进电机
    uint8_t dir = (tank->stepping_motor_polarity + 1) % 2;
    if(delta_step > 0)
        Motor_Set(tank->connect_motor_num, 2, dir, delta_step, step_velocity, 400, 2400, 800);
    else 
        Motor_Set(tank->connect_motor_num, 2, !dir, -delta_step, step_velocity, 400, 2400, 800);
}

/**
 * @brief 微动开关对应的外部中断的回调函数
 */
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
