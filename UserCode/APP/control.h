#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "gamepad.h"
#include "H_Tmc2209.h"
#include "WaterTank.h"
#include "PID.h"
#include "bsp_jy901.h"
#include "jy901.h"

void Gamepad_Control(void);
void FOC_Set_Speed(uint8_t motor_num, int16_t speed);
void speed_control(uint8_t speed_X, uint8_t speed_Y);
void Scheduler_Set_DroneBalanceControlFlag(void);
uint8_t Scheduler_Get_DroneBalanceControlFlag(void);

#endif