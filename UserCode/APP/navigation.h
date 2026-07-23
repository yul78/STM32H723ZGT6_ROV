#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "main.h"

#define NAV_TASK_PERIOD_MS                 100U
#define NAV_YAW_OFFSET_DEG                 65.0f
#define NAV_MOTOR1_POLARITY                1
#define NAV_MOTOR2_POLARITY                1
#define NAV_MAX_TURN_SPEED                 700.0f
#define NAV_FORWARD_SPEED              900.0f
#define NAV_HEADING_ALIGN_DEG              15.0f
#define NAV_HEADING_STOP_FORWARD_DEG       60.0f
#define NAV_OUTSIDE_MARGIN_M               1.5f
#define NAV_INSIDE_MARGIN_M                1.0f


typedef struct
{
    float target_center_latitude;  // 目标圆心纬度
    float target_center_longitude; // 目标圆心经度
    float target_radius_m;         // 目标圆半径，单位：米
    uint8_t valid;                 // 目标数值是否有效，1表示有效，0表示无效
} NavigationTarget_t;

/* Call once during system init. */
void Navigation_Init(void);

/* Set activity-circle center and radius in meters. */
void Navigation_SetTarget(float center_lat, float center_lon, float radius_m);

/* Enable or disable automatic return. */
void Navigation_Enable(uint8_t enable);

/* Call every NAV_TASK_PERIOD_MS. */
void Navigation_Task(void);

/* Returns 1 when the module currently considers the vehicle outside the circle. */
uint8_t Navigation_IsOutside(void);

/* Stop both motors, clear return state, and disable automatic return until Navigation_Enable(1). */
void Navigation_Stop(void);

#endif
