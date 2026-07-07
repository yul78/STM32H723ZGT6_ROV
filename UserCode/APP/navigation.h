#ifndef __NAVIGATION_H
#define __NAVIGATION_H

#include "main.h"

#define NAV_TASK_PERIOD_MS                 100U
#define NAV_YAW_OFFSET_DEG                 65.0f
#define NAV_MOTOR1_POLARITY                1
#define NAV_MOTOR2_POLARITY                1
#define NAV_MAX_TURN_SPEED                 700.0f
#define NAV_MAX_FORWARD_SPEED              900.0f
#define NAV_HEADING_ALIGN_DEG              15.0f
#define NAV_HEADING_STOP_FORWARD_DEG       60.0f
#define NAV_OUTSIDE_MARGIN_M               1.5f
#define NAV_INSIDE_MARGIN_M                1.0f
#define NAV_TURN_PID_KP                    8.0f
#define NAV_TURN_PID_KI                    0.0f
#define NAV_TURN_PID_KD                    0.0f
#define NAV_FORWARD_PID_KP                 120.0f
#define NAV_FORWARD_PID_KI                 0.0f
#define NAV_FORWARD_PID_KD                 0.0f

typedef struct
{
    float target_center_latitude;
    float target_center_longitude;
    float target_radius_m;
    uint8_t valid;
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

/* Stop both motors and clear return state. */
void Navigation_Stop(void);

#endif