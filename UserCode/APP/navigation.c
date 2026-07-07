#include "navigation.h"
#include <string.h>
#include "app_gps.h"
#include "control.h"
#include "jy901.h"
#include "PID.h"

static NavigationTarget_t navigation_target;
static uint8_t navigation_enabled;
static uint8_t navigation_outside;

static void Navigation_StopMotors(void)
{
    FOC_Set_Speed(1U, 0);
    FOC_Set_Speed(2U, 0);
}

void Navigation_Init(void)
{
    memset(&navigation_target, 0, sizeof(navigation_target));
    navigation_enabled = 0U;
    navigation_outside = 0U;
    Navigation_Stop();
}

void Navigation_SetTarget(float center_lat, float center_lon, float radius_m)
{
    navigation_target.target_center_latitude = center_lat;
    navigation_target.target_center_longitude = center_lon;
    navigation_target.target_radius_m = radius_m;
    navigation_target.valid = 1U;
}

void Navigation_Enable(uint8_t enable)
{
    navigation_enabled = enable ? 1U : 0U;
    if (navigation_enabled == 0U)
    {
        Navigation_Stop();
    }
}

void Navigation_Task(void)
{
    if (navigation_enabled == 0U)
    {
        Navigation_StopMotors();
    }
}

uint8_t Navigation_IsOutside(void)
{
    return navigation_outside;
}

void Navigation_Stop(void)
{
    navigation_outside = 0U;
    Navigation_StopMotors();
}
