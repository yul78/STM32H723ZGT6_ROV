#include "navigation.h"
#include <string.h>
#include <math.h>
#include "app_gps.h"
#include "control.h"
#include "jy901.h"
#include "PID.h"

static NavigationTarget_t navigation_target;
static uint8_t navigation_enabled;
static uint8_t navigation_outside;

#define NAV_DEG_TO_RAD                         0.01745329251994329577f
#define NAV_METERS_PER_DEG_LAT                 111320.0f

static uint8_t Navigation_TargetIsValid(void)
{
    return (navigation_target.valid != 0U) && (navigation_target.target_radius_m > 0.0f);
}

static uint8_t Navigation_GpsIsValid(void)
{
    return (fabsf(gps_data.latitude) > 0.000001f) || (fabsf(gps_data.longitude) > 0.000001f);
}

static float Navigation_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float Navigation_NormalizeAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static void Navigation_ComputeOffsetMeters(float current_lat, float current_lon, float target_lat, float target_lon, float *north_m, float *east_m)
{
    float lat_delta_deg = target_lat - current_lat;
    float lon_delta_deg = target_lon - current_lon;
    float ref_lat_rad = target_lat * NAV_DEG_TO_RAD;

    *north_m = lat_delta_deg * NAV_METERS_PER_DEG_LAT;
    *east_m = lon_delta_deg * NAV_METERS_PER_DEG_LAT * cosf(ref_lat_rad);
}

static float Navigation_ComputeDistanceMeters(float north_m, float east_m)
{
    return sqrtf((north_m * north_m) + (east_m * east_m));
}

static float Navigation_ComputeTargetHeadingDeg(float north_m, float east_m)
{
    return atan2f(east_m, north_m) * (180.0f / 3.14159265358979323846f);
}

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
    navigation_target.valid = (radius_m > 0.0f) ? 1U : 0U;
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
    float north_m = 0.0f;
    float east_m = 0.0f;
    float distance_m = 0.0f;

    if ((navigation_enabled == 0U) || (Navigation_TargetIsValid() == 0U) || (Navigation_GpsIsValid() == 0U))
    {
        Navigation_Stop();
        return;
    }

    Navigation_ComputeOffsetMeters(
        gps_data.latitude,
        gps_data.longitude,
        navigation_target.target_center_latitude,
        navigation_target.target_center_longitude,
        &north_m,
        &east_m);

    distance_m = Navigation_ComputeDistanceMeters(north_m, east_m);

    if (navigation_outside == 0U)
    {
        if (distance_m > (navigation_target.target_radius_m + NAV_OUTSIDE_MARGIN_M))
        {
            navigation_outside = 1U;
        }
    }
    else
    {
        if (distance_m <= (navigation_target.target_radius_m - NAV_INSIDE_MARGIN_M))
        {
            Navigation_Stop();
            return;
        }
    }

    if (navigation_outside == 0U)
    {
        Navigation_StopMotors();
        return;
    }

    (void)Navigation_ComputeTargetHeadingDeg(north_m, east_m);
    Navigation_StopMotors();
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
