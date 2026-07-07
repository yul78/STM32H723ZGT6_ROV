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

static pid_control_t navigation_turn_pid = {
    .kp = NAV_TURN_PID_KP,
    .ki = NAV_TURN_PID_KI,
    .kd = NAV_TURN_PID_KD,
    .integral = 0.0f,
    .last_error = 0.0f,
    .output_limit = NAV_MAX_TURN_SPEED,
    .integral_limit = NAV_MAX_TURN_SPEED,
    .a = 0.0f,
    .filtered_error = 0.0f
};

static pid_control_t navigation_forward_pid = {
    .kp = NAV_FORWARD_PID_KP,
    .ki = NAV_FORWARD_PID_KI,
    .kd = NAV_FORWARD_PID_KD,
    .integral = 0.0f,
    .last_error = 0.0f,
    .output_limit = NAV_MAX_FORWARD_SPEED,
    .integral_limit = NAV_MAX_FORWARD_SPEED,
    .a = 0.0f,
    .filtered_error = 0.0f
};

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

static void Navigation_ResetPid(pid_control_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->filtered_error = 0.0f;
}

static float Navigation_PidStep(pid_control_t *pid, float target, float actual)
{
    float error = target - actual;
    float output = 0.0f;

    pid->filtered_error = error;
    pid->integral += pid->filtered_error;
    pid->integral = Navigation_ClampFloat(pid->integral, -pid->integral_limit, pid->integral_limit);

    output = (pid->kp * pid->filtered_error) +
             (pid->ki * pid->integral) +
             (pid->kd * (pid->filtered_error - pid->last_error));

    pid->last_error = pid->filtered_error;
    return Navigation_ClampFloat(output, -pid->output_limit, pid->output_limit);
}

static int16_t Navigation_ComputeTurnCommand(float heading_error_deg)
{
    float turn_output = Navigation_PidStep(&navigation_turn_pid, heading_error_deg, 0.0f);
    return (int16_t)(turn_output * (float)NAV_MOTOR1_POLARITY);
}

static int16_t Navigation_ComputeForwardCommand(float distance_m, float radius_m, float abs_heading_error_deg)
{
    float outside_distance_m = distance_m - radius_m;
    float forward_output = 0.0f;

    if (abs_heading_error_deg >= NAV_HEADING_STOP_FORWARD_DEG)
    {
        return 0;
    }

    forward_output = Navigation_PidStep(&navigation_forward_pid, outside_distance_m, 0.0f);

    if (abs_heading_error_deg > NAV_HEADING_ALIGN_DEG)
    {
        float scale = (NAV_HEADING_STOP_FORWARD_DEG - abs_heading_error_deg) /
                      (NAV_HEADING_STOP_FORWARD_DEG - NAV_HEADING_ALIGN_DEG);
        scale = Navigation_ClampFloat(scale, 0.0f, 1.0f);
        forward_output *= scale;
    }

    if (forward_output < 0.0f)
    {
        forward_output = 0.0f;
    }

    return (int16_t)(forward_output * (float)NAV_MOTOR2_POLARITY);
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
    /* When outside the circle, yaw correction has priority; forward thrust is reduced or blocked until heading error is small enough. */
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

    {
        float target_heading_deg = Navigation_ComputeTargetHeadingDeg(north_m, east_m);
        float current_heading_deg = Navigation_NormalizeAngleDeg(jy901_data.yaw + NAV_YAW_OFFSET_DEG);
        float heading_error_deg = Navigation_NormalizeAngleDeg(target_heading_deg - current_heading_deg);
        float abs_heading_error_deg = fabsf(heading_error_deg);
        int16_t turn_command = Navigation_ComputeTurnCommand(heading_error_deg);
        int16_t forward_command = Navigation_ComputeForwardCommand(distance_m, navigation_target.target_radius_m, abs_heading_error_deg);

        FOC_Set_Speed(1U, turn_command);
        FOC_Set_Speed(2U, forward_command);
    }
}

uint8_t Navigation_IsOutside(void)
{
    return navigation_outside;
}

void Navigation_Stop(void)
{
    navigation_outside = 0U;
    Navigation_ResetPid(&navigation_turn_pid);
    Navigation_ResetPid(&navigation_forward_pid);
    Navigation_StopMotors();
}
