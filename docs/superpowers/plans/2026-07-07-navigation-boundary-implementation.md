# Navigation Boundary Return Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a self-contained navigation module that stops inside the configured activity circle and, when outside, turns the torpedo-shaped vehicle toward the circle center and drives it back inside using the existing bow steering thruster and stern propulsion thruster.

**Architecture:** Keep all changes inside `UserCode/APP/navigation.h` and `UserCode/APP/navigation.c`. `navigation.h` exposes configuration macros and a small public API; `navigation.c` owns target/state storage, local GPS geometry helpers, file-local PID math compatible with `pid_control_t`, hysteresis-based boundary detection, and the 100 ms control loop that reads `gps_data` and `jy901_data` then calls `FOC_Set_Speed()`.

**Tech Stack:** STM32H723 bare-metal firmware, C, Keil MDK-ARM (`UV4.exe`), JY901 yaw data, GPS latitude/longitude app layer, existing `pid_control_t` layout from `UserCode/APP/PID.h`

## Global Constraints

- Only modify `UserCode/APP/navigation.h` and `UserCode/APP/navigation.c`.
- `Navigation_Task()` must be designed for a fixed `100 ms` call period.
- Inside the activity circle, both thruster outputs must be `0`.
- When outside the activity circle, motor 1 handles pure yaw correction and motor 2 handles forward/backward propulsion.
- Re-entering the activity circle must stop immediately; do not keep approaching the center.
- `NAV_YAW_OFFSET_DEG` and both motor polarities must be macros in `navigation.h`.
- The yaw convention is: `yaw = 0°` means the body x-axis points northeast `65°`; rotating from northeast `65°` toward due north makes yaw increase positively up to `+180°`, and rotating the opposite direction makes yaw decrease negatively down to `-180°`.
- If auto-return is disabled, the radius is invalid, the target is unset, or GPS data is invalid, stop both motors.
- Do not modify `UserCode/APP/PID.c`, `UserCode/APP/PID.h`, `UserCode/APP/app_gps.c`, `UserCode/APP/app_gps.h`, `UserCode/MID/JY901.c`, `UserCode/MID/JY901.h`, or `UserCode/APP/control.c`; `navigation.c` must adapt to those existing interfaces.
- `UserCode/APP/PID.c:24-52` keeps `pid_calculate()` file-local, so `navigation.c` must implement its own file-local PID step using the same `pid_control_t` fields instead of trying to call that static function.

---

## File Structure

- `UserCode/APP/navigation.h` — public macros, target struct, function declarations, and concise usage comments for external callers.
- `UserCode/APP/navigation.c` — private module state, local PID helpers, GPS geometry helpers, hysteresis logic, heading control, forward-speed gating, and `FOC_Set_Speed()` output.
- `MDK-ARM/Project.uvprojx` — build target only; no file-list changes required because `navigation.c` and `navigation.h` already exist.

### Task 1: Build the public navigation API and safe stubs

**Files:**
- Modify: `UserCode/APP/navigation.h:1-11`
- Modify: `UserCode/APP/navigation.c:1-2`
- Test: `MDK-ARM/Project.uvprojx`

**Interfaces:**
- Consumes: `void FOC_Set_Speed(uint8_t motor_num, int16_t speed);` from `UserCode/APP/control.h`
- Produces: 
  - `void Navigation_Init(void);`
  - `void Navigation_SetTarget(float center_lat, float center_lon, float radius_m);`
  - `void Navigation_Enable(uint8_t enable);`
  - `void Navigation_Task(void);`
  - `uint8_t Navigation_IsOutside(void);`
  - `void Navigation_Stop(void);`
  - `typedef struct { float target_center_latitude; float target_center_longitude; float target_radius_m; uint8_t valid; } NavigationTarget_t;`

- [ ] **Step 1: Write the failing build-first API skeleton**

Replace `UserCode/APP/navigation.h` with:

```c
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

void Navigation_Init(void);
void Navigation_SetTarget(float center_lat, float center_lon, float radius_m);
void Navigation_Enable(uint8_t enable);
void Navigation_Task(void);
uint8_t Navigation_IsOutside(void);
void Navigation_Stop(void);

#endif
```

Replace `UserCode/APP/navigation.c` with this intentionally incomplete skeleton:

```c
#include "navigation.h"
#include <string.h>
#include "app_gps.h"
#include "control.h"
#include "jy901.h"
#include "PID.h"

static NavigationTarget_t navigation_target;
static uint8_t navigation_enabled;
static uint8_t navigation_outside;

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
```

- [ ] **Step 2: Run build to verify it fails**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`

Expected: FAIL with an undefined symbol or implicit declaration error mentioning `Navigation_StopMotors`.

- [ ] **Step 3: Write the minimal implementation to make the API build-safe**

Update `UserCode/APP/navigation.c` by adding the missing helper before `Navigation_Init()`:

```c
static void Navigation_StopMotors(void)
{
    FOC_Set_Speed(1U, 0);
    FOC_Set_Speed(2U, 0);
}
```

Add short public usage comments above the declarations in `UserCode/APP/navigation.h`:

```c
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
```

- [ ] **Step 4: Run build to verify it passes**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`

Expected: PASS with zero compile errors from `UserCode/APP/navigation.c`.

- [ ] **Step 5: Commit**

```bash
git add UserCode/APP/navigation.h UserCode/APP/navigation.c
git commit -m "feat: add navigation module api skeleton"
```

### Task 2: Add target validation, boundary detection, and heading geometry

**Files:**
- Modify: `UserCode/APP/navigation.c`
- Test: `MDK-ARM/Project.uvprojx`

**Interfaces:**
- Consumes:
  - `extern gps_data_t gps_data;` from `UserCode/APP/app_gps.h`
  - `extern jy901_data_t jy901_data;` from `UserCode/MID/JY901.h`
- Produces:
  - `static uint8_t Navigation_TargetIsValid(void);`
  - `static uint8_t Navigation_GpsIsValid(void);`
  - `static float Navigation_ClampFloat(float value, float min_value, float max_value);`
  - `static float Navigation_NormalizeAngleDeg(float angle_deg);`
  - `static void Navigation_ComputeOffsetMeters(float current_lat, float current_lon, float target_lat, float target_lon, float *north_m, float *east_m);`
  - `static float Navigation_ComputeDistanceMeters(float north_m, float east_m);`
  - `static float Navigation_ComputeTargetHeadingDeg(float north_m, float east_m);`

- [ ] **Step 1: Write the failing geometry-first control flow**

Replace `Navigation_Task()` in `UserCode/APP/navigation.c` with this intentionally incomplete version:

```c
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
```

- [ ] **Step 2: Run build to verify it fails**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`

Expected: FAIL with undefined symbol errors mentioning `Navigation_TargetIsValid`, `Navigation_GpsIsValid`, or `Navigation_ComputeOffsetMeters`.

- [ ] **Step 3: Write the minimal geometry and validation helpers**

Add these includes near the top of `UserCode/APP/navigation.c`:

```c
#include <math.h>
```

Add these file-local helpers above `Navigation_Init()`:

```c
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
```

Tighten `Navigation_SetTarget()` so invalid radius clears the target instead of blindly marking it valid:

```c
void Navigation_SetTarget(float center_lat, float center_lon, float radius_m)
{
    navigation_target.target_center_latitude = center_lat;
    navigation_target.target_center_longitude = center_lon;
    navigation_target.target_radius_m = radius_m;
    navigation_target.valid = (radius_m > 0.0f) ? 1U : 0U;
}
```

- [ ] **Step 4: Run build to verify it passes**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`

Expected: PASS. `Navigation_Task()` should compile with full target validation, GPS validation, local meter conversion, and hysteresis-based outside detection while still stopping the motors whenever the vehicle is inside the circle.

- [ ] **Step 5: Commit**

```bash
git add UserCode/APP/navigation.c
git commit -m "feat: add navigation boundary detection"
```

### Task 3: Add heading control, thrust gating, and the final return-to-circle loop

**Files:**
- Modify: `UserCode/APP/navigation.c`
- Modify: `UserCode/APP/navigation.h`
- Test: `MDK-ARM/Project.uvprojx`

**Interfaces:**
- Consumes:
  - `extern jy901_data_t jy901_data;` from `UserCode/MID/JY901.h`
  - `void FOC_Set_Speed(uint8_t motor_num, int16_t speed);` from `UserCode/APP/control.h`
  - `typedef struct { float kp; float ki; float kd; float integral; float last_error; float output_limit; float integral_limit; float a; float filtered_error; } pid_control_t;` from `UserCode/APP/PID.h`
- Produces:
  - `static void Navigation_ResetPid(pid_control_t *pid);`
  - `static float Navigation_PidStep(pid_control_t *pid, float target, float actual);`
  - `static int16_t Navigation_ComputeTurnCommand(float heading_error_deg);`
  - `static int16_t Navigation_ComputeForwardCommand(float distance_m, float radius_m, float abs_heading_error_deg);`

- [ ] **Step 1: Write the failing control-loop wiring**

Add these new statics near the top of `UserCode/APP/navigation.c`:

```c
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
```

Replace `Navigation_Stop()` with:

```c
void Navigation_Stop(void)
{
    navigation_outside = 0U;
    Navigation_ResetPid(&navigation_turn_pid);
    Navigation_ResetPid(&navigation_forward_pid);
    Navigation_StopMotors();
}
```

Replace the bottom of `Navigation_Task()` starting at `(void)Navigation_ComputeTargetHeadingDeg(north_m, east_m);` with:

```c
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
```

- [ ] **Step 2: Run build to verify it fails**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`

Expected: FAIL with undefined symbol errors mentioning `Navigation_ResetPid`, `Navigation_ComputeTurnCommand`, or `Navigation_ComputeForwardCommand`.

- [ ] **Step 3: Write the minimal final control implementation**

Add these helper implementations above `Navigation_Init()` in `UserCode/APP/navigation.c`:

```c
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
```

Tighten `Navigation_Enable()` so disabling also clears controller accumulation:

```c
void Navigation_Enable(uint8_t enable)
{
    navigation_enabled = enable ? 1U : 0U;
    if (navigation_enabled == 0U)
    {
        Navigation_Stop();
    }
}
```

Add one module comment near `Navigation_Task()` explaining the heading-first behavior:

```c
/* When outside the circle, yaw correction has priority; forward thrust is reduced or blocked until heading error is small enough. */
```

- [ ] **Step 4: Run build to verify it passes**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`

Expected: PASS. `Navigation_Task()` now reads GPS + yaw, detects inside/outside with hysteresis, turns toward the circle center using motor 1, gates forward thrust on motor 2 based on heading error, and stops immediately on re-entry.

- [ ] **Step 5: Run targeted diff review for the two allowed files**

Run: `git diff -- UserCode/APP/navigation.h UserCode/APP/navigation.c`

Expected: Only `UserCode/APP/navigation.h` and `UserCode/APP/navigation.c` appear, with macro-based polarity/yaw configuration, documented public API, and no edits to GPS/JY901/PID/control files.

- [ ] **Step 6: Commit**

```bash
git add UserCode/APP/navigation.h UserCode/APP/navigation.c
git commit -m "feat: add circle return navigation control"
```

## Self-Review

- **Spec coverage:** Task 1 covers the public API and macro-based configuration. Task 2 covers target validation, GPS validation, local meter conversion, and hysteresis-based outside detection. Task 3 covers yaw sign handling, heading normalization, motor-1 yaw control, motor-2 forward gating, immediate stop on re-entry, and the requirement to keep changes limited to `navigation.h` and `navigation.c`.
- **Placeholder scan:** No `TODO`, `TBD`, or “implement later” placeholders remain. Every code-changing step includes concrete code and every verification step includes an exact command.
- **Type consistency:** All public signatures are introduced in Task 1 and reused unchanged later. PID helpers use `pid_control_t` exactly as declared in `UserCode/APP/PID.h`, and the final task still relies only on `gps_data`, `jy901_data`, and `FOC_Set_Speed()`.
