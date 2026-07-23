# STM32H723 Architecture Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move business orchestration out of `Core/Src/main.c`, introduce `Device / Service / App` layers, keep `UserCode/foc_lib/` untouched internally, and land the refactor in working buildable increments.

**Architecture:** The refactor keeps CubeMX/HAL at the platform boundary, wraps existing hardware-facing modules in `Device` adapters, moves business state machines into `Service`, and reduces `main.c` to HAL init plus `AppMain_Init()` and `AppMain_RunOnce()`. Runtime stays bare-metal and scheduler-driven; ISR code only captures bytes, updates driver state, and flips flags.

**Tech Stack:** STM32H723 bare-metal C, STM32 HAL, Keil MDK-ARM target `Project`, existing `UserCode/MID/*`, existing `UserCode/BSP/*`, existing `UserCode/foc_lib/*`

## Global Constraints
- Keep runtime model as a lightweight scheduler; do not introduce RTOS or a dynamic event bus.
- Do not modify files inside `UserCode/foc_lib/`; only wrap its public entry points.
- Variable names use module-prefix snake case.
- Function names use type/module prefix + CamelCase.
- `Core/` and CubeMX-generated HAL files stay focused on startup and ISR entry only.
- ISR code must not contain business branching.
- Every task must end in a buildable Keil project state.
- Use compile-driven red/green checks because the repository currently has no project-owned unit-test harness.

---

## File Map

### Create
- `UserCode/App/app_main.h` — top-level application entry points
- `UserCode/App/app_main.c` — application wiring and run loop body
- `UserCode/App/app_config.h` — app-scoped object declarations and composition helpers
- `UserCode/App/app_config.c` — instantiate adapters/services and connect dependencies
- `UserCode/Service/scheduler/scheduler_service.h` — scheduler types and API
- `UserCode/Service/scheduler/scheduler_service.c` — periodic task table execution
- `UserCode/Device/actuator/actuator_stepper.h` — stepper adapter interface over `H_Tmc2209`
- `UserCode/Device/actuator/actuator_stepper.c` — stepper adapter implementation
- `UserCode/Device/actuator/actuator_thruster.h` — thruster adapter interface over `app_thrusters` + `foc_lib`
- `UserCode/Device/actuator/actuator_thruster.c` — thruster adapter implementation
- `UserCode/Device/sensor/input_gamepad.h` — gamepad adapter API and normalized state
- `UserCode/Device/sensor/input_gamepad.c` — gamepad adapter implementation
- `UserCode/Device/sensor/sensor_imu.h` — IMU snapshot API
- `UserCode/Device/sensor/sensor_imu.c` — IMU adapter implementation
- `UserCode/Device/sensor/sensor_gps.h` — GPS snapshot API
- `UserCode/Device/sensor/sensor_gps.c` — GPS adapter implementation
- `UserCode/Device/sensor/sensor_water_leak.h` — water leak adapter API
- `UserCode/Device/sensor/sensor_water_leak.c` — water leak adapter implementation
- `UserCode/Service/input/input_service.h` — operator input command API
- `UserCode/Service/input/input_service.c` — handoff from gamepad state to normalized command
- `UserCode/Service/propulsion/propulsion_service.h` — thruster control service API
- `UserCode/Service/propulsion/propulsion_service.c` — propulsion control implementation
- `UserCode/Service/navigation/navigation_service.h` — IMU/GPS state service API
- `UserCode/Service/navigation/navigation_service.c` — navigation state update implementation
- `UserCode/Service/ballast/ballast_service.h` — ballast state machine API
- `UserCode/Service/ballast/ballast_service.c` — non-blocking ballast implementation
- `UserCode/Service/safety/safety_service.h` — leak/emergency strategy API
- `UserCode/Service/safety/safety_service.c` — non-blocking safety implementation

### Modify
- `Core/Src/main.c` — replace direct business loop with `AppMain_*` calls
- `UserCode/BSP/bsp_interrupt.h` — stop including APP business headers directly
- `UserCode/BSP/bsp_interrupt.c` — route ISR events to adapters/flags only
- `UserCode/APP/control.h` — retire legacy control declarations or turn into compatibility shim
- `UserCode/APP/control.c` — remove direct orchestration logic or convert into thin compatibility wrapper
- `UserCode/APP/WaterTank.h` — shrink to compatibility shim during migration, then retire
- `UserCode/APP/WaterTank.c` — remove blocking logic and bridge to `BallastService_*`
- `UserCode/APP/app_thrusters.h` — expose only stable wrapper-facing declarations
- `UserCode/APP/app_thrusters.c` — keep as thin init wrapper or redirect to adapter ownership
- `UserCode/APP/app_gps.h` — bridge old GPS access to navigation/device layer
- `UserCode/APP/app_gps.c` — route legacy task path into adapter/service composition
- `MDK-ARM/Project.uvprojx` — add all new source/header groups to target `Project`
- `MDK-ARM/Project.uvoptx` — refresh file group visibility if Keil workspace tracking is required locally

### Build / Verification Command
- Build: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
- Success signal: exit code 0 and final line containing `0 Error(s)`
- Failure signal: undefined symbol, missing file in project, duplicate symbol, or compile error

## Task 1: Extract App entry points and scheduler shell

**Files:**
- Create: `UserCode/App/app_main.h`
- Create: `UserCode/App/app_main.c`
- Create: `UserCode/App/app_config.h`
- Create: `UserCode/App/app_config.c`
- Create: `UserCode/Service/scheduler/scheduler_service.h`
- Create: `UserCode/Service/scheduler/scheduler_service.c`
- Modify: `Core/Src/main.c`
- Modify: `MDK-ARM/Project.uvprojx`

**Interfaces:**
- Consumes: `HAL_GetTick(void)`, existing CubeMX init functions in `Core/Src/main.c`
- Produces:
  - `void AppMain_Init(void);`
  - `void AppMain_RunOnce(void);`
  - `typedef void (*scheduler_task_fn_t)(void *context);`
  - `void SchedulerService_Init(void);`
  - `void SchedulerService_Register(const scheduler_task_t *task);`
  - `void SchedulerService_RunOnce(uint32_t scheduler_now_ms);`

- [ ] **Step 1: Write the failing compile check by rewiring `main.c` first**

```c
#include "app_main.h"

/* USER CODE BEGIN 2 */
AppMain_Init();
/* USER CODE END 2 */

while (1)
{
    AppMain_RunOnce();
}
```

- [ ] **Step 2: Run build to verify it fails because `AppMain_*` does not exist yet**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: FAIL with an undefined identifier or unresolved symbol for `AppMain_Init` / `AppMain_RunOnce`

- [ ] **Step 3: Add the scheduler shell and app entry headers**

`UserCode/Service/scheduler/scheduler_service.h`
```c
#ifndef SCHEDULER_SERVICE_H
#define SCHEDULER_SERVICE_H

#include "main.h"

typedef void (*scheduler_task_fn_t)(void *context);

typedef struct
{
    const char *task_name;
    uint16_t task_period_ms;
    uint32_t task_last_tick;
    scheduler_task_fn_t task_run;
    void *task_context;
} scheduler_task_t;

void SchedulerService_Init(void);
void SchedulerService_Register(const scheduler_task_t *task);
void SchedulerService_RunOnce(uint32_t scheduler_now_ms);

#endif
```

`UserCode/App/app_main.h`
```c
#ifndef APP_MAIN_H
#define APP_MAIN_H

void AppMain_Init(void);
void AppMain_RunOnce(void);

#endif
```

- [ ] **Step 4: Add minimal implementation that preserves existing behavior hooks without moving business logic yet**

`UserCode/Service/scheduler/scheduler_service.c`
```c
#include "scheduler_service.h"

#define scheduler_task_capacity 8U

static scheduler_task_t scheduler_task_table[scheduler_task_capacity];
static uint8_t scheduler_task_count;

void SchedulerService_Init(void)
{
    scheduler_task_count = 0U;
}

void SchedulerService_Register(const scheduler_task_t *task)
{
    if ((task != NULL) && (scheduler_task_count < scheduler_task_capacity))
    {
        scheduler_task_table[scheduler_task_count] = *task;
        scheduler_task_count++;
    }
}

void SchedulerService_RunOnce(uint32_t scheduler_now_ms)
{
    for (uint8_t scheduler_task_index = 0U; scheduler_task_index < scheduler_task_count; ++scheduler_task_index)
    {
        scheduler_task_t *scheduler_task = &scheduler_task_table[scheduler_task_index];
        if ((scheduler_now_ms - scheduler_task->task_last_tick) >= scheduler_task->task_period_ms)
        {
            scheduler_task->task_last_tick = scheduler_now_ms;
            scheduler_task->task_run(scheduler_task->task_context);
        }
    }
}
```

`UserCode/App/app_main.c`
```c
#include "app_main.h"
#include "app_config.h"
#include "scheduler_service.h"
#include "main.h"

void AppMain_Init(void)
{
    AppConfig_Init();
}

void AppMain_RunOnce(void)
{
    SchedulerService_RunOnce(HAL_GetTick());
}
```

`UserCode/App/app_config.h`
```c
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

void AppConfig_Init(void);

#endif
```

`UserCode/App/app_config.c`
```c
#include "app_config.h"
#include "scheduler_service.h"

static void AppConfig_IdleTask(void *context)
{
    (void)context;
}

void AppConfig_Init(void)
{
    static scheduler_task_t app_idle_task =
    {
        .task_name = "app_idle_task",
        .task_period_ms = 20U,
        .task_last_tick = 0U,
        .task_run = AppConfig_IdleTask,
        .task_context = 0,
    };

    SchedulerService_Init();
    SchedulerService_Register(&app_idle_task);
}
```

- [ ] **Step 5: Add the new files to Keil target `Project`**

Add these paths under new groups `App` and `Service` in `MDK-ARM/Project.uvprojx`:
```text
..\UserCode\App\app_main.c
..\UserCode\App\app_config.c
..\UserCode\Service\scheduler\scheduler_service.c
```

- [ ] **Step 6: Run build to verify the shell passes**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: PASS with `0 Error(s)`

- [ ] **Step 7: Commit**

```bash
git add Core/Src/main.c MDK-ARM/Project.uvprojx UserCode/App/app_main.h UserCode/App/app_main.c UserCode/App/app_config.h UserCode/App/app_config.c UserCode/Service/scheduler/scheduler_service.h UserCode/Service/scheduler/scheduler_service.c
git commit -m "refactor: add app shell and scheduler scaffold"
```

### Task 2: Introduce Device adapters around existing hardware modules

**Files:**
- Create: `UserCode/Device/actuator/actuator_stepper.h`
- Create: `UserCode/Device/actuator/actuator_stepper.c`
- Create: `UserCode/Device/actuator/actuator_thruster.h`
- Create: `UserCode/Device/actuator/actuator_thruster.c`
- Create: `UserCode/Device/sensor/input_gamepad.h`
- Create: `UserCode/Device/sensor/input_gamepad.c`
- Create: `UserCode/Device/sensor/sensor_imu.h`
- Create: `UserCode/Device/sensor/sensor_imu.c`
- Create: `UserCode/Device/sensor/sensor_gps.h`
- Create: `UserCode/Device/sensor/sensor_gps.c`
- Create: `UserCode/Device/sensor/sensor_water_leak.h`
- Create: `UserCode/Device/sensor/sensor_water_leak.c`
- Modify: `UserCode/APP/app_thrusters.h`
- Modify: `UserCode/APP/app_thrusters.c`
- Modify: `MDK-ARM/Project.uvprojx`

**Interfaces:**
- Consumes:
  - `Gamepad_Init`, `Gamepad_RxCallback`, `Gamepad_GetData` from `UserCode/APP/gamepad.h`
  - `Foc_Init`, `Foc_Loop`, `Foc_Set_Speed` entry points exposed outside `UserCode/foc_lib/`
  - `Motor_Set`, `Motor_Stop`, `Motor_GetStep` from `UserCode/MID/H_Tmc2209.h`
  - `gps_data` from `UserCode/APP/app_gps.h`
  - `jy901_data` from `UserCode/BSP/bsp_jy901.h` / `jy901.h`
  - `Water_Check`, `voltage_value` from `UserCode/APP/WaterADC.h`
- Produces:
  - `void ActuatorStepper_Init(actuator_stepper_t *self, uint8_t actuator_motor_id, uint8_t actuator_polarity);`
  - `void ActuatorStepper_SetTargetStep(actuator_stepper_t *self, int32_t actuator_target_step, uint32_t actuator_step_velocity);`
  - `void ActuatorStepper_Stop(actuator_stepper_t *self);`
  - `uint32_t ActuatorStepper_GetProgress(const actuator_stepper_t *self);`
  - `void ActuatorThruster_Init(actuator_thruster_t *self, uint8_t actuator_thruster_id);`
  - `void ActuatorThruster_SetSpeed(actuator_thruster_t *self, int16_t actuator_speed);`
  - `void InputGamepad_Init(input_gamepad_t *self);`
  - `void InputGamepad_OnByte(input_gamepad_t *self, uint8_t input_byte);`
  - `void SensorImu_Snapshot(sensor_imu_t *self, sensor_imu_snapshot_t *sensor_snapshot);`
  - `void SensorGps_Snapshot(sensor_gps_t *self, sensor_gps_snapshot_t *sensor_snapshot);`
  - `uint8_t SensorWaterLeak_IsDetected(sensor_water_leak_t *self);`

- [ ] **Step 1: Write the failing compile check by referencing the adapters from `app_config.c` before they exist**

```c
#include "actuator_stepper.h"
#include "actuator_thruster.h"
#include "input_gamepad.h"
#include "sensor_imu.h"
#include "sensor_gps.h"
#include "sensor_water_leak.h"

static actuator_stepper_t ballast_front_stepper;
static actuator_stepper_t ballast_rear_stepper;
static actuator_thruster_t propulsion_left_right_thruster;
static actuator_thruster_t propulsion_vertical_thruster;
static input_gamepad_t operator_gamepad;
static sensor_imu_t navigation_imu;
static sensor_gps_t navigation_gps;
static sensor_water_leak_t safety_water_leak;
```

- [ ] **Step 2: Run build to verify it fails because the new adapter headers and symbols do not exist yet**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: FAIL with missing include or undefined type errors for `actuator_stepper_t`, `actuator_thruster_t`, or sensor adapter types

- [ ] **Step 3: Add the adapter headers with explicit ownership-friendly structs**

`UserCode/Device/actuator/actuator_stepper.h`
```c
#ifndef ACTUATOR_STEPPER_H
#define ACTUATOR_STEPPER_H

#include "main.h"

typedef struct
{
    uint8_t actuator_motor_id;
    uint8_t actuator_polarity;
} actuator_stepper_t;

void ActuatorStepper_Init(actuator_stepper_t *self, uint8_t actuator_motor_id, uint8_t actuator_polarity);
void ActuatorStepper_SetTargetStep(actuator_stepper_t *self, int32_t actuator_target_step, uint32_t actuator_step_velocity);
void ActuatorStepper_Stop(actuator_stepper_t *self);
uint32_t ActuatorStepper_GetProgress(const actuator_stepper_t *self);

#endif
```

`UserCode/Device/actuator/actuator_thruster.h`
```c
#ifndef ACTUATOR_THRUSTER_H
#define ACTUATOR_THRUSTER_H

#include "main.h"

typedef struct
{
    uint8_t actuator_thruster_id;
} actuator_thruster_t;

void ActuatorThruster_Init(actuator_thruster_t *self, uint8_t actuator_thruster_id);
void ActuatorThruster_SetSpeed(actuator_thruster_t *self, int16_t actuator_speed);
void ActuatorThruster_Stop(actuator_thruster_t *self);

#endif
```

`UserCode/Device/sensor/input_gamepad.h`
```c
#ifndef INPUT_GAMEPAD_H
#define INPUT_GAMEPAD_H

#include "main.h"

typedef struct
{
    int16_t input_forward;
    int16_t input_vertical;
    uint16_t input_buttons;
    uint8_t input_updated;
} input_gamepad_state_t;

typedef struct
{
    input_gamepad_state_t input_state;
} input_gamepad_t;

void InputGamepad_Init(input_gamepad_t *self);
void InputGamepad_OnByte(input_gamepad_t *self, uint8_t input_byte);
void InputGamepad_RunOnce(input_gamepad_t *self);
const input_gamepad_state_t *InputGamepad_GetState(const input_gamepad_t *self);

#endif
```

- [ ] **Step 4: Implement the adapters as thin wrappers only; do not move business rules here**

`UserCode/Device/actuator/actuator_stepper.c`
```c
#include "actuator_stepper.h"
#include "H_Tmc2209.h"

void ActuatorStepper_Init(actuator_stepper_t *self, uint8_t actuator_motor_id, uint8_t actuator_polarity)
{
    self->actuator_motor_id = actuator_motor_id;
    self->actuator_polarity = actuator_polarity;
}

void ActuatorStepper_SetTargetStep(actuator_stepper_t *self, int32_t actuator_target_step, uint32_t actuator_step_velocity)
{
    uint8_t actuator_direction = (actuator_target_step >= 0) ? self->actuator_polarity : (uint8_t)!self->actuator_polarity;
    uint32_t actuator_step_count = (actuator_target_step >= 0) ? (uint32_t)actuator_target_step : (uint32_t)(-actuator_target_step);
    Motor_Set(self->actuator_motor_id, 2U, actuator_direction, actuator_step_count, actuator_step_velocity, 400U, 2400U, 800U);
}

void ActuatorStepper_Stop(actuator_stepper_t *self)
{
    Motor_Stop(self->actuator_motor_id);
}

uint32_t ActuatorStepper_GetProgress(const actuator_stepper_t *self)
{
    return Motor_GetStep(self->actuator_motor_id);
}
```

`UserCode/Device/actuator/actuator_thruster.c`
```c
#include "actuator_thruster.h"
#include "app_thrusters.h"
#include "foc.h"

void ActuatorThruster_Init(actuator_thruster_t *self, uint8_t actuator_thruster_id)
{
    self->actuator_thruster_id = actuator_thruster_id;
    APP_Thrusters_Init();
}

void ActuatorThruster_SetSpeed(actuator_thruster_t *self, int16_t actuator_speed)
{
    Foc_Set_Speed(self->actuator_thruster_id, (float)actuator_speed);
}

void ActuatorThruster_Stop(actuator_thruster_t *self)
{
    Foc_Set_Speed(self->actuator_thruster_id, 0.0f);
}
```

`UserCode/Device/sensor/sensor_water_leak.c`
```c
#include "sensor_water_leak.h"
#include "WaterADC.h"

void SensorWaterLeak_Init(sensor_water_leak_t *self)
{
    (void)self;
    WaterADC_Init();
}

uint8_t SensorWaterLeak_IsDetected(sensor_water_leak_t *self)
{
    (void)self;
    return Water_Check();
}
```

- [ ] **Step 5: Wire adapter init into `AppConfig_Init()`**

```c
ActuatorStepper_Init(&ballast_front_stepper, 1U, 1U);
ActuatorStepper_Init(&ballast_rear_stepper, 2U, 0U);
ActuatorThruster_Init(&propulsion_left_right_thruster, 1U);
ActuatorThruster_Init(&propulsion_vertical_thruster, 2U);
InputGamepad_Init(&operator_gamepad);
SensorWaterLeak_Init(&safety_water_leak);
```

- [ ] **Step 6: Add the new source files to Keil and build green**

Add these paths under new groups `Device/actuator` and `Device/sensor` in `MDK-ARM/Project.uvprojx`:
```text
..\UserCode\Device\actuator\actuator_stepper.c
..\UserCode\Device\actuator\actuator_thruster.c
..\UserCode\Device\sensor\input_gamepad.c
..\UserCode\Device\sensor\sensor_imu.c
..\UserCode\Device\sensor\sensor_gps.c
..\UserCode\Device\sensor\sensor_water_leak.c
```

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: PASS with `0 Error(s)`

- [ ] **Step 7: Commit**

```bash
git add MDK-ARM/Project.uvprojx UserCode/App/app_config.c UserCode/APP/app_thrusters.h UserCode/APP/app_thrusters.c UserCode/Device/actuator/actuator_stepper.h UserCode/Device/actuator/actuator_stepper.c UserCode/Device/actuator/actuator_thruster.h UserCode/Device/actuator/actuator_thruster.c UserCode/Device/sensor/input_gamepad.h UserCode/Device/sensor/input_gamepad.c UserCode/Device/sensor/sensor_imu.h UserCode/Device/sensor/sensor_imu.c UserCode/Device/sensor/sensor_gps.h UserCode/Device/sensor/sensor_gps.c UserCode/Device/sensor/sensor_water_leak.h UserCode/Device/sensor/sensor_water_leak.c
git commit -m "refactor: add device adapters for app services"
```

### Task 3: Add input, navigation, and propulsion services

**Files:**
- Create: `UserCode/Service/input/input_service.h`
- Create: `UserCode/Service/input/input_service.c`
- Create: `UserCode/Service/navigation/navigation_service.h`
- Create: `UserCode/Service/navigation/navigation_service.c`
- Create: `UserCode/Service/propulsion/propulsion_service.h`
- Create: `UserCode/Service/propulsion/propulsion_service.c`
- Modify: `UserCode/App/app_config.c`
- Modify: `UserCode/App/app_main.c`
- Modify: `UserCode/APP/control.h`
- Modify: `UserCode/APP/control.c`
- Modify: `MDK-ARM/Project.uvprojx`

**Interfaces:**
- Consumes:
  - `input_gamepad_t` from Task 2
  - `sensor_imu_t`, `sensor_gps_t` from Task 2
  - `actuator_thruster_t` from Task 2
- Produces:
  - `void InputService_Init(input_service_t *self, input_gamepad_t *input_gamepad);`
  - `uint8_t InputService_TakeCommand(input_service_t *self, input_command_t *input_command);`
  - `void NavigationService_Init(navigation_service_t *self, sensor_imu_t *sensor_imu, sensor_gps_t *sensor_gps);`
  - `void NavigationService_RunOnce(navigation_service_t *self);`
  - `const navigation_state_t *NavigationService_GetState(const navigation_service_t *self);`
  - `void PropulsionService_Init(propulsion_service_t *self, actuator_thruster_t *propulsion_horizontal, actuator_thruster_t *propulsion_vertical);`
  - `void PropulsionService_ApplyCommand(propulsion_service_t *self, const input_command_t *input_command);`
  - `void PropulsionService_StopAll(propulsion_service_t *self);`

- [ ] **Step 1: Write the failing compile check by registering service-backed scheduler callbacks in `app_config.c`**

```c
#include "input_service.h"
#include "navigation_service.h"
#include "propulsion_service.h"

static input_service_t operator_input_service;
static navigation_service_t vehicle_navigation_service;
static propulsion_service_t vehicle_propulsion_service;

static void AppConfig_InputTask(void *context)
{
    InputService_RunOnce((input_service_t *)context);
}

static void AppConfig_NavigationTask(void *context)
{
    NavigationService_RunOnce((navigation_service_t *)context);
}
```

- [ ] **Step 2: Run build to verify it fails before the new services exist**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: FAIL with missing include or undefined type/function errors for `input_service_t`, `NavigationService_RunOnce`, or `PropulsionService_Init`

- [ ] **Step 3: Add the service headers with explicit command/state contracts**

`UserCode/Service/input/input_service.h`
```c
#ifndef INPUT_SERVICE_H
#define INPUT_SERVICE_H

#include "input_gamepad.h"

typedef struct
{
    int16_t input_forward_command;
    int16_t input_vertical_command;
    uint16_t input_buttons;
} input_command_t;

typedef struct
{
    input_gamepad_t *input_gamepad;
    input_command_t input_last_command;
} input_service_t;

void InputService_Init(input_service_t *self, input_gamepad_t *input_gamepad);
void InputService_RunOnce(input_service_t *self);
uint8_t InputService_TakeCommand(input_service_t *self, input_command_t *input_command);

#endif
```

`UserCode/Service/propulsion/propulsion_service.h`
```c
#ifndef PROPULSION_SERVICE_H
#define PROPULSION_SERVICE_H

#include "actuator_thruster.h"
#include "input_service.h"

typedef struct
{
    actuator_thruster_t *propulsion_horizontal;
    actuator_thruster_t *propulsion_vertical;
} propulsion_service_t;

void PropulsionService_Init(propulsion_service_t *self, actuator_thruster_t *propulsion_horizontal, actuator_thruster_t *propulsion_vertical);
void PropulsionService_ApplyCommand(propulsion_service_t *self, const input_command_t *input_command);
void PropulsionService_StopAll(propulsion_service_t *self);

#endif
```

- [ ] **Step 4: Implement the services and remove new work from legacy `Gamepad_Control()`**

`UserCode/Service/input/input_service.c`
```c
#include "input_service.h"

void InputService_Init(input_service_t *self, input_gamepad_t *input_gamepad)
{
    self->input_gamepad = input_gamepad;
    self->input_last_command.input_forward_command = 0;
    self->input_last_command.input_vertical_command = 0;
    self->input_last_command.input_buttons = 0U;
}

void InputService_RunOnce(input_service_t *self)
{
    const input_gamepad_state_t *input_state = InputGamepad_GetState(self->input_gamepad);
    self->input_last_command.input_forward_command = input_state->input_forward;
    self->input_last_command.input_vertical_command = input_state->input_vertical;
    self->input_last_command.input_buttons = input_state->input_buttons;
}

uint8_t InputService_TakeCommand(input_service_t *self, input_command_t *input_command)
{
    *input_command = self->input_last_command;
    return 1U;
}
```

`UserCode/Service/propulsion/propulsion_service.c`
```c
#include "propulsion_service.h"

void PropulsionService_Init(propulsion_service_t *self, actuator_thruster_t *propulsion_horizontal, actuator_thruster_t *propulsion_vertical)
{
    self->propulsion_horizontal = propulsion_horizontal;
    self->propulsion_vertical = propulsion_vertical;
}

void PropulsionService_ApplyCommand(propulsion_service_t *self, const input_command_t *input_command)
{
    ActuatorThruster_SetSpeed(self->propulsion_horizontal, input_command->input_forward_command);
    ActuatorThruster_SetSpeed(self->propulsion_vertical, input_command->input_vertical_command);
}

void PropulsionService_StopAll(propulsion_service_t *self)
{
    ActuatorThruster_Stop(self->propulsion_horizontal);
    ActuatorThruster_Stop(self->propulsion_vertical);
}
```

`UserCode/APP/control.c`
```c
#include "control.h"

void Gamepad_Control(void)
{
}

void FOC_Set_Speed(uint8_t motor_num, int16_t speed)
{
    (void)motor_num;
    (void)speed;
}
```

- [ ] **Step 5: Register the service tasks in `AppConfig_Init()` and expose service accessors for `app_main.c`**

`UserCode/App/app_config.h`
```c
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "input_service.h"
#include "navigation_service.h"
#include "propulsion_service.h"

void AppConfig_Init(void);
input_service_t *AppConfig_GetInputService(void);
propulsion_service_t *AppConfig_GetPropulsionService(void);
navigation_service_t *AppConfig_GetNavigationService(void);

#endif
```

`UserCode/App/app_config.c`
```c
static scheduler_task_t input_task =
{
    .task_name = "input_task",
    .task_period_ms = 10U,
    .task_last_tick = 0U,
    .task_run = AppConfig_InputTask,
    .task_context = &operator_input_service,
};

static scheduler_task_t navigation_task =
{
    .task_name = "navigation_task",
    .task_period_ms = 20U,
    .task_last_tick = 0U,
    .task_run = AppConfig_NavigationTask,
    .task_context = &vehicle_navigation_service,
};

input_service_t *AppConfig_GetInputService(void)
{
    return &operator_input_service;
}

propulsion_service_t *AppConfig_GetPropulsionService(void)
{
    return &vehicle_propulsion_service;
}

navigation_service_t *AppConfig_GetNavigationService(void)
{
    return &vehicle_navigation_service;
}
```

`UserCode/App/app_main.c`
```c
#include "propulsion_service.h"
#include "input_service.h"
#include "app_config.h"

void AppMain_RunOnce(void)
{
    input_command_t input_command;
    SchedulerService_RunOnce(HAL_GetTick());
    if (InputService_TakeCommand(AppConfig_GetInputService(), &input_command) != 0U)
    {
        PropulsionService_ApplyCommand(AppConfig_GetPropulsionService(), &input_command);
    }
}
```

- [ ] **Step 6: Add service files to Keil and build green**

Add these paths under new groups `Service/input`, `Service/navigation`, and `Service/propulsion` in `MDK-ARM/Project.uvprojx`:
```text
..\UserCode\Service\input\input_service.c
..\UserCode\Service\navigation\navigation_service.c
..\UserCode\Service\propulsion\propulsion_service.c
```

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: PASS with `0 Error(s)`

- [ ] **Step 7: Commit**

```bash
git add MDK-ARM/Project.uvprojx UserCode/App/app_config.c UserCode/App/app_main.c UserCode/APP/control.h UserCode/APP/control.c UserCode/Service/input/input_service.h UserCode/Service/input/input_service.c UserCode/Service/navigation/navigation_service.h UserCode/Service/navigation/navigation_service.c UserCode/Service/propulsion/propulsion_service.h UserCode/Service/propulsion/propulsion_service.c
git commit -m "refactor: move input navigation and propulsion into services"
```

### Task 4: Replace blocking water-tank flow with Ballast and Safety services

**Files:**
- Create: `UserCode/Service/ballast/ballast_service.h`
- Create: `UserCode/Service/ballast/ballast_service.c`
- Create: `UserCode/Service/safety/safety_service.h`
- Create: `UserCode/Service/safety/safety_service.c`
- Modify: `UserCode/App/app_config.c`
- Modify: `UserCode/App/app_main.c`
- Modify: `UserCode/APP/WaterTank.h`
- Modify: `UserCode/APP/WaterTank.c`
- Modify: `Core/Src/main.c`
- Modify: `MDK-ARM/Project.uvprojx`

**Interfaces:**
- Consumes:
  - `actuator_stepper_t` from Task 2
  - `sensor_water_leak_t` from Task 2
  - `navigation_service_t` from Task 3
  - limit-switch events surfaced by GPIO interrupt path in Task 5
- Produces:
  - `void BallastService_Init(ballast_service_t *self, actuator_stepper_t *ballast_front_stepper, actuator_stepper_t *ballast_rear_stepper);`
  - `ballast_service_t *BallastService_GetDefault(void);`
  - `void BallastService_RequestFill(ballast_service_t *self, ballast_tank_id_t ballast_tank_id, float ballast_volume_ml);`
  - `void BallastService_RequestDrain(ballast_service_t *self, ballast_tank_id_t ballast_tank_id, float ballast_volume_ml);`
  - `void BallastService_RunOnce(ballast_service_t *self);`
  - `void BallastService_HandleLimitEvent(ballast_service_t *self, ballast_tank_id_t ballast_tank_id, ballast_limit_event_t ballast_limit_event);`
  - `void SafetyService_Init(safety_service_t *self, sensor_water_leak_t *safety_water_leak, ballast_service_t *safety_ballast);`
  - `void SafetyService_RunOnce(safety_service_t *self);`
  - `uint8_t SafetyService_IsEmergencyActive(const safety_service_t *self);`
  - `safety_service_t *AppConfig_GetSafetyService(void);`
  - `ballast_service_t *AppConfig_GetBallastService(void);`

- [ ] **Step 1: Write the failing compile check by removing the blocking leak logic from `main.c` and registering a scheduler-backed safety task instead**

```c
/* delete direct Water_Check() / Water_Tank_Draining_To_Empty() / while(1) block */
```

`UserCode/App/app_config.c`
```c
#include "safety_service.h"

static void AppConfig_SafetyTask(void *context)
{
    SafetyService_RunOnce((safety_service_t *)context);
}
```

Expected architecture after this task: `AppMain_RunOnce()` keeps only `SchedulerService_RunOnce(HAL_GetTick());` and safety work runs from the scheduler table.

- [ ] **Step 2: Run build to verify it fails before ballast and safety services exist**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: FAIL with missing include or undefined symbol errors for `SafetyService_RunOnce`, `ballast_service_t`, or `safety_service_t`

- [ ] **Step 3: Add non-blocking ballast service contracts**

`UserCode/Service/ballast/ballast_service.h`
```c
#ifndef BALLAST_SERVICE_H
#define BALLAST_SERVICE_H

#include "actuator_stepper.h"

typedef enum
{
    ballast_tank_front = 0,
    ballast_tank_rear = 1,
} ballast_tank_id_t;

typedef enum
{
    ballast_limit_none = 0,
    ballast_limit_empty = 1,
    ballast_limit_full = 2,
} ballast_limit_event_t;

typedef struct
{
    actuator_stepper_t *ballast_front_stepper;
    actuator_stepper_t *ballast_rear_stepper;
    float ballast_front_volume_ml;
    float ballast_rear_volume_ml;
    float ballast_front_target_ml;
    float ballast_rear_target_ml;
    uint8_t ballast_front_busy;
    uint8_t ballast_rear_busy;
} ballast_service_t;

void BallastService_Init(ballast_service_t *self, actuator_stepper_t *ballast_front_stepper, actuator_stepper_t *ballast_rear_stepper);
ballast_service_t *BallastService_GetDefault(void);
void BallastService_RequestFill(ballast_service_t *self, ballast_tank_id_t ballast_tank_id, float ballast_volume_ml);
void BallastService_RequestDrain(ballast_service_t *self, ballast_tank_id_t ballast_tank_id, float ballast_volume_ml);
void BallastService_RunOnce(ballast_service_t *self);
void BallastService_HandleLimitEvent(ballast_service_t *self, ballast_tank_id_t ballast_tank_id, ballast_limit_event_t ballast_limit_event);
void BallastService_IrqOnLimit(ballast_tank_id_t ballast_tank_id, ballast_limit_event_t ballast_limit_event);

#endif
```

`UserCode/Service/safety/safety_service.h`
```c
#ifndef SAFETY_SERVICE_H
#define SAFETY_SERVICE_H

#include "sensor_water_leak.h"
#include "ballast_service.h"

typedef struct
{
    sensor_water_leak_t *safety_water_leak;
    ballast_service_t *safety_ballast;
    uint8_t safety_emergency_active;
} safety_service_t;

void SafetyService_Init(safety_service_t *self, sensor_water_leak_t *safety_water_leak, ballast_service_t *safety_ballast);
void SafetyService_RunOnce(safety_service_t *self);
uint8_t SafetyService_IsEmergencyActive(const safety_service_t *self);

#endif
```

- [ ] **Step 4: Implement the non-blocking emergency drain path and bridge the legacy tank API**

`UserCode/Service/ballast/ballast_service.c`
```c
#include "ballast_service.h"

static ballast_service_t *ballast_service_default_owner;

void BallastService_Init(ballast_service_t *self, actuator_stepper_t *ballast_front_stepper, actuator_stepper_t *ballast_rear_stepper)
{
    ballast_service_default_owner = self;
    self->ballast_front_stepper = ballast_front_stepper;
    self->ballast_rear_stepper = ballast_rear_stepper;
}

ballast_service_t *BallastService_GetDefault(void)
{
    return ballast_service_default_owner;
}
```

`UserCode/Service/safety/safety_service.c`
```c
#include "safety_service.h"

void SafetyService_Init(safety_service_t *self, sensor_water_leak_t *safety_water_leak, ballast_service_t *safety_ballast)
{
    self->safety_water_leak = safety_water_leak;
    self->safety_ballast = safety_ballast;
    self->safety_emergency_active = 0U;
}

void SafetyService_RunOnce(safety_service_t *self)
{
    if (SensorWaterLeak_IsDetected(self->safety_water_leak) != 0U)
    {
        self->safety_emergency_active = 1U;
        BallastService_RequestDrain(self->safety_ballast, ballast_tank_front, 1000.0f);
        BallastService_RequestDrain(self->safety_ballast, ballast_tank_rear, 1000.0f);
    }
}

uint8_t SafetyService_IsEmergencyActive(const safety_service_t *self)
{
    return self->safety_emergency_active;
}
```

`UserCode/APP/WaterTank.c`
```c
#include "WaterTank.h"
#include "ballast_service.h"

void Water_Tank_Filling(water_tank_t *tank, float delta_xML)
{
    BallastService_RequestFill(BallastService_GetDefault(),
        (tank == &tank_front) ? ballast_tank_front : ballast_tank_rear,
        delta_xML);
}

void Water_Tank_Draining(water_tank_t *tank, float delta_xML)
{
    BallastService_RequestDrain(BallastService_GetDefault(),
        (tank == &tank_front) ? ballast_tank_front : ballast_tank_rear,
        delta_xML);
}

void Water_Tank_Draining_To_Empty(water_tank_t *tank)
{
    BallastService_RequestDrain(BallastService_GetDefault(),
        (tank == &tank_front) ? ballast_tank_front : ballast_tank_rear,
        1000.0f);
}
```

- [ ] **Step 5: Register ballast and safety tasks in `AppConfig_Init()` and expose accessors**

`UserCode/App/app_config.h`
```c
ballast_service_t *AppConfig_GetBallastService(void);
safety_service_t *AppConfig_GetSafetyService(void);
```

`UserCode/App/app_config.c`
```c
static ballast_service_t vehicle_ballast_service;
static safety_service_t vehicle_safety_service;

static void AppConfig_BallastTask(void *context)
{
    BallastService_RunOnce((ballast_service_t *)context);
}

static void AppConfig_SafetyTask(void *context)
{
    SafetyService_RunOnce((safety_service_t *)context);
}

ballast_service_t *AppConfig_GetBallastService(void)
{
    return &vehicle_ballast_service;
}

safety_service_t *AppConfig_GetSafetyService(void)
{
    return &vehicle_safety_service;
}
```

Register:
```c
static scheduler_task_t ballast_task =
{
    .task_name = "ballast_task",
    .task_period_ms = 20U,
    .task_last_tick = 0U,
    .task_run = AppConfig_BallastTask,
    .task_context = &vehicle_ballast_service,
};

static scheduler_task_t safety_task =
{
    .task_name = "safety_task",
    .task_period_ms = 50U,
    .task_last_tick = 0U,
    .task_run = AppConfig_SafetyTask,
    .task_context = &vehicle_safety_service,
};
```

- [ ] **Step 6: Add new service files to Keil, build green, then run a hardware smoke check**

Add these paths to `MDK-ARM/Project.uvprojx`:
```text
..\UserCode\Service\ballast\ballast_service.c
..\UserCode\Service\safety\safety_service.c
```

Run build: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: PASS with `0 Error(s)`

Hardware smoke check:
```text
1. Power on board.
2. Trigger normal gamepad input; confirm main loop stays responsive.
3. Simulate leak input; confirm board does not enter while(1).
4. Confirm ballast request persists across scheduler ticks instead of blocking delay loops.
```

- [ ] **Step 7: Commit**

```bash
git add Core/Src/main.c MDK-ARM/Project.uvprojx UserCode/App/app_config.c UserCode/App/app_main.c UserCode/APP/WaterTank.h UserCode/APP/WaterTank.c UserCode/Service/ballast/ballast_service.h UserCode/Service/ballast/ballast_service.c UserCode/Service/safety/safety_service.h UserCode/Service/safety/safety_service.c
git commit -m "refactor: replace blocking ballast flow with services"
```

### Task 5: Decouple ISR routing and finish naming migration for touched modules

**Files:**
- Modify: `UserCode/BSP/bsp_interrupt.h`
- Modify: `UserCode/BSP/bsp_interrupt.c`
- Modify: `UserCode/Device/sensor/input_gamepad.h`
- Modify: `UserCode/Device/sensor/input_gamepad.c`
- Modify: `UserCode/Service/ballast/ballast_service.h`
- Modify: `UserCode/Service/ballast/ballast_service.c`
- Modify: `UserCode/App/app_config.c`
- Modify: `UserCode/App/app_main.c`
- Modify: `MDK-ARM/Project.uvprojx` if file groups changed

**Interfaces:**
- Consumes:
  - `InputGamepad_OnByte(input_gamepad_t *self, uint8_t input_byte);`
  - `BallastService_HandleLimitEvent(ballast_service_t *self, ballast_tank_id_t ballast_tank_id, ballast_limit_event_t ballast_limit_event);`
  - existing `HAL_UART_Receive_IT`, `HAL_UARTEx_ReceiveToIdle_DMA`, `Foc_Loop`
- Produces:
  - `void InputGamepad_IrqOnByte(uint8_t input_byte);`
  - `void BallastService_IrqOnLimit(ballast_tank_id_t ballast_tank_id, ballast_limit_event_t ballast_limit_event);`
  - `void AppConfig_IrqOnBalanceTick(void);`

- [ ] **Step 1: Write the failing compile check by replacing direct APP calls inside `bsp_interrupt.c` with adapter/service hooks**

```c
if (huart == &gamepad_huart)
{
    InputGamepad_IrqOnByte(gamepad_rxByte);
    HAL_UART_Receive_IT(huart, &gamepad_rxByte, 1U);
}
```

```c
if (GPIO_Pin == tank_front_full_Pin)
{
    BallastService_IrqOnLimit(ballast_tank_front, ballast_limit_full);
}
```

- [ ] **Step 2: Run build to verify it fails before the new IRQ hook functions exist**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: FAIL with undefined symbol errors for `InputGamepad_IrqOnByte` or `BallastService_IrqOnLimit`

- [ ] **Step 3: Add the thin IRQ handoff functions and keep ISR code business-free**

`UserCode/Device/sensor/input_gamepad.h`
```c
void InputGamepad_IrqOnByte(uint8_t input_byte);
```

`UserCode/Device/sensor/input_gamepad.c`
```c
#include "input_gamepad.h"
#include "gamepad.h"

static input_gamepad_t *input_gamepad_irq_owner;

void InputGamepad_Init(input_gamepad_t *self)
{
    input_gamepad_irq_owner = self;
    Gamepad_Init(&gamepad_huart);
}

void InputGamepad_IrqOnByte(uint8_t input_byte)
{
    if (input_gamepad_irq_owner != 0)
    {
        Gamepad_RxCallback(input_byte);
    }
}
```

`UserCode/Service/ballast/ballast_service.h`
```c
void BallastService_IrqOnLimit(ballast_tank_id_t ballast_tank_id, ballast_limit_event_t ballast_limit_event);
```

`UserCode/Service/ballast/ballast_service.c`
```c
#include "ballast_service.h"

static ballast_service_t *ballast_service_irq_owner;

void BallastService_Init(ballast_service_t *self, actuator_stepper_t *ballast_front_stepper, actuator_stepper_t *ballast_rear_stepper)
{
    ballast_service_irq_owner = self;
    self->ballast_front_stepper = ballast_front_stepper;
    self->ballast_rear_stepper = ballast_rear_stepper;
}

void BallastService_IrqOnLimit(ballast_tank_id_t ballast_tank_id, ballast_limit_event_t ballast_limit_event)
{
    if (ballast_service_irq_owner != 0)
    {
        BallastService_HandleLimitEvent(ballast_service_irq_owner, ballast_tank_id, ballast_limit_event);
    }
}
```

- [ ] **Step 4: Clean up touched names to the approved naming convention**

Apply this exact rename set in migrated code:
```text
scheduler_task_count        keep
vehicle_ballast_service     keep
vehicle_navigation_service  keep
operator_input_service      keep
AppMain_Init                keep
AppMain_RunOnce             keep
SchedulerService_RunOnce    keep
BallastService_RequestFill  keep
BallastService_RequestDrain keep
SafetyService_RunOnce       keep
```

Delete or stop calling these legacy orchestration entry points from scheduler/app code:
```text
Gamepad_Control
Drone_Balance_Control
Scheduler_Set_DroneBalanceControlFlag
Scheduler_Get_DroneBalanceControlFlag
```

- [ ] **Step 5: Build green and run focused interrupt smoke verification**

Run build: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: PASS with `0 Error(s)`

Hardware smoke check:
```text
1. Confirm gamepad bytes still arrive and drive propulsion.
2. Confirm GPS and IMU reception still continue after ISR cleanup.
3. Confirm tank limit switches update ballast state without direct WaterTank ISR state mutation.
4. Confirm TIM1/TIM8 still call Foc_Loop() and nothing else business-level happens there.
```

- [ ] **Step 6: Commit**

```bash
git add UserCode/BSP/bsp_interrupt.h UserCode/BSP/bsp_interrupt.c UserCode/Device/sensor/input_gamepad.h UserCode/Device/sensor/input_gamepad.c UserCode/Service/ballast/ballast_service.h UserCode/Service/ballast/ballast_service.c UserCode/App/app_config.c UserCode/App/app_main.c
git commit -m "refactor: decouple interrupts from app business logic"
```

### Task 6: Final integration cleanup and legacy wrapper removal

**Files:**
- Modify: `Core/Src/main.c`
- Modify: `UserCode/APP/control.h`
- Modify: `UserCode/APP/control.c`
- Modify: `UserCode/APP/WaterTank.h`
- Modify: `UserCode/APP/WaterTank.c`
- Modify: `UserCode/APP/app_gps.h`
- Modify: `UserCode/APP/app_gps.c`
- Modify: `UserCode/APP/app_thrusters.h`
- Modify: `UserCode/APP/app_thrusters.c`
- Modify: `README.md`

**Interfaces:**
- Consumes: all final `App / Service / Device` interfaces from Tasks 1-5
- Produces: a codebase where `Core/Src/main.c` is only HAL init + app handoff, and legacy APP modules are wrappers or removed from active control flow

- [ ] **Step 1: Write the failing compile check by deleting the last direct business calls from `main.c` and the last scheduler references from legacy APP files**

`Core/Src/main.c` target shape:
```c
/* USER CODE BEGIN 2 */
AppMain_Init();
/* USER CODE END 2 */

while (1)
{
    AppMain_RunOnce();
}
```

Legacy wrappers target shape:
```c
/* control.c */
void Gamepad_Control(void)
{
}
```

```c
/* app_gps.c */
void APP_GPS_Task(void)
{
}
```

- [ ] **Step 2: Run build to catch any final hidden dependency on old orchestration symbols**

Run: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: FAIL only if any active code path still depends on old direct orchestration symbols

- [ ] **Step 3: Remove or shrink the leftover wrappers until build goes green**

Allowed end states:
```text
- Keep legacy APP entry points as empty compatibility shims for one migration cycle.
- Or remove them from headers and source if no call sites remain.
```

Disallowed end states:
```text
- main.c directly calling Gamepad_Control / JY901_Task / APP_GPS_Task / Water_Tank_Update_Handler
- any new business while(1) loop outside main
- any new direct dependency from Service into H_Tmc2209 globals
```

- [ ] **Step 4: Run final build and final hardware regression**

Run build: `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
Expected: PASS with `0 Error(s)`

Hardware regression checklist:
```text
1. Power-on init completes.
2. Gamepad-driven propulsion still works.
3. GPS/IMU snapshots still update.
4. Ballast commands run without blocking the loop.
5. Leak handling no longer halts the firmware.
6. Limit switches still clamp ballast state.
7. foc_lib behavior is unchanged from the outside.
```

- [ ] **Step 5: Update top-level documentation and commit**

`README.md` section to add:
```md
## Firmware Layers
- `Core/` and `Drivers/`: HAL startup and MCU platform code
- `UserCode/Device/`: hardware adapters over existing drivers
- `UserCode/Service/`: business state machines and control logic
- `UserCode/App/`: application composition and scheduler entry points
- `UserCode/foc_lib/`: stable FOC library, wrapped only from outside
```

Commit:
```bash
git add Core/Src/main.c UserCode/APP/control.h UserCode/APP/control.c UserCode/APP/WaterTank.h UserCode/APP/WaterTank.c UserCode/APP/app_gps.h UserCode/APP/app_gps.c UserCode/APP/app_thrusters.h UserCode/APP/app_thrusters.c README.md
git commit -m "refactor: finish layered firmware migration"
```

## Risks and review checkpoints
- `app_thrusters.c` currently wraps FOC init; confirm there is only one owner calling FOC init after Task 2 to avoid duplicate initialization.
- `bsp_interrupt.c` currently mixes timer and UART logic; review each ISR path after Task 5 so no business code remains except `Foc_Loop()`.
- `WaterTank.c` currently owns blocking and stateful logic; after Task 4, verify every call path is non-blocking.
- `Project.uvprojx` must be updated in every task that adds sources; a missing file in Keil can look like an architecture bug when it is only a project configuration issue.
- If legacy wrappers stay for one migration cycle, review that no scheduler/app code still depends on them before Task 6 closes.

## Verification summary
- Build after every task with `UV4.exe -b "E:/My_MCU_Project/STM32H723/DW/MDK-ARM/Project.uvprojx" -t "Project"`
- Use focused board smoke checks after Tasks 4, 5, and 6
- Treat any return of blocking loops, direct ISR business branching, or direct `Motor[]` access from `Service` as a failed regression
