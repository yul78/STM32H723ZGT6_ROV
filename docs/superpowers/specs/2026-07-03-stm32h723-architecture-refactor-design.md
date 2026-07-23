# STM32H723 固件架构重构设计

- 日期：2026-07-03
- 项目：DW / STM32H723 水下无人机固件
- 范围：代码框架重构设计，不包含 `UserCode/foc_lib/` 内部修改

## 1. 背景与目标
当前工程虽然已经按 `Core / APP / BSP / MID / foc_lib` 做了物理分目录，但逻辑边界并不清晰：`Core/Src/main.c` 同时承担初始化、调度、业务决策、异常处理和调试输出；`UserCode/BSP/bsp_interrupt.c` 将中断回调、设备接收、控制调度混在一起；`UserCode/APP/WaterTank.c` 直接操作步进电机驱动与全局状态；多个模块跨层直接读写全局变量。

本次重构目标是：在适合裸机 MCU 的前提下，建立清晰、稳定、可迁移的软件分层，降低耦合、统一命名、明确状态所有权，并为后续重构提供一条可执行的迁移路线。

## 2. 约束与原则
- 允许大幅重构目录与模块边界。
- 运行模型保持“轻量调度”，不引入 RTOS 或重型事件框架。
- 允许使用 `struct / struct* / function pointer` 等 C 风格封装模拟面向对象。
- `UserCode/foc_lib/` 视为稳定底层库，不改其内部实现，只在外部包一层适配。
- 变量名采用**模块前缀+下划线**。
- 函数名采用**类型前缀+驼峰**。
- CubeMX/HAL 生成区尽量不承载业务逻辑。
- ISR 中不再写业务分支，中断只负责采样、缓存、置位和重新挂接。

## 3. 当前架构反模式
### 3.1 主循环上帝对象
`Core/Src/main.c` 中直接串联 `Gamepad_Control()`、`JY901_Task()`、`APP_GPS_Task()`、`Water_Tank_*()`、`Water_Check()` 等逻辑，还夹杂按钮到水舱动作映射、调试打印、阻塞式异常处理。调度层与业务层没有分开。

### 3.2 中断层泄漏业务逻辑
`UserCode/BSP/bsp_interrupt.c` 中的 `HAL_TIM_PeriodElapsedCallback()` 直接触发 `Scheduler_Set_DroneBalanceControlFlag()`、`Foc_Loop()`；`HAL_UART_RxCpltCallback()` 直接调用 `Gamepad_RxCallback()`、GPS FIFO 写入。ISR 层承担了本该属于应用编排层或驱动适配层的职责。

### 3.3 应用层直接依赖具体驱动
`UserCode/APP/WaterTank.c` 直接调用 `Motor_Set()`、`Motor_Stop()`、`Motor_GetStep()`，还直接访问 `Motor[]` 全局数组。高层状态机无法脱离 TMC2209 实现单独理解和替换。

### 3.4 状态与数据所有权混乱
`tank_front/tank_rear`、`jy901_data`、`gps_data`、`Motor[]` 等状态散落在不同模块，既没有清晰 owner，也没有统一访问边界，导致读写路径不透明。

### 3.5 阻塞式安全流程
`Water_Tank_Draining_To_Empty()` 与 `Draining_Test()` 使用 `while(1)` + delay 的阻塞方式，`main.c` 在进水检测后又直接 `while(1)` 停死。异常处理和安全策略没有独立服务层承接。

### 3.6 命名风格与抽象风格不统一
同一工程内并存 `Water_Tank_Init`、`Gamepad_Control`、`FOC_Set_Speed`、`drone_balance_control_flag`、`jy901_data` 等不同风格，进一步放大理解和迁移成本。

## 4. 推荐架构
推荐采用 **轻量调度 + 分层服务 + 设备接口适配** 架构，而不是继续以 `main.c` 为中心做功能拼接。核心原则如下：
- 中断只做事件采集/驱动喂数，不做业务决策。
- 驱动层只暴露能力，不表达业务含义。
- 服务层负责状态机、控制策略、安全逻辑。
- 应用编排层只负责初始化、任务顺序和任务周期。
- 通过 C 风格接口对象封装底层实现，让业务层依赖抽象能力，而不是依赖具体驱动文件。

### 4.1 Layer 1: Platform / HAL
位置：`Core/`、`Drivers/`

职责：
- 片上资源初始化
- HAL ISR 入口
- 芯片级启动与底层配置

规则：
- 不在这里写业务状态机
- 不在这里写控制策略

### 4.2 Layer 2: Device Adapter
建议目录：`UserCode/Device/`
- `actuator/`
- `sensor/`
- `comm/`
- `common/`

职责：
- 把具体设备封装成可替换对象
- 对上提供统一接口结构体
- 只表达设备能力，不表达业务意图

建议对象：
- `ActuatorStepper_*`：包装 `MID/H_Tmc2209.*`
- `ActuatorThruster_*`：包装 `APP/app_thrusters.*` 与 `foc_lib` 外部调用
- `SensorImu_*`：包装 JY901 能力
- `SensorGps_*`：包装 GPS 收数与解析能力
- `InputGamepad_*`：包装手柄接收与解析能力
- `SensorWaterLeak_*`：包装 ADC 进水检测

示例接口：
```c
typedef struct
{
    void (*start)(void *self);
    void (*stop)(void *self);
    void (*setTarget)(void *self, int32_t target);
    int32_t (*getProgress)(void *self);
    void *context;
} actuator_stepper_if_t;
```

### 4.3 Layer 3: Domain Service
建议目录：`UserCode/Service/`
- `scheduler/`
- `input/`
- `propulsion/`
- `ballast/`
- `navigation/`
- `safety/`
- `telemetry/`

职责划分：
- `input/`：读取并标准化手柄输入，输出统一 command/state
- `propulsion/`：推进器目标解算、速度命令输出，只调用 thruster adapter
- `ballast/`：水舱状态机、注排水目标、水量估算、限位事件处理
- `navigation/`：IMU/GPS 数据快照与姿态/位置状态维护
- `safety/`：漏水、姿态异常、失控保护、紧急排水策略，必须改成非阻塞状态机
- `scheduler/`：维护轻量任务表和周期执行标志

### 4.4 Layer 4: App Orchestrator
建议目录：`UserCode/App/`
- `app_main.c`
- `app_main.h`
- `app_config.c`
- `app_runtime.c`

职责：
- 系统初始化顺序
- 模块装配（dependency wiring）
- 周期任务调度
- 主循环执行节拍

最终 `main.c` 只保留：
1. HAL/Cube 初始化
2. `AppMain_Init()`
3. `while (1) { AppMain_RunOnce(); }`

## 5. 运行模型
采用**轻量周期调度器**，不走复杂消息总线。

建议把当前隐式的“20ms 大循环 + 若干 ISR”整理成显式任务表：
- 1ms/2ms：高频底层维护（如果需要，仅置位，不做业务）
- 10ms：输入采样、推进更新
- 20ms：姿态/导航更新
- 20ms：水舱状态机更新
- 50ms/100ms：安全检查、遥测输出

调度器结构建议：
```c
typedef struct
{
    const char *task_name;
    uint16_t task_period_ms;
    uint32_t task_last_tick;
    void (*task_run)(void *context);
    void *task_context;
} scheduler_task_t;
```

## 6. 推荐目录布局
```text
UserCode/
├── App/
│   ├── app_main.c
│   ├── app_main.h
│   ├── app_config.c
│   └── app_runtime.c
├── Service/
│   ├── scheduler/
│   ├── input/
│   ├── propulsion/
│   ├── ballast/
│   ├── navigation/
│   ├── safety/
│   └── telemetry/
├── Device/
│   ├── actuator/
│   ├── sensor/
│   ├── comm/
│   └── common/
├── BSP/
├── MID/
└── foc_lib/
```

含义：
- `BSP/MID/foc_lib`：保留底层与已有驱动能力
- `Device`：新增适配层，把旧驱动转成稳定接口
- `Service`：新增业务层，承接所有状态机和控制逻辑
- `App`：新增编排层，替代现在臃肿的 `main.c`

## 7. 命名规范
### 7.1 变量名
采用**模块前缀 + 下划线**：
- `gamepad_state`
- `ballast_front_tank`
- `navigation_attitude`
- `safety_leak_detected`
- `scheduler_task_table`

### 7.2 函数名
采用**类型/模块前缀 + 驼峰**：
- `GamepadInput_Init`
- `GamepadInput_RunOnce`
- `BallastService_RequestFill`
- `BallastService_HandleLimitEvent`
- `PropulsionService_SetTarget`
- `SafetyService_RunOnce`
- `AppMain_Init`
- `AppMain_RunOnce`

### 7.3 统一原则
- 一个模块的对外函数名全部同前缀
- 一个模块的内部静态变量全部同模块前缀
- 缩写统一：GPS/IMU/ADC/FOC 保持大写，不混写
- 新设计中不再混用 `Water_Tank_* / Gamepad_* / app_* / bsp_*` 这类多来源风格

## 8. 必须保持稳定的边界
1. **`UserCode/foc_lib/` 不改内部内容**  
   只允许在 `Device/actuator/` 外围建立 thruster adapter，对外隐藏 `Foc_Init()`、`Foc_Loop()`、`Foc_Set_Speed()` 的调用细节。

2. **CubeMX/HAL 生成区尽量不承载业务**  
   `Core/Src/main.c`、`stm32h7xx_it.c` 等只保留底层入口与少量装配代码。

3. **ISR 中不再写业务分支**  
   ISR 只做三件事：
   - 采样/收字节
   - 更新驱动层缓存或事件标志
   - 重新挂接中断/DMA

## 9. 可复用的现有能力
以下能力建议保留并包适配，不要重写：
- `UserCode/APP/gamepad.c` 中已有的手柄协议解析能力，尤其 `Gamepad_RxCallback()`、`Gamepad_GetData()`
- `UserCode/MID/H_Tmc2209.*` 中已有的步进电机控制能力，如 `Motor_Set()`、`Motor_Stop()`、`Motor_GetStep()`
- `UserCode/BSP` + `UserCode/MID` 中现有的 GPS/JY901 数据接收与解析能力
- `UserCode/foc_lib/` 及其外部调用入口 `Foc_Init()`、`Foc_Loop()`

重构重点不是推倒已有成熟算法，而是把它们放进正确的层级和依赖方向里。

## 10. 迁移计划
### Phase 1：抽出编排层
关键文件：
- `Core/Src/main.c`
- 新增 `UserCode/App/app_main.*`
- 新增 `UserCode/Service/scheduler/*`

目标：
- 把 `main.c` 中业务循环迁出
- 建立 `AppMain_Init()` / `AppMain_RunOnce()`
- 明确任务周期与调用顺序

### Phase 2：拆 ISR 与业务层
关键文件：
- `UserCode/BSP/bsp_interrupt.c`
- `Core/Src/stm32h7xx_it.c`
- `UserCode/APP/gamepad.c`
- GPS/JY901 对应 BSP/MID 文件

目标：
- ISR 只负责投递数据或置位标志
- 把控制标志、解析调用转移到 Device/Service 层

### Phase 3：建立 Device Adapter 层
关键文件：
- 新增 `UserCode/Device/actuator/*`
- 新增 `UserCode/Device/sensor/*`
- `UserCode/MID/H_Tmc2209.*`
- `UserCode/APP/app_thrusters.*`

目标：
- 用接口对象包装 stepper、thruster、IMU、GPS、water leak detector
- `foc_lib` 不改，只在 adapter 中包起来

### Phase 4：重写业务服务层
关键文件：
- `UserCode/APP/WaterTank.*`
- `UserCode/APP/control.c`
- 新增 `UserCode/Service/ballast/*`
- 新增 `UserCode/Service/propulsion/*`
- 新增 `UserCode/Service/safety/*`
- 新增 `UserCode/Service/navigation/*`

目标：
- 把 `WaterTank` 重写成非阻塞 ballast service
- 把 `control.c` 拆成 input → command → propulsion/ballast 的清晰流向
- 把漏水处理从“阻塞排空+停死”改成 safety service 的状态机

### Phase 5：命名统一与旧接口下线
关键文件：
- 所有新建 service/device/app 文件
- 逐步收缩旧 `APP/` 下直接暴露的历史接口

目标：
- 按统一命名规范重命名
- 删除跨层直连路径
- 保留兼容期尽量短，避免长期双轨接口

## 11. 验证标准
设计落地后，验证至少覆盖：
1. Keil 工程可完整编译通过，新增目录与文件已纳入工程。
2. 上电后各服务初始化顺序明确，`foc_lib` 仍能正常初始化并工作。
3. `AppMain_RunOnce()` 周期稳定，没有新的阻塞循环。
4. 手柄输入仍能被接收、解析并转成推进/水舱命令。
5. 推进器速度控制经 adapter 调用 `foc_lib`，行为与重构前一致或更清晰。
6. 注排水状态机不再直接读写底层电机全局数组，限位事件可正确收敛状态。
7. 漏水、姿态异常等由 safety service 非阻塞处理，不再出现 `while(1)` 卡死主循环的设计。
8. 新增/迁移模块的函数名与变量名全部符合本次约定。

## 12. 最终建议
这次最值得做的，不是简单把文件挪整齐，而是建立三个长期稳定的边界：
- **设备能力边界**：Device adapter
- **业务语义边界**：Domain service
- **调度编排边界**：App orchestrator

只要这三个边界建立起来，`main.c`、`bsp_interrupt.c`、`WaterTank.c` 目前的混乱就会自然收敛；而 `foc_lib` 也能以黑盒能力的方式稳定接入，不会被这次重构波及。
