# AI 工程交接上下文

本文档用于配合 `git log` 快速接手当前工程。目标是让新的 AI 或开发者不用重新摸索 CubeMX、Keil、FDCAN、DJI 电机调试链路和 PC 回归测试。

## 项目概览

- 工程路径：`D:\desktop\2026RC\Control\single_motor_test`
- 当前基线提交：`482df95 通用化 DJI 单电机调试任务`
- 工程性质：真实 STM32CubeMX 生成工程，不是手工仿造结构。
- 目标芯片：`STM32H723ZGTx`
- IDE/工具链：`MDK-ARM`，Keil 使用 ArmClang/AC6。
- RTOS：CMSIS-RTOS2 / FreeRTOS。
- 应用代码语言：C。
- 当前功能：单 DJI 电机调试工程，编译期选择 CAN1/CAN2/CAN3、电机 ID 1-8、电机型号 M2006/C610 或 M3508/C620。
- 当前默认配置：`CAN2 + ID2 + M2006/C610`，配置入口在 `Core/Inc/dji_motor_debug_config.h`。

## 关键文件

- `single_motor_test.ioc`：CubeMX 工程源文件，硬件和中间件配置从这里生成。
- `MDK-ARM/single_motor_test.uvprojx`：Keil 工程文件，已设置 `<uAC6>1</uAC6>`，不要随意改回 ARMCC5。
- `MDK-ARM/single_motor_test.uvoptx`：Keil 选项文件，可能包含 Watch 配置；源文件增删改名时也要同步检查。
- `Core/Inc/dji_motor_debug_config.h`：单电机调试编译期配置，包含 CAN 通道、电机 ID、电机类型。
- `Core/Inc/dji_motor.h` 和 `Core/Src/dji_motor.c`：DJI 电机通用协议层，负责反馈解析、电流单位换算、控制帧 ID/slot 打包。
- `Core/Inc/dji_motor_task.h` 和 `Core/Src/dji_motor_task.c`：RTOS 电机调试任务、FDCAN 收发、调试变量更新。
- `Core/Inc/dji_motor_task_logic.h` 和 `Core/Src/dji_motor_task_logic.c`：任务纯控制逻辑，供生产任务和 PC 测试共用。
- `Core/Inc/pid.h` 和 `Core/Src/pid.c`：增量式 PID 和位置式 PID，使用真实 `dt_s`。
- `tests/pc/test_dji_motor_control.c`：PC 侧回归测试，覆盖 PID、反馈解析、电流换算、控制帧打包和任务纯逻辑。
- `.vscode/tasks.json`：VS Code 任务，包含 PC 测试和 Keil 构建入口。

## 编译期调试配置

配置文件：`Core/Inc/dji_motor_debug_config.h`

- `DJI_DEBUG_CAN_CHANNEL`：只能为 `1U`、`2U` 或 `3U`。
- `DJI_DEBUG_MOTOR_ID`：只能为 `1U` 到 `8U`。
- `DJI_DEBUG_MOTOR_TYPE`：只能为 `DJI_MOTOR_TYPE_M2006` 或 `DJI_MOTOR_TYPE_M3508`。

非法值会通过 `#error` 在编译期报错。当前默认值为：

```c
#define DJI_DEBUG_CAN_CHANNEL      (2U)
#define DJI_DEBUG_MOTOR_ID         (2U)
#define DJI_DEBUG_MOTOR_TYPE       DJI_MOTOR_TYPE_M2006
```

## DJI 电机协议规则

- 反馈帧 ID：`0x200 + motor_id`，即 ID1-ID8 对应 `0x201` 到 `0x208`。
- 控制帧 ID：ID1-ID4 使用 `0x200`，ID5-ID8 使用 `0x1FF`。
- 控制帧 slot：
  - ID1/ID5 -> `data[0..1]`
  - ID2/ID6 -> `data[2..3]`
  - ID3/ID7 -> `data[4..5]`
  - ID4/ID8 -> `data[6..7]`
- 编码器一圈：`8192`
- M2006/C610：
  - 减速比：`36.0f`
  - 电流换算：`10A <-> raw 16384`
- M3508/C620：
  - 减速比：`3591.0f / 187.0f`，约 `19.20320856`
  - 该值来自 RoboMaster M3508 P19 规格参数，不使用近似 `19:1`
  - 电流换算：`20A <-> raw 16384`

## Keil Watch 调试入口

Keil Watch 只需要添加：

```c
g_dji_motor_debug
```

主要字段：

- `enable`：总使能。`false` 时发送 0A，并重置 PID 状态。
- `mode`：控制模式，`0` 停止，`1` 电流模式，`2` 速度模式，`3` 位置模式。
- `target_current_A`：电流模式目标电流。
- `target_speed_rpm`：速度模式目标速度；位置模式下由位置环输出更新。
- `target_angle_deg`：位置模式目标总角度。
- `feedback_speed_rpm`：反馈输出轴速度。
- `feedback_angle_deg`：反馈单圈输出轴角度。
- `feedback_total_angle_deg`：反馈累计输出轴角度。
- `feedback_current_A`：反馈电流。
- `feedback_temperature_c`：电机温度。
- `command_current_A`：最终发送电流，单位 A。
- `command_current_raw`：最终发送电流 raw 值。
- `configured_can_channel`、`configured_motor_id`、`configured_motor_type`：只读配置回显。
- `feedback_can_id`、`control_can_id`、`control_slot`：只读协议派生值。
- `rx_count`、`tx_count`、`error_count`：CAN 收发和错误计数。
- `speed_pid_params`：速度环 PID 参数，运行时可在 Watch 中调节。
- `angle_pid_params`：位置环 PID 参数，运行时可在 Watch 中调节。

推荐首次上电调试顺序：

1. 保持 `enable = false`，确认 Watch 变量和反馈字段可观察。
2. 设置 `mode = 1`，用小电流测试方向和反馈是否正常。
3. 设置 `mode = 2`，从 `target_speed_rpm = 100` 以下开始验证速度闭环。
4. 设置 `mode = 3`，确认累计角度方向正确后再测试位置闭环。
5. 如果响应太软，优先小步提高 `speed_pid_params.kp` 和 `speed_pid_params.output_limit`。

## 控制频率和 PID

- 速度环：1kHz，增量式 PID，输入 `target_speed_rpm` 和 `feedback_speed_rpm`，输出 `command_current_A`。
- 位置环：100Hz，位置式 PID，输入 `target_angle_deg` 和 `feedback_total_angle_deg`，输出 `target_speed_rpm`。
- PID 使用真实 `dt_s`：速度环 `0.001f`，位置环 `0.01f`。
- PID 参数单位和输出单位直接对应调试变量，不做隐藏 x10/x100 缩放。
- 当前默认参数偏保守，目标是安全起转和方便调试，不追求开箱高速响应。

## 构建和测试

PC 回归测试：

```powershell
gcc -std=c99 -Wall -Wextra -ICore/Inc tests/pc/test_dji_motor_control.c Core/Src/pid.c Core/Src/dji_motor.c Core/Src/dji_motor_task_logic.c -o tests/pc/test_dji_motor_control.exe
.\tests\pc\test_dji_motor_control.exe
```

期望结果：

```text
all tests passed
```

Keil 构建：

```powershell
D:\Keil_v5\UV4\UV4.exe -b MDK-ARM\single_motor_test.uvprojx -j0
```

构建日志位置：

```text
MDK-ARM\single_motor_test\single_motor_test.build_log.htm
```

期望结果包含：

```text
compiling dji_motor_task_logic.c...
compiling dji_motor_task.c...
"single_motor_test\single_motor_test.axf" - 0 Error(s)
```

## Git 和协作规则

- 除非用户明确要求提交 git，否则不要自动提交。
- 用户明确要求提交时，需要写详细中文提交信息。
- 提交后必须检查 `git log --encoding=UTF-8 -1 --format=fuller` 和 `git cat-file -p HEAD`，确认中文无乱码、排版无错位。
- 构建产物、Keil 用户界面临时文件和 PC 测试 exe 由 `.gitignore` 忽略，不应提交。

## 当前注意事项

- `MDK-ARM/single_motor_test.uvprojx` 中的 `<uAC6>1</uAC6>` 很重要。当前 FreeRTOS port 使用 clang/GCC 风格内联汇编，改回 ARMCC5 会导致 `portmacro.h` 编译失败。
- `MDK-ARM/EventRecorderStub.scvd`、`MDK-ARM/single_motor_test/`、`MDK-ARM/startup_stm32h723xx.lst`、`tests/pc/test_dji_motor_control.exe` 是可再生成产物，不应提交。
- 如果重新用 CubeMX 生成代码，要重点检查用户代码区是否保留，以及 `pid.c`、`dji_motor.c`、`dji_motor_task_logic.c`、`dji_motor_task.c` 是否仍在 Keil 工程文件中。
- 如果 CAN 没有反馈，优先检查物理 CAN 线、终端电阻、电调 ID、当前 `DJI_DEBUG_CAN_CHANNEL` 对应的 FDCAN 引脚和滤波器反馈 ID。

## 新 AI 接手流程

1. 先读 `git log --oneline --decorate -5`，确认当前提交历史。
2. 再读本文档，建立工程上下文。
3. 查看 `git status --short --ignored`，区分源码改动和被忽略构建产物。
4. 需要改控制任务时，优先查看 `Core/Src/dji_motor_task.c` 和 `Core/Src/dji_motor_task_logic.c`。
5. 需要改协议解析或电流换算时，查看 `Core/Src/dji_motor.c`。
6. 需要改 PID 行为时，查看 `Core/Src/pid.c` 并同步更新 PC 测试。
7. 完成修改后至少运行 PC 回归测试；涉及 Keil 工程、嵌入式编译或源文件增删改名时，再运行 Keil 构建。
8. 只有用户明确要求时才提交，并按中文详细提交信息规则执行。

## Keil 源文件重命名易错点

- 重命名或新增 `.c` 文件时，必须同时检查 `MDK-ARM/single_motor_test.uvprojx` 和 `MDK-ARM/single_motor_test.uvoptx`。
- 不能只看源码 include 或 PC 侧 gcc 测试是否通过。
- 必须重新运行 Keil 构建，并检查 `MDK-ARM/single_motor_test/single_motor_test.build_log.htm` 中实际 `compiling xxx.c...` 的文件名。
- 典型失败形态：`armclang: error: no such file or directory: '../Core/Src/old_name.c'`。
- 具体例子：把 `dji_m2006.c` 改名为 `dji_motor.c` 后，两个 Keil 工程文件都必须引用 `../Core/Src/dji_motor.c`。
