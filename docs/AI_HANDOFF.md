# AI 工程交接上下文

本文档用于配合 `git log` 快速接手当前工程，目标是让新的 AI 或开发者不需要重新摸索 CubeMX、Keil、FDCAN 和 M2006 控制链路。

## 项目概览

- 工程路径：`D:\desktop\2026RC\Control\single_motor_test`
- 当前基线提交：`f85d987 初始化 M2006 单电机 CubeMX 控制工程`
- 工程性质：真实 STM32CubeMX 生成工程，不是手工仿造工程结构。
- 目标芯片：`STM32H723ZGTx`
- IDE/工具链：`MDK-ARM`，Keil 使用 ArmClang/AC6。
- RTOS：CMSIS-RTOS2 / FreeRTOS。
- 应用代码语言：C。
- 当前功能：通过 CAN3 控制单个 DJI M2006 电机和 C610 电调，支持电流、速度、位置三种调试模式。

## 关键文件

- `single_motor_test.ioc`：CubeMX 工程源文件，FDCAN3、FreeRTOS、PF6/PF7 等硬件配置从这里生成。
- `MDK-ARM/single_motor_test.uvprojx`：Keil 工程文件，已设置 `<uAC6>1</uAC6>`，不要随意改回 ARMCC5。
- `Core/Inc/m2006_task.h`：M2006 控制模式、Keil Watch 调试结构体 `M2006_Debug`、全局变量 `g_m2006_debug`。
- `Core/Src/m2006_task.c`：RTOS 电机任务、CAN3 收发、速度环/位置环调度、调试变量更新。
- `Core/Inc/dji_m2006.h` 和 `Core/Src/dji_m2006.c`：M2006/C610 协议解析、电流单位换算、0x200 控制帧 slot1 打包。
- `Core/Inc/pid.h` 和 `Core/Src/pid.c`：增量式 PID 和位置式 PID，使用真实 `dt_s`，没有隐藏 x10/x100 缩放。
- `tests/pc/test_m2006_control.c`：PC 侧单元测试，覆盖 PID、反馈解析、电流换算和控制帧打包。
- `.vscode/tasks.json`：VS Code 任务，包含 PC 测试和 Keil 构建入口。

## M2006 控制链路

- CAN 通道：FDCAN3。
- 引脚：PF6 为 FDCAN3_RX，PF7 为 FDCAN3_TX。
- 帧格式：Classic CAN，8-byte data frame。
- 电调：C610，ID 为 1。
- 控制帧：标准帧 `0x200`，ID1 对应第 1 个电机 slot，即 data[0..1]。
- 反馈帧：标准帧 `0x201`。
- 电流单位：对外调试使用 `A`，内部发送前转换为 C610 raw current。
- 角度单位：`deg`。
- 速度单位：`rpm`。
- 减速比：`36.0f`。
- 当前默认换算：`10A` 对应 C610 raw `16384`。

## Keil Watch 调试入口

Keil Watch 只需要添加：

```c
g_m2006_debug
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
- `rx_count`、`tx_count`、`error_count`：CAN 收发和错误计数。
- `speed_pid_params`：速度环 PID 参数，运行时可在 Watch 中调节。
- `angle_pid_params`：位置环 PID 参数，运行时可在 Watch 中调节。

推荐首次上电调试顺序：

1. `mode = 1`，小电流测试方向和反馈是否正常。
2. `mode = 2`，从 `target_speed_rpm = 100` 以下开始验证速度闭环。
3. `mode = 3`，确认累计角度方向正确后再测试位置闭环。
4. 如果响应太软，优先小步提高 `speed_pid_params.kp` 和 `speed_pid_params.output_limit`。

## 控制频率和 PID

- 速度环：1kHz，增量式 PID，输入 `target_speed_rpm` 和 `feedback_speed_rpm`，输出 `command_current_A`。
- 位置环：100Hz，位置式 PID，输入 `target_angle_deg` 和 `feedback_total_angle_deg`，输出 `target_speed_rpm`。
- PID 使用真实 `dt_s`：速度环 `0.001f`，位置环 `0.01f`。
- PID 参数单位和输出单位直接对应调试变量，不做隐藏 x10/x100 缩放。
- 当前默认参数偏保守，目标是安全起转和方便调试，不追求开箱高速响应。

## 构建和测试

PC 单元测试：

```powershell
gcc -std=c99 -Wall -Wextra -ICore/Inc tests/pc/test_m2006_control.c Core/Src/pid.c Core/Src/dji_m2006.c -o tests/pc/test_m2006_control.exe
.\tests\pc\test_m2006_control.exe
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
"single_motor_test\single_motor_test.axf" - 0 Error(s)
```

## Git 和协作规则

- 除非用户明确要求提交 git，否则不要自动提交。
- 用户明确要求提交时，需要写详细中文提交信息。
- 提交后必须检查 `git log -1 --format=fuller` 和 `git cat-file -p HEAD`，确认中文无乱码、排版无错位。
- 构建产物、Keil 用户界面临时文件和 PC 测试 exe 由 `.gitignore` 忽略。
- `MDK-ARM/single_motor_test.uvoptx` 是 Keil 工程选项文件，可能包含 Watch 配置；如果其中记录了 `g_m2006_debug`，可以作为调试便利配置提交。

## 当前注意事项

- `MDK-ARM/single_motor_test.uvprojx` 中的 `<uAC6>1</uAC6>` 很重要。当前 FreeRTOS port 使用 clang/GCC 风格内联汇编，改回 ARMCC5 会导致 `portmacro.h` 编译失败。
- `MDK-ARM/EventRecorderStub.scvd`、`MDK-ARM/single_motor_test/`、`MDK-ARM/startup_stm32h723xx.lst`、`tests/pc/test_m2006_control.exe` 是可再生成产物，不应提交。
- 如果重新用 CubeMX 生成代码，要重点检查用户代码区是否保留，以及 `pid.c`、`dji_m2006.c`、`m2006_task.c` 是否仍在 Keil 工程文件中。
- 如果 CAN 没有反馈，优先检查物理 CAN 线、终端电阻、C610 ID、FDCAN3 引脚 PF6/PF7 和滤波器 `0x201`。

## 新 AI 接手流程

1. 先读 `git log --oneline --decorate -5`，确认当前提交历史。
2. 再读本文档，建立工程上下文。
3. 查看 `git status --short --ignored`，区分源码改动和被忽略构建产物。
4. 需要改控制逻辑时，优先查看 `Core/Src/m2006_task.c`。
5. 需要改协议解析或电流换算时，查看 `Core/Src/dji_m2006.c`。
6. 需要改 PID 行为时，查看 `Core/Src/pid.c` 并同步更新 PC 测试。
7. 完成修改后至少运行 PC 单元测试；涉及 Keil 工程或嵌入式编译时再运行 Keil 构建。
8. 只有用户明确要求时才提交，并按中文详细提交信息规则执行。
