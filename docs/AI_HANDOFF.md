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
- 当前功能：单电机调试工程，编译期选择 DJI 协议或 RobStride/EDULITE 协议。
- DJI 支持：M2006/C610、M3508/C620，电机 ID 1-8。
- RobStride 支持：EDULITE 05 / EL05，默认私有协议扩展帧。
- 当前默认配置：`CAN2 + ID2 + M2006/C610`，配置入口在 `Application/Inc/dji_motor_debug_config.h`。

## 关键文件

- `Core/Inc` 和 `Core/Src`：CubeMX / HAL / BSP / RTOS 生成或强相关 glue 文件。
- `Application/Inc` 和 `Application/Src`：手写协议、控制、调试任务和 PC 可测逻辑。
- `single_motor_test.ioc`：CubeMX 工程源文件，硬件和中间件配置从这里生成。
- `MDK-ARM/single_motor_test.uvprojx`：Keil 工程文件，已设置 `<uAC6>1</uAC6>`，不要随意改回 ARMCC5。
- `MDK-ARM/single_motor_test.uvoptx`：Keil 选项文件，可能包含 Watch 配置；源文件增删改名时也要同步检查。
- `Application/Inc/dji_motor_debug_config.h`：单电机调试编译期配置，包含 CAN 通道、电机 ID、电机类型。
- `Application/Inc/motor_debug_config.h`：总协议和 CAN 通道选择，默认 `MOTOR_DEBUG_PROTOCOL_DJI`。
- `Application/Inc/dji_motor.h` 和 `Application/Src/dji_motor.c`：DJI 电机通用协议层，负责反馈解析、电流单位换算、控制帧 ID/slot 打包。
- `Application/Inc/dji_motor_task.h` 和 `Application/Src/dji_motor_task.c`：RTOS 电机调试任务、FDCAN 收发、调试变量更新。
- `Application/Inc/dji_motor_task_logic.h` 和 `Application/Src/dji_motor_task_logic.c`：任务纯控制逻辑，供生产任务和 PC 测试共用。
- `Application/Inc/robstride_motor.h` 和 `Application/Src/robstride_motor.c`：RobStride/EDULITE 私有协议层，负责扩展帧 ID、运控帧和反馈解析。
- `Application/Inc/robstride_motor_task.h` 和 `Application/Src/robstride_motor_task.c`：RobStride/EDULITE RTOS 调试任务和 Watch 入口。
- `Application/Inc/pid.h`、`Application/Inc/pid_config.h` 和 `Application/Src/pid.c`：增量式 PID 固定基础版，位置式 PID 支持 basic/advanced 编译期裁剪。
- `Application/Inc/app_math.h`：应用层通用数学小工具，目前提供 `App_Math_ClampFloat()`，供 PID、DJI 和 RobStride 协议层复用。
- `Application/Inc/app_dsp.h` 和 `Application/Src/app_dsp.c`：CMSIS-DSP 可选接入薄封装，当前封装 `arm_dot_prod_f32()` 和 `arm_fir_f32()`，PC 侧保留纯 C fallback。
- `Application/Inc/app_dsp_test_task.h` 和 `Application/Src/app_dsp_test_task.c`：低优先级 RTOS DSP 验证任务，Watch 入口为 `g_app_dsp_test_debug`。
- `Application/Inc/app_time_profiler.h` 和 `Application/Src/app_time_profiler.c`：RTOS tick + SysTick 时间戳耗时统计封装，用于板上观察任务循环执行时间。
- `tests/pc/test_dji_motor_control.c`：PC 侧回归测试，覆盖 PID、反馈解析、电流换算、控制帧打包和任务纯逻辑。
- `tests/pc/test_pid_advanced_control.c`：Advanced PID PC 侧回归测试，覆盖积分限幅、积分分离、抗积分饱和、微分抗冲击、微分滤波、输出低通和 reset。
- `tests/pc/test_robstride_motor_control.c`：RobStride/EDULITE PC 侧协议回归测试。
- `.vscode/tasks.json`：VS Code 任务，包含 PC 测试和 Keil 构建入口。

## 编译期调试配置

配置文件：`Application/Inc/dji_motor_debug_config.h`

总配置文件：`Application/Inc/motor_debug_config.h`

- `MOTOR_DEBUG_PROTOCOL`：`MOTOR_DEBUG_PROTOCOL_DJI` 或 `MOTOR_DEBUG_PROTOCOL_ROBSTRIDE`。
- `MOTOR_DEBUG_CAN_CHANNEL`：只能为 `1U`、`2U` 或 `3U`，默认 `2U`。

DJI 配置：

- `DJI_DEBUG_CAN_CHANNEL`：只能为 `1U`、`2U` 或 `3U`。
- `DJI_DEBUG_MOTOR_ID`：只能为 `1U` 到 `8U`。
- `DJI_DEBUG_MOTOR_TYPE`：只能为 `DJI_MOTOR_TYPE_M2006` 或 `DJI_MOTOR_TYPE_M3508`。

非法值会通过 `#error` 在编译期报错。当前默认值为：

```c
#define DJI_DEBUG_CAN_CHANNEL      (2U)
#define DJI_DEBUG_MOTOR_ID         (2U)
#define DJI_DEBUG_MOTOR_TYPE       DJI_MOTOR_TYPE_M2006
```

RobStride 配置文件：`Application/Inc/robstride_motor_debug_config.h`

- `ROBSTRIDE_DEBUG_MOTOR_ID`：只能为 `0U` 到 `255U` 中的非零值，默认 `0x7F`。
- `ROBSTRIDE_DEBUG_HOST_CAN_ID`：默认 `0xFDU`。
- `ROBSTRIDE_DEBUG_MOTOR_TYPE`：当前只支持 `ROBSTRIDE_MOTOR_TYPE_EL05`。

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
- `dsp_cmsis_enabled`、`dsp_build_flags`：只读 DSP 编译配置回显；`dsp_build_flags` bit0 表示 CMSIS-DSP 接入宏启用，bit1 表示 `ARM_MATH_CM7`。
- `time_profiler_enabled`、`last_loop_us`、`max_loop_us`、`avg_loop_us`：只读任务循环耗时统计，单位 us。
- `speed_pid_params`：速度环 PID 参数，运行时可在 Watch 中调节。
- `angle_pid_params`：位置环 PID 参数，运行时可在 Watch 中调节。

RobStride/EDULITE Watch 入口：

```c
g_robstride_motor_debug
```

主要字段：

- `enable`：总使能。`false` 时周期性发送 Type 4 disable，保证安全停机。
- `clear_fault_on_disable`：发送 disable 时 data[0] 是否置 1 清故障。
- `set_zero_request`：置 `true` 后发送一次 Type 6 set zero，然后任务自动清零。
- `target_position_rad`、`target_velocity_rad_s`、`target_torque_Nm`：Type 1 运控目标。
- `kp`、`kd`：Type 1 运控刚度和阻尼。
- `feedback_position_rad`、`feedback_velocity_rad_s`、`feedback_torque_Nm`、`feedback_temperature_c`：Type 2 反馈。
- `mode_state`、`fault_code`：从 Type 2 扩展 ID 解析出的模式和故障字段。
- `last_rx_can_id`、`last_tx_can_id`：最近一次收发扩展帧 ID。
- `dsp_cmsis_enabled`、`dsp_build_flags`：只读 DSP 编译配置回显；`dsp_build_flags` bit0 表示 CMSIS-DSP 接入宏启用，bit1 表示 `ARM_MATH_CM7`。
- `time_profiler_enabled`、`last_loop_us`、`max_loop_us`、`avg_loop_us`：只读任务循环耗时统计，单位 us。

### DSP 验证 Watch

```c
g_app_dsp_test_debug
```

- `enable`：默认开启；置 0 可暂停低频 DSP 自检任务。
- `run_count`、`pass_count`、`fail_count`：DSP 自检运行和结果计数。
- `last_dot_result`、`expected_dot_result`：`App_DSP_DotProductF32()` 固定向量点积验证。
- `last_fir_output`、`expected_fir_output`：`App_DSP_FIRF32()` 固定 FIR 输出验证。
- `last_error`：点积和 FIR 两项中的最大误差。
- `last_run_us`、`max_run_us`：单次 DSP 自检耗时，单位 us。
- `dsp_cmsis_enabled`、`dsp_build_flags`：只读 DSP 编译配置回显。

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
- 增量式 PID 只有基础版；速度环参数类型为 `PID_Incremental_Param_Config`。
- 位置式 PID 版本由 `Application/Inc/pid_config.h` 的 `PID_POSITION_CONFIG_VARIANT` 编译期选择，默认 `PID_POSITION_VARIANT_BASIC`。
- Basic 位置式 PID 保留基础 P/I/D、deadband 和输出限幅；`I_Outlimit` 在 Basic 编译配置下不存在。
- Advanced 位置式 PID 启用积分限幅、积分分离、反向计算抗积分饱和、D 项设定值加权、对测量值求导、一阶微分滤波和输出级低通；`I_Outlimit`、`setpoint_weight_b`、`setpoint_weight_c`、`derivative_filter_N`、`anti_windup_gain`、`integral_separation_threshold`、`output_filter_N` 只在 Advanced 编译配置下存在。
- `I_Outlimit > 0` 时启用积分限幅；`I_Outlimit <= 0` 时关闭积分限幅，不再提供单独的 Watch 布尔开关。
- `integral_separation_threshold > 0` 时启用积分分离：误差绝对值超过阈值时暂停积分，但不清零已有积分。
- `output_filter_N > 0` 时启用输出级一阶低通，作用在输出限幅和抗饱和之后的最终输出上。
- 默认 Basic 固件的 Keil Watch 中不会看到位置环积分限幅字段；需要调积分限幅或抗饱和时，先切换 `PID_POSITION_CONFIG_VARIANT` 到 `PID_POSITION_VARIANT_ADVANCED` 并重新编译。
- 通用 float 钳位函数位于 `Application/Inc/app_math.h`，语义保持简单：小于下限返回下限，大于上限返回上限，否则返回原值；不要在无测试覆盖时改变 `NaN` 或 `min > max` 行为。

## CMSIS-DSP 和性能观测

- Keil 工程已经包含 `../Middlewares/ST/ARM/DSP/Inc`，并在 C 编译宏中定义 `APP_DSP_ENABLE_CMSIS_DSP=1U` 和 `ARM_MATH_CM7`。
- 已按 CubeMX GUI 执行过 Generate Code；`.ioc` 中 X-CUBE-ALGOBUILD DSP Library 保持启用。
- CubeMX 生成的 X-CUBE 中间件根目录只自动放入 DSP 头文件。当前最小验证所需的 CMSIS-DSP V1.10.0 头文件和 `arm_dot_prod_f32.c`、`arm_fir_f32.c`、`arm_fir_init_f32.c` 来自同一个 `C:\Users\ayou\STM32Cube\Repository\Packs\STMicroelectronics\X-CUBE-ALGOBUILD\1.4.0` 示例工程，避免头/源码版本混用。
- `app_dsp` 隔离 `arm_math.h` 依赖，当前真实调用 `arm_dot_prod_f32()` 和 `arm_fir_f32()`；PC 测试默认 `APP_DSP_ENABLE_CMSIS_DSP=0`，不依赖 STM32/CMSIS-DSP 头文件。
- `app_dsp_test_task` 是独立低优先级 RTOS 验证任务，10Hz 固定数据测试点积和 FIR，方便后续板上 Watch 验证 CMSIS-DSP 链接和运行。
- 当前 PID、钳位和协议换算仍保持手写标量实现；CMSIS-DSP 优先留给后续 IMU、矩阵、批量滤波、FFT 或多电机批量向量计算。
- `app_time_profiler` 在 STM32H723 Keil 目标默认启用，ms 来源为 RTOS tick `osKernelGetTickCount()`，us 细分来源为当前 `SysTick->VAL` 和 `SysTick->LOAD`，通过两次时间戳做差得到执行耗时。
- 当前工程 HAL time base 是 TIM6，`HAL_IncTick()` 由 TIM6 回调驱动；任务耗时统计不使用 HAL tick，也不使用 DWT/SWO/CYCCNT。
- PC 侧 `app_time_profiler` 默认禁用，避免 PC 测试依赖 RTOS 或 STM32 SysTick 寄存器。
- `Core/Inc/FreeRTOSConfig.h` 中 `configENABLE_FPU` 当前仍为 `0`。如果后续要在多个 FreeRTOS 任务里大量使用浮点或 CMSIS-DSP，需要单独验证 FPU 上下文保存策略，不要把它和算法替换混在一次改动里。

## RobStride / EDULITE 05 协议

- CAN：Classic CAN 2.0 扩展帧，29-bit ID，默认 1 Mbps。
- 扩展 ID：bit28..24 为通信类型，bit23..8 为数据区 2，bit7..0 为目标电机 ID。
- Type 1 运控帧：data 为 big-endian `position/velocity/kp/kd`，前馈 torque raw 放在扩展 ID 的数据区 2。
- Type 2 反馈帧：data 为 big-endian `position/velocity/torque/temperature`，电机 ID、fault、mode 从扩展 ID 解析。
- EL05 范围：位置 `-12.57..12.57 rad`，速度 `-50..50 rad/s`，力矩 `-6..6 Nm`，Kp `0..500`，Kd `0..5`。
- 首版不实现持久参数写入、保存、波特率修改和协议切换，避免误改设备配置。

## 构建和测试

PC 回归测试：

```powershell
gcc -std=c99 -Wall -Wextra -ICore/Inc -IApplication/Inc tests/pc/test_dji_motor_control.c Application/Src/pid.c Application/Src/dji_motor.c Application/Src/dji_motor_task_logic.c -o tests/pc/test_dji_motor_control.exe
.\tests\pc\test_dji_motor_control.exe
```

RobStride PC 回归测试：

```powershell
gcc -std=c99 -Wall -Wextra -ICore/Inc -IApplication/Inc tests/pc/test_robstride_motor_control.c Application/Src/robstride_motor.c -o tests/pc/test_robstride_motor_control.exe
.\tests\pc\test_robstride_motor_control.exe
```

Advanced PID PC 回归测试：

```powershell
gcc -std=c99 -Wall -Wextra -DPID_POSITION_CONFIG_VARIANT=PID_POSITION_VARIANT_ADVANCED -ICore/Inc -IApplication/Inc tests/pc/test_pid_advanced_control.c Application/Src/pid.c -o tests/pc/test_pid_advanced_control.exe
.\tests\pc\test_pid_advanced_control.exe
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
compiling app_dsp.c...
compiling app_time_profiler.c...
compiling dji_motor_task.c...
compiling robstride_motor.c...
compiling robstride_motor_task.c...
"single_motor_test\single_motor_test.axf" - 0 Error(s)
```

## Git 和协作规则

- 除非用户明确要求提交 git，否则不要自动提交。
- 用户明确要求提交时，需要写详细中文提交信息。
- 提交后必须检查 `git log --encoding=UTF-8 -1 --format=fuller` 和 `git cat-file -p HEAD`，确认中文无乱码、排版无错位。
- 构建产物、Keil 用户界面临时文件和 PC 测试 exe 由 `.gitignore` 忽略，不应提交。

## 当前注意事项

- `MDK-ARM/single_motor_test.uvprojx` 中的 `<uAC6>1</uAC6>` 很重要。当前 FreeRTOS port 使用 clang/GCC 风格内联汇编，改回 ARMCC5 会导致 `portmacro.h` 编译失败。
- `MDK-ARM/EventRecorderStub.scvd`、`MDK-ARM/single_motor_test/`、`MDK-ARM/startup_stm32h723xx.lst`、`tests/pc/test_dji_motor_control.exe`、`tests/pc/test_robstride_motor_control.exe` 是可再生成产物，不应提交。
- 如果重新用 CubeMX 生成代码，要重点检查用户代码区是否保留，以及 `pid.c`、`app_dsp.c`、`app_time_profiler.c`、`dji_motor.c`、`dji_motor_task_logic.c`、`dji_motor_task.c`、`robstride_motor.c`、`robstride_motor_task.c` 是否仍在 Keil 工程文件中。
- RobStride 需要 FDCAN 扩展滤波器，`fdcan.c` 中各 FDCAN 实例的 `ExtFiltersNbr` 当前为 `1`。
- 如果 CAN 没有反馈，优先检查物理 CAN 线、终端电阻、电调 ID、当前 `DJI_DEBUG_CAN_CHANNEL` 对应的 FDCAN 引脚和滤波器反馈 ID。

## 新 AI 接手流程

1. 先读 `git log --oneline --decorate -5`，确认当前提交历史。
2. 再读本文档，建立工程上下文。
3. 查看 `git status --short --ignored`，区分源码改动和被忽略构建产物。
4. 需要改 DJI 控制任务时，优先查看 `Application/Src/dji_motor_task.c` 和 `Application/Src/dji_motor_task_logic.c`。
5. 需要改 DJI 协议解析或电流换算时，查看 `Application/Src/dji_motor.c`。
6. 需要改 RobStride/EDULITE 协议时，查看 `Application/Src/robstride_motor.c`。
7. 需要改 PID 行为时，查看 `Application/Inc/pid_config.h`、`Application/Inc/pid.h`、`Application/Src/pid.c`，并同步更新 Basic 和 Advanced PC 测试。
8. 需要复用小型数学工具时，优先查看 `Application/Inc/app_math.h`，避免在各协议或控制模块里重复实现。
9. 需要检查 CMSIS-DSP 或板上周期统计时，查看 `Application/Inc/app_dsp.h`、`Application/Src/app_dsp.c`、`Application/Inc/app_time_profiler.h`、`Application/Src/app_time_profiler.c`。
10. 完成修改后至少运行 PC 回归测试；涉及 Keil 工程、嵌入式编译或源文件增删改名时，再运行 Keil 构建。
11. 只有用户明确要求时才提交，并按中文详细提交信息规则执行。

## Keil 源文件重命名易错点

- 重命名或新增 `.c` 文件时，必须同时检查 `MDK-ARM/single_motor_test.uvprojx` 和 `MDK-ARM/single_motor_test.uvoptx`。
- 不能只看源码 include 或 PC 侧 gcc 测试是否通过。
- 必须重新运行 Keil 构建，并检查 `MDK-ARM/single_motor_test/single_motor_test.build_log.htm` 中实际 `compiling xxx.c...` 的文件名。
- 典型失败形态：`armclang: error: no such file or directory: '../Core/Src/old_name.c'`。
- 具体例子：把 `dji_m2006.c` 改名为 `dji_motor.c` 后，两个 Keil 工程文件都必须引用 `../Application/Src/dji_motor.c`。
