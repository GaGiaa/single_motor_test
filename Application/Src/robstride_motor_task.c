#include "robstride_motor_task.h"

#include "app_time_profiler.h"
#include "app_dsp.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "robstride_motor_debug_config.h"

#include <stddef.h>

RobStride_Motor_Debug g_robstride_motor_debug = {
    .enable = false,
    .clear_fault_on_disable = false,
    .set_zero_request = false,
    .request_device_id = false,
    .request_active_report = false,
    .active_report_enable = true,
    .target_position_rad = 0.0f,
    .target_velocity_rad_s = 0.0f,
    .target_torque_Nm = 0.0f,
    .kp = 0.0f,
    .kd = 0.0f,
    .feedback_position_rad = 0.0f,
    .feedback_velocity_rad_s = 0.0f,
    .feedback_torque_Nm = 0.0f,
    .feedback_temperature_c = 0.0f,
    .mode_state = 0U,
    .fault_code = 0U,
    .configured_can_channel = MOTOR_DEBUG_CAN_CHANNEL,
    .configured_motor_id = ROBSTRIDE_DEBUG_MOTOR_ID,
    .configured_host_can_id = ROBSTRIDE_DEBUG_HOST_CAN_ID,
    .configured_motor_type = ROBSTRIDE_DEBUG_MOTOR_TYPE,
    .device_id_request_count = 0U,
    .device_id_response_count = 0U,
    .active_report_request_count = 0U,
    .last_discovered_motor_id = 0U,
    .last_device_uid = {0},
    .last_device_id_rx_can_id = 0U,
    .device_id_valid = false,
    .type17_read = {
        .request = false,
        .branch_count = 0U,
        .request_count = 0U,
        .success_count = 0U,
        .send_fail_count = 0U,
        .unhandled_count = 0U,
        .fail_count = 0U,
        .last_fail_status = 0U,
        .last_u8_value = 0U,
        .param_index = ROBSTRIDE_PARAM_CAN_ID,
        .param_target = ROBSTRIDE_READ_PARAM_TARGET_CAN_ID,
        .last_tx_can_id = 0U,
        .last_success_rx_can_id = 0U,
        .last_fail_rx_can_id = 0U,
        .last_u8_value_valid = false,
    },
    .raw_rx_count = 0U,
    .last_unhandled_rx_can_id = 0U,
    .last_unhandled_rx_data = {0},
    .last_rx_can_id = 0U,
    .last_tx_can_id = 0U,
    .rx_count = 0U,
    .tx_count = 0U,
    .error_count = 0U,
    .dsp_cmsis_enabled = false,
    .dsp_build_flags = 0U,
    .time_profiler_enabled = false,
    .last_loop_us = 0U,
    .max_loop_us = 0U,
    .avg_loop_us = 0U,
};

static const RobStride_Motor_Config s_robstride_motor_config = {
    .motor_type = ROBSTRIDE_DEBUG_MOTOR_TYPE,
    .motor_id = ROBSTRIDE_DEBUG_MOTOR_ID,
    .host_can_id = ROBSTRIDE_DEBUG_HOST_CAN_ID,
    .params = {
        .position_min_rad = ROBSTRIDE_EL05_POSITION_MIN_RAD,
        .position_max_rad = ROBSTRIDE_EL05_POSITION_MAX_RAD,
        .velocity_min_rad_s = ROBSTRIDE_EL05_VELOCITY_MIN_RAD_S,
        .velocity_max_rad_s = ROBSTRIDE_EL05_VELOCITY_MAX_RAD_S,
        .torque_min_Nm = ROBSTRIDE_EL05_TORQUE_MIN_NM,
        .torque_max_Nm = ROBSTRIDE_EL05_TORQUE_MAX_NM,
        .kp_min = ROBSTRIDE_EL05_KP_MIN,
        .kp_max = ROBSTRIDE_EL05_KP_MAX,
        .kd_min = ROBSTRIDE_EL05_KD_MIN,
        .kd_max = ROBSTRIDE_EL05_KD_MAX,
    },
};

static RobStride_Motor_State s_robstride_motor_state;
static App_TimeProfiler_Stats s_robstride_motor_loop_time;

static FDCAN_HandleTypeDef *RobStride_Motor_GetConfiguredCan(void)
{
#if (MOTOR_DEBUG_CAN_CHANNEL == 1U)
    return &hfdcan1;
#elif (MOTOR_DEBUG_CAN_CHANNEL == 2U)
    return &hfdcan2;
#else
    return &hfdcan3;
#endif
}

static void RobStride_Motor_UpdateDebugConfig(void)
{
    g_robstride_motor_debug.configured_can_channel = MOTOR_DEBUG_CAN_CHANNEL;
    g_robstride_motor_debug.configured_motor_id = s_robstride_motor_config.motor_id;
    g_robstride_motor_debug.configured_host_can_id = s_robstride_motor_config.host_can_id;
    g_robstride_motor_debug.configured_motor_type = s_robstride_motor_config.motor_type;
    g_robstride_motor_debug.type17_read.param_index =
        RobStride_Motor_SelectReadableParamIndex((RobStride_ReadParam_Target)g_robstride_motor_debug.type17_read.param_target);
    g_robstride_motor_debug.dsp_cmsis_enabled = App_DSP_IsCmsisEnabled();
    g_robstride_motor_debug.dsp_build_flags = App_DSP_GetBuildFlags();
}

static void RobStride_Motor_UpdateDebugPerf(void)
{
    g_robstride_motor_debug.time_profiler_enabled = s_robstride_motor_loop_time.enabled;
    g_robstride_motor_debug.last_loop_us = s_robstride_motor_loop_time.last_us;
    g_robstride_motor_debug.max_loop_us = s_robstride_motor_loop_time.max_us;
    g_robstride_motor_debug.avg_loop_us = s_robstride_motor_loop_time.avg_us;
}

static void RobStride_Motor_UpdateDebugFeedback(void)
{
    g_robstride_motor_debug.feedback_position_rad = s_robstride_motor_state.position_rad;
    g_robstride_motor_debug.feedback_velocity_rad_s = s_robstride_motor_state.velocity_rad_s;
    g_robstride_motor_debug.feedback_torque_Nm = s_robstride_motor_state.torque_Nm;
    g_robstride_motor_debug.feedback_temperature_c = s_robstride_motor_state.temperature_c;
    g_robstride_motor_debug.mode_state = s_robstride_motor_state.mode_state;
    g_robstride_motor_debug.fault_code = s_robstride_motor_state.fault_code;
}

static HAL_StatusTypeDef RobStride_Motor_CAN_Start(void)
{
    FDCAN_FilterTypeDef filter = {0};
    FDCAN_HandleTypeDef *hfdcan = RobStride_Motor_GetConfiguredCan();

    filter.IdType = FDCAN_EXTENDED_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0U;
    filter.FilterID2 = 0U;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void RobStride_Motor_DrainRxFifo(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t discovered_motor_id;
    uint8_t discovered_uid[8];
    uint8_t read_u8_value;
    uint8_t failure_status;
    uint16_t type17_param_index = g_robstride_motor_debug.type17_read.param_index;
    uint32_t type17_completed_count = g_robstride_motor_debug.type17_read.success_count +
                                      g_robstride_motor_debug.type17_read.fail_count;
    bool type17_pending = g_robstride_motor_debug.type17_read.request_count >
                          type17_completed_count;
    FDCAN_HandleTypeDef *hfdcan = RobStride_Motor_GetConfiguredCan();

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            ++g_robstride_motor_debug.error_count;
            return;
        }

        if (rx_header.IdType != FDCAN_EXTENDED_ID || rx_header.DataLength != FDCAN_DLC_BYTES_8) {
            continue;
        }

        ++g_robstride_motor_debug.raw_rx_count;

        if (RobStride_Motor_ParseDeviceIdResponse(rx_header.Identifier,
                                                  rx_data,
                                                  &discovered_motor_id,
                                                  discovered_uid)) {
            g_robstride_motor_debug.last_discovered_motor_id = discovered_motor_id;
            for (uint8_t i = 0U; i < 8U; ++i) {
                g_robstride_motor_debug.last_device_uid[i] = discovered_uid[i];
            }
            g_robstride_motor_debug.last_device_id_rx_can_id = rx_header.Identifier;
            g_robstride_motor_debug.last_rx_can_id = rx_header.Identifier;
            g_robstride_motor_debug.device_id_valid = true;
            ++g_robstride_motor_debug.device_id_response_count;
            ++g_robstride_motor_debug.rx_count;
            continue;
        }

        if (RobStride_Motor_ParseReadSingleUint8Response(rx_header.Identifier,
                                                         rx_data,
                                                         type17_param_index,
                                                         s_robstride_motor_config.host_can_id,
                                                         s_robstride_motor_config.motor_id,
                                                         &read_u8_value)) {
            g_robstride_motor_debug.type17_read.last_u8_value = read_u8_value;
            g_robstride_motor_debug.type17_read.last_success_rx_can_id = rx_header.Identifier;
            g_robstride_motor_debug.last_rx_can_id = rx_header.Identifier;
            g_robstride_motor_debug.type17_read.last_u8_value_valid = true;
            ++g_robstride_motor_debug.type17_read.success_count;
            ++g_robstride_motor_debug.rx_count;
            continue;
        }

        if (RobStride_Motor_ParseReadSingleParamFailure(rx_header.Identifier,
                                                        rx_data,
                                                        type17_param_index,
                                                        s_robstride_motor_config.host_can_id,
                                                        s_robstride_motor_config.motor_id,
                                                        &failure_status)) {
            g_robstride_motor_debug.type17_read.last_fail_status = failure_status;
            g_robstride_motor_debug.type17_read.last_fail_rx_can_id = rx_header.Identifier;
            g_robstride_motor_debug.last_rx_can_id = rx_header.Identifier;
            ++g_robstride_motor_debug.type17_read.fail_count;
            ++g_robstride_motor_debug.rx_count;
            continue;
        }

        if (RobStride_Motor_UpdateFeedback(&s_robstride_motor_state, rx_header.Identifier, rx_data)) {
            g_robstride_motor_debug.last_rx_can_id = rx_header.Identifier;
            RobStride_Motor_UpdateDebugFeedback();
            ++g_robstride_motor_debug.rx_count;
            continue;
        }

        g_robstride_motor_debug.last_unhandled_rx_can_id = rx_header.Identifier;
        for (uint8_t i = 0U; i < 8U; ++i) {
            g_robstride_motor_debug.last_unhandled_rx_data[i] = rx_data[i];
        }
        if (type17_pending) {
            ++g_robstride_motor_debug.type17_read.unhandled_count;
        }
    }
}

static bool RobStride_Motor_SendFrame(const RobStride_Motor_Frame *frame)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    FDCAN_HandleTypeDef *hfdcan = RobStride_Motor_GetConfiguredCan();

    if (frame == NULL) {
        ++g_robstride_motor_debug.error_count;
        return false;
    }

    tx_header.Identifier = frame->can_id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)frame->data) == HAL_OK) {
        g_robstride_motor_debug.last_tx_can_id = frame->can_id;
        ++g_robstride_motor_debug.tx_count;
        return true;
    } else {
        ++g_robstride_motor_debug.error_count;
        return false;
    }
}

static void RobStride_Motor_RunControlStep(bool *last_enable)
{
    RobStride_Motor_Frame frame;
    RobStride_Motor_Command command;
    uint16_t type17_param_index;

    g_robstride_motor_debug.type17_read.param_index =
        RobStride_Motor_SelectReadableParamIndex((RobStride_ReadParam_Target)g_robstride_motor_debug.type17_read.param_target);
    type17_param_index = g_robstride_motor_debug.type17_read.param_index;

    if (g_robstride_motor_debug.request_device_id && !g_robstride_motor_debug.enable) {
        RobStride_Motor_MakeGetDeviceIdFrame(&s_robstride_motor_config, &frame);
        if (RobStride_Motor_SendFrame(&frame)) {
            ++g_robstride_motor_debug.device_id_request_count;
        }
        g_robstride_motor_debug.request_device_id = false;
        return;
    }

    if (g_robstride_motor_debug.request_active_report && !g_robstride_motor_debug.enable) {
        RobStride_Motor_MakeActiveReportFrame(&s_robstride_motor_config,
                                              g_robstride_motor_debug.active_report_enable,
                                              &frame);
        if (RobStride_Motor_SendFrame(&frame)) {
            ++g_robstride_motor_debug.active_report_request_count;
        }
        g_robstride_motor_debug.request_active_report = false;
        return;
    }

    if (g_robstride_motor_debug.type17_read.request && g_robstride_motor_debug.enable) {
        return;
    }

    if (g_robstride_motor_debug.type17_read.request && !g_robstride_motor_debug.enable) {
        bool send_ok;

        ++g_robstride_motor_debug.type17_read.branch_count;
        RobStride_Motor_MakeReadSingleParamFrame(&s_robstride_motor_config,
                                                 type17_param_index,
                                                 &frame);
        send_ok = RobStride_Motor_SendFrame(&frame);
        if (send_ok) {
            ++g_robstride_motor_debug.type17_read.request_count;
            g_robstride_motor_debug.type17_read.last_tx_can_id = frame.can_id;
        } else {
            ++g_robstride_motor_debug.type17_read.send_fail_count;
        }
        g_robstride_motor_debug.type17_read.request = false;
        return;
    }

    if (g_robstride_motor_debug.set_zero_request) {
        RobStride_Motor_MakeSetZeroFrame(&s_robstride_motor_config, &frame);
        (void)RobStride_Motor_SendFrame(&frame);
        g_robstride_motor_debug.set_zero_request = false;
    }

    if (!g_robstride_motor_debug.enable) {
        RobStride_Motor_MakeDisableFrame(&s_robstride_motor_config,
                                         g_robstride_motor_debug.clear_fault_on_disable,
                                         &frame);
        (void)RobStride_Motor_SendFrame(&frame);
        if (last_enable != NULL) {
            *last_enable = false;
        }
        return;
    }

    if (last_enable != NULL && !*last_enable) {
        RobStride_Motor_MakeEnableFrame(&s_robstride_motor_config, &frame);
        (void)RobStride_Motor_SendFrame(&frame);
        *last_enable = true;
    }

    command.position_rad = g_robstride_motor_debug.target_position_rad;
    command.velocity_rad_s = g_robstride_motor_debug.target_velocity_rad_s;
    command.torque_Nm = g_robstride_motor_debug.target_torque_Nm;
    command.kp = g_robstride_motor_debug.kp;
    command.kd = g_robstride_motor_debug.kd;
    RobStride_Motor_MakeMotionControlFrame(&s_robstride_motor_config, &command, &frame);
    (void)RobStride_Motor_SendFrame(&frame);
}

void RobStride_Motor_Task_Run(void *argument)
{
    bool last_enable = false;
    uint32_t wake_tick;

    (void)argument;

    RobStride_Motor_UpdateDebugConfig();
    RobStride_Motor_Init(&s_robstride_motor_state, &s_robstride_motor_config);
    App_TimeProfiler_Init();
    App_TimeProfiler_Reset(&s_robstride_motor_loop_time);
    RobStride_Motor_UpdateDebugConfig();
    RobStride_Motor_UpdateDebugPerf();

    if (RobStride_Motor_CAN_Start() != HAL_OK) {
        ++g_robstride_motor_debug.error_count;
    }

    wake_tick = osKernelGetTickCount();
    for (;;) {
        const App_TimeProfiler_Timestamp loop_start = App_TimeProfiler_Begin();
        RobStride_Motor_DrainRxFifo();
        RobStride_Motor_RunControlStep(&last_enable);
        App_TimeProfiler_End(&s_robstride_motor_loop_time, loop_start);
        RobStride_Motor_UpdateDebugPerf();
        osDelayUntil(wake_tick + 1U);
        wake_tick += 1U;
    }
}

#if (MOTOR_DEBUG_PROTOCOL == MOTOR_DEBUG_PROTOCOL_ROBSTRIDE)
static bool RobStride_Motor_IsConfiguredCan(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL) {
        return false;
    }

#if (MOTOR_DEBUG_CAN_CHANNEL == 1U)
    return hfdcan->Instance == FDCAN1;
#elif (MOTOR_DEBUG_CAN_CHANNEL == 2U)
    return hfdcan->Instance == FDCAN2;
#else
    return hfdcan->Instance == FDCAN3;
#endif
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan != NULL &&
        RobStride_Motor_IsConfiguredCan(hfdcan) &&
        (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        RobStride_Motor_DrainRxFifo();
    }
}
#endif
