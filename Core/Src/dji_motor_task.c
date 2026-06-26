#include "dji_motor_task.h"

#include "cmsis_os.h"
#include "dji_motor_debug_config.h"
#include "dji_motor_task_logic.h"
#include "fdcan.h"

#include <stddef.h>

#define DJI_MOTOR_SPEED_DT_S            (0.001f)
#define DJI_MOTOR_ANGLE_DT_S            (0.01f)

DJI_Motor_Debug g_dji_motor_debug = {
    .enable = false,
    .mode = DJI_MOTOR_MODE_STOP,
    .target_current_A = 0.0f,
    .target_speed_rpm = 0.0f,
    .target_angle_deg = 0.0f,
    .feedback_speed_rpm = 0.0f,
    .feedback_angle_deg = 0.0f,
    .feedback_total_angle_deg = 0.0f,
    .feedback_current_A = 0.0f,
    .feedback_temperature_c = 0U,
    .command_current_A = 0.0f,
    .command_current_raw = 0,
    .configured_can_channel = DJI_DEBUG_CAN_CHANNEL,
    .configured_motor_id = DJI_DEBUG_MOTOR_ID,
    .configured_motor_type = DJI_DEBUG_MOTOR_TYPE,
    .feedback_can_id = 0U,
    .control_can_id = 0U,
    .control_slot = 0U,
    .rx_count = 0U,
    .tx_count = 0U,
    .error_count = 0U,
    .speed_pid_params = {
        .kp = 0.004f,
        .ki = (DJI_DEBUG_MOTOR_TYPE == DJI_MOTOR_TYPE_M3508) ? 0.20f : 0.35f,
        .kd = 0.0f,
        .I_Outlimit = (DJI_DEBUG_MOTOR_TYPE == DJI_MOTOR_TYPE_M3508) ? 1.0f : 1.5f,
        .isIOutlimit = true,
        .output_limit = (DJI_DEBUG_MOTOR_TYPE == DJI_MOTOR_TYPE_M3508) ? 1.5f : 2.0f,
        .deadband = 3.0f,
    },
    .angle_pid_params = {
        .kp = 1.5f,
        .ki = 0.0f,
        .kd = 0.0f,
        .I_Outlimit = 0.0f,
        .isIOutlimit = true,
        .output_limit = (DJI_DEBUG_MOTOR_TYPE == DJI_MOTOR_TYPE_M3508) ? 90.0f : 120.0f,
        .deadband = 1.0f,
    },
};

static const DJI_Motor_Config s_dji_motor_config = {
    .motor_type = DJI_DEBUG_MOTOR_TYPE,
    .motor_id = DJI_DEBUG_MOTOR_ID,
    .control_slot = (uint8_t)((DJI_DEBUG_MOTOR_ID - 1U) % 4U),
    .feedback_id = DJI_MOTOR_FEEDBACK_BASE_ID + DJI_DEBUG_MOTOR_ID,
    .control_id = (DJI_DEBUG_MOTOR_ID <= 4U) ? DJI_MOTOR_CONTROL_LOW_ID : DJI_MOTOR_CONTROL_HIGH_ID,
    .gear_ratio = (DJI_DEBUG_MOTOR_TYPE == DJI_MOTOR_TYPE_M3508) ? DJI_MOTOR_M3508_GEAR_RATIO : DJI_MOTOR_M2006_GEAR_RATIO,
    .max_current_A = (DJI_DEBUG_MOTOR_TYPE == DJI_MOTOR_TYPE_M3508) ? DJI_ESC_C620_MAX_CURRENT_A : DJI_ESC_C610_MAX_CURRENT_A,
    .max_current_raw = DJI_ESC_CURRENT_MAX_RAW,
};

static DJI_Motor_State s_dji_motor_state;
static PID_Incremental s_dji_motor_speed_pid;
static PID_Position s_dji_motor_angle_pid;

static FDCAN_HandleTypeDef *DJI_Motor_GetConfiguredCan(void)
{
#if (DJI_DEBUG_CAN_CHANNEL == 1U)
    return &hfdcan1;
#elif (DJI_DEBUG_CAN_CHANNEL == 2U)
    return &hfdcan2;
#else
    return &hfdcan3;
#endif
}

static bool DJI_Motor_IsConfiguredCan(const FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL) {
        return false;
    }

#if (DJI_DEBUG_CAN_CHANNEL == 1U)
    return hfdcan->Instance == FDCAN1;
#elif (DJI_DEBUG_CAN_CHANNEL == 2U)
    return hfdcan->Instance == FDCAN2;
#else
    return hfdcan->Instance == FDCAN3;
#endif
}

static void DJI_Motor_UpdateDebugConfig(void)
{
    g_dji_motor_debug.configured_can_channel = DJI_DEBUG_CAN_CHANNEL;
    g_dji_motor_debug.configured_motor_id = s_dji_motor_config.motor_id;
    g_dji_motor_debug.configured_motor_type = s_dji_motor_config.motor_type;
    g_dji_motor_debug.feedback_can_id = s_dji_motor_config.feedback_id;
    g_dji_motor_debug.control_can_id = s_dji_motor_config.control_id;
    g_dji_motor_debug.control_slot = s_dji_motor_config.control_slot;
}

static void DJI_Motor_UpdateDebugFeedback(void)
{
    g_dji_motor_debug.feedback_speed_rpm = s_dji_motor_state.feedback_speed_rpm;
    g_dji_motor_debug.feedback_angle_deg = s_dji_motor_state.feedback_angle_deg;
    g_dji_motor_debug.feedback_total_angle_deg = s_dji_motor_state.feedback_total_angle_deg;
    g_dji_motor_debug.feedback_current_A = s_dji_motor_state.feedback_current_A;
    g_dji_motor_debug.feedback_temperature_c = s_dji_motor_state.feedback_temperature_c;
}

static HAL_StatusTypeDef DJI_Motor_CAN_Start(void)
{
    FDCAN_FilterTypeDef filter = {0};
    FDCAN_HandleTypeDef *hfdcan = DJI_Motor_GetConfiguredCan();

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = s_dji_motor_config.feedback_id;
    filter.FilterID2 = 0x7FFU;

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

static void DJI_Motor_DrainRxFifo(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    FDCAN_HandleTypeDef *hfdcan = DJI_Motor_GetConfiguredCan();

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            ++g_dji_motor_debug.error_count;
            return;
        }

        if (rx_header.IdType == FDCAN_STANDARD_ID &&
            rx_header.Identifier == s_dji_motor_config.feedback_id &&
            rx_header.DataLength == FDCAN_DLC_BYTES_8) {
            DJI_Motor_UpdateFeedback(&s_dji_motor_state, rx_data);
            DJI_Motor_UpdateDebugFeedback();
            ++g_dji_motor_debug.rx_count;
        }
    }
}

static void DJI_Motor_SendCurrent(float current_A)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8];
    FDCAN_HandleTypeDef *hfdcan = DJI_Motor_GetConfiguredCan();

    g_dji_motor_debug.command_current_raw = DJI_Motor_CurrentAToRaw(&s_dji_motor_config, current_A);
    g_dji_motor_debug.command_current_A = DJI_Motor_RawCurrentToA(&s_dji_motor_config, g_dji_motor_debug.command_current_raw);
    DJI_Motor_PackCurrentFrame(&s_dji_motor_config, g_dji_motor_debug.command_current_raw, tx_data);

    tx_header.Identifier = s_dji_motor_config.control_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data) == HAL_OK) {
        ++g_dji_motor_debug.tx_count;
    } else {
        ++g_dji_motor_debug.error_count;
    }
}

static float DJI_Motor_CalcCommandCurrent(uint32_t tick_count)
{
    return DJI_Motor_CalcCommandCurrentStep(&g_dji_motor_debug,
                                            &s_dji_motor_speed_pid,
                                            &s_dji_motor_angle_pid,
                                            tick_count);
}

void DJI_Motor_Task_Run(void *argument)
{
    (void)argument;

    DJI_Motor_UpdateDebugConfig();
    DJI_Motor_Init(&s_dji_motor_state, &s_dji_motor_config);
    PID_Incremental_Init(&s_dji_motor_speed_pid, &g_dji_motor_debug.speed_pid_params, DJI_MOTOR_SPEED_DT_S);
    PID_Position_Init(&s_dji_motor_angle_pid, &g_dji_motor_debug.angle_pid_params, DJI_MOTOR_ANGLE_DT_S);

    if (DJI_Motor_CAN_Start() != HAL_OK) {
        ++g_dji_motor_debug.error_count;
    }

    uint32_t tick_count = 0U;
    uint32_t wake_tick = osKernelGetTickCount();

    for (;;) {
        DJI_Motor_DrainRxFifo();
        DJI_Motor_SendCurrent(DJI_Motor_CalcCommandCurrent(tick_count));
        ++tick_count;
        osDelayUntil(wake_tick + 1U);
        wake_tick += 1U;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan != NULL &&
        DJI_Motor_IsConfiguredCan(hfdcan) &&
        (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        DJI_Motor_DrainRxFifo();
    }
}


