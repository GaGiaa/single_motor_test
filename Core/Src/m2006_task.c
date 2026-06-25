#include "m2006_task.h"

#include "cmsis_os.h"
#include "fdcan.h"

#include <stddef.h>

#define M2006_SPEED_DT_S            (0.001f)
#define M2006_ANGLE_DT_S            (0.01f)
#define M2006_POSITION_DIVIDER      (10U)

M2006_Debug g_m2006_debug = {
    .enable = false,
    .mode = M2006_MODE_STOP,
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
    .rx_count = 0U,
    .tx_count = 0U,
    .error_count = 0U,
    .speed_pid_params = {
        .kp = 0.004f,
        .ki = 0.35f,
        .kd = 0.0f,
        .I_Outlimit = 1.5f,
        .isIOutlimit = true,
        .output_limit = 2.0f,
        .deadband = 3.0f,
    },
    .angle_pid_params = {
        .kp = 1.5f,
        .ki = 0.0f,
        .kd = 0.0f,
        .I_Outlimit = 0.0f,
        .isIOutlimit = true,
        .output_limit = 120.0f,
        .deadband = 1.0f,
    },
};

static DJI_M2006_State s_m2006_motor;
static PID_Incremental s_m2006_speed_pid;
static PID_Position s_m2006_angle_pid;

static void M2006_UpdateDebugFeedback(void)
{
    g_m2006_debug.feedback_speed_rpm = s_m2006_motor.feedback_speed_rpm;
    g_m2006_debug.feedback_angle_deg = s_m2006_motor.feedback_angle_deg;
    g_m2006_debug.feedback_total_angle_deg = s_m2006_motor.feedback_total_angle_deg;
    g_m2006_debug.feedback_current_A = s_m2006_motor.feedback_current_A;
    g_m2006_debug.feedback_temperature_c = s_m2006_motor.feedback_temperature_c;
}

static HAL_StatusTypeDef M2006_CAN3_Start(void)
{
    FDCAN_FilterTypeDef filter = {0};

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = DJI_M2006_FEEDBACK_ID;
    filter.FilterID2 = 0x7FFU;

    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &filter) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

static void M2006_DrainRxFifo(void)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(&hfdcan3, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
            ++g_m2006_debug.error_count;
            return;
        }

        if (rx_header.IdType == FDCAN_STANDARD_ID &&
            rx_header.Identifier == DJI_M2006_FEEDBACK_ID &&
            rx_header.DataLength == FDCAN_DLC_BYTES_8) {
            DJI_M2006_UpdateFeedback(&s_m2006_motor, rx_data);
            M2006_UpdateDebugFeedback();
            ++g_m2006_debug.rx_count;
        }
    }
}

static void M2006_SendCurrent(float current_A)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8];

    g_m2006_debug.command_current_raw = DJI_M2006_CurrentAToRaw(current_A);
    g_m2006_debug.command_current_A = DJI_M2006_RawCurrentToA(g_m2006_debug.command_current_raw);
    DJI_M2006_PackCurrentFrame(g_m2006_debug.command_current_raw, tx_data);

    tx_header.Identifier = DJI_M2006_CONTROL_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &tx_header, tx_data) == HAL_OK) {
        ++g_m2006_debug.tx_count;
    } else {
        ++g_m2006_debug.error_count;
    }
}

static float M2006_CalcCommandCurrent(uint32_t tick_count)
{
    float command_current_A = 0.0f;

    if (!g_m2006_debug.enable || g_m2006_debug.mode == M2006_MODE_STOP) {
        PID_Incremental_Reset(&s_m2006_speed_pid);
        PID_Position_Reset(&s_m2006_angle_pid);
        return 0.0f;
    }

    if (g_m2006_debug.mode == M2006_MODE_CURRENT) {
        command_current_A = g_m2006_debug.target_current_A;
    } else {
        s_m2006_speed_pid.params = g_m2006_debug.speed_pid_params;
        s_m2006_angle_pid.params = g_m2006_debug.angle_pid_params;

        if (g_m2006_debug.mode == M2006_MODE_POSITION &&
            (tick_count % M2006_POSITION_DIVIDER) == 0U) {
            g_m2006_debug.target_speed_rpm = PID_Position_Calc(&s_m2006_angle_pid,
                                                               g_m2006_debug.target_angle_deg,
                                                               g_m2006_debug.feedback_total_angle_deg);
        }

        command_current_A = PID_Incremental_Calc(&s_m2006_speed_pid,
                                                 g_m2006_debug.target_speed_rpm,
                                                 g_m2006_debug.feedback_speed_rpm);
    }

    return command_current_A;
}

void M2006_Task_Run(void *argument)
{
    (void)argument;

    DJI_M2006_Init(&s_m2006_motor);
    PID_Incremental_Init(&s_m2006_speed_pid, &g_m2006_debug.speed_pid_params, M2006_SPEED_DT_S);
    PID_Position_Init(&s_m2006_angle_pid, &g_m2006_debug.angle_pid_params, M2006_ANGLE_DT_S);

    if (M2006_CAN3_Start() != HAL_OK) {
        ++g_m2006_debug.error_count;
    }

    uint32_t tick_count = 0U;
    uint32_t wake_tick = osKernelGetTickCount();

    for (;;) {
        M2006_DrainRxFifo();
        M2006_SendCurrent(M2006_CalcCommandCurrent(tick_count));
        ++tick_count;
        osDelayUntil(wake_tick + 1U);
        wake_tick += 1U;
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan != NULL &&
        hfdcan->Instance == FDCAN3 &&
        (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        M2006_DrainRxFifo();
    }
}
