#include "m2006_axis.h"

/* C610 current command: 1000 command units correspond to 1 A. */
#define M2006_CURRENT_LIMIT               5000.0f
#define M2006_SPEED_KP                    2.0f
#define M2006_SPEED_KI                    8.0f
#define M2006_SPEED_INTEGRAL_LIMIT        2500.0f
#define M2006_ACCEL_LIMIT_RPM_S           18000.0f
#define M2006_MAX_ALLOWED_RPM             9000
#define M2006_HOLD_POSITION_KP_RPM_REV     120.0f
#define M2006_HOLD_MAX_SPEED_RPM           300.0f
#define M2006_CONTROL_PERIOD_MS           10U
#define M2006_FEEDBACK_TIMEOUT_MS         100U
#define M2006_STOP_REPEAT_COUNT           3U
#define M2006_ENCODER_COUNTS_PER_REV       8192
#define M2006_ENCODER_HALF_RANGE           4096

typedef struct
{
  volatile uint16_t ecd;
  volatile uint16_t last_ecd;
  volatile int64_t total_ecd;
  volatile int16_t speed_rpm;
  volatile int16_t torque_current;
  volatile uint32_t feedback_tick_ms;
  volatile uint32_t message_count;
  volatile uint8_t feedback_ready;
  volatile uint8_t position_initialized;
  /* Revision: establish the Y path zero on its first valid position read. */
  volatile uint8_t position_reference_ready;
  volatile int64_t position_zero_ecd;

  FDCAN_HandleTypeDef *hfdcan;
  int16_t target_speed_rpm;
  float command_speed_rpm;
  float speed_integral;
  float hold_target_position_rev;
  uint32_t last_control_ms;
  M2006_AxisState_t state;
  M2006_AxisFault_t fault;
} M2006_Axis_t;

static M2006_Axis_t m2006_axis = {0};

static float M2006_ClampFloat(float value, float min_value, float max_value)
{
  if (value > max_value)
  {
    return max_value;
  }
  if (value < min_value)
  {
    return min_value;
  }
  return value;
}

static float M2006_MoveTowards(float value, float target, float max_step)
{
  if (target > value + max_step)
  {
    return value + max_step;
  }
  if (target < value - max_step)
  {
    return value - max_step;
  }
  return target;
}

static int16_t M2006_ClampTargetRpm(int16_t target_rpm)
{
  if (target_rpm > M2006_MAX_ALLOWED_RPM)
  {
    return M2006_MAX_ALLOWED_RPM;
  }
  if (target_rpm < -M2006_MAX_ALLOWED_RPM)
  {
    return (int16_t)-M2006_MAX_ALLOWED_RPM;
  }
  return target_rpm;
}

static uint8_t M2006_FeedbackTimedOut(uint32_t now_ms,
                                      uint32_t feedback_tick_ms)
{
  /* The feedback ISR may advance its tick after the caller sampled now_ms. */
  int32_t feedback_age_ms = (int32_t)(now_ms - feedback_tick_ms);

  return (feedback_age_ms > (int32_t)M2006_FEEDBACK_TIMEOUT_MS) ? 1U : 0U;
}

static HAL_StatusTypeDef M2006_SendCurrent(int16_t current)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};
  uint8_t data[8] = {0};

  if (m2006_axis.hfdcan == 0)
  {
    return HAL_ERROR;
  }

  tx_header.Identifier = M2006_AXIS_CONTROL_ID;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  data[0] = (uint8_t)((uint16_t)current >> 8);
  data[1] = (uint8_t)current;
  return HAL_FDCAN_AddMessageToTxFifoQ(m2006_axis.hfdcan,
                                       &tx_header,
                                       data);
}

static HAL_StatusTypeDef M2006_ZeroOutput(void)
{
  uint32_t repeat;
  HAL_StatusTypeDef status = HAL_ERROR;

  for (repeat = 0U; repeat < M2006_STOP_REPEAT_COUNT; repeat++)
  {
    if (M2006_SendCurrent(0) == HAL_OK)
    {
      status = HAL_OK;
    }
    if ((repeat + 1U) < M2006_STOP_REPEAT_COUNT)
    {
      HAL_Delay(1U);
    }
  }

  return status;
}

static void M2006_SetFault(M2006_AxisFault_t fault)
{
  m2006_axis.target_speed_rpm = 0;
  m2006_axis.command_speed_rpm = 0.0f;
  m2006_axis.speed_integral = 0.0f;
  m2006_axis.fault = fault;
  m2006_axis.state = M2006_AXIS_FAULT;
  if (M2006_ZeroOutput() != HAL_OK)
  {
    m2006_axis.fault = M2006_AXIS_FAULT_CAN_TX;
  }
}

static int16_t M2006_SpeedPi(float target_rpm,
                             int16_t feedback_rpm,
                             float dt_s)
{
  float error = target_rpm - (float)feedback_rpm;
  float proportional = M2006_SPEED_KP * error;
  float candidate_integral;
  float unsaturated;
  float output;

  candidate_integral = m2006_axis.speed_integral +
                       M2006_SPEED_KI * error * dt_s;
  candidate_integral = M2006_ClampFloat(candidate_integral,
                                        -M2006_SPEED_INTEGRAL_LIMIT,
                                        M2006_SPEED_INTEGRAL_LIMIT);
  unsaturated = proportional + candidate_integral;
  output = M2006_ClampFloat(unsaturated,
                            -M2006_CURRENT_LIMIT,
                            M2006_CURRENT_LIMIT);

  if ((unsaturated == output) ||
      ((output >= M2006_CURRENT_LIMIT) && (error < 0.0f)) ||
      ((output <= -M2006_CURRENT_LIMIT) && (error > 0.0f)))
  {
    m2006_axis.speed_integral = candidate_integral;
  }

  output = proportional + m2006_axis.speed_integral;
  output = M2006_ClampFloat(output,
                            -M2006_CURRENT_LIMIT,
                            M2006_CURRENT_LIMIT);
  return (int16_t)output;
}

HAL_StatusTypeDef M2006_Axis_Init(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_FilterTypeDef filter = {0};

  if (hfdcan == 0)
  {
    return HAL_ERROR;
  }

  m2006_axis.hfdcan = hfdcan;
  m2006_axis.feedback_ready = 0U;
  m2006_axis.position_initialized = 0U;
  m2006_axis.position_reference_ready = 0U;
  m2006_axis.total_ecd = 0;
  m2006_axis.position_zero_ecd = 0;
  m2006_axis.message_count = 0U;
  m2006_axis.target_speed_rpm = 0;
  m2006_axis.command_speed_rpm = 0.0f;
  m2006_axis.speed_integral = 0.0f;
  m2006_axis.hold_target_position_rev = 0.0f;
  m2006_axis.state = M2006_AXIS_IDLE;
  m2006_axis.fault = M2006_AXIS_FAULT_NONE;

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = M2006_AXIS_FEEDBACK_ID;
  filter.FilterID2 = 0x7FFU;

  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_FDCAN_ActivateNotification(hfdcan,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return M2006_ZeroOutput();
}

void M2006_Axis_OnFeedback(const uint8_t data[8], uint32_t now_ms)
{
  uint16_t ecd;
  int32_t ecd_delta;

  if (data == 0)
  {
    return;
  }

  ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  if (m2006_axis.position_initialized == 0U)
  {
    m2006_axis.total_ecd = 0;
    m2006_axis.position_initialized = 1U;
  }
  else
  {
    ecd_delta = (int32_t)ecd - (int32_t)m2006_axis.last_ecd;
    if (ecd_delta > M2006_ENCODER_HALF_RANGE)
    {
      ecd_delta -= M2006_ENCODER_COUNTS_PER_REV;
    }
    else if (ecd_delta < -M2006_ENCODER_HALF_RANGE)
    {
      ecd_delta += M2006_ENCODER_COUNTS_PER_REV;
    }
    m2006_axis.total_ecd += ecd_delta;
  }
  m2006_axis.ecd = ecd;
  m2006_axis.last_ecd = ecd;
  m2006_axis.speed_rpm =
      (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  m2006_axis.torque_current =
      (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  m2006_axis.feedback_tick_ms = now_ms;
  m2006_axis.message_count++;
  m2006_axis.feedback_ready = 1U;
}

uint8_t M2006_Axis_StartSpeed(int16_t target_rpm)
{
  uint32_t now_ms = HAL_GetTick();

  if (m2006_axis.feedback_ready == 0U)
  {
    M2006_SetFault(M2006_AXIS_FAULT_NO_FEEDBACK);
    return 0U;
  }
  if (M2006_FeedbackTimedOut(now_ms,
                            m2006_axis.feedback_tick_ms) != 0U)
  {
    M2006_SetFault(M2006_AXIS_FAULT_FEEDBACK_TIMEOUT);
    return 0U;
  }

  m2006_axis.target_speed_rpm = M2006_ClampTargetRpm(target_rpm);
  m2006_axis.command_speed_rpm = 0.0f;
  m2006_axis.speed_integral = 0.0f;
  m2006_axis.last_control_ms = now_ms;
  m2006_axis.fault = M2006_AXIS_FAULT_NONE;
  m2006_axis.state = M2006_AXIS_RUNNING;
  return 1U;
}

uint8_t M2006_Axis_SetSpeedTarget(int16_t target_rpm)
{
  if (m2006_axis.state != M2006_AXIS_RUNNING)
  {
    return 0U;
  }

  m2006_axis.target_speed_rpm = M2006_ClampTargetRpm(target_rpm);
  return 1U;
}

uint8_t M2006_Axis_StartPositionHold(float target_position_rev)
{
  float current_position_rev;
  uint32_t now_ms = HAL_GetTick();

  if (target_position_rev != target_position_rev)
  {
    return 0U;
  }
  if (m2006_axis.feedback_ready == 0U)
  {
    M2006_SetFault(M2006_AXIS_FAULT_NO_FEEDBACK);
    return 0U;
  }
  if ((M2006_FeedbackTimedOut(now_ms,
                             m2006_axis.feedback_tick_ms) != 0U) ||
      (M2006_Axis_GetPositionRev(&current_position_rev) == 0U))
  {
    M2006_SetFault(M2006_AXIS_FAULT_FEEDBACK_TIMEOUT);
    return 0U;
  }

  m2006_axis.hold_target_position_rev = target_position_rev;
  m2006_axis.target_speed_rpm = 0;
  m2006_axis.command_speed_rpm = 0.0f;
  m2006_axis.speed_integral = 0.0f;
  m2006_axis.last_control_ms = now_ms;
  m2006_axis.fault = M2006_AXIS_FAULT_NONE;
  m2006_axis.state = M2006_AXIS_HOLDING;
  return 1U;
}

void M2006_Axis_Update(uint32_t now_ms)
{
  uint32_t elapsed_ms;
  float dt_s;
  float speed_step;
  float current_position_rev;
  float hold_speed_rpm;
  int16_t current;

  if ((m2006_axis.state != M2006_AXIS_RUNNING) &&
      (m2006_axis.state != M2006_AXIS_HOLDING))
  {
    return;
  }
  if (M2006_FeedbackTimedOut(now_ms,
                            m2006_axis.feedback_tick_ms) != 0U)
  {
    M2006_SetFault(M2006_AXIS_FAULT_FEEDBACK_TIMEOUT);
    return;
  }

  elapsed_ms = now_ms - m2006_axis.last_control_ms;
  if (elapsed_ms < M2006_CONTROL_PERIOD_MS)
  {
    return;
  }
  m2006_axis.last_control_ms = now_ms;
  dt_s = (float)elapsed_ms / 1000.0f;
  if (dt_s > 0.05f)
  {
    dt_s = 0.05f;
  }

  if (m2006_axis.state == M2006_AXIS_HOLDING)
  {
    if (M2006_Axis_GetPositionRev(&current_position_rev) == 0U)
    {
      M2006_SetFault(M2006_AXIS_FAULT_FEEDBACK_TIMEOUT);
      return;
    }

    hold_speed_rpm = M2006_HOLD_POSITION_KP_RPM_REV *
                     (m2006_axis.hold_target_position_rev -
                      current_position_rev);
    hold_speed_rpm = M2006_ClampFloat(hold_speed_rpm,
                                      -M2006_HOLD_MAX_SPEED_RPM,
                                      M2006_HOLD_MAX_SPEED_RPM);
    if (hold_speed_rpm >= 0.0f)
    {
      m2006_axis.target_speed_rpm = (int16_t)(hold_speed_rpm + 0.5f);
    }
    else
    {
      m2006_axis.target_speed_rpm = (int16_t)(hold_speed_rpm - 0.5f);
    }
  }

  speed_step = M2006_ACCEL_LIMIT_RPM_S * dt_s;
  m2006_axis.command_speed_rpm =
      M2006_MoveTowards(m2006_axis.command_speed_rpm,
                        (float)m2006_axis.target_speed_rpm,
                        speed_step);
  current = M2006_SpeedPi(m2006_axis.command_speed_rpm,
                          m2006_axis.speed_rpm,
                          dt_s);
  if (M2006_SendCurrent(current) != HAL_OK)
  {
    M2006_SetFault(M2006_AXIS_FAULT_CAN_TX);
  }
}

void M2006_Axis_Stop(void)
{
  m2006_axis.target_speed_rpm = 0;
  m2006_axis.command_speed_rpm = 0.0f;
  m2006_axis.speed_integral = 0.0f;
  if (M2006_ZeroOutput() == HAL_OK)
  {
    m2006_axis.state = M2006_AXIS_IDLE;
  }
  else
  {
    m2006_axis.fault = M2006_AXIS_FAULT_CAN_TX;
    m2006_axis.state = M2006_AXIS_FAULT;
  }
}

uint8_t M2006_Axis_HasFeedback(void)
{
  return m2006_axis.feedback_ready;
}

uint8_t M2006_Axis_HasFault(void)
{
  return (m2006_axis.state == M2006_AXIS_FAULT) ? 1U : 0U;
}

M2006_AxisFault_t M2006_Axis_GetFault(void)
{
  return m2006_axis.fault;
}

uint8_t M2006_Axis_GetPositionRev(float *position_rev)
{
  int64_t total_ecd;
  int64_t position_zero_ecd;
  uint32_t feedback_tick_ms;
  uint32_t now_ms;
  uint8_t position_initialized;
  uint32_t primask;

  if (position_rev == 0)
  {
    return 0U;
  }

  now_ms = HAL_GetTick();
  primask = __get_PRIMASK();
  __disable_irq();
  total_ecd = m2006_axis.total_ecd;
  feedback_tick_ms = m2006_axis.feedback_tick_ms;
  position_initialized = m2006_axis.position_initialized;
  if ((position_initialized != 0U) &&
      (M2006_FeedbackTimedOut(now_ms, feedback_tick_ms) == 0U) &&
      (m2006_axis.position_reference_ready == 0U))
  {
    m2006_axis.position_zero_ecd = total_ecd;
    m2006_axis.position_reference_ready = 1U;
  }
  position_zero_ecd = m2006_axis.position_zero_ecd;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((position_initialized == 0U) ||
      (M2006_FeedbackTimedOut(now_ms, feedback_tick_ms) != 0U))
  {
    return 0U;
  }

  *position_rev = (float)(total_ecd - position_zero_ecd) /
                  (float)M2006_ENCODER_COUNTS_PER_REV;
  return 1U;
}
