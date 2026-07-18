/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "m2006_axis.h"
#include "route_plan.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t error;
  float position_rad;
  float last_position_rad;
  float position_unwrapped_rad;
  float velocity_rad_s;
  float torque_nm;
  float pmax_rad;
  float vmax_rad_s;
  float tmax_nm;
  uint32_t msg_count;
  uint32_t last_feedback_tick;
  uint8_t position_initialized;
} DM3519_Feedback_t;

typedef enum
{
  X_SYNC_FAULT_NONE = 0U,
  X_SYNC_FAULT_POSITION_ERROR = 1U,
  X_SYNC_FAULT_FEEDBACK_TIMEOUT = 2U,
  X_SYNC_FAULT_TARGET_TIMEOUT = 3U
} X_SyncFault_t;

typedef struct
{
  float target_position_rad;
  uint32_t start_tick;
  uint32_t last_command_tick;
  uint32_t stable_start_tick;
  uint8_t within_tolerance;
} X_TargetCorrection_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DM3519_X_MOTOR1_ID                 0x01U
#define DM3519_X_MOTOR2_ID                 0x02U
#define DM3519_X_MOTOR1_MASTER_ID          0x03U
#define DM3519_X_MOTOR2_MASTER_ID          0x04U
#define DM3519_VELOCITY_MODE_ID            0x200U
#define DM3519_CONTROL_STD_ID(id)          (DM3519_VELOCITY_MODE_ID + (id))
#define DM3519_PARAM_STD_ID                0x7FFU
#define DM3519_PARAM_READ_CMD              0x33U
#define DM3519_PARAM_PMAX_RID              0x15U
#define DM3519_PARAM_VMAX_RID              0x16U
#define DM3519_PARAM_TMAX_RID              0x17U
#define DM3519_FALLBACK_PMAX_RAD           12.5f
#define DM3519_FALLBACK_VMAX_RAD_S         45.0f
#define DM3519_FALLBACK_TMAX_NM            18.0f
#define DM3519_X_REDUCTION_RATIO           19.2f

#define X_RUN_SPEED_RAD_S                  15.0f
#define X_COMMAND_REFRESH_MS               20U
#define X_SYNC_KP                          1.0f
#define X_SYNC_MAX_CORRECTION_RAD_S        1.5f
#define X_SYNC_POSITION_FAULT_RAD          19.2f
#define X_SYNC_FEEDBACK_TIMEOUT_MS         100U
#define X_TARGET_KP                        0.5f
#define X_TARGET_MAX_VELOCITY_RAD_S        2.0f
#define X_TARGET_TOLERANCE_RAD             2.0f
#define X_TARGET_STABLE_TIME_MS            200U
#define X_TARGET_SETTLE_TIMEOUT_MS         3000U
#define X_STOP_REPEAT_COUNT                3U
#define X_STOP_REPEAT_INTERVAL_MS          2U
#define Y_TARGET_SETTLE_TIMEOUT_MS         3000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile DM3519_Feedback_t dm3519_x_motor1 = {
  .pmax_rad = DM3519_FALLBACK_PMAX_RAD,
  .vmax_rad_s = DM3519_FALLBACK_VMAX_RAD_S,
  .tmax_nm = DM3519_FALLBACK_TMAX_NM
};
static volatile DM3519_Feedback_t dm3519_x_motor2 = {
  .pmax_rad = DM3519_FALLBACK_PMAX_RAD,
  .vmax_rad_s = DM3519_FALLBACK_VMAX_RAD_S,
  .tmax_nm = DM3519_FALLBACK_TMAX_NM
};
static volatile float x_motor1_position_zero_rad = 0.0f;
static volatile float x_motor2_position_zero_rad = 0.0f;
static volatile X_SyncFault_t x_sync_fault = X_SYNC_FAULT_NONE;
static uint8_t x_position_reference_ready = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint32_t FDCAN_DLC_FromLength(uint8_t len)
{
  if (len == 4U)
  {
    return FDCAN_DLC_BYTES_4;
  }

  if (len == 8U)
  {
    return FDCAN_DLC_BYTES_8;
  }

  Error_Handler();
  return FDCAN_DLC_BYTES_0;
}

static void FDCAN_SendStandardFrame(FDCAN_HandleTypeDef *hfdcan,
                                    uint16_t std_id,
                                    uint8_t *data,
                                    uint8_t len)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};

  tx_header.Identifier = std_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_FromLength(len);
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data) != HAL_OK)
  {
    Error_Handler();
  }
}

static void DM3519_EnableVelocityMode(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
  uint8_t data[8] = {
    0xFFU, 0xFFU, 0xFFU, 0xFFU,
    0xFFU, 0xFFU, 0xFFU, 0xFCU
  };

  FDCAN_SendStandardFrame(hfdcan, DM3519_CONTROL_STD_ID(motor_id), data, 8U);
}

static void DM3519_SetVelocity(FDCAN_HandleTypeDef *hfdcan,
                               uint8_t motor_id,
                               float velocity_rad_s)
{
  union
  {
    float value;
    uint8_t bytes[4];
  } velocity;
  uint8_t data[4];

  velocity.value = velocity_rad_s;
  data[0] = velocity.bytes[0];
  data[1] = velocity.bytes[1];
  data[2] = velocity.bytes[2];
  data[3] = velocity.bytes[3];

  FDCAN_SendStandardFrame(hfdcan, DM3519_CONTROL_STD_ID(motor_id), data, 4U);
}

static void DM3519_CAN1_Start(void)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID2 = 0x7FFU;

  filter.FilterIndex = 0U;
  filter.FilterID1 = DM3519_X_MOTOR1_MASTER_ID;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  filter.FilterIndex = 1U;
  filter.FilterID1 = DM3519_X_MOTOR2_MASTER_ID;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0U) != HAL_OK)
  {
    Error_Handler();
  }
}

static void DM3519_RequestMappingRange(uint8_t motor_id, uint8_t rid)
{
  uint8_t data[8] = {0};

  data[0] = motor_id;
  data[2] = DM3519_PARAM_READ_CMD;
  data[3] = rid;
  FDCAN_SendStandardFrame(&hfdcan1, DM3519_PARAM_STD_ID, data, 8U);
}

static void DM3519_RequestMappingRanges(uint8_t motor_id)
{
  DM3519_RequestMappingRange(motor_id, DM3519_PARAM_PMAX_RID);
  HAL_Delay(2U);
  DM3519_RequestMappingRange(motor_id, DM3519_PARAM_VMAX_RID);
  HAL_Delay(2U);
  DM3519_RequestMappingRange(motor_id, DM3519_PARAM_TMAX_RID);
  HAL_Delay(2U);
}

static float DM3519_UintToFloat(uint16_t value,
                                float min_value,
                                float max_value,
                                uint8_t bits)
{
  uint32_t max_int = (1UL << bits) - 1UL;

  return ((float)value * (max_value - min_value) / (float)max_int) + min_value;
}

static uint8_t DM3519_DecodeMappingRange(volatile DM3519_Feedback_t *motor,
                                         uint8_t motor_id,
                                         const uint8_t data[8])
{
  union
  {
    float value;
    uint8_t bytes[4];
  } range;

  if ((data[0] != motor_id) ||
      (data[1] != 0x00U) ||
      (data[2] != DM3519_PARAM_READ_CMD))
  {
    return 0U;
  }

  range.bytes[0] = data[4];
  range.bytes[1] = data[5];
  range.bytes[2] = data[6];
  range.bytes[3] = data[7];

  if (range.value <= 0.0f)
  {
    return 1U;
  }

  if (data[3] == DM3519_PARAM_PMAX_RID)
  {
    motor->pmax_rad = range.value;
    motor->position_initialized = 0U;
  }
  else if (data[3] == DM3519_PARAM_VMAX_RID)
  {
    motor->vmax_rad_s = range.value;
  }
  else if (data[3] == DM3519_PARAM_TMAX_RID)
  {
    motor->tmax_nm = range.value;
  }

  return 1U;
}

static void DM3519_DecodeFeedback(volatile DM3519_Feedback_t *motor,
                                  uint8_t motor_id,
                                  const uint8_t data[8])
{
  uint16_t position_raw;
  uint16_t velocity_raw;
  uint16_t torque_raw;
  float position_rad;
  float position_delta_rad;

  if (DM3519_DecodeMappingRange(motor, motor_id, data) != 0U)
  {
    return;
  }

  if ((data[0] & 0x0FU) != (motor_id & 0x0FU))
  {
    return;
  }

  position_raw = ((uint16_t)data[1] << 8) | data[2];
  velocity_raw = ((uint16_t)data[3] << 4) | ((uint16_t)data[4] >> 4);
  torque_raw = (((uint16_t)data[4] & 0x0FU) << 8) | data[5];

  motor->error = data[0] >> 4;
  position_rad = DM3519_UintToFloat(position_raw,
                                     -motor->pmax_rad,
                                     motor->pmax_rad,
                                     16U);

  if (motor->position_initialized == 0U)
  {
    motor->position_unwrapped_rad = position_rad;
    motor->position_initialized = 1U;
  }
  else
  {
    position_delta_rad = position_rad - motor->last_position_rad;
    if (position_delta_rad > motor->pmax_rad)
    {
      position_delta_rad -= 2.0f * motor->pmax_rad;
    }
    else if (position_delta_rad < -motor->pmax_rad)
    {
      position_delta_rad += 2.0f * motor->pmax_rad;
    }
    motor->position_unwrapped_rad += position_delta_rad;
  }

  motor->position_rad = position_rad;
  motor->last_position_rad = position_rad;
  motor->velocity_rad_s = DM3519_UintToFloat(velocity_raw,
                                              -motor->vmax_rad_s,
                                              motor->vmax_rad_s,
                                              12U);
  motor->torque_nm = DM3519_UintToFloat(torque_raw,
                                         -motor->tmax_nm,
                                         motor->tmax_nm,
                                         12U);
  motor->msg_count++;
  motor->last_feedback_tick = HAL_GetTick();
}

static void X_SetMotorVelocities(float motor1_velocity_rad_s,
                                 float motor2_velocity_rad_s)
{
  DM3519_SetVelocity(&hfdcan1,
                      DM3519_X_MOTOR1_ID,
                      motor1_velocity_rad_s);
  DM3519_SetVelocity(&hfdcan1,
                      DM3519_X_MOTOR2_ID,
                      motor2_velocity_rad_s);
}

static void X_SetPairedVelocity(float motor1_velocity_rad_s)
{
  X_SetMotorVelocities(motor1_velocity_rad_s, -motor1_velocity_rad_s);
}

static void X_EnablePairedVelocityMode(void)
{
  DM3519_EnableVelocityMode(&hfdcan1, DM3519_X_MOTOR1_ID);
  DM3519_SetVelocity(&hfdcan1, DM3519_X_MOTOR1_ID, 0.0f);
  DM3519_EnableVelocityMode(&hfdcan1, DM3519_X_MOTOR2_ID);
  DM3519_SetVelocity(&hfdcan1, DM3519_X_MOTOR2_ID, 0.0f);
  HAL_Delay(5U);
  X_SetPairedVelocity(0.0f);
}

static void X_Stop(void)
{
  uint32_t repeat;

  for (repeat = 0U; repeat < X_STOP_REPEAT_COUNT; repeat++)
  {
    X_SetPairedVelocity(0.0f);
    if ((repeat + 1U) < X_STOP_REPEAT_COUNT)
    {
      HAL_Delay(X_STOP_REPEAT_INTERVAL_MS);
    }
  }
}

static float X_ClampVelocity(float velocity_rad_s, float base_velocity_rad_s)
{
  if (base_velocity_rad_s > 0.0f)
  {
    if (velocity_rad_s < 0.0f)
    {
      return 0.0f;
    }
    if (velocity_rad_s > X_RUN_SPEED_RAD_S)
    {
      return X_RUN_SPEED_RAD_S;
    }
  }
  else if (base_velocity_rad_s < 0.0f)
  {
    if (velocity_rad_s > 0.0f)
    {
      return 0.0f;
    }
    if (velocity_rad_s < -X_RUN_SPEED_RAD_S)
    {
      return -X_RUN_SPEED_RAD_S;
    }
  }
  else
  {
    return 0.0f;
  }

  return velocity_rad_s;
}

static float X_ClampSignedVelocity(float velocity_rad_s, float limit_rad_s)
{
  if (velocity_rad_s > limit_rad_s)
  {
    return limit_rad_s;
  }
  if (velocity_rad_s < -limit_rad_s)
  {
    return -limit_rad_s;
  }
  return velocity_rad_s;
}

static void X_LatchSyncFault(X_SyncFault_t fault)
{
  if (x_sync_fault == X_SYNC_FAULT_NONE)
  {
    x_sync_fault = fault;
  }
  X_SetMotorVelocities(0.0f, 0.0f);
  M2006_Axis_Stop();
}

static uint8_t X_EnsurePositionReference(uint32_t now)
{
  uint32_t motor1_feedback_tick;
  uint32_t motor2_feedback_tick;
  uint8_t motor1_initialized;
  uint8_t motor2_initialized;
  uint32_t primask;

  if (x_position_reference_ready != 0U)
  {
    return 1U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  motor1_feedback_tick = dm3519_x_motor1.last_feedback_tick;
  motor2_feedback_tick = dm3519_x_motor2.last_feedback_tick;
  motor1_initialized = dm3519_x_motor1.position_initialized;
  motor2_initialized = dm3519_x_motor2.position_initialized;
  if ((motor1_initialized != 0U) && (motor2_initialized != 0U))
  {
    x_motor1_position_zero_rad = dm3519_x_motor1.position_unwrapped_rad;
    x_motor2_position_zero_rad = dm3519_x_motor2.position_unwrapped_rad;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((motor1_initialized == 0U) ||
      (motor2_initialized == 0U) ||
      ((now - motor1_feedback_tick) > X_SYNC_FEEDBACK_TIMEOUT_MS) ||
      ((now - motor2_feedback_tick) > X_SYNC_FEEDBACK_TIMEOUT_MS))
  {
    X_LatchSyncFault(X_SYNC_FAULT_FEEDBACK_TIMEOUT);
    return 0U;
  }

  x_position_reference_ready = 1U;
  return 1U;
}

static uint8_t X_GetSynchronizedPositions(uint32_t now,
                                          float *motor1_position_rad,
                                          float *motor2_position_rad)
{
  float position_error_rad;
  uint32_t motor1_feedback_tick;
  uint32_t motor2_feedback_tick;
  uint8_t motor1_initialized;
  uint8_t motor2_initialized;
  uint32_t primask;

  if ((x_sync_fault != X_SYNC_FAULT_NONE) ||
      (X_EnsurePositionReference(now) == 0U))
  {
    X_SetMotorVelocities(0.0f, 0.0f);
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *motor1_position_rad = dm3519_x_motor1.position_unwrapped_rad -
                         x_motor1_position_zero_rad;
  *motor2_position_rad = -(dm3519_x_motor2.position_unwrapped_rad -
                           x_motor2_position_zero_rad);
  motor1_feedback_tick = dm3519_x_motor1.last_feedback_tick;
  motor2_feedback_tick = dm3519_x_motor2.last_feedback_tick;
  motor1_initialized = dm3519_x_motor1.position_initialized;
  motor2_initialized = dm3519_x_motor2.position_initialized;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if ((motor1_initialized == 0U) ||
      (motor2_initialized == 0U) ||
      ((now - motor1_feedback_tick) > X_SYNC_FEEDBACK_TIMEOUT_MS) ||
      ((now - motor2_feedback_tick) > X_SYNC_FEEDBACK_TIMEOUT_MS))
  {
    X_LatchSyncFault(X_SYNC_FAULT_FEEDBACK_TIMEOUT);
    return 0U;
  }

  position_error_rad = *motor1_position_rad - *motor2_position_rad;
  if ((position_error_rad > X_SYNC_POSITION_FAULT_RAD) ||
      (position_error_rad < -X_SYNC_POSITION_FAULT_RAD))
  {
    X_LatchSyncFault(X_SYNC_FAULT_POSITION_ERROR);
    return 0U;
  }

  return 1U;
}

static void X_SetSynchronizedVelocity(float base_velocity_rad_s, uint32_t now)
{
  float motor1_position_rad;
  float motor2_position_rad;
  float position_error_rad;
  float correction_rad_s;
  float motor1_velocity_rad_s;
  float motor2_velocity_rad_s;

  if (X_GetSynchronizedPositions(now,
                                 &motor1_position_rad,
                                 &motor2_position_rad) == 0U)
  {
    return;
  }

  if (base_velocity_rad_s == 0.0f)
  {
    X_SetMotorVelocities(0.0f, 0.0f);
    return;
  }

  position_error_rad = motor1_position_rad - motor2_position_rad;
  correction_rad_s = X_SYNC_KP * position_error_rad /
                     DM3519_X_REDUCTION_RATIO;
  correction_rad_s = X_ClampSignedVelocity(correction_rad_s,
                                            X_SYNC_MAX_CORRECTION_RAD_S);

  motor1_velocity_rad_s = X_ClampVelocity(base_velocity_rad_s -
                                           correction_rad_s,
                                           base_velocity_rad_s);
  motor2_velocity_rad_s = X_ClampVelocity(base_velocity_rad_s +
                                           correction_rad_s,
                                           base_velocity_rad_s);
  X_SetMotorVelocities(motor1_velocity_rad_s, -motor2_velocity_rad_s);
}

static uint8_t X_GetAxisPosition(uint32_t now, float *position_rad)
{
  float motor1_position_rad;
  float motor2_position_rad;

  if (X_GetSynchronizedPositions(now,
                                 &motor1_position_rad,
                                 &motor2_position_rad) == 0U)
  {
    return 0U;
  }

  *position_rad = 0.5f *
                  (motor1_position_rad + motor2_position_rad) /
                  DM3519_X_REDUCTION_RATIO;
  return 1U;
}

static uint8_t X_PrepareTargetMotion(float target_position_rad,
                                     float speed_rad_s,
                                     uint32_t now,
                                     uint32_t *run_time_ms,
                                     float *velocity_rad_s,
                                     float *motor_target_position_rad)
{
  float current_position_rad;

  if ((X_GetAxisPosition(now, &current_position_rad) == 0U) ||
      (RoutePlan_CalculateRunTimeMs(current_position_rad,
                                    target_position_rad,
                                    speed_rad_s,
                                    run_time_ms) == 0U))
  {
    return 0U;
  }

  if (target_position_rad > current_position_rad)
  {
    *velocity_rad_s = speed_rad_s;
  }
  else if (target_position_rad < current_position_rad)
  {
    *velocity_rad_s = -speed_rad_s;
  }
  else
  {
    *velocity_rad_s = 0.0f;
  }
  *motor_target_position_rad = target_position_rad *
                               DM3519_X_REDUCTION_RATIO;
  return 1U;
}

static void X_TargetCorrectionStart(X_TargetCorrection_t *correction,
                                    float target_position_rad,
                                    uint32_t now)
{
  correction->target_position_rad = target_position_rad;
  correction->start_tick = now;
  correction->last_command_tick = now - X_COMMAND_REFRESH_MS;
  correction->stable_start_tick = 0U;
  correction->within_tolerance = 0U;
  X_SetPairedVelocity(0.0f);
}

static int8_t X_TargetCorrectionUpdate(X_TargetCorrection_t *correction,
                                       uint32_t now)
{
  float motor1_position_rad;
  float motor2_position_rad;
  float motor1_error_rad;
  float motor2_error_rad;
  float motor1_abs_error_rad;
  float motor2_abs_error_rad;
  float average_error_rad;
  float common_velocity_rad_s;
  float sync_correction_rad_s;
  float motor1_velocity_rad_s;
  float motor2_velocity_rad_s;

  if (x_sync_fault != X_SYNC_FAULT_NONE)
  {
    return -1;
  }
  if ((now - correction->start_tick) >= X_TARGET_SETTLE_TIMEOUT_MS)
  {
    X_LatchSyncFault(X_SYNC_FAULT_TARGET_TIMEOUT);
    return -1;
  }
  if ((now - correction->last_command_tick) < X_COMMAND_REFRESH_MS)
  {
    return 0;
  }
  correction->last_command_tick = now;

  if (X_GetSynchronizedPositions(now,
                                 &motor1_position_rad,
                                 &motor2_position_rad) == 0U)
  {
    return -1;
  }

  motor1_error_rad = correction->target_position_rad - motor1_position_rad;
  motor2_error_rad = correction->target_position_rad - motor2_position_rad;
  motor1_abs_error_rad = (motor1_error_rad >= 0.0f) ?
                         motor1_error_rad : -motor1_error_rad;
  motor2_abs_error_rad = (motor2_error_rad >= 0.0f) ?
                         motor2_error_rad : -motor2_error_rad;

  if ((motor1_abs_error_rad <= X_TARGET_TOLERANCE_RAD) &&
      (motor2_abs_error_rad <= X_TARGET_TOLERANCE_RAD))
  {
    X_SetMotorVelocities(0.0f, 0.0f);
    if (correction->within_tolerance == 0U)
    {
      correction->within_tolerance = 1U;
      correction->stable_start_tick = now;
    }
    else if ((now - correction->stable_start_tick) >=
             X_TARGET_STABLE_TIME_MS)
    {
      return 1;
    }
  }
  else
  {
    correction->within_tolerance = 0U;
    average_error_rad = 0.5f * (motor1_error_rad + motor2_error_rad);
    common_velocity_rad_s = X_TARGET_KP * average_error_rad /
                            DM3519_X_REDUCTION_RATIO;
    common_velocity_rad_s =
      X_ClampSignedVelocity(common_velocity_rad_s,
                            X_TARGET_MAX_VELOCITY_RAD_S);

    sync_correction_rad_s =
      X_SYNC_KP * (motor1_position_rad - motor2_position_rad) /
      DM3519_X_REDUCTION_RATIO;
    sync_correction_rad_s =
      X_ClampSignedVelocity(sync_correction_rad_s,
                            X_SYNC_MAX_CORRECTION_RAD_S);

    motor1_velocity_rad_s =
      X_ClampSignedVelocity(common_velocity_rad_s - sync_correction_rad_s,
                            X_TARGET_MAX_VELOCITY_RAD_S);
    motor2_velocity_rad_s =
      X_ClampSignedVelocity(common_velocity_rad_s + sync_correction_rad_s,
                            X_TARGET_MAX_VELOCITY_RAD_S);
    X_SetMotorVelocities(motor1_velocity_rad_s, -motor2_velocity_rad_s);
  }

  return 0;
}

static void Motion_StopAll(void)
{
  X_Stop();
  M2006_Axis_Stop();
}

static uint8_t Y_PrepareTargetMotion(float target_position_rev,
                                     uint16_t speed_rpm,
                                     uint32_t *run_time_ms,
                                     int16_t *target_rpm)
{
  float current_position_rev;

  if ((M2006_Axis_GetPositionRev(&current_position_rev) == 0U) ||
      (RoutePlan_CalculateRunTimeMs(current_position_rev,
                                    target_position_rev,
                                    (float)speed_rpm / 60.0f,
                                    run_time_ms) == 0U))
  {
    return 0U;
  }

  if (target_position_rev > current_position_rev)
  {
    *target_rpm = (int16_t)speed_rpm;
  }
  else if (target_position_rev < current_position_rev)
  {
    *target_rpm = -(int16_t)speed_rpm;
  }
  else
  {
    *target_rpm = 0;
  }
  return 1U;
}

static uint8_t Y_TargetReached(float target_position_rev,
                               int16_t target_rpm,
                               uint8_t *reached)
{
  float current_position_rev;

  if (M2006_Axis_GetPositionRev(&current_position_rev) == 0U)
  {
    return 0U;
  }

  if (target_rpm > 0)
  {
    *reached = (current_position_rev >= target_position_rev) ? 1U : 0U;
  }
  else if (target_rpm < 0)
  {
    *reached = (current_position_rev <= target_position_rev) ? 1U : 0U;
  }
  else
  {
    *reached = 1U;
  }
  return 1U;
}

static uint8_t XY_RunSelectedAxes(const MotionSegment_t *segment,
                                  uint8_t run_x,
                                  uint8_t run_y)
{
  uint32_t start_tick;
  uint32_t last_x_command_tick;
  uint32_t x_run_time_ms = 0U;
  uint32_t y_run_time_ms = 0U;
  float x_velocity_rad_s = 0.0f;
  float x_motor_target_position_rad = 0.0f;
  int16_t y_target_rpm = 0;
  uint8_t x_running;
  uint8_t x_correcting = 0U;
  uint8_t y_running;
  X_TargetCorrection_t correction;

  if ((run_x != 0U) &&
      (X_PrepareTargetMotion(segment->x_target_position_rad,
                             segment->x_speed_rad_s,
                             HAL_GetTick(),
                             &x_run_time_ms,
                             &x_velocity_rad_s,
                             &x_motor_target_position_rad) == 0U))
  {
    Motion_StopAll();
    return 0U;
  }

  if ((run_y != 0U) &&
      (Y_PrepareTargetMotion(segment->y_target_position_rev,
                             segment->y_speed_rpm,
                             &y_run_time_ms,
                             &y_target_rpm) == 0U))
  {
    Motion_StopAll();
    return 0U;
  }

  x_running = ((run_x != 0U) && (x_run_time_ms > 0U)) ? 1U : 0U;
  y_running = ((run_y != 0U) && (y_run_time_ms > 0U)) ? 1U : 0U;
  start_tick = HAL_GetTick();
  last_x_command_tick = start_tick;

  if (x_running != 0U)
  {
    X_SetSynchronizedVelocity(x_velocity_rad_s, start_tick);
    if (x_sync_fault != X_SYNC_FAULT_NONE)
    {
      Motion_StopAll();
      return 0U;
    }
  }
  if ((y_running != 0U) &&
      (M2006_Axis_StartSpeed(y_target_rpm) == 0U))
  {
    Motion_StopAll();
    return 0U;
  }

  while ((x_running != 0U) ||
         (x_correcting != 0U) ||
         (y_running != 0U))
  {
    uint32_t now = HAL_GetTick();
    uint32_t elapsed_ms = now - start_tick;

    if (((run_x != 0U) && (x_sync_fault != X_SYNC_FAULT_NONE)) ||
        ((run_y != 0U) && (M2006_Axis_HasFault() != 0U)))
    {
      Motion_StopAll();
      return 0U;
    }

    if (x_running != 0U)
    {
      if (elapsed_ms >= x_run_time_ms)
      {
        X_TargetCorrectionStart(&correction,
                                x_motor_target_position_rad,
                                now);
        x_running = 0U;
        x_correcting = 1U;
      }
      else if ((now - last_x_command_tick) >= X_COMMAND_REFRESH_MS)
      {
        last_x_command_tick = now;
        X_SetSynchronizedVelocity(x_velocity_rad_s, now);
      }
    }

    if (x_correcting != 0U)
    {
      int8_t correction_status = X_TargetCorrectionUpdate(&correction, now);

      if (correction_status < 0)
      {
        Motion_StopAll();
        return 0U;
      }
      if (correction_status > 0)
      {
        X_Stop();
        x_correcting = 0U;
      }
    }

    if (y_running != 0U)
    {
      uint8_t y_reached;

      M2006_Axis_Update(now);
      if ((M2006_Axis_HasFault() != 0U) ||
          (Y_TargetReached(segment->y_target_position_rev,
                           y_target_rpm,
                           &y_reached) == 0U))
      {
        Motion_StopAll();
        return 0U;
      }
      if (y_reached != 0U)
      {
        M2006_Axis_Stop();
        y_running = 0U;
      }
      else if ((elapsed_ms >= y_run_time_ms) &&
               ((elapsed_ms - y_run_time_ms) >=
                Y_TARGET_SETTLE_TIMEOUT_MS))
      {
        Motion_StopAll();
        return 0U;
      }
    }

    HAL_Delay(1U);
  }

  return 1U;
}

static uint8_t XY_RunSegment(const MotionSegment_t *segment)
{
  if (RoutePlan_ValidateSegment(segment) == 0U)
  {
    Motion_StopAll();
    return 0U;
  }

  if ((segment->x_enabled != 0U) &&
      (segment->y_enabled != 0U) &&
      (segment->synchronized != 0U))
  {
    return XY_RunSelectedAxes(segment, 1U, 1U);
  }

  /* Revision: a non-synchronized segment controls one axis, never X then Y. */
  if ((segment->x_enabled != 0U) &&
      (XY_RunSelectedAxes(segment, 1U, 0U) == 0U))
  {
    return 0U;
  }
  else if ((segment->y_enabled != 0U) &&
           (XY_RunSelectedAxes(segment, 0U, 1U) == 0U))
  {
    return 0U;
  }

  return 1U;
}

uint8_t Route_Run(uint8_t from_position, uint8_t to_position)
{
  const RoutePlan_t *route = RoutePlan_Find(from_position, to_position);
  uint8_t segment_index;

  if ((route == 0) || (route->configured == 0U))
  {
    Motion_StopAll();
    return 0U;
  }

  Motion_StopAll();

  for (segment_index = 0U;
       segment_index < route->segment_count;
       segment_index++)
  {
    if (XY_RunSegment(&route->segments[segment_index]) == 0U)
    {
      Motion_StopAll();
      return 0U;
    }
  }

  Motion_StopAll();
  return 1U;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_SPI6_Init();
  MX_TIM2_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART10_UART_Init();
  /* USER CODE BEGIN 2 */
  DM3519_CAN1_Start();
  if (M2006_Axis_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  Motion_StopAll();
  HAL_Delay(100U);

  X_EnablePairedVelocityMode();
  DM3519_RequestMappingRanges(DM3519_X_MOTOR1_ID);
  DM3519_RequestMappingRanges(DM3519_X_MOTOR2_ID);
  HAL_Delay(20U);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }

  if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
      (rx_header.DataLength != FDCAN_DLC_BYTES_8))
  {
    return;
  }

  if (hfdcan->Instance == FDCAN1)
  {
    if (rx_header.Identifier == DM3519_X_MOTOR1_MASTER_ID)
    {
      DM3519_DecodeFeedback(&dm3519_x_motor1,
                            DM3519_X_MOTOR1_ID,
                            rx_data);
    }
    else if (rx_header.Identifier == DM3519_X_MOTOR2_MASTER_ID)
    {
      DM3519_DecodeFeedback(&dm3519_x_motor2,
                            DM3519_X_MOTOR2_ID,
                            rx_data);
    }
  }
  else if ((hfdcan->Instance == FDCAN2) &&
           (rx_header.Identifier == M2006_AXIS_FEEDBACK_ID))
  {
    M2006_Axis_OnFeedback(rx_data, HAL_GetTick());
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
