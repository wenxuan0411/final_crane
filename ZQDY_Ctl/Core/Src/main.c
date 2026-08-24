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
#include <math.h>
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
  float gear_ratio;
  float pmax_rad;
  float vmax_rad_s;
  float tmax_nm;
  uint32_t msg_count;
  uint32_t last_feedback_tick;
  uint8_t position_initialized;
  uint8_t gear_ratio_received;
} DM3519_Feedback_t;

typedef enum
{
  X_SYNC_FAULT_NONE = 0U,
  X_SYNC_FAULT_POSITION_ERROR = 1U,
  X_SYNC_FAULT_FEEDBACK_TIMEOUT = 2U
} X_SyncFault_t;

typedef enum
{
  BEAN_UNKNOWN = 0U,
  BEAN_YELLOW,
  BEAN_GREEN,
  BEAN_WHITE
} BeanType_t;

typedef struct
{
  uint8_t pick_position;
  BeanType_t bean;
  uint8_t target_box_id;
  uint8_t place_position;
} TransportTask_t;

typedef uint8_t (*GripAction_t)(void);

typedef enum
{
  Z_ASYNC_MOVE_IDLE = 0U,
  Z_ASYNC_MOVE_RUNNING,
  Z_ASYNC_MOVE_FAULT
} Z_AsyncMoveState_t;

typedef struct
{
  Z_AsyncMoveState_t state;
  float target_position_mm;
  float speed_rad_s;
  float direction_sign;
  float mm_per_motor_rad;
  uint32_t start_tick;
  uint32_t last_command_tick;
  uint32_t move_timeout_ms;
} Z_AsyncMove_t;

typedef struct
{
  uint8_t rx_byte;
  char line[16];
  volatile uint8_t length;
  volatile uint8_t ready;
  volatile uint8_t overflow;
} VisionUartReceiver_t;

typedef enum
{
  VISION_STATUS_IDLE = 0U,
  VISION_STATUS_WAITING,
  VISION_STATUS_READY,
  VISION_STATUS_TIMEOUT,
  VISION_STATUS_INVALID_DATA,
  VISION_STATUS_UART_ERROR
} VisionStatus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DM3519_X_MOTOR1_ID                 0x01U
#define DM3519_X_MOTOR2_ID                 0x02U
#define DM3519_Z_MOTOR_ID                  0x05U
#define DM3519_X_MOTOR1_MASTER_ID          0x03U
#define DM3519_X_MOTOR2_MASTER_ID          0x04U
#define DM3519_Z_MOTOR_MASTER_ID           0x06U
#define DM3519_VELOCITY_MODE_ID            0x200U
#define DM3519_CONTROL_STD_ID(id)          (DM3519_VELOCITY_MODE_ID + (id))
#define DM3519_PARAM_STD_ID                0x7FFU
#define DM3519_PARAM_READ_CMD              0x33U
#define DM3519_PARAM_GEAR_RATIO_RID        0x14U
#define DM3519_PARAM_PMAX_RID              0x15U
#define DM3519_PARAM_VMAX_RID              0x16U
#define DM3519_PARAM_TMAX_RID              0x17U
#define DM3519_FALLBACK_PMAX_RAD           12.5f
#define DM3519_FALLBACK_VMAX_RAD_S         45.0f
#define DM3519_FALLBACK_TMAX_NM            18.0f
#define DM3519_FALLBACK_GEAR_RATIO          (3591.0f / 187.0f)
#define X_RUN_SPEED_RAD_S                  35.0f
#define X_FEEDBACK_RAD_PER_MM               0.0262274447f
#define X_COMMAND_REFRESH_MS               20U
#define X_ACCEL_TIME_RATIO                 0.25f
#define X_CRUISE_TIME_RATIO                0.50f
#define X_DECEL_TIME_RATIO                 0.25f
#define X_TRAPEZOID_AREA_RATIO             \
  ((0.5f * X_ACCEL_TIME_RATIO) + X_CRUISE_TIME_RATIO + \
   (0.5f * X_DECEL_TIME_RATIO))
#define X_SYNC_KP                          1.0f
#define X_SYNC_MAX_CORRECTION_RAD_S        1.5f
#define X_SYNC_POSITION_FAULT_MM           20.0f
#define X_SYNC_POSITION_FAULT_RAD          \
  (X_SYNC_POSITION_FAULT_MM * X_FEEDBACK_RAD_PER_MM)
#define X_SYNC_FEEDBACK_TIMEOUT_MS         100U
#define X_FEEDBACK_ACQUIRE_TIMEOUT_MS      300U
#define X_STOP_REPEAT_COUNT                3U
#define X_STOP_REPEAT_INTERVAL_MS          2U
#define X_POSITION_MIN_MM                   (-2750.0f)
#define X_POSITION_MAX_MM                   2900.0f
#define Z_OUTPUT_MM_PER_RAD                 26.998f
#define Z_POSITION_MIN_MM                   0.0f
#define Z_POSITION_MAX_MM                   305.0f
#define Z_POSITION_TOLERANCE_MM             0.5f
#define Z_APPROACH_TIME_S                   0.10f
#define Z_TEST_TARGET_POSITION_MM           290.0f
#define Z_TEST_SPEED_RAD_S                  7.0f
#define Z_GRIP_RETRACT_TARGET_MM            285.0f
#define Z_GRIP_01_XY_START_POSITION_MM      255.0f
#define Z_GRIP_02_XY_START_POSITION_MM      205.0f
#define Z_GRIP_03_XY_START_POSITION_MM      285.0f
#define RELEASE_04_08_Z_PREPARE_X_DISTANCE_MM 10.0f
#define RELEASE_05_06_07_Z_PREPARE_X_DISTANCE_MM 500.0f
#define RELEASE_Z_PREPARE_Y_DISTANCE_MM       100.0f
#define RELEASE_Z_TARGET_POSITION_MM         145.0f
#define Z_COMMAND_REFRESH_MS                20U
#define Z_FEEDBACK_TIMEOUT_MS               100U
#define Z_FEEDBACK_ACQUIRE_TIMEOUT_MS       300U
#define Z_MOVE_TIMEOUT_MARGIN_MS            1000U
#define Z_STOP_REPEAT_COUNT                 3U
#define Z_STOP_REPEAT_INTERVAL_MS           2U
#define Y_TARGET_SETTLE_TIMEOUT_MS         3000U
#define Y_FINE_APPROACH_KP_RPM_REV          600.0f
#define Y_HOLD_ENTRY_TOLERANCE_MM           1.0f
#define Y_HOLD_ENTRY_MAX_SPEED_RPM          100.0f
#define START_KEY_PRESSED_STATE             GPIO_PIN_RESET
#define START_KEY_DEBOUNCE_MS               30U
#define TRANSPORT_TASK_COUNT                 3U
#define SECOND_RUN_GREEN_OFFSET_MM           5.0f
#define SECOND_RUN_WHITE_OFFSET_MM           10.0f
#define BEAN_TYPE_COUNT                      3U
#define VISION_COLOR_RESULT_COUNT            3U
#define VISION_DIGIT_RESULT_COUNT            5U
#define VISION_RESULT_TIMEOUT_MS             15000U
#define VISION_COMMAND_INTERVAL_MS           20U
#define VISION_STOP_REPEAT_COUNT              3U
#define HOST_READY_TX_TIMEOUT_MS             100U
#define HOST_READY_TX_INTERVAL_MS            100U
#define RUN_RESULT_READY                      0U
#define RUN_RESULT_SUCCESS                    1U
#define RUN_RESULT_VISION_FAILED              2U
#define RUN_RESULT_MOTION_FAILED              3U
#define EMERGENCY_STOP_REPEAT_COUNT           3U
#define EMERGENCY_STOP_SPIN_COUNT             100000U
#define SERVO_MIN_PULSE_US                   500U
#define SERVO_MAX_PULSE_US                   2500U
#define SERVO_MAX_ANGLE_DEG                  180.0f
#define GRIP_ACTION_INTERVAL_MS              1000U
#define GRIP_ACTION_INTERVAL               1000U
#define GRIP_ACTION_MS              				 500U
#define CRANE_MOTION_ENABLED                  1U
#define PA15_Z_START_ONLY                     0U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static volatile DM3519_Feedback_t dm3519_x_motor1 = {
  .gear_ratio = DM3519_FALLBACK_GEAR_RATIO,
  .pmax_rad = DM3519_FALLBACK_PMAX_RAD,
  .vmax_rad_s = DM3519_FALLBACK_VMAX_RAD_S,
  .tmax_nm = DM3519_FALLBACK_TMAX_NM
};
static volatile DM3519_Feedback_t dm3519_x_motor2 = {
  .gear_ratio = DM3519_FALLBACK_GEAR_RATIO,
  .pmax_rad = DM3519_FALLBACK_PMAX_RAD,
  .vmax_rad_s = DM3519_FALLBACK_VMAX_RAD_S,
  .tmax_nm = DM3519_FALLBACK_TMAX_NM
};
static volatile DM3519_Feedback_t dm3519_z_motor = {
  .gear_ratio = DM3519_FALLBACK_GEAR_RATIO,
  .pmax_rad = DM3519_FALLBACK_PMAX_RAD,
  .vmax_rad_s = DM3519_FALLBACK_VMAX_RAD_S,
  .tmax_nm = DM3519_FALLBACK_TMAX_NM
};
static volatile float z_position_zero_rad = 0.0f;
static volatile uint8_t z_position_reference_ready = 0U;
static volatile float x_motor1_position_zero_rad = 0.0f;
static volatile float x_motor2_position_zero_rad = 0.0f;
static volatile X_SyncFault_t x_sync_fault = X_SYNC_FAULT_NONE;
static uint8_t x_position_reference_ready = 0U;
static uint8_t gripper_servo_pwm_started = 0U;
static uint8_t rotation_servo_pwm_started = 0U;
static Z_AsyncMove_t z_async_move = {
  .state = Z_ASYNC_MOVE_IDLE
};
static uint8_t release_z_prepare_armed = 0U;
static uint8_t release_z_prepare_started = 0U;
static const MotionGroup_t *release_z_prepare_group = 0;
static float release_z_prepare_x_distance_mm = 0.0f;
static uint8_t release_z_prepare_use_y_position = 0U;
static float release_z_prepare_y_target_position_mm = 0.0f;
static VisionUartReceiver_t color_vision_receiver = {0};
static VisionUartReceiver_t digit_vision_receiver = {0};
  static volatile uint8_t emergency_stop_active = 0U;
static float grip_depth_offset_mm = 0.0f;
volatile float y_calibration_position_rev = 0.0f;
volatile uint8_t y_calibration_position_valid = 0U;
volatile uint8_t xy_test_result = 0U;
static uint32_t host_ready_last_tx_tick = 0U;
static uint8_t host_ready_sent = 0U;
volatile VisionStatus_t vision_status = VISION_STATUS_IDLE;
volatile uint8_t color_vision_result[VISION_COLOR_RESULT_COUNT] = {0};
volatile uint8_t digit_vision_result[VISION_DIGIT_RESULT_COUNT] = {0};
volatile uint8_t z_feedback_valid = 0U;
volatile uint8_t z_feedback_error = 0U;
volatile uint8_t z_feedback_gear_ratio_valid = 0U;
volatile uint32_t z_feedback_message_count = 0U;
volatile float z_feedback_position_rad = 0.0f;
volatile float z_feedback_unwrapped_rad = 0.0f;
volatile float z_feedback_delta_rad = 0.0f;
volatile float z_feedback_velocity_rad_s = 0.0f;
volatile float z_feedback_torque_nm = 0.0f;
volatile float z_feedback_position_mm = 0.0f;
volatile float z_feedback_gear_ratio = DM3519_FALLBACK_GEAR_RATIO;
static TransportTask_t transport_task_table[TRANSPORT_TASK_COUNT] = {0};
static const uint8_t transport_task_order[TRANSPORT_TASK_COUNT] =
{
  2U, 0U, 1U
};
static const GripAction_t
grip_action_table[TRANSPORT_TASK_COUNT][BEAN_TYPE_COUNT] =
{
  {GRIP_01_YELLOW, GRIP_01_GREEN, GRIP_01_WHITE},
  {GRIP_02_YELLOW, GRIP_02_GREEN, GRIP_02_WHITE},
  {GRIP_03_YELLOW, GRIP_03_GREEN, GRIP_03_WHITE}
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint8_t X_TargetMmToFeedbackRad(float target_position_mm,
                                       float *target_position_rad);
static uint8_t X_HasReachedTrigger(uint32_t now,
                                   float trigger_position_mm,
                                   float direction_velocity_rad_s,
                                   uint8_t *reached);
static uint8_t TransportPlan_Run(void);
static uint8_t Z_AsyncMove_Update(uint32_t now);
static uint8_t Vision_RecognizeAndBuildTransportPlan(void);
static void ReleaseZ_Disarm(void);
static uint8_t ReleaseZ_ArmForRoute(uint8_t from_position,
                                    uint8_t to_position);
static uint8_t ReleaseZ_UpdateForGroup(const MotionGroup_t *group,
                                       uint32_t now);
static uint8_t ReleaseZ_EnsureAtReleaseHeight(void);
static void Crane_RunOnce(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void GripperServo_SetAngle(float angle_deg)
{
  uint32_t pulse_us;

  if (angle_deg < 0.0f)
  {
    angle_deg = 0.0f;
  }
  else if (angle_deg > SERVO_MAX_ANGLE_DEG)
  {
    angle_deg = SERVO_MAX_ANGLE_DEG;
  }

  pulse_us = SERVO_MIN_PULSE_US +
             (uint32_t)((angle_deg *
                         (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) /
                         SERVO_MAX_ANGLE_DEG) + 0.5f);
  __HAL_TIM_SET_COMPARE(&GRIPPER_SERVO_TIM,
                        GRIPPER_SERVO_CHANNEL,
                        pulse_us);

  if (gripper_servo_pwm_started == 0U)
  {
    if (HAL_TIM_PWM_Start(&GRIPPER_SERVO_TIM,
                          GRIPPER_SERVO_CHANNEL) != HAL_OK)
    {
      Error_Handler();
    }
    gripper_servo_pwm_started = 1U;
  }
}

static void RotationServo_SetAngle(float angle_deg)
{
  uint32_t pulse_us;

  if (angle_deg < 0.0f)
  {
    angle_deg = 0.0f;
  }
  else if (angle_deg > SERVO_MAX_ANGLE_DEG)
  {
    angle_deg = SERVO_MAX_ANGLE_DEG;
  }

  pulse_us = SERVO_MIN_PULSE_US +
             (uint32_t)((angle_deg *
                         (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) /
                         SERVO_MAX_ANGLE_DEG) + 0.5f);
  __HAL_TIM_SET_COMPARE(&ROTATION_SERVO_TIM,
                        ROTATION_SERVO_CHANNEL,
                        pulse_us);

  if (rotation_servo_pwm_started == 0U)
  {
    if (HAL_TIM_PWM_Start(&ROTATION_SERVO_TIM,
                          ROTATION_SERVO_CHANNEL) != HAL_OK)
    {
      Error_Handler();
    }
    rotation_servo_pwm_started = 1U;
  }
}

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

static HAL_StatusTypeDef FDCAN_TrySendStandardFrame(
    FDCAN_HandleTypeDef *hfdcan,
    uint16_t std_id,
    const uint8_t *data,
    uint8_t len)
{
  FDCAN_TxHeaderTypeDef tx_header = {0};

  if ((hfdcan == 0) ||
      (data == 0) ||
      (hfdcan->State != HAL_FDCAN_STATE_BUSY))
  {
    return HAL_ERROR;
  }

  tx_header.Identifier = std_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_FromLength(len);
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0U;

  return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data);
}

static void FDCAN_SendStandardFrame(FDCAN_HandleTypeDef *hfdcan,
                                    uint16_t std_id,
                                    uint8_t *data,
                                    uint8_t len)
{
  if (FDCAN_TrySendStandardFrame(hfdcan,
                                 std_id,
                                 data,
                                 len) != HAL_OK)
  {
    Error_Handler();
  }
}

void Crane_EmergencyStop(void)
{
  static const uint8_t zero_velocity[4] = {0U, 0U, 0U, 0U};
  static const uint8_t zero_current[8] =
      {0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
  uint32_t repeat;

  if (emergency_stop_active != 0U)
  {
    return;
  }
  emergency_stop_active = 1U;
  z_async_move.state = Z_ASYNC_MOVE_IDLE;

  for (repeat = 0U;
       repeat < EMERGENCY_STOP_REPEAT_COUNT;
       repeat++)
  {
    volatile uint32_t spin;

    (void)FDCAN_TrySendStandardFrame(
        &hfdcan1,
        DM3519_CONTROL_STD_ID(DM3519_X_MOTOR1_ID),
        zero_velocity,
        sizeof(zero_velocity));
    (void)FDCAN_TrySendStandardFrame(
        &hfdcan1,
        DM3519_CONTROL_STD_ID(DM3519_X_MOTOR2_ID),
        zero_velocity,
        sizeof(zero_velocity));
    (void)FDCAN_TrySendStandardFrame(
        &hfdcan1,
        DM3519_CONTROL_STD_ID(DM3519_Z_MOTOR_ID),
        zero_velocity,
        sizeof(zero_velocity));
    (void)FDCAN_TrySendStandardFrame(
        &hfdcan2,
        M2006_AXIS_CONTROL_ID,
        zero_current,
        sizeof(zero_current));

    for (spin = 0U;
         spin < EMERGENCY_STOP_SPIN_COUNT;
         spin++)
    {
      __NOP();
    }
  }
}

static void DM3519_EnableVelocityMode(FDCAN_HandleTypeDef *hfdcan,
                                      uint8_t motor_id)
{
  uint8_t data[8] = {
    0xFFU, 0xFFU, 0xFFU, 0xFFU,
    0xFFU, 0xFFU, 0xFFU, 0xFCU
  };

  FDCAN_SendStandardFrame(hfdcan,
                          DM3519_CONTROL_STD_ID(motor_id),
                          data,
                          8U);
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

  filter.FilterIndex = 2U;
  filter.FilterID1 = DM3519_Z_MOTOR_MASTER_ID;
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
  DM3519_RequestMappingRange(motor_id, DM3519_PARAM_GEAR_RATIO_RID);
  HAL_Delay(2U);
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

  if (data[3] == DM3519_PARAM_GEAR_RATIO_RID)
  {
    motor->gear_ratio = range.value;
    motor->gear_ratio_received = 1U;
  }
  else if (data[3] == DM3519_PARAM_PMAX_RID)
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

static void Z_SetVelocity(float velocity_rad_s)
{
  DM3519_SetVelocity(&hfdcan1, DM3519_Z_MOTOR_ID, velocity_rad_s);
}

static void Z_EnableVelocityMode(void)
{
  DM3519_EnableVelocityMode(&hfdcan1, DM3519_Z_MOTOR_ID);
  Z_SetVelocity(0.0f);
  HAL_Delay(5U);
  Z_SetVelocity(0.0f);
}

static void Z_Stop(void)
{
  uint32_t repeat;

  for (repeat = 0U; repeat < Z_STOP_REPEAT_COUNT; repeat++)
  {
    Z_SetVelocity(0.0f);
    if ((repeat + 1U) < Z_STOP_REPEAT_COUNT)
    {
      HAL_Delay(Z_STOP_REPEAT_INTERVAL_MS);
    }
  }
}

static uint8_t Z_AcquireFreshFeedback(void)
{
  uint32_t start_tick;
  uint32_t last_command_tick;
  uint32_t initial_msg_count;

  initial_msg_count = dm3519_z_motor.msg_count;
  start_tick = HAL_GetTick();
  last_command_tick = start_tick - Z_COMMAND_REFRESH_MS;

  while ((HAL_GetTick() - start_tick) < Z_FEEDBACK_ACQUIRE_TIMEOUT_MS)
  {
    uint32_t now = HAL_GetTick();

    if ((now - last_command_tick) >= Z_COMMAND_REFRESH_MS)
    {
      last_command_tick = now;
      Z_SetVelocity(0.0f);
    }

    if ((dm3519_z_motor.msg_count != initial_msg_count) &&
        (dm3519_z_motor.position_initialized != 0U) &&
        ((now - dm3519_z_motor.last_feedback_tick) <=
         Z_FEEDBACK_TIMEOUT_MS))
    {
      return 1U;
    }

    M2006_Axis_Update(now);
    HAL_Delay(1U);
  }

  Z_Stop();
  return 0U;
}

static uint8_t Z_EstablishPositionReference(void)
{
  uint32_t primask;

  if (Z_AcquireFreshFeedback() == 0U)
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  z_position_zero_rad = dm3519_z_motor.position_unwrapped_rad;
  z_position_reference_ready = 1U;
  if (primask == 0U)
  {
    __enable_irq();
  }

  return 1U;
}

static uint8_t Z_GetPositionMm(float *position_mm)
{
  float wrapped_position_rad;
  float unwrapped_position_rad;
  float velocity_rad_s;
  float torque_nm;
  float gear_ratio;
  float zero_rad;
  uint32_t feedback_tick;
  uint32_t message_count;
  uint8_t error;
  uint8_t position_initialized;
  uint8_t reference_ready;
  uint8_t gear_ratio_received;
  uint32_t now;
  uint32_t primask;

  if (position_mm == 0)
  {
    return 0U;
  }

  now = HAL_GetTick();
  primask = __get_PRIMASK();
  __disable_irq();
  wrapped_position_rad = dm3519_z_motor.position_rad;
  unwrapped_position_rad = dm3519_z_motor.position_unwrapped_rad;
  velocity_rad_s = dm3519_z_motor.velocity_rad_s;
  torque_nm = dm3519_z_motor.torque_nm;
  gear_ratio = dm3519_z_motor.gear_ratio;
  zero_rad = z_position_zero_rad;
  feedback_tick = dm3519_z_motor.last_feedback_tick;
  message_count = dm3519_z_motor.msg_count;
  error = dm3519_z_motor.error;
  position_initialized = dm3519_z_motor.position_initialized;
  reference_ready = z_position_reference_ready;
  gear_ratio_received = dm3519_z_motor.gear_ratio_received;
  if (primask == 0U)
  {
    __enable_irq();
  }

  z_feedback_error = error;
  z_feedback_message_count = message_count;
  z_feedback_position_rad = wrapped_position_rad;
  z_feedback_unwrapped_rad = unwrapped_position_rad;
  z_feedback_delta_rad = unwrapped_position_rad - zero_rad;
  z_feedback_velocity_rad_s = velocity_rad_s;
  z_feedback_torque_nm = torque_nm;
  z_feedback_gear_ratio = gear_ratio;
  z_feedback_gear_ratio_valid = gear_ratio_received;

  if ((reference_ready == 0U) ||
      (position_initialized == 0U) ||
      ((now - feedback_tick) > Z_FEEDBACK_TIMEOUT_MS))
  {
    z_feedback_valid = 0U;
    return 0U;
  }

  *position_mm = (z_feedback_delta_rad / gear_ratio) *
                 Z_OUTPUT_MM_PER_RAD;
  z_feedback_position_mm = *position_mm;
  z_feedback_valid = 1U;
  return 1U;
}

static void Z_AsyncMove_Cancel(void)
{
  z_async_move.state = Z_ASYNC_MOVE_IDLE;
  Z_Stop();
}

static uint8_t Z_AsyncMove_Update(uint32_t now)
{
  float current_position_mm;
  float remaining_mm;
  float approach_speed_rad_s;

  if (z_async_move.state == Z_ASYNC_MOVE_IDLE)
  {
    return 1U;
  }
  if (z_async_move.state == Z_ASYNC_MOVE_FAULT)
  {
    return 0U;
  }

  if (((now - z_async_move.start_tick) >=
       z_async_move.move_timeout_ms) ||
      (Z_GetPositionMm(&current_position_mm) == 0U) ||
      (current_position_mm < Z_POSITION_MIN_MM) ||
      (current_position_mm > Z_POSITION_MAX_MM))
  {
    Z_Stop();
    z_async_move.state = Z_ASYNC_MOVE_FAULT;
    return 0U;
  }

  if (((z_async_move.direction_sign > 0.0f) &&
       (current_position_mm >=
        (z_async_move.target_position_mm - Z_POSITION_TOLERANCE_MM))) ||
      ((z_async_move.direction_sign < 0.0f) &&
       (current_position_mm <=
        (z_async_move.target_position_mm + Z_POSITION_TOLERANCE_MM))))
  {
    Z_Stop();
    z_async_move.state = Z_ASYNC_MOVE_IDLE;
    return 1U;
  }

  if ((now - z_async_move.last_command_tick) >= Z_COMMAND_REFRESH_MS)
  {
    z_async_move.last_command_tick = now;
    remaining_mm = z_async_move.target_position_mm - current_position_mm;
    if (remaining_mm < 0.0f)
    {
      remaining_mm = -remaining_mm;
    }
    approach_speed_rad_s =
        remaining_mm /
        (z_async_move.mm_per_motor_rad * Z_APPROACH_TIME_S);
    if (approach_speed_rad_s > z_async_move.speed_rad_s)
    {
      approach_speed_rad_s = z_async_move.speed_rad_s;
    }
    Z_SetVelocity(z_async_move.direction_sign * approach_speed_rad_s);
  }

  return 1U;
}

static uint8_t Z_AsyncMove_Start(float target_position_mm,
                                 float speed_rad_s)
{
  float current_position_mm;
  float distance_mm;
  float expected_time_ms;
  uint32_t now;

  if ((target_position_mm != target_position_mm) ||
      (speed_rad_s != speed_rad_s) ||
      (target_position_mm < Z_POSITION_MIN_MM) ||
      (target_position_mm > Z_POSITION_MAX_MM) ||
      (speed_rad_s <= 0.0f) ||
      (speed_rad_s > dm3519_z_motor.vmax_rad_s) ||
      (z_position_reference_ready == 0U) ||
      (z_async_move.state == Z_ASYNC_MOVE_RUNNING))
  {
    Z_AsyncMove_Cancel();
    return 0U;
  }

  if ((Z_AcquireFreshFeedback() == 0U) ||
      (Z_GetPositionMm(&current_position_mm) == 0U) ||
      (z_feedback_gear_ratio <= 0.0f) ||
      (current_position_mm < Z_POSITION_MIN_MM) ||
      (current_position_mm > Z_POSITION_MAX_MM))
  {
    Z_AsyncMove_Cancel();
    return 0U;
  }

  distance_mm = target_position_mm - current_position_mm;
  if ((distance_mm <= Z_POSITION_TOLERANCE_MM) &&
      (distance_mm >= -Z_POSITION_TOLERANCE_MM))
  {
    Z_AsyncMove_Cancel();
    return 1U;
  }

  if (distance_mm > 0.0f)
  {
    z_async_move.direction_sign = 1.0f;
  }
  else
  {
    z_async_move.direction_sign = -1.0f;
    distance_mm = -distance_mm;
  }

  z_async_move.mm_per_motor_rad =
      Z_OUTPUT_MM_PER_RAD / z_feedback_gear_ratio;
  expected_time_ms =
      distance_mm * 1000.0f /
      (speed_rad_s * z_async_move.mm_per_motor_rad);
  if (expected_time_ms > 4294966000.0f)
  {
    Z_AsyncMove_Cancel();
    return 0U;
  }

  now = HAL_GetTick();
  z_async_move.target_position_mm = target_position_mm;
  z_async_move.speed_rad_s = speed_rad_s;
  z_async_move.start_tick = now;
  z_async_move.last_command_tick = now - Z_COMMAND_REFRESH_MS;
  z_async_move.move_timeout_ms =
      (uint32_t)(expected_time_ms + 0.5f) +
      Z_MOVE_TIMEOUT_MARGIN_MS;
  z_async_move.state = Z_ASYNC_MOVE_RUNNING;

  return Z_AsyncMove_Update(now);
}

static uint8_t Z_AsyncMove_Wait(void)
{
  while (z_async_move.state == Z_ASYNC_MOVE_RUNNING)
  {
    uint32_t now = HAL_GetTick();

    if (Z_AsyncMove_Update(now) == 0U)
    {
      return 0U;
    }
    M2006_Axis_Update(now);
    HAL_Delay(1U);
  }

  return (z_async_move.state == Z_ASYNC_MOVE_FAULT) ? 0U : 1U;
}

static void ReleaseZ_Disarm(void)
{
  release_z_prepare_armed = 0U;
  release_z_prepare_started = 0U;
  release_z_prepare_group = 0;
  release_z_prepare_x_distance_mm = 0.0f;
  release_z_prepare_use_y_position = 0U;
  release_z_prepare_y_target_position_mm = 0.0f;
}

static uint8_t ReleaseZ_ArmForRoute(uint8_t from_position,
                                    uint8_t to_position)
{
  const RoutePlan_t *route = RoutePlan_Find(from_position, to_position);
  uint8_t group_index;

  ReleaseZ_Disarm();
  if ((route == 0) ||
      (route->configured == 0U) ||
      (route->group_count == 0U))
  {
    return 0U;
  }

  group_index = route->group_count;
  while (group_index > 0U)
  {
    group_index--;
    if (route->groups[group_index].x_enabled != 0U)
    {
      release_z_prepare_group = &route->groups[group_index];
      if (((from_position == 1U) && (to_position == 4U)) ||
          ((from_position == 2U) && (to_position == 8U)))
      {
        if ((release_z_prepare_group->y_segments == 0) ||
            (release_z_prepare_group->y_segment_count == 0U))
        {
          ReleaseZ_Disarm();
          return 0U;
        }
        release_z_prepare_use_y_position = 1U;
        release_z_prepare_y_target_position_mm =
            release_z_prepare_group->y_segments[
                release_z_prepare_group->y_segment_count - 1U]
                .y_target_position_mm;
      }
      else
      {
        release_z_prepare_x_distance_mm =
            ((to_position == 4U) || (to_position == 8U)) ?
            RELEASE_04_08_Z_PREPARE_X_DISTANCE_MM :
            RELEASE_05_06_07_Z_PREPARE_X_DISTANCE_MM;
      }
      release_z_prepare_armed = 1U;
      return 1U;
    }
  }

  return 0U;
}

static uint8_t ReleaseZ_EnsureAtReleaseHeight(void)
{
  if (release_z_prepare_armed == 0U)
  {
    return Z_move(RELEASE_Z_TARGET_POSITION_MM, Z_TEST_SPEED_RAD_S);
  }

  if (release_z_prepare_started == 0U)
  {
    if ((Z_AsyncMove_Wait() == 0U) ||
        (Z_AsyncMove_Start(RELEASE_Z_TARGET_POSITION_MM,
                           Z_TEST_SPEED_RAD_S) == 0U))
    {
      ReleaseZ_Disarm();
      return 0U;
    }
    release_z_prepare_started = 1U;
  }

  if (Z_AsyncMove_Wait() == 0U)
  {
    ReleaseZ_Disarm();
    return 0U;
  }

  ReleaseZ_Disarm();
  return 1U;
}

static uint8_t Z_RetractForXYMotion(float xy_start_position_mm)
{
  float current_position_mm;

  if ((xy_start_position_mm < Z_POSITION_MIN_MM) ||
      (xy_start_position_mm > Z_GRIP_RETRACT_TARGET_MM))
  {
    return 0U;
  }

  if (Z_AsyncMove_Start(Z_GRIP_RETRACT_TARGET_MM,
                        Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }

  while (z_async_move.state == Z_ASYNC_MOVE_RUNNING)
  {
    uint32_t now = HAL_GetTick();

    if ((Z_AsyncMove_Update(now) == 0U) ||
        (Z_GetPositionMm(&current_position_mm) == 0U))
    {
      Z_AsyncMove_Cancel();
      return 0U;
    }
    if (current_position_mm >= xy_start_position_mm)
    {
      return 1U;
    }

    M2006_Axis_Update(now);
    HAL_Delay(1U);
  }

  return (z_async_move.state == Z_ASYNC_MOVE_FAULT) ? 0U : 1U;
}

uint8_t Z_move(float target_position_mm, float speed_rad_s)
{
  float current_position_mm;
  float distance_mm;
  float direction_sign;
  float mm_per_motor_rad;
  float expected_time_ms;
  uint32_t move_timeout_ms;
  uint32_t start_tick;
  uint32_t last_command_tick;

  if (Z_AsyncMove_Wait() == 0U)
  {
    return 0U;
  }

  if ((target_position_mm != target_position_mm) ||
      (speed_rad_s != speed_rad_s) ||
      (target_position_mm < Z_POSITION_MIN_MM) ||
      (target_position_mm > Z_POSITION_MAX_MM) ||
      (speed_rad_s <= 0.0f) ||
      (speed_rad_s > dm3519_z_motor.vmax_rad_s) ||
      (z_position_reference_ready == 0U))
  {
    Z_Stop();
    return 0U;
  }

  if ((Z_AcquireFreshFeedback() == 0U) ||
      (Z_GetPositionMm(&current_position_mm) == 0U) ||
      (z_feedback_gear_ratio <= 0.0f) ||
      (current_position_mm < Z_POSITION_MIN_MM) ||
      (current_position_mm > Z_POSITION_MAX_MM))
  {
    Z_Stop();
    return 0U;
  }

  distance_mm = target_position_mm - current_position_mm;
  if ((distance_mm <= Z_POSITION_TOLERANCE_MM) &&
      (distance_mm >= -Z_POSITION_TOLERANCE_MM))
  {
    Z_Stop();
    return 1U;
  }

  if (distance_mm > 0.0f)
  {
    direction_sign = 1.0f;
  }
  else
  {
    direction_sign = -1.0f;
    distance_mm = -distance_mm;
  }

  mm_per_motor_rad = Z_OUTPUT_MM_PER_RAD / z_feedback_gear_ratio;
  expected_time_ms = distance_mm * 1000.0f /
                     (speed_rad_s * mm_per_motor_rad);
  if (expected_time_ms > 4294966000.0f)
  {
    Z_Stop();
    return 0U;
  }
  move_timeout_ms = (uint32_t)(expected_time_ms + 0.5f) +
                    Z_MOVE_TIMEOUT_MARGIN_MS;

  start_tick = HAL_GetTick();
  last_command_tick = start_tick - Z_COMMAND_REFRESH_MS;
  while ((HAL_GetTick() - start_tick) < move_timeout_ms)
  {
    uint32_t now = HAL_GetTick();

    if ((Z_GetPositionMm(&current_position_mm) == 0U) ||
        (current_position_mm < Z_POSITION_MIN_MM) ||
        (current_position_mm > Z_POSITION_MAX_MM))
    {
      Z_Stop();
      return 0U;
    }

    if (((direction_sign > 0.0f) &&
         (current_position_mm >=
          (target_position_mm - Z_POSITION_TOLERANCE_MM))) ||
        ((direction_sign < 0.0f) &&
         (current_position_mm <=
          (target_position_mm + Z_POSITION_TOLERANCE_MM))))
    {
      Z_Stop();
      return 1U;
    }

    if ((now - last_command_tick) >= Z_COMMAND_REFRESH_MS)
    {
      float remaining_mm = target_position_mm - current_position_mm;
      float approach_speed_rad_s;

      last_command_tick = now;
      if (remaining_mm < 0.0f)
      {
        remaining_mm = -remaining_mm;
      }
      approach_speed_rad_s = remaining_mm /
                             (mm_per_motor_rad * Z_APPROACH_TIME_S);
      if (approach_speed_rad_s > speed_rad_s)
      {
        approach_speed_rad_s = speed_rad_s;
      }
      Z_SetVelocity(direction_sign * approach_speed_rad_s);
    }

    M2006_Axis_Update(now);
    HAL_Delay(1U);
  }

  Z_Stop();
  return 0U;
}

uint8_t Z_START(void)
{

  if (Z_move(245.0f, Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }

  RotationServo_SetAngle(ROTATION_SERVO_START_ANGLE);
  GripperServo_SetAngle(GRIPPER_SERVO_OPEN_ANGLE);
  return 1U;
}

uint8_t GRIP_02_YELLOW(void)
{
  if (Z_move(79.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL);

  return Z_RetractForXYMotion(Z_GRIP_02_XY_START_POSITION_MM);
}

uint8_t GRIP_02_GREEN(void)
{
  if (Z_move(80.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL);

  return Z_RetractForXYMotion(Z_GRIP_02_XY_START_POSITION_MM);
}

uint8_t GRIP_02_WHITE(void)
{
  if (Z_move(80.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL);

  return Z_RetractForXYMotion(Z_GRIP_02_XY_START_POSITION_MM);
}

uint8_t GRIP_01_YELLOW(void)
{
  if (Z_move(129.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  return Z_RetractForXYMotion(Z_GRIP_01_XY_START_POSITION_MM);
}

uint8_t GRIP_01_GREEN(void)
{
  if (Z_move(130.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL);

  return Z_RetractForXYMotion(Z_GRIP_01_XY_START_POSITION_MM);
}

uint8_t GRIP_01_WHITE(void)
{	
  if (Z_move(130.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL);

  return Z_RetractForXYMotion(Z_GRIP_01_XY_START_POSITION_MM);
}

uint8_t GRIP_03_YELLOW(void)
{
  if (Z_move(179.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL);

  return Z_RetractForXYMotion(Z_GRIP_03_XY_START_POSITION_MM);
}

uint8_t GRIP_03_GREEN(void)
{
  if (Z_move(180.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL);

  return Z_RetractForXYMotion(Z_GRIP_03_XY_START_POSITION_MM);
}
	
uint8_t GRIP_03_WHITE(void)
{
  if (Z_move(185.0f - grip_depth_offset_mm,
             Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_INTERVAL_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  HAL_Delay(GRIP_ACTION_INTERVAL);

  return Z_RetractForXYMotion(Z_GRIP_03_XY_START_POSITION_MM);
}

uint8_t RELEASE_04_08(void)
{
  RotationServo_SetAngle(ROTATION_SERVO_END_ANGLE);
	HAL_Delay(GRIP_ACTION_INTERVAL_MS);
  if (ReleaseZ_EnsureAtReleaseHeight() == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_OPEN_ANGLE);
  HAL_Delay(GRIP_ACTION_MS);

  RotationServo_SetAngle(ROTATION_SERVO_START_ANGLE);
  return Z_move(285.0f, Z_TEST_SPEED_RAD_S);
}

uint8_t RELEASE_05_06_07(void)
{
  if (ReleaseZ_EnsureAtReleaseHeight() == 0U)
  {
    return 0U;
  }
  HAL_Delay(GRIP_ACTION_MS);

  GripperServo_SetAngle(GRIPPER_SERVO_OPEN_ANGLE);
  HAL_Delay(GRIP_ACTION_MS);

  return Z_AsyncMove_Start(285.0f, Z_TEST_SPEED_RAD_S);
}

static float X_ClampVelocity(float velocity_rad_s,
                             float base_velocity_rad_s,
                             float velocity_limit_rad_s)
{
  if (base_velocity_rad_s > 0.0f)
  {
    if (velocity_rad_s < 0.0f)
    {
      return 0.0f;
    }
    if (velocity_rad_s > velocity_limit_rad_s)
    {
      return velocity_limit_rad_s;
    }
  }
  else if (base_velocity_rad_s < 0.0f)
  {
    if (velocity_rad_s > 0.0f)
    {
      return 0.0f;
    }
    if (velocity_rad_s < -velocity_limit_rad_s)
    {
      return -velocity_limit_rad_s;
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

static uint8_t X_AcquireFreshFeedback(void)
{
  uint32_t start_tick;
  uint32_t last_command_tick;
  uint32_t initial_motor1_msg_count;
  uint32_t initial_motor2_msg_count;
  uint32_t motor1_msg_count;
  uint32_t motor2_msg_count;
  uint32_t motor1_feedback_tick;
  uint32_t motor2_feedback_tick;
  uint8_t motor1_initialized;
  uint8_t motor2_initialized;
  uint32_t primask;

  if ((x_sync_fault != X_SYNC_FAULT_NONE) &&
      (x_sync_fault != X_SYNC_FAULT_FEEDBACK_TIMEOUT))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  initial_motor1_msg_count = dm3519_x_motor1.msg_count;
  initial_motor2_msg_count = dm3519_x_motor2.msg_count;
  if (primask == 0U)
  {
    __enable_irq();
  }

  start_tick = HAL_GetTick();
  last_command_tick = start_tick - X_COMMAND_REFRESH_MS;
  while ((HAL_GetTick() - start_tick) < X_FEEDBACK_ACQUIRE_TIMEOUT_MS)
  {
    uint32_t now = HAL_GetTick();

    if ((now - last_command_tick) >= X_COMMAND_REFRESH_MS)
    {
      last_command_tick = now;
      X_SetMotorVelocities(0.0f, 0.0f);
    }
    if (Z_AsyncMove_Update(now) == 0U)
    {
      return 0U;
    }
    M2006_Axis_Update(now);

    primask = __get_PRIMASK();
    __disable_irq();
    motor1_msg_count = dm3519_x_motor1.msg_count;
    motor2_msg_count = dm3519_x_motor2.msg_count;
    motor1_feedback_tick = dm3519_x_motor1.last_feedback_tick;
    motor2_feedback_tick = dm3519_x_motor2.last_feedback_tick;
    motor1_initialized = dm3519_x_motor1.position_initialized;
    motor2_initialized = dm3519_x_motor2.position_initialized;
    if (primask == 0U)
    {
      __enable_irq();
    }

    if ((motor1_msg_count != initial_motor1_msg_count) &&
        (motor2_msg_count != initial_motor2_msg_count) &&
        (motor1_initialized != 0U) &&
        (motor2_initialized != 0U) &&
        ((now - motor1_feedback_tick) <= X_SYNC_FEEDBACK_TIMEOUT_MS) &&
        ((now - motor2_feedback_tick) <= X_SYNC_FEEDBACK_TIMEOUT_MS))
    {
      if (x_position_reference_ready == 0U)
      {
        primask = __get_PRIMASK();
        __disable_irq();
        x_motor1_position_zero_rad = dm3519_x_motor1.position_unwrapped_rad;
        x_motor2_position_zero_rad = dm3519_x_motor2.position_unwrapped_rad;
        if (primask == 0U)
        {
          __enable_irq();
        }
        x_position_reference_ready = 1U;
      }

      x_sync_fault = X_SYNC_FAULT_NONE;
      return 1U;
    }

    HAL_Delay(1U);
  }

  X_LatchSyncFault(X_SYNC_FAULT_FEEDBACK_TIMEOUT);
  return 0U;
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
  float motor1_gear_ratio;
  float motor2_gear_ratio;
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
  motor1_gear_ratio = dm3519_x_motor1.gear_ratio;
  motor2_gear_ratio = dm3519_x_motor2.gear_ratio;
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
      (motor1_gear_ratio <= 0.0f) ||
      (motor2_gear_ratio <= 0.0f) ||
      ((now - motor1_feedback_tick) > X_SYNC_FEEDBACK_TIMEOUT_MS) ||
      ((now - motor2_feedback_tick) > X_SYNC_FEEDBACK_TIMEOUT_MS))
  {
    X_LatchSyncFault(X_SYNC_FAULT_FEEDBACK_TIMEOUT);
    return 0U;
  }

  *motor1_position_rad /= motor1_gear_ratio;
  *motor2_position_rad /= motor2_gear_ratio;
  position_error_rad = *motor1_position_rad - *motor2_position_rad;
  if ((position_error_rad > X_SYNC_POSITION_FAULT_RAD) ||
      (position_error_rad < -X_SYNC_POSITION_FAULT_RAD))
  {
    X_LatchSyncFault(X_SYNC_FAULT_POSITION_ERROR);
    return 0U;
  }

  return 1U;
}

static void X_SetSynchronizedVelocityLimited(float base_velocity_rad_s,
                                             float velocity_limit_rad_s,
                                             uint32_t now)
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
  correction_rad_s = X_SYNC_KP * position_error_rad;
  correction_rad_s = X_ClampSignedVelocity(correction_rad_s,
                                            X_SYNC_MAX_CORRECTION_RAD_S);

  motor1_velocity_rad_s = X_ClampVelocity(base_velocity_rad_s -
                                           correction_rad_s,
                                           base_velocity_rad_s,
                                           velocity_limit_rad_s);
  motor2_velocity_rad_s = X_ClampVelocity(base_velocity_rad_s +
                                           correction_rad_s,
                                           base_velocity_rad_s,
                                           velocity_limit_rad_s);
  X_SetMotorVelocities(motor1_velocity_rad_s, -motor2_velocity_rad_s);
}

static void X_SetSynchronizedVelocity(float base_velocity_rad_s, uint32_t now)
{
  X_SetSynchronizedVelocityLimited(base_velocity_rad_s,
                                   X_RUN_SPEED_RAD_S,
                                   now);
}

static float X_GetTrapezoidVelocity(float peak_velocity_rad_s,
                                    uint32_t elapsed_ms,
                                    uint32_t total_time_ms)
{
  float time_ratio;

  if ((total_time_ms == 0U) || (elapsed_ms >= total_time_ms))
  {
    return 0.0f;
  }

  time_ratio = (float)elapsed_ms / (float)total_time_ms;
  if (time_ratio < X_ACCEL_TIME_RATIO)
  {
    return peak_velocity_rad_s * time_ratio / X_ACCEL_TIME_RATIO;
  }
  if (time_ratio < (X_ACCEL_TIME_RATIO + X_CRUISE_TIME_RATIO))
  {
    return peak_velocity_rad_s;
  }

  return peak_velocity_rad_s * (1.0f - time_ratio) /
         X_DECEL_TIME_RATIO;
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
                  (motor1_position_rad + motor2_position_rad);
  return 1U;
}

static uint8_t ReleaseZ_UpdateForGroup(const MotionGroup_t *group,
                                       uint32_t now)
{
  float x_position_rad;
  float x_position_mm;
  float y_position_rev;
  float y_position_mm;
  float remaining_mm;

  if ((release_z_prepare_armed == 0U) ||
      (release_z_prepare_started != 0U) ||
      (group != release_z_prepare_group))
  {
    return 1U;
  }

  if (z_async_move.state == Z_ASYNC_MOVE_FAULT)
  {
    return 0U;
  }
  if (z_async_move.state == Z_ASYNC_MOVE_RUNNING)
  {
    return 1U;
  }

  if (release_z_prepare_use_y_position != 0U)
  {
    if (M2006_Axis_GetPositionRev(&y_position_rev) == 0U)
    {
      return 0U;
    }
    y_position_mm = y_position_rev / ROUTE_Y_FEEDBACK_REV_PER_MM;
    remaining_mm = release_z_prepare_y_target_position_mm - y_position_mm;
  }
  else
  {
    if (X_GetAxisPosition(now, &x_position_rad) == 0U)
    {
      return 0U;
    }
    x_position_mm = x_position_rad / X_FEEDBACK_RAD_PER_MM;
    remaining_mm = group->x_target_position_mm - x_position_mm;
  }
  if (remaining_mm < 0.0f)
  {
    remaining_mm = -remaining_mm;
  }
  if (((release_z_prepare_use_y_position != 0U) &&
       (remaining_mm > RELEASE_Z_PREPARE_Y_DISTANCE_MM)) ||
      ((release_z_prepare_use_y_position == 0U) &&
       (remaining_mm > release_z_prepare_x_distance_mm)))
  {
    return 1U;
  }

  /* Keep the gripper closed; Release_Dispatch opens it after Route_Run ends. */
  if (Z_AsyncMove_Start(RELEASE_Z_TARGET_POSITION_MM,
                        Z_TEST_SPEED_RAD_S) == 0U)
  {
    return 0U;
  }
  release_z_prepare_started = 1U;
  return 1U;
}

static uint8_t X_HasReachedTrigger(uint32_t now,
                                   float trigger_position_mm,
                                   float direction_velocity_rad_s,
                                   uint8_t *reached)
{
  float position_rad;
  float position_mm;

  if ((reached == 0) ||
      (direction_velocity_rad_s == 0.0f) ||
      (X_GetAxisPosition(now, &position_rad) == 0U))
  {
    return 0U;
  }

  position_mm = position_rad / X_FEEDBACK_RAD_PER_MM;
  if (direction_velocity_rad_s > 0.0f)
  {
    *reached = (position_mm >= trigger_position_mm) ? 1U : 0U;
  }
  else
  {
    *reached = (position_mm <= trigger_position_mm) ? 1U : 0U;
  }

  return 1U;
}

static uint8_t X_PrepareTargetMotion(float target_position_mm,
                                     float speed_rad_s,
                                     uint32_t now,
                                     uint32_t *run_time_ms,
                                     float *velocity_rad_s)
{
  float current_position_rad;
  float target_position_rad;

  if (X_AcquireFreshFeedback() == 0U)
  {
    return 0U;
  }

  now = HAL_GetTick();
  if ((X_TargetMmToFeedbackRad(target_position_mm,
                               &target_position_rad) == 0U) ||
      (X_GetAxisPosition(now, &current_position_rad) == 0U) ||
      (RoutePlan_CalculateRunTimeMs(current_position_rad,
                                    target_position_rad,
                                    speed_rad_s * X_TRAPEZOID_AREA_RATIO,
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
  return 1U;
}

static uint8_t X_TargetMmToFeedbackRad(float target_position_mm,
                                       float *target_position_rad)
{
  if ((target_position_rad == 0) ||
      (target_position_mm != target_position_mm) ||
      (target_position_mm < X_POSITION_MIN_MM) ||
      (target_position_mm > X_POSITION_MAX_MM))
  {
    return 0U;
  }

  *target_position_rad = target_position_mm * X_FEEDBACK_RAD_PER_MM;
  return 1U;
}

static void Motion_StopXY(void)
{
  X_Stop();
  M2006_Axis_Stop();
}

static void Motion_StopAll(void)
{
  ReleaseZ_Disarm();
  Motion_StopXY();
  Z_AsyncMove_Cancel();
}

static uint8_t Y_TargetMmToFeedbackRev(float target_position_mm,
                                       float *target_position_rev)
{
  if ((target_position_rev == 0) ||
      (target_position_mm != target_position_mm) ||
      (target_position_mm < ROUTE_Y_POSITION_MIN_MM) ||
      (target_position_mm > ROUTE_Y_POSITION_MAX_MM))
  {
    return 0U;
  }

  *target_position_rev = target_position_mm *
                         ROUTE_Y_FEEDBACK_REV_PER_MM;
  return 1U;
}

static uint8_t Y_PrepareTargetMotion(float target_position_mm,
                                     uint16_t speed_rpm,
                                     float *target_position_rev,
                                     uint32_t *run_time_ms,
                                     int16_t *target_rpm)
{
  float current_position_rev;

  if ((Y_TargetMmToFeedbackRev(target_position_mm,
                               target_position_rev) == 0U) ||
      (M2006_Axis_GetPositionRev(&current_position_rev) == 0U) ||
      (RoutePlan_CalculateRunTimeMs(current_position_rev,
                                    *target_position_rev,
                                    (float)speed_rpm / 60.0f,
                                    run_time_ms) == 0U))
  {
    return 0U;
  }

  if (*target_position_rev > current_position_rev)
  {
    *target_rpm = (int16_t)speed_rpm;
  }
  else if (*target_position_rev < current_position_rev)
  {
    *target_rpm = -(int16_t)speed_rpm;
  }
  else
  {
    *target_rpm = 0;
  }
  return 1U;
}

static uint8_t Y_UpdateApproachTarget(float target_position_rev,
                                      uint16_t max_speed_rpm,
                                      uint8_t *ready_to_hold)
{
  float current_position_rev;
  float position_error_rev;
  float absolute_error_rev;
  float feedback_speed_abs_rpm;
  float fine_approach_speed_rpm;
  float speed_rpm;
  int16_t feedback_speed_rpm;
  int16_t target_rpm;

  if ((ready_to_hold == 0) ||
      (M2006_Axis_GetPositionRev(&current_position_rev) == 0U) ||
      (M2006_Axis_GetSpeedRpm(&feedback_speed_rpm) == 0U))
  {
    return 0U;
  }

  position_error_rev = target_position_rev - current_position_rev;
  absolute_error_rev = (position_error_rev >= 0.0f) ?
                       position_error_rev : -position_error_rev;
  feedback_speed_abs_rpm = (feedback_speed_rpm >= 0) ?
                           (float)feedback_speed_rpm :
                           -(float)feedback_speed_rpm;
  if (absolute_error_rev <=
      (Y_HOLD_ENTRY_TOLERANCE_MM * ROUTE_Y_FEEDBACK_REV_PER_MM) &&
      feedback_speed_abs_rpm <= Y_HOLD_ENTRY_MAX_SPEED_RPM)
  {
    *ready_to_hold = 1U;
    return 1U;
  }

  /* v^2 = 120 * a * distance, with v in rpm and distance in revolutions. */
  speed_rpm = sqrtf(120.0f * M2006_AXIS_ACCEL_LIMIT_RPM_S *
                    absolute_error_rev);
  fine_approach_speed_rpm = Y_FINE_APPROACH_KP_RPM_REV *
                            absolute_error_rev;
  if (speed_rpm > fine_approach_speed_rpm)
  {
    speed_rpm = fine_approach_speed_rpm;
  }
  if (speed_rpm > (float)max_speed_rpm)
  {
    speed_rpm = (float)max_speed_rpm;
  }
  if (position_error_rev >= 0.0f)
  {
    target_rpm = (int16_t)(speed_rpm + 0.5f);
  }
  else
  {
    target_rpm = (int16_t)(-speed_rpm - 0.5f);
  }
  if (M2006_Axis_SetSpeedTarget(target_rpm) == 0U)
  {
    return 0U;
  }

  *ready_to_hold = 0U;
  return 1U;
}

static uint8_t Y_StartPreparedMotion(float target_position_rev,
                                     uint32_t run_time_ms,
                                     int16_t target_rpm,
                                     uint8_t *running)
{
  if (run_time_ms > 0U)
  {
    if (M2006_Axis_StartSpeed(target_rpm) == 0U)
    {
      return 0U;
    }
    *running = 1U;
  }
  else
  {
    if (M2006_Axis_StartPositionHold(target_position_rev) == 0U)
    {
      return 0U;
    }
    *running = 0U;
  }

  return 1U;
}

static uint8_t XY_RunGroup(const MotionGroup_t *group)
{
  const YMotionSegment_t *y_segment = 0;
  uint32_t x_start_tick;
  uint32_t last_x_command_tick;
  uint32_t y_start_tick = 0U;
  uint32_t x_run_time_ms = 0U;
  uint32_t y_run_time_ms = 0U;
  float x_velocity_rad_s = 0.0f;
  float y_target_position_rev = 0.0f;
  int16_t y_target_rpm = 0;
  uint8_t x_running;
  uint8_t y_running = 0U;
  uint8_t y_segment_active;
  uint8_t y_segment_index = 0U;
  uint8_t y_waiting_for_x_position = 0U;

  if (RoutePlan_ValidateGroup(group) == 0U)
  {
    Motion_StopAll();
    return 0U;
  }

  if ((group->x_enabled != 0U) &&
      (X_PrepareTargetMotion(group->x_target_position_mm,
                             group->x_speed_rad_s,
                             HAL_GetTick(),
                             &x_run_time_ms,
                             &x_velocity_rad_s) == 0U))
  {
    Motion_StopAll();
    return 0U;
  }
  if (group->y_segment_count > 0U)
  {
    y_segment = &group->y_segments[0];
    y_waiting_for_x_position = y_segment->wait_for_x_position;
  }
  if ((y_segment != 0) &&
      (y_waiting_for_x_position == 0U) &&
      (Y_PrepareTargetMotion(y_segment->y_target_position_mm,
                             y_segment->y_speed_rpm,
                             &y_target_position_rev,
                             &y_run_time_ms,
                             &y_target_rpm) == 0U))
  {
    Motion_StopAll();
    return 0U;
  }

  x_running = ((group->x_enabled != 0U) &&
               (x_run_time_ms > 0U)) ? 1U : 0U;
  y_segment_active = (y_segment != 0) ? 1U : 0U;
  x_start_tick = HAL_GetTick();
  last_x_command_tick = x_start_tick;
  y_start_tick = x_start_tick;

  if (x_running != 0U)
  {
    X_SetSynchronizedVelocity(
      X_GetTrapezoidVelocity(x_velocity_rad_s, 0U, x_run_time_ms),
      x_start_tick);
    if (x_sync_fault != X_SYNC_FAULT_NONE)
    {
      Motion_StopAll();
      return 0U;
    }
  }
  if ((y_segment_active != 0U) &&
      (y_waiting_for_x_position == 0U) &&
      (Y_StartPreparedMotion(y_target_position_rev,
                             y_run_time_ms,
                             y_target_rpm,
                             &y_running) == 0U))
  {
    Motion_StopAll();
    return 0U;
  }

  while ((x_running != 0U) ||
         (y_segment_active != 0U))
  {
    uint32_t now = HAL_GetTick();
    uint32_t x_elapsed_ms = now - x_start_tick;

    if ((Z_AsyncMove_Update(now) == 0U) ||
        (ReleaseZ_UpdateForGroup(group, now) == 0U) ||
        ((group->x_enabled != 0U) &&
         (x_sync_fault != X_SYNC_FAULT_NONE)) ||
        ((group->y_segment_count > 0U) &&
         (M2006_Axis_HasFault() != 0U)))
    {
      Motion_StopAll();
      return 0U;
    }

    if (x_running != 0U)
    {
      if (x_elapsed_ms >= x_run_time_ms)
      {
        X_Stop();
        x_running = 0U;
      }
      else if ((now - last_x_command_tick) >= X_COMMAND_REFRESH_MS)
      {
        last_x_command_tick = now;
        X_SetSynchronizedVelocity(
          X_GetTrapezoidVelocity(x_velocity_rad_s,
                                 x_elapsed_ms,
                                 x_run_time_ms),
          now);
      }
    }

    if (y_segment_active != 0U)
    {
      uint8_t ready_to_hold;

      if (y_waiting_for_x_position != 0U)
      {
        uint8_t trigger_reached;

        if (X_HasReachedTrigger(now,
                                y_segment->x_trigger_position_mm,
                                x_velocity_rad_s,
                                &trigger_reached) == 0U)
        {
          Motion_StopAll();
          return 0U;
        }
        if (trigger_reached != 0U)
        {
          if ((Y_PrepareTargetMotion(y_segment->y_target_position_mm,
                                     y_segment->y_speed_rpm,
                                     &y_target_position_rev,
                                     &y_run_time_ms,
                                     &y_target_rpm) == 0U) ||
              (Y_StartPreparedMotion(y_target_position_rev,
                                     y_run_time_ms,
                                     y_target_rpm,
                                     &y_running) == 0U))
          {
            Motion_StopAll();
            return 0U;
          }
          y_start_tick = now;
          y_waiting_for_x_position = 0U;
        }
        else if (x_running == 0U)
        {
          Motion_StopAll();
          return 0U;
        }
      }

      if (y_waiting_for_x_position == 0U)
      {
        ready_to_hold = (y_running == 0U) ? 1U : 0U;
        if ((y_running != 0U) &&
            (Y_UpdateApproachTarget(y_target_position_rev,
                                    y_segment->y_speed_rpm,
                                    &ready_to_hold) == 0U))
        {
          Motion_StopAll();
          return 0U;
        }
        if (ready_to_hold != 0U)
        {
          /* Keep zero speed after arrival; do not pull back to the theoretical
             position with the position-hold controller. */
          if ((y_running != 0U) &&
              (M2006_Axis_StartSpeed(0) == 0U))
          {
            Motion_StopAll();
            return 0U;
          }

          y_segment_index++;
          if (y_segment_index >= group->y_segment_count)
          {
            y_segment_active = 0U;
            y_running = 0U;
          }
          else
          {
            y_segment = &group->y_segments[y_segment_index];
            y_waiting_for_x_position = y_segment->wait_for_x_position;
            y_running = 0U;
            if ((y_waiting_for_x_position == 0U) &&
                ((Y_PrepareTargetMotion(y_segment->y_target_position_mm,
                                        y_segment->y_speed_rpm,
                                        &y_target_position_rev,
                                        &y_run_time_ms,
                                        &y_target_rpm) == 0U) ||
                 (Y_StartPreparedMotion(y_target_position_rev,
                                        y_run_time_ms,
                                        y_target_rpm,
                                        &y_running) == 0U)))
            {
              Motion_StopAll();
              return 0U;
            }
            if (y_waiting_for_x_position == 0U)
            {
              y_start_tick = now;
            }
          }
        }
        else if (((now - y_start_tick) >= y_run_time_ms) &&
                 (((now - y_start_tick) - y_run_time_ms) >=
                  Y_TARGET_SETTLE_TIMEOUT_MS))
        {
          Motion_StopAll();
          return 0U;
        }
      }
    }

    M2006_Axis_Update(HAL_GetTick());
    if (M2006_Axis_HasFault() != 0U)
    {
      Motion_StopAll();
      return 0U;
    }

    HAL_Delay(1U);
  }

  return 1U;
}

uint8_t Route_Run(uint8_t from_position, uint8_t to_position)
{
  const RoutePlan_t *route = RoutePlan_Find(from_position, to_position);
  uint8_t group_index;

  if ((route == 0) || (route->configured == 0U))
  {
    Motion_StopAll();
    return 0U;
  }

  Motion_StopXY();

  for (group_index = 0U;
       group_index < route->group_count;
       group_index++)
  {
    if (XY_RunGroup(&route->groups[group_index]) == 0U)
    {
      Motion_StopAll();
      return 0U;
    }
  }

  X_Stop();
  if (Z_AsyncMove_Wait() == 0U)
  {
    Motion_StopAll();
    return 0U;
  }
  return 1U;
}

static uint8_t Route_RunToPlace(uint8_t from_position,
                                uint8_t place_position)
{
  if (ReleaseZ_ArmForRoute(from_position, place_position) == 0U)
  {
    Motion_StopAll();
    return 0U;
  }

  if (Route_Run(from_position, place_position) == 0U)
  {
    ReleaseZ_Disarm();
    return 0U;
  }

  return 1U;
}

static uint8_t Bean_ToTargetBoxId(BeanType_t bean)
{
  switch (bean)
  {
    case BEAN_YELLOW:
      return 1U;

    case BEAN_GREEN:
      return 2U;

    case BEAN_WHITE:
      return 3U;

    default:
      return 0U;
  }
}

static uint8_t Bean_GetTransportRunCount(BeanType_t bean)
{
  switch (bean)
  {
    case BEAN_YELLOW:
    case BEAN_GREEN:
    case BEAN_WHITE:
      return 1U;

    default:
      return 0U;
  }
}

static void Host_SendReady(void)
{
  static uint8_t ready_message[] = "READY";

  if (HAL_UART_Transmit(&huart1,
                        ready_message,
                        sizeof(ready_message) - 1U,
                        HOST_READY_TX_TIMEOUT_MS) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Host_Ready_Update(uint32_t now)
{
  if ((host_ready_sent == 0U) ||
      ((now - host_ready_last_tx_tick) >= HOST_READY_TX_INTERVAL_MS))
  {
    Host_SendReady();
    host_ready_last_tx_tick = now;
    host_ready_sent = 1U;
  }
}

static void Vision_ArmReceiver(UART_HandleTypeDef *huart,
                               VisionUartReceiver_t *receiver)
{
  if (HAL_UART_Receive_IT(huart, &receiver->rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Vision_UART_Init(void)
{
  Vision_ArmReceiver(&huart10, &color_vision_receiver);
  Vision_ArmReceiver(&huart7, &digit_vision_receiver);
}

static void Vision_ClearReceiver(VisionUartReceiver_t *receiver)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  receiver->length = 0U;
  receiver->ready = 0U;
  receiver->overflow = 0U;
  receiver->line[0] = '\0';
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void Vision_OnRxByte(VisionUartReceiver_t *receiver)
{
  uint8_t byte = receiver->rx_byte;

  if (receiver->ready == 0U)
  {
    if (byte == (uint8_t)'\n')
    {
      receiver->line[receiver->length] = '\0';
      receiver->ready = 1U;
    }
    else if (byte != (uint8_t)'\r')
    {
      if (receiver->length < (sizeof(receiver->line) - 1U))
      {
        receiver->line[receiver->length] = (char)byte;
        receiver->length++;
      }
      else
      {
        receiver->length = 0U;
        receiver->overflow = 1U;
      }
    }
  }
}

static uint8_t Vision_CopyLine(VisionUartReceiver_t *receiver,
                               char *line,
                               uint8_t line_size)
{
  uint8_t index;
  uint8_t length;
  uint8_t ready;
  uint8_t overflow;
  uint32_t primask;

  if ((line == 0) || (line_size == 0U))
  {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  length = receiver->length;
  ready = receiver->ready;
  overflow = receiver->overflow;
  if ((ready != 0U) &&
      (overflow == 0U) &&
      (length < line_size))
  {
    for (index = 0U; index < length; index++)
    {
      line[index] = receiver->line[index];
    }
    line[length] = '\0';
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  return ((ready != 0U) &&
          (overflow == 0U) &&
          (length < line_size)) ? 1U : 0U;
}

static uint8_t Vision_ParseCsv(const char *line,
                               uint8_t expected_count,
                               uint8_t minimum_value,
                               uint8_t maximum_value,
                               uint8_t *values)
{
  const char *cursor = line;
  uint8_t value_index;
  uint8_t compare_index;

  if ((line == 0) || (values == 0) || (expected_count == 0U))
  {
    return 0U;
  }

  for (value_index = 0U;
       value_index < expected_count;
       value_index++)
  {
    uint16_t value = 0U;
    uint8_t digit_count = 0U;

    while ((*cursor >= '0') && (*cursor <= '9'))
    {
      value = (uint16_t)(value * 10U) +
              (uint16_t)((uint8_t)*cursor - (uint8_t)'0');
      digit_count++;
      cursor++;
    }

    if ((digit_count == 0U) ||
        (value < minimum_value) ||
        (value > maximum_value))
    {
      return 0U;
    }
    values[value_index] = (uint8_t)value;

    if ((value_index + 1U) < expected_count)
    {
      if (*cursor != ',')
      {
        return 0U;
      }
      cursor++;
    }
    else if (*cursor != '\0')
    {
      return 0U;
    }
  }

  for (value_index = 0U;
       value_index < expected_count;
       value_index++)
  {
    for (compare_index = (uint8_t)(value_index + 1U);
         compare_index < expected_count;
         compare_index++)
    {
      if (values[value_index] == values[compare_index])
      {
        return 0U;
      }
    }
  }

  return 1U;
}

static uint8_t Vision_SendCommandToBoth(const uint8_t *command,
                                        uint16_t command_length)
{
  if ((command == 0) || (command_length == 0U))
  {
    return 0U;
  }

  if ((HAL_UART_Transmit(&huart10,
                         command,
                         command_length,
                         100U) != HAL_OK) ||
      (HAL_UART_Transmit(&huart7,
                         command,
                         command_length,
                         100U) != HAL_OK))
  {
    return 0U;
  }

  return 1U;
}

static void Vision_StopResultStreaming(void)
{
  static const uint8_t stop_command[] = "STOP\n";
  uint32_t repeat;

  for (repeat = 0U; repeat < VISION_STOP_REPEAT_COUNT; repeat++)
  {
    (void)Vision_SendCommandToBoth(stop_command,
                                   sizeof(stop_command) - 1U);
    if ((repeat + 1U) < VISION_STOP_REPEAT_COUNT)
    {
      HAL_Delay(VISION_COMMAND_INTERVAL_MS);
    }
  }
}

static uint8_t Vision_WaitForResults(uint8_t *colors, uint8_t *digits)
{
  static const uint8_t start_command[] = "START\n";
  char color_line[16];
  char digit_line[16];
  uint32_t start_tick = HAL_GetTick();
  uint32_t last_command_tick =
      start_tick - VISION_COMMAND_INTERVAL_MS;
  uint8_t color_valid = 0U;
  uint8_t digit_valid = 0U;
  uint8_t invalid_data_seen = 0U;

  while ((HAL_GetTick() - start_tick) < VISION_RESULT_TIMEOUT_MS)
  {
    uint32_t now = HAL_GetTick();

    if (vision_status == VISION_STATUS_UART_ERROR)
    {
      return 0U;
    }

    if ((now - last_command_tick) >= VISION_COMMAND_INTERVAL_MS)
    {
      last_command_tick = now;
      if (Vision_SendCommandToBoth(start_command,
                                   sizeof(start_command) - 1U) == 0U)
      {
        vision_status = VISION_STATUS_UART_ERROR;
        return 0U;
      }
    }

    if ((color_valid == 0U) &&
        (color_vision_receiver.ready != 0U))
    {
      if ((Vision_CopyLine(&color_vision_receiver,
                           color_line,
                           sizeof(color_line)) != 0U) &&
          (Vision_ParseCsv(color_line,
                           VISION_COLOR_RESULT_COUNT,
                           1U,
                           3U,
                           colors) != 0U))
      {
        color_valid = 1U;
      }
      else
      {
        invalid_data_seen = 1U;
        Vision_ClearReceiver(&color_vision_receiver);
      }
    }

    if ((digit_valid == 0U) &&
        (digit_vision_receiver.ready != 0U))
    {
      if ((Vision_CopyLine(&digit_vision_receiver,
                           digit_line,
                           sizeof(digit_line)) != 0U) &&
          (Vision_ParseCsv(digit_line,
                           VISION_DIGIT_RESULT_COUNT,
                           1U,
                           5U,
                           digits) != 0U))
      {
        digit_valid = 1U;
      }
      else
      {
        invalid_data_seen = 1U;
        Vision_ClearReceiver(&digit_vision_receiver);
      }
    }

    if ((color_valid != 0U) && (digit_valid != 0U))
    {
      return 1U;
    }

    M2006_Axis_Update(now);
    HAL_Delay(1U);
  }

  vision_status = (invalid_data_seen != 0U) ?
                  VISION_STATUS_INVALID_DATA :
                  VISION_STATUS_TIMEOUT;
  return 0U;
}

static uint8_t Vision_RecognizeAndBuildTransportPlan(void)
{
  uint8_t colors[VISION_COLOR_RESULT_COUNT];
  uint8_t digits[VISION_DIGIT_RESULT_COUNT];
  uint8_t index;

  Vision_ClearReceiver(&color_vision_receiver);
  Vision_ClearReceiver(&digit_vision_receiver);
  vision_status = VISION_STATUS_WAITING;

  if (Vision_WaitForResults(colors, digits) == 0U)
  {
    Vision_StopResultStreaming();
    return 0U;
  }
  Vision_StopResultStreaming();

  for (index = 0U; index < TRANSPORT_TASK_COUNT; index++)
  {
    uint8_t digit_position;
    uint8_t target_box_id =
        Bean_ToTargetBoxId((BeanType_t)colors[index]);

    transport_task_table[index].pick_position = index + 1U;
    transport_task_table[index].bean = (BeanType_t)colors[index];
    transport_task_table[index].target_box_id = target_box_id;
    transport_task_table[index].place_position = 0U;

    for (digit_position = 0U;
         digit_position < VISION_DIGIT_RESULT_COUNT;
         digit_position++)
    {
      if (digits[digit_position] == target_box_id)
      {
        transport_task_table[index].place_position =
            (uint8_t)(digit_position + 4U);
        break;
      }
    }
    if (transport_task_table[index].place_position == 0U)
    {
      vision_status = VISION_STATUS_INVALID_DATA;
      return 0U;
    }
  }

  for (index = 0U; index < VISION_COLOR_RESULT_COUNT; index++)
  {
    color_vision_result[index] = colors[index];
  }
  for (index = 0U; index < VISION_DIGIT_RESULT_COUNT; index++)
  {
    digit_vision_result[index] = digits[index];
  }

  vision_status = VISION_STATUS_READY;
  return 1U;
}

static uint8_t Grip_Dispatch(uint8_t pick_position, BeanType_t bean)
{
  GripAction_t grip_action;

  if ((pick_position < 1U) ||
      (pick_position > TRANSPORT_TASK_COUNT) ||
      (bean < BEAN_YELLOW) ||
      (bean > BEAN_WHITE))
  {
    return 0U;
  }

  grip_action = grip_action_table[pick_position - 1U]
                                  [(uint8_t)bean - 1U];
  if (grip_action == 0)
  {
    return 0U;
  }

  return grip_action();
}

static uint8_t Release_Dispatch(uint8_t place_position)
{
  switch (place_position)
  {
    case 4U:
    case 8U:
      return RELEASE_04_08();

    case 5U:
    case 6U:
    case 7U:
      return RELEASE_05_06_07();

    default:
      return 0U;
  }
}

static uint8_t TransportPlan_IsRouteConfigured(uint8_t from_position,
                                                uint8_t to_position)
{
  const RoutePlan_t *route = RoutePlan_Find(from_position, to_position);

  return ((route != 0) && (route->configured != 0U)) ? 1U : 0U;
}

static uint8_t TransportPlan_Validate(void)
{
  uint8_t current_position = 0U;
  uint8_t bean_mask = 0U;
  uint8_t place_mask = 0U;
  uint8_t task_order_index;
  uint8_t run_index;
  uint8_t task_index;

  for (task_index = 0U;
       task_index < TRANSPORT_TASK_COUNT;
       task_index++)
  {
    const TransportTask_t *task = &transport_task_table[task_index];
    uint8_t bean_bit;
    uint8_t place_bit;

    if ((task->pick_position != (task_index + 1U)) ||
        (task->bean < BEAN_YELLOW) ||
        (task->bean > BEAN_WHITE) ||
        (task->target_box_id != Bean_ToTargetBoxId(task->bean)) ||
        (task->place_position < 4U) ||
        (task->place_position > 8U))
    {
      return 0U;
    }

    bean_bit = (uint8_t)(1U << ((uint8_t)task->bean - 1U));
    place_bit = (uint8_t)(1U << (task->place_position - 4U));
    if (((bean_mask & bean_bit) != 0U) ||
        ((place_mask & place_bit) != 0U))
    {
      return 0U;
    }
    bean_mask |= bean_bit;
    place_mask |= place_bit;
  }

  for (task_order_index = 0U;
       task_order_index < TRANSPORT_TASK_COUNT;
       task_order_index++)
  {
    const TransportTask_t *task;
    uint8_t run_count;

    task_index = transport_task_order[task_order_index];
    if (task_index >= TRANSPORT_TASK_COUNT)
    {
      return 0U;
    }

    task = &transport_task_table[task_index];
    run_count = Bean_GetTransportRunCount(task->bean);
    if (run_count == 0U)
    {
      return 0U;
    }

    for (run_index = 0U;
         run_index < run_count;
         run_index++)
    {
      if ((TransportPlan_IsRouteConfigured(current_position,
                                            task->pick_position) == 0U) ||
          (TransportPlan_IsRouteConfigured(task->pick_position,
                                            task->place_position) == 0U))
      {
        return 0U;
      }

      current_position = task->place_position;
    }
  }

  if (TransportPlan_IsRouteConfigured(current_position, 0U) == 0U)
  {
    return 0U;
  }

  return (bean_mask == 0x07U) ? 1U : 0U;
}

static uint8_t TransportPlan_Run(void)
{
  uint8_t current_position = 0U;
  uint8_t task_order_index;
  uint8_t run_index;
  uint8_t task_index;

  if ((TransportPlan_Validate() == 0U) ||
      (Z_START() == 0U))
  {
    Motion_StopAll();
    return 0U;
  }

  for (task_order_index = 0U;
       task_order_index < TRANSPORT_TASK_COUNT;
       task_order_index++)
  {
    const TransportTask_t *task;
    uint8_t run_count;

    task_index = transport_task_order[task_order_index];
    task = &transport_task_table[task_index];
    run_count = Bean_GetTransportRunCount(task->bean);
    for (run_index = 0U;
         run_index < run_count;
         run_index++)
    {
      if (run_index == 1U)
      {
        grip_depth_offset_mm =
            (task->bean == BEAN_WHITE) ?
            SECOND_RUN_WHITE_OFFSET_MM :
            SECOND_RUN_GREEN_OFFSET_MM;
      }
      else
      {
        grip_depth_offset_mm = 0.0f;
      }

      if ((Route_Run(current_position, task->pick_position) == 0U) ||
          (Grip_Dispatch(task->pick_position, task->bean) == 0U) ||
          (Route_RunToPlace(task->pick_position,
                            task->place_position) == 0U) ||
          (Release_Dispatch(task->place_position) == 0U))
      {
        Motion_StopAll();
        return 0U;
      }

      current_position = task->place_position;
    }
  }

  if (Route_Run(current_position, 0U) == 0U)
  {
    Motion_StopAll();
    return 0U;
  }

  grip_depth_offset_mm = 0.0f;
  Motion_StopAll();
  return 1U;
}

static void Crane_RunOnce(void)
{
  if (PA15_Z_START_ONLY != 0U)
  {
    if (Z_START() == 0U)
    {
      Motion_StopAll();
      Crane_EmergencyStop();
      xy_test_result = RUN_RESULT_MOTION_FAILED;
    }
    else
    {
      xy_test_result = RUN_RESULT_SUCCESS;
    }
  }
  else if (Vision_RecognizeAndBuildTransportPlan() == 0U)
  {
    Motion_StopAll();
    xy_test_result = RUN_RESULT_VISION_FAILED;
  }
  else if (TransportPlan_Run() == 0U)
  {
    Motion_StopAll();
    Crane_EmergencyStop();
    xy_test_result = RUN_RESULT_MOTION_FAILED;
  }
  else
  {
    xy_test_result = RUN_RESULT_SUCCESS;
  }
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
  if (CRANE_MOTION_ENABLED != 0U)
  {
    Vision_UART_Init();
  }
  RotationServo_SetAngle(ROTATION_SERVO_END_ANGLE);
  GripperServo_SetAngle(GRIPPER_SERVO_GRIP_ANGLE);
  if (CRANE_MOTION_ENABLED != 0U)
  {
    DM3519_CAN1_Start();
    if (M2006_Axis_Init(&hfdcan2) != HAL_OK)
    {
      Error_Handler();
    }
    Motion_StopAll();
    HAL_Delay(100U);
    X_EnablePairedVelocityMode();
    Z_EnableVelocityMode();
    DM3519_RequestMappingRanges(DM3519_X_MOTOR1_ID);
    DM3519_RequestMappingRanges(DM3519_X_MOTOR2_ID);
    DM3519_RequestMappingRanges(DM3519_Z_MOTOR_ID);
    X_SetPairedVelocity(0.0f);
    Z_SetVelocity(0.0f);
    HAL_Delay(20U);
    if (Z_EstablishPositionReference() == 0U)
    {
      Error_Handler();
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (CRANE_MOTION_ENABLED != 0U)
    {
      float position_rev;
      float z_position_mm;

      M2006_Axis_Update(HAL_GetTick());
      (void)Z_GetPositionMm(&z_position_mm);
      if (M2006_Axis_GetPositionRev(&position_rev) != 0U)
      {
        y_calibration_position_rev = position_rev;
        y_calibration_position_valid = 1U;
      }
      else
      {
        y_calibration_position_valid = 0U;
      }

      if ((xy_test_result == RUN_RESULT_READY) ||
          (xy_test_result == RUN_RESULT_VISION_FAILED))
      {
        Host_Ready_Update(HAL_GetTick());
      }

      if (((xy_test_result == RUN_RESULT_READY) ||
           (xy_test_result == RUN_RESULT_VISION_FAILED)) &&
          (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) ==
           START_KEY_PRESSED_STATE))
      {
        HAL_Delay(START_KEY_DEBOUNCE_MS);
        if (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) ==
            START_KEY_PRESSED_STATE)
        {
          Crane_RunOnce();

          while (HAL_GPIO_ReadPin(START_KEY_GPIO_Port, START_KEY_Pin) ==
                 START_KEY_PRESSED_STATE)
          {
            M2006_Axis_Update(HAL_GetTick());
            HAL_Delay(10U);
          }
          HAL_Delay(START_KEY_DEBOUNCE_MS);
        }
      }
    }

    HAL_Delay(10U);
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
  uint32_t pending_messages;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
  {
    return;
  }

  pending_messages = HAL_FDCAN_GetRxFifoFillLevel(hfdcan,
                                                   FDCAN_RX_FIFO0);
  while (pending_messages > 0U)
  {
    pending_messages--;
    if (HAL_FDCAN_GetRxMessage(hfdcan,
                               FDCAN_RX_FIFO0,
                               &rx_header,
                               rx_data) != HAL_OK)
    {
      break;
    }

    if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
        (rx_header.DataLength != FDCAN_DLC_BYTES_8))
    {
      continue;
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
      else if (rx_header.Identifier == DM3519_Z_MOTOR_MASTER_ID)
      {
        DM3519_DecodeFeedback(&dm3519_z_motor,
                              DM3519_Z_MOTOR_ID,
                              rx_data);
      }
    }
    else if ((hfdcan->Instance == FDCAN2) &&
             (rx_header.Identifier == M2006_AXIS_FEEDBACK_ID))
    {
      M2006_Axis_OnFeedback(rx_data, HAL_GetTick());
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART10)
  {
    Vision_OnRxByte(&color_vision_receiver);
    Vision_ArmReceiver(&huart10, &color_vision_receiver);
  }
  else if (huart->Instance == UART7)
  {
    Vision_OnRxByte(&digit_vision_receiver);
    Vision_ArmReceiver(&huart7, &digit_vision_receiver);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART10)
  {
    color_vision_receiver.length = 0U;
    color_vision_receiver.ready = 0U;
    color_vision_receiver.overflow = 0U;
    vision_status = VISION_STATUS_UART_ERROR;
    (void)HAL_UART_Receive_IT(&huart10,
                             &color_vision_receiver.rx_byte,
                             1U);
  }
  else if (huart->Instance == UART7)
  {
    digit_vision_receiver.length = 0U;
    digit_vision_receiver.ready = 0U;
    digit_vision_receiver.overflow = 0U;
    vision_status = VISION_STATUS_UART_ERROR;
    (void)HAL_UART_Receive_IT(&huart7,
                             &digit_vision_receiver.rx_byte,
                             1U);
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
  Crane_EmergencyStop();
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
