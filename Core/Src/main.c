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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint16_t angle;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperature;
  uint32_t msg_count;
} M2006_Measure_t;

typedef struct
{
  float integral;
  float prev_error;
} M2006_SpeedPid_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DM3519_X_MOTOR1_ID                 0x01U
#define DM3519_X_MOTOR2_ID                 0x02U
#define DM3519_Z_MOTOR_ID                  0x03U
#define DM3519_VELOCITY_MODE_ID            0x200U
#define DM3519_CONTROL_STD_ID(id)          (DM3519_VELOCITY_MODE_ID + (id))

#define X_RUN_SPEED_RAD_S                  15.0f
#define X_COMMAND_REFRESH_MS               20U
#define X_STOP_REPEAT_COUNT                3U
#define X_STOP_REPEAT_INTERVAL_MS          2U
#define ACTION_INTERVAL_MS                 1500U
#define Z_INITIAL_SPEED_RAD_S              6.0f
#define Z_INITIAL_TIME_MS                  4000U

#define Z_PRE_GRIP_SPEED_RAD_S             -7.0f
#define Z_PRE_GRIP_TIME_MS                 2000U
#define Z_POST_GRIP_SPEED_RAD_S            7.0f
#define Z_POST_GRIP_TIME_MS                3000U

#define SERVO_MIN_PULSE_US                 500U
#define SERVO_MAX_PULSE_US                 2500U
#define SERVO_MAX_ANGLE_DEG                180.0f
#define SERVO_RELEASE_ANGLE_DEG            68.0f
#define SERVO_GRIP_ANGLE_DEG               110.0f
#define SERVO_TRAVEL_TIME_MS               2000U

#define M2006_CAN_ID_CONTROL               0x200U
#define M2006_CAN_ID_MOTOR1                0x201U
#define M2006_RUN_SPEED_RPM                3600
#define M2006_CONTROL_PERIOD_MS            10U
#define M2006_PID_KP                       8.0f
#define M2006_PID_KI                       1.20f
#define M2006_PID_KD                       0.0f
#define M2006_PID_INTEGRAL_LIMIT           5000.0f
#define M2006_PID_OUTPUT_LIMIT             8000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t servo_pwm_started = 0U;
static volatile M2006_Measure_t m2006_motor = {0};
static M2006_SpeedPid_t m2006_speed_pid = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t Competition_ShouldRunAfterReset(void)
{
  uint8_t pin_reset;
  uint8_t power_reset;

  pin_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0U) ? 1U : 0U;
  power_reset = ((__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0U) ||
                 (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != 0U)) ? 1U : 0U;

  __HAL_RCC_CLEAR_RESET_FLAGS();
  return (uint8_t)((pin_reset != 0U) && (power_reset == 0U));
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
}

static void X_SetPairedVelocity(float motor1_velocity_rad_s)
{
  DM3519_SetVelocity(&hfdcan1,
                      DM3519_X_MOTOR1_ID,
                      motor1_velocity_rad_s);
  DM3519_SetVelocity(&hfdcan1,
                      DM3519_X_MOTOR2_ID,
                      -motor1_velocity_rad_s);
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

static void X_RunForTime(int8_t quhuo_forward, uint32_t run_time_ms)
{
  float direction = (quhuo_forward >= 0) ? 1.0f : -1.0f;
  uint32_t start_tick = HAL_GetTick();
  uint32_t elapsed_ms = 0U;

  while (elapsed_ms < run_time_ms)
  {
    X_SetPairedVelocity(direction * X_RUN_SPEED_RAD_S);

    elapsed_ms = HAL_GetTick() - start_tick;
    if (elapsed_ms < run_time_ms)
    {
      uint32_t remaining_ms = run_time_ms - elapsed_ms;
      HAL_Delay((remaining_ms < X_COMMAND_REFRESH_MS) ?
                remaining_ms : X_COMMAND_REFRESH_MS);
    }

    elapsed_ms = HAL_GetTick() - start_tick;
  }

  X_Stop();
  HAL_Delay(ACTION_INTERVAL_MS);
}

static void Z_RunForTime(float velocity_rad_s, uint32_t run_time_ms)
{
  DM3519_SetVelocity(&hfdcan1, DM3519_Z_MOTOR_ID, velocity_rad_s);
  HAL_Delay(run_time_ms);
  DM3519_SetVelocity(&hfdcan1, DM3519_Z_MOTOR_ID, 0.0f);
}

static void Servo_SetAngle(float angle_deg)
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
             (uint32_t)(angle_deg *
                        (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) /
                        SERVO_MAX_ANGLE_DEG);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_us);

  if (servo_pwm_started == 0U)
  {
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
    {
      Error_Handler();
    }
    servo_pwm_started = 1U;
  }

  HAL_Delay(SERVO_TRAVEL_TIME_MS);
}

static void Gripper_RunAction(void)
{
  /* Z axis and servo actions are temporarily disabled for XY-only testing. */
  /* Z_RunForTime(Z_PRE_GRIP_SPEED_RAD_S, Z_PRE_GRIP_TIME_MS); */
  /* Servo_SetAngle(SERVO_GRIP_ANGLE_DEG); */
  /* Z_RunForTime(Z_POST_GRIP_SPEED_RAD_S, Z_POST_GRIP_TIME_MS); */
  HAL_Delay(ACTION_INTERVAL_MS);
}

static void Release_RunAction(void)
{
  /* Z axis and servo actions are temporarily disabled for XY-only testing. */
  /* Z_RunForTime(-Z_POST_GRIP_SPEED_RAD_S, Z_POST_GRIP_TIME_MS); */
  /* Servo_SetAngle(SERVO_RELEASE_ANGLE_DEG); */
  /* Z_RunForTime(-Z_PRE_GRIP_SPEED_RAD_S, Z_PRE_GRIP_TIME_MS); */
  HAL_Delay(ACTION_INTERVAL_MS);
}

static float ClampFloat(float value, float min_value, float max_value)
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

static int16_t ClampCurrent(float value)
{
  if (value > (float)M2006_PID_OUTPUT_LIMIT)
  {
    return M2006_PID_OUTPUT_LIMIT;
  }
  if (value < (float)-M2006_PID_OUTPUT_LIMIT)
  {
    return (int16_t)-M2006_PID_OUTPUT_LIMIT;
  }
  return (int16_t)value;
}

static void M2006_Decode(const uint8_t data[8])
{
  m2006_motor.angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  m2006_motor.speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  m2006_motor.given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  m2006_motor.temperature = data[6];
  m2006_motor.msg_count++;
}

static void M2006_SetCurrent(int16_t current)
{
  uint8_t data[8] = {0};

  data[0] = (uint8_t)((uint16_t)current >> 8);
  data[1] = (uint8_t)current;
  FDCAN_SendStandardFrame(&hfdcan2, M2006_CAN_ID_CONTROL, data, 8U);
}

static void M2006_SpeedPidReset(void)
{
  m2006_speed_pid.integral = 0.0f;
  m2006_speed_pid.prev_error = 0.0f;
}

static int16_t M2006_SpeedPidCalc(int16_t target_rpm,
                                  int16_t feedback_rpm,
                                  float dt_s)
{
  float error;
  float derivative;
  float output;

  if (dt_s <= 0.0f)
  {
    dt_s = 0.001f;
  }

  error = (float)target_rpm - (float)feedback_rpm;
  m2006_speed_pid.integral += error * dt_s;
  m2006_speed_pid.integral = ClampFloat(m2006_speed_pid.integral,
                                        -M2006_PID_INTEGRAL_LIMIT,
                                        M2006_PID_INTEGRAL_LIMIT);
  derivative = (error - m2006_speed_pid.prev_error) / dt_s;
  output = (M2006_PID_KP * error) +
           (M2006_PID_KI * m2006_speed_pid.integral) +
           (M2006_PID_KD * derivative);
  m2006_speed_pid.prev_error = error;

  return ClampCurrent(output);
}

static void M2006_CAN2_Start(void)
{
  FDCAN_FilterTypeDef filter = {0};

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = M2006_CAN_ID_MOTOR1;
  filter.FilterID2 = 0x7FFU;

  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan2,
                                     FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0U) != HAL_OK)
  {
    Error_Handler();
  }
}

static void M2006_Stop(void)
{
  M2006_SpeedPidReset();
  M2006_SetCurrent(0);
}

static void Y_RunForTime(int8_t direction, uint32_t run_time_ms)
{
  uint32_t start_tick = HAL_GetTick();
  uint32_t last_control_tick = start_tick;
  int16_t target_rpm = (direction >= 0) ? M2006_RUN_SPEED_RPM : -M2006_RUN_SPEED_RPM;

  M2006_SpeedPidReset();
  while ((HAL_GetTick() - start_tick) < run_time_ms)
  {
    uint32_t now = HAL_GetTick();
    if ((now - last_control_tick) >= M2006_CONTROL_PERIOD_MS)
    {
      float dt_s = (float)(now - last_control_tick) / 1000.0f;
      int16_t current;

      last_control_tick = now;
      current = M2006_SpeedPidCalc(target_rpm, m2006_motor.speed_rpm, dt_s);
      M2006_SetCurrent(current);
    }

    HAL_Delay(1U);
  }

  M2006_Stop();
  HAL_Delay(ACTION_INTERVAL_MS);
}

static void Competition_RunSequence(void)
{
  /* Initial action: raise Z axis, open gripper, then wait before Flow 1. */
  Z_RunForTime(Z_INITIAL_SPEED_RAD_S, Z_INITIAL_TIME_MS);
  Servo_SetAngle(SERVO_RELEASE_ANGLE_DEG);
  HAL_Delay(ACTION_INTERVAL_MS);

  /* Flow 1: Y 0->1, X A->C, grip, X C->A', Y 1->7, X A'->D, release. */
  Y_RunForTime(-1, 3879U);
  X_RunForTime(1, 9183U);
  Gripper_RunAction();
  X_RunForTime(-1, 5673U);
  Y_RunForTime(1, 6429U);
  X_RunForTime(-1, 9402U);
  Release_RunAction();

  /* Flow 2: X D->A'', Y 7->2, X A''->C, grip, X C->A', Y 2->5, X A'->E. */
  X_RunForTime(1, 4387U);
  Y_RunForTime(1, 715U);
  X_RunForTime(1, 10688U);
  Gripper_RunAction();
  X_RunForTime(-1, 5673U);
  Y_RunForTime(-1, 6429U);
  X_RunForTime(-1, 10391U);
  Release_RunAction();

  /* Flow 3: X E->A', Y 5->3, X A'->B, grip, then pass obstacle points 9 and 9'. */
  X_RunForTime(1, 10391U);
  Y_RunForTime(1, 2858U);
  X_RunForTime(1, 4576U);
  Gripper_RunAction();
  Y_RunForTime(1, 1100U);
  X_RunForTime(-1, 4576U);
  Y_RunForTime(-1, 2200U);
  X_RunForTime(-1, 10391U);
  Y_RunForTime(1, 1407U);
  Release_RunAction();

  /* Final action: close the gripper before ending the competition sequence. */
  Servo_SetAngle(SERVO_GRIP_ANGLE_DEG);
  HAL_Delay(ACTION_INTERVAL_MS);

  X_Stop();
  M2006_Stop();
  DM3519_SetVelocity(&hfdcan1, DM3519_Z_MOTOR_ID, 0.0f);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t run_competition = Competition_ShouldRunAfterReset();

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
  M2006_CAN2_Start();
  M2006_SpeedPidReset();
  X_Stop();
  M2006_Stop();
  DM3519_SetVelocity(&hfdcan1, DM3519_Z_MOTOR_ID, 0.0f);
  HAL_Delay(100U);

  X_EnablePairedVelocityMode();
  DM3519_EnableVelocityMode(&hfdcan1, DM3519_Z_MOTOR_ID);
  HAL_Delay(20U);

  if (run_competition != 0U)
  {
    Competition_RunSequence();
  }

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

  if (((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) ||
      (hfdcan->Instance != FDCAN2))
  {
    return;
  }

  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
  {
    return;
  }

  if ((rx_header.IdType == FDCAN_STANDARD_ID) &&
      (rx_header.Identifier == M2006_CAN_ID_MOTOR1) &&
      (rx_header.DataLength == FDCAN_DLC_BYTES_8))
  {
    M2006_Decode(rx_data);
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
