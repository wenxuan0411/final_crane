/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

uint8_t Z_move(float target_position_mm, float speed_rad_s);
uint8_t Z_START(void);
uint8_t GRIP_01_YELLOW(void);
uint8_t GRIP_01_GREEN(void);
uint8_t GRIP_01_WHITE(void);
uint8_t GRIP_02_YELLOW(void);
uint8_t GRIP_02_GREEN(void);
uint8_t GRIP_02_WHITE(void);
uint8_t GRIP_03_YELLOW(void);
uint8_t GRIP_03_GREEN(void);
uint8_t GRIP_03_WHITE(void);
uint8_t RELEASE_04_08(void);
uint8_t RELEASE_05_06_07(void);
extern volatile uint8_t z_feedback_valid;
extern volatile uint8_t z_feedback_error;
extern volatile uint8_t z_feedback_gear_ratio_valid;
extern volatile uint32_t z_feedback_message_count;
extern volatile float z_feedback_position_rad;
extern volatile float z_feedback_unwrapped_rad;
extern volatile float z_feedback_delta_rad;
extern volatile float z_feedback_velocity_rad_s;
extern volatile float z_feedback_torque_nm;
extern volatile float z_feedback_position_mm;
extern volatile float z_feedback_gear_ratio;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_KEY_Pin GPIO_PIN_15
#define START_KEY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
