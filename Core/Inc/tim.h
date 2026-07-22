/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim2;

/* USER CODE BEGIN Private defines */

#define GRIPPER_SERVO_TIM             htim2
#define GRIPPER_SERVO_CHANNEL         TIM_CHANNEL_1
#define GRIPPER_SERVO_GPIO_PORT       GPIOA
#define GRIPPER_SERVO_GPIO_PIN        GPIO_PIN_0
#define GRIPPER_SERVO_OPEN_ANGLE      68.0f
#define GRIPPER_SERVO_GRIP_ANGLE      110.0f

#define ROTATION_SERVO_TIM            htim2
#define ROTATION_SERVO_CHANNEL        TIM_CHANNEL_3
#define ROTATION_SERVO_GPIO_PORT      GPIOA
#define ROTATION_SERVO_GPIO_PIN       GPIO_PIN_2
#define ROTATION_SERVO_START_ANGLE    50.0f
#define ROTATION_SERVO_END_ANGLE      135.0f

/* USER CODE END Private defines */

void MX_TIM2_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

