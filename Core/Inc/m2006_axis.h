#ifndef M2006_AXIS_H
#define M2006_AXIS_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#define M2006_AXIS_CONTROL_ID       0x200U
#define M2006_AXIS_FEEDBACK_ID      0x201U

typedef enum
{
  M2006_AXIS_IDLE = 0,
  M2006_AXIS_RUNNING,
  M2006_AXIS_HOLDING,
  M2006_AXIS_FAULT
} M2006_AxisState_t;

typedef enum
{
  M2006_AXIS_FAULT_NONE = 0,
  M2006_AXIS_FAULT_NO_FEEDBACK,
  M2006_AXIS_FAULT_FEEDBACK_TIMEOUT,
  M2006_AXIS_FAULT_CAN_TX
} M2006_AxisFault_t;

HAL_StatusTypeDef M2006_Axis_Init(FDCAN_HandleTypeDef *hfdcan);
void M2006_Axis_OnFeedback(const uint8_t data[8], uint32_t now_ms);

uint8_t M2006_Axis_StartSpeed(int16_t target_rpm);
uint8_t M2006_Axis_SetSpeedTarget(int16_t target_rpm);
uint8_t M2006_Axis_StartPositionHold(float target_position_rev);
void M2006_Axis_Update(uint32_t now_ms);
void M2006_Axis_Stop(void);

uint8_t M2006_Axis_HasFeedback(void);
uint8_t M2006_Axis_HasFault(void);
M2006_AxisFault_t M2006_Axis_GetFault(void);
uint8_t M2006_Axis_GetPositionRev(float *position_rev);

#endif /* M2006_AXIS_H */
