#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H

#include <stdint.h>

#define ROUTE_Y_FEEDBACK_REV_PER_MM  0.44913793f
#define ROUTE_Y_POSITION_MIN_MM      (-920.0f)
#define ROUTE_Y_POSITION_MAX_MM      1220.0f

typedef struct
{
  float y_target_position_mm;
  uint16_t y_speed_rpm;
  uint8_t wait_for_x_position;
  float x_trigger_position_mm;
} YMotionSegment_t;

typedef struct
{
  uint8_t x_enabled;
  float x_target_position_mm;
  float x_speed_rad_s;
  const YMotionSegment_t *y_segments;
  uint8_t y_segment_count;
} MotionGroup_t;

typedef struct
{
  uint8_t from_position;
  uint8_t to_position;
  const MotionGroup_t *groups;
  uint8_t group_count;
  uint8_t configured;
} RoutePlan_t;

const RoutePlan_t *RoutePlan_Find(uint8_t from_position,
                                  uint8_t to_position);
uint8_t RoutePlan_ValidateGroup(const MotionGroup_t *group);
uint8_t RoutePlan_CalculateRunTimeMs(float current_position,
                                     float target_position,
                                     float speed_per_second,
                                     uint32_t *run_time_ms);
uint8_t Route_Run(uint8_t from_position, uint8_t to_position);

#endif /* ROUTE_PLAN_H */
