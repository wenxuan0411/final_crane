#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H

#include <stdint.h>

typedef struct
{
  uint8_t x_enabled;
  float x_target_position_rad;
  float x_speed_rad_s;
  uint8_t y_enabled;
  float y_target_position_rev;
  uint16_t y_speed_rpm;
  uint8_t synchronized;
} MotionSegment_t;

typedef struct
{
  uint8_t from_position;
  uint8_t to_position;
  const MotionSegment_t *segments;
  uint8_t segment_count;
  uint8_t configured;
} RoutePlan_t;

const RoutePlan_t *RoutePlan_Find(uint8_t from_position,
                                  uint8_t to_position);
uint8_t RoutePlan_ValidateSegment(const MotionSegment_t *segment);
uint8_t RoutePlan_CalculateRunTimeMs(float current_position,
                                     float target_position,
                                     float speed_per_second,
                                     uint32_t *run_time_ms);
uint8_t Route_Run(uint8_t from_position, uint8_t to_position);

#endif /* ROUTE_PLAN_H */
