#ifndef ROUTE_PLAN_H
#define ROUTE_PLAN_H

#include <stdint.h>

typedef struct
{
  float x_speed_rad_s;
  uint32_t x_time_ms;
  int16_t y_speed_rpm;
  uint32_t y_time_ms;
  uint32_t pause_after_ms;
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
uint8_t Route_Run(uint8_t from_position, uint8_t to_position);

#endif /* ROUTE_PLAN_H */
