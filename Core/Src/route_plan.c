#include "route_plan.h"
#include <float.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
/* Revision: raise the X path-speed validation limit from 15 to 35 rad/s. */
#define ROUTE_X_MAX_SPEED_RAD_S 35.0f
#define ROUTE_Y_MAX_SPEED_RPM   6000U
#define UNCONFIGURED_STEP       {0U, 0.0f, 0.0f, 0U, 0.0f, 0U, 0U}

/*
 * Route template format:
 * { X enabled, X absolute target (mm), X speed (rad/s),
 *   Y enabled, Y target (rev), Y speed (rpm), synchronized }
 * Revision: both axes may be enabled only for synchronized motion. A
 * non-synchronized segment controls exactly one axis; synchronization is
 * ignored for one axis.
 *
 * Keep configured = 0U in route_table until all segments of that route
 * have been measured and filled in.
 */

static const MotionSegment_t route_1_to_4[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_1_to_5[] =
{
  {1U, -43.781250f, 10.0f, 1U, 200.0f, 3000U, 1U},
  {1U, -141.015625f, 15.0f, 1U, -28.0f, 3600U, 1U},
};

static const MotionSegment_t route_1_to_6[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_1_to_7[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_1_to_8[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_2_to_4[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_2_to_5[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_2_to_6[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_2_to_7[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_2_to_8[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_3_to_4[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_3_to_5[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_3_to_6[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_3_to_7[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_3_to_8[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_4_to_1[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_4_to_2[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_4_to_3[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_5_to_1[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_5_to_2[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_5_to_3[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_6_to_1[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_6_to_2[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_6_to_3[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_7_to_1[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_7_to_2[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_7_to_3[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_8_to_1[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_8_to_2[] =
{
  UNCONFIGURED_STEP,
};

static const MotionSegment_t route_8_to_3[] =
{
  UNCONFIGURED_STEP,
};

static const RoutePlan_t route_table[] =
{
  {1U, 4U, route_1_to_4, (uint8_t)ARRAY_SIZE(route_1_to_4), 0U},
  {1U, 5U, route_1_to_5, (uint8_t)ARRAY_SIZE(route_1_to_5), 1U},
  {1U, 6U, route_1_to_6, (uint8_t)ARRAY_SIZE(route_1_to_6), 0U},
  {1U, 7U, route_1_to_7, (uint8_t)ARRAY_SIZE(route_1_to_7), 0U},
  {1U, 8U, route_1_to_8, (uint8_t)ARRAY_SIZE(route_1_to_8), 0U},

  {2U, 4U, route_2_to_4, (uint8_t)ARRAY_SIZE(route_2_to_4), 0U},
  {2U, 5U, route_2_to_5, (uint8_t)ARRAY_SIZE(route_2_to_5), 0U},
  {2U, 6U, route_2_to_6, (uint8_t)ARRAY_SIZE(route_2_to_6), 0U},
  {2U, 7U, route_2_to_7, (uint8_t)ARRAY_SIZE(route_2_to_7), 0U},
  {2U, 8U, route_2_to_8, (uint8_t)ARRAY_SIZE(route_2_to_8), 0U},

  {3U, 4U, route_3_to_4, (uint8_t)ARRAY_SIZE(route_3_to_4), 0U},
  {3U, 5U, route_3_to_5, (uint8_t)ARRAY_SIZE(route_3_to_5), 0U},
  {3U, 6U, route_3_to_6, (uint8_t)ARRAY_SIZE(route_3_to_6), 0U},
  {3U, 7U, route_3_to_7, (uint8_t)ARRAY_SIZE(route_3_to_7), 0U},
  {3U, 8U, route_3_to_8, (uint8_t)ARRAY_SIZE(route_3_to_8), 0U},

  {4U, 1U, route_4_to_1, (uint8_t)ARRAY_SIZE(route_4_to_1), 0U},
  {4U, 2U, route_4_to_2, (uint8_t)ARRAY_SIZE(route_4_to_2), 0U},
  {4U, 3U, route_4_to_3, (uint8_t)ARRAY_SIZE(route_4_to_3), 0U},
  {5U, 1U, route_5_to_1, (uint8_t)ARRAY_SIZE(route_5_to_1), 0U},
  {5U, 2U, route_5_to_2, (uint8_t)ARRAY_SIZE(route_5_to_2), 0U},
  {5U, 3U, route_5_to_3, (uint8_t)ARRAY_SIZE(route_5_to_3), 0U},
  {6U, 1U, route_6_to_1, (uint8_t)ARRAY_SIZE(route_6_to_1), 0U},
  {6U, 2U, route_6_to_2, (uint8_t)ARRAY_SIZE(route_6_to_2), 0U},
  {6U, 3U, route_6_to_3, (uint8_t)ARRAY_SIZE(route_6_to_3), 0U},
  {7U, 1U, route_7_to_1, (uint8_t)ARRAY_SIZE(route_7_to_1), 0U},
  {7U, 2U, route_7_to_2, (uint8_t)ARRAY_SIZE(route_7_to_2), 0U},
  {7U, 3U, route_7_to_3, (uint8_t)ARRAY_SIZE(route_7_to_3), 0U},
  {8U, 1U, route_8_to_1, (uint8_t)ARRAY_SIZE(route_8_to_1), 0U},
  {8U, 2U, route_8_to_2, (uint8_t)ARRAY_SIZE(route_8_to_2), 0U},
  {8U, 3U, route_8_to_3, (uint8_t)ARRAY_SIZE(route_8_to_3), 0U},
};

const RoutePlan_t *RoutePlan_Find(uint8_t from_position,
                                  uint8_t to_position)
{
  uint32_t route_index;

  for (route_index = 0U;
       route_index < (uint32_t)ARRAY_SIZE(route_table);
       route_index++)
  {
    if ((route_table[route_index].from_position == from_position) &&
        (route_table[route_index].to_position == to_position))
    {
      return &route_table[route_index];
    }
  }

  return 0;
}

static uint8_t RoutePlan_IsFinite(float value)
{
  return ((value == value) &&
          (value <= FLT_MAX) &&
          (value >= -FLT_MAX)) ? 1U : 0U;
}

uint8_t RoutePlan_ValidateSegment(const MotionSegment_t *segment)
{
  if ((segment == 0) ||
      ((segment->x_enabled == 0U) && (segment->y_enabled == 0U)) ||
      ((segment->x_enabled != 0U) &&
       (segment->y_enabled != 0U) &&
       (segment->synchronized == 0U)))
  {
    return 0U;
  }

  if ((segment->x_enabled != 0U) &&
      ((RoutePlan_IsFinite(segment->x_target_position_mm) == 0U) ||
       (RoutePlan_IsFinite(segment->x_speed_rad_s) == 0U) ||
       (segment->x_speed_rad_s <= 0.0f) ||
       (segment->x_speed_rad_s > ROUTE_X_MAX_SPEED_RAD_S)))
  {
    return 0U;
  }

  if ((segment->y_enabled != 0U) &&
      ((RoutePlan_IsFinite(segment->y_target_position_rev) == 0U) ||
       (segment->y_speed_rpm == 0U) ||
       (segment->y_speed_rpm > ROUTE_Y_MAX_SPEED_RPM)))
  {
    return 0U;
  }

  return 1U;
}

uint8_t RoutePlan_CalculateRunTimeMs(float current_position,
                                     float target_position,
                                     float speed_per_second,
                                     uint32_t *run_time_ms)
{
  float distance;
  float duration_ms;

  if ((run_time_ms == 0) ||
      (RoutePlan_IsFinite(current_position) == 0U) ||
      (RoutePlan_IsFinite(target_position) == 0U) ||
      (RoutePlan_IsFinite(speed_per_second) == 0U) ||
      (speed_per_second <= 0.0f))
  {
    return 0U;
  }

  distance = target_position - current_position;
  if (distance < 0.0f)
  {
    distance = -distance;
  }
  duration_ms = distance * 1000.0f / speed_per_second;
  if (duration_ms > 4294967040.0f)
  {
    return 0U;
  }

  *run_time_ms = (uint32_t)(duration_ms + 0.5f);
  return 1U;
}
