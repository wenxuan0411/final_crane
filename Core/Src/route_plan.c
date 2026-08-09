#include "route_plan.h"
#include <float.h>

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
/* Revision: raise the X path-speed validation limit from 15 to 35 rad/s. */
#define ROUTE_X_MAX_SPEED_RAD_S 35.0f
#define ROUTE_Y_MAX_SPEED_RPM   11000U
#define Y_SEGMENT(target_mm, speed_rpm) \
  {target_mm, speed_rpm, 0U, 0.0f}
#define Y_SEGMENT_AFTER_X(target_mm, speed_rpm, x_trigger_mm) \
  {target_mm, speed_rpm, 1U, x_trigger_mm}
#define X_ONLY_GROUP(x_target_mm, x_speed) \
  {1U, x_target_mm, x_speed, 0, 0U}
#define Y_ONLY_GROUP(y_list) \
  {0U, 0.0f, 0.0f, y_list, (uint8_t)ARRAY_SIZE(y_list)}
#define XY_GROUP(x_target_mm, x_speed, y_list) \
  {1U, x_target_mm, x_speed, y_list, (uint8_t)ARRAY_SIZE(y_list)}

/*
 * A motion group contains zero or one X motion and zero or more consecutive
 * Y motions. X and the first Y segment start together. Later Y segments start
 * as soon as the previous Y target is reached without restarting X, unless
 * Y_SEGMENT_AFTER_X is used to wait for an X feedback position first.
 *
 * X-only group: set y_segments = 0 and y_segment_count = 0U.
 * Y-only group: set x_enabled = 0U and provide one or more Y segments.
 *
 * Keep configured = 0U in route_table until all segments of that route
 * have been measured and filled in.
 */
static const YMotionSegment_t route_0_to_1_y[] =
{
  Y_SEGMENT(513.0f, 9000U),
};

static const MotionGroup_t route_0_to_1[] =
{
  XY_GROUP(2144.0f, 30.0f, route_0_to_1_y),
};

static const YMotionSegment_t route_0_to_3_y[] =
{
  Y_SEGMENT(230.0f, 9000U),
	Y_SEGMENT_AFTER_X(13.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_0_to_3[] =
{
  XY_GROUP(1884.0f, 30.0f, route_0_to_3_y),
};

static const YMotionSegment_t route_1_to_4_y[] =
{
  Y_SEGMENT(-230.0f, 9000U),
  Y_SEGMENT_AFTER_X(883.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_1_to_4[] =
{
  XY_GROUP(-1375.0f, 30.0f, route_1_to_4_y),
};

static const YMotionSegment_t route_1_to_5_y[] =
{
  Y_SEGMENT(-230.0f, 9000U),
  Y_SEGMENT_AFTER_X(408.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_1_to_5[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_1_to_5_y),
};

static const YMotionSegment_t route_1_to_6_y[] =
{
  Y_SEGMENT_AFTER_X(-230.0f, 9000U, 1293.0f),
  Y_SEGMENT_AFTER_X(10.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_1_to_6[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_1_to_6_y),
};

static const YMotionSegment_t route_1_to_7_y[] =
{
  Y_SEGMENT_AFTER_X(-230.0f, 9000U, 1293.0f),
	Y_SEGMENT_AFTER_X(-390.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_1_to_7[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_1_to_7_y),
};

static const YMotionSegment_t route_1_to_8_y[] =
{
	Y_SEGMENT(230.0f, 9000U),
  Y_SEGMENT_AFTER_X(-865.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_1_to_8[] =
{
  XY_GROUP(-1375.0f, 30.0f, route_1_to_8_y),
};

static const YMotionSegment_t route_2_to_4_y[] =
{
	Y_SEGMENT(-230.0f, 9000U),
  Y_SEGMENT_AFTER_X(883.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_2_to_4[] =
{
  XY_GROUP(-1375.0f, 30.0f, route_2_to_4_y),
};

static const YMotionSegment_t route_2_to_5_y[] =
{
  Y_SEGMENT_AFTER_X(408.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_2_to_5[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_2_to_5_y),
};

static const YMotionSegment_t route_2_to_6_y[] =
{
  Y_SEGMENT_AFTER_X(230.0f, 9000U, 1293.0f),
  Y_SEGMENT_AFTER_X(10.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_2_to_6[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_2_to_6_y),
};

static const YMotionSegment_t route_2_to_7_y[] =
{
  Y_SEGMENT(230.0f, 9000U),
  Y_SEGMENT_AFTER_X(-390.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_2_to_7[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_2_to_7_y),
};

static const YMotionSegment_t route_2_to_8_y[] =
{
  Y_SEGMENT(230.0f, 9000U),
  Y_SEGMENT_AFTER_X(-865.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_2_to_8[] =
{
  XY_GROUP(-1375.0f, 30.0f, route_2_to_8_y),
};

static const YMotionSegment_t route_3_to_4_y[] =
{
  Y_SEGMENT(-230.0f, 9000U),
  Y_SEGMENT_AFTER_X(883.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_3_to_4[] =
{
  XY_GROUP(-1375.0f, 30.0f, route_3_to_4_y),
};

static const YMotionSegment_t route_3_to_5_y[] =
{
  Y_SEGMENT(-230.0f, 9000U),
  Y_SEGMENT_AFTER_X(408.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_3_to_5[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_3_to_5_y),
};

static const YMotionSegment_t route_3_to_6_y[] =
{
	Y_SEGMENT(-230.0f, 9000U),
  Y_SEGMENT_AFTER_X(230.0f, 9000U, 1293.0f),
  Y_SEGMENT_AFTER_X(10.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_3_to_6[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_3_to_6_y),
};

static const YMotionSegment_t route_3_to_7_y[] =
{
  Y_SEGMENT(230.0f, 9000U),
  Y_SEGMENT_AFTER_X(-390.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_3_to_7[] =
{
  XY_GROUP(-1603.0f, 30.0f, route_3_to_7_y),
};

static const YMotionSegment_t route_3_to_8_y[] =
{
  Y_SEGMENT(230.0f, 9000U),
  Y_SEGMENT_AFTER_X(-865.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_3_to_8[] =
{
  XY_GROUP(-1375.0f, 30.0f, route_3_to_8_y),
};

static const YMotionSegment_t route_4_to_1_y[] =
{
  Y_SEGMENT_AFTER_X(513.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_4_to_1[] =
{
  XY_GROUP(2144.0f, 30.0f, route_4_to_1_y),
};

static const YMotionSegment_t route_4_to_2_y[] =
{
	Y_SEGMENT(220.0f, 9000U),
  Y_SEGMENT_AFTER_X(-487.0f, 10000U, -707.0f),
};

static const MotionGroup_t route_4_to_2[] =
{
  XY_GROUP(2144.0f, 30.0f, route_4_to_2_y),
};

static const YMotionSegment_t route_4_to_3_y[] =
{
	Y_SEGMENT(230.0f, 9000U),
  Y_SEGMENT_AFTER_X(13.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_4_to_3[] =
{
  XY_GROUP(1884.0f, 30.0f, route_4_to_3_y),
};

static const YMotionSegment_t route_5_to_1_y[] =
{
  Y_SEGMENT_AFTER_X(513.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_5_to_1[] =
{
  XY_GROUP(2144.0f, 30.0f, route_5_to_1_y),
};

static const YMotionSegment_t route_5_to_2_y[] =
{
  Y_SEGMENT_AFTER_X(-487.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_5_to_2[] =
{
  XY_GROUP(2144.0f, 30.0f, route_5_to_2_y),
};

static const YMotionSegment_t route_5_to_3_y[] =
{
  Y_SEGMENT_AFTER_X(13.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_5_to_3[] =
{
  XY_GROUP(1884.0f, 30.0f, route_5_to_3_y),
};

static const YMotionSegment_t route_6_to_1_y[] =
{
  Y_SEGMENT(513.0f, 9000U),
};

static const MotionGroup_t route_6_to_1[] =
{
  XY_GROUP(2144.0f, 30.0f, route_6_to_1_y),
};

static const YMotionSegment_t route_6_to_2_y[] =
{
  Y_SEGMENT(-487.0f, 9000U),
};

static const MotionGroup_t route_6_to_2[] =
{
  XY_GROUP(2144.0f, 30.0f, route_6_to_2_y),
};

static const YMotionSegment_t route_6_to_3_y[] =
{
  Y_SEGMENT(230.0f, 9000U),
  Y_SEGMENT_AFTER_X(13.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_6_to_3[] =
{
  XY_GROUP(1884.0f, 30.0f, route_6_to_3_y),
};

static const YMotionSegment_t route_7_to_1_y[] =
{
  Y_SEGMENT_AFTER_X(513.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_7_to_1[] =
{
  XY_GROUP(2144.0f, 30.0f, route_7_to_1_y),
};

static const YMotionSegment_t route_7_to_2_y[] =
{
  Y_SEGMENT_AFTER_X(-487.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_7_to_2[] =
{
  XY_GROUP(2144.0f, 30.0f, route_7_to_2_y),
};

static const YMotionSegment_t route_7_to_3_y[] =
{
  Y_SEGMENT_AFTER_X(13.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_7_to_3[] =
{
  XY_GROUP(1884.0f, 30.0f, route_7_to_3_y),
};

static const YMotionSegment_t route_8_to_1_y[] =
{
	Y_SEGMENT(-230.0f, 9000U),
  Y_SEGMENT_AFTER_X(513.0f, 9500U, -707.0f),
};

static const MotionGroup_t route_8_to_1[] =
{
  XY_GROUP(2144.0f, 30.0f, route_8_to_1_y),
};

static const YMotionSegment_t route_8_to_2_y[] =
{
  Y_SEGMENT_AFTER_X(-487.0f, 9000U, -707.0f),
};

static const MotionGroup_t route_8_to_2[] =
{
  XY_GROUP(2144.0f, 30.0f, route_8_to_2_y),
};

static const YMotionSegment_t route_8_to_3_y[] =
{
	Y_SEGMENT(-230.0f, 9000U),
  Y_SEGMENT_AFTER_X(13.0f, 9000U, 1293.0f),
};

static const MotionGroup_t route_8_to_3[] =
{
  XY_GROUP(1884.0f, 30.0f, route_8_to_3_y),
};

static const YMotionSegment_t route_4_to_0_y[] =
{
  Y_SEGMENT(250.0f, 9000U),
};

static const MotionGroup_t route_4_to_0[] =
{
  XY_GROUP(0.0f, 30.0f, route_4_to_0_y),
};

static const YMotionSegment_t route_5_to_0_y[] =
{
  Y_SEGMENT(250.0f, 9000U),
};

static const MotionGroup_t route_5_to_0[] =
{
  XY_GROUP(0.0f, 30.0f, route_5_to_0_y),
};

static const YMotionSegment_t route_6_to_0_y[] =
{
  Y_SEGMENT(250.0f, 9000U),
};

static const MotionGroup_t route_6_to_0[] =
{
  XY_GROUP(0.0f, 30.0f, route_6_to_0_y),
};

static const YMotionSegment_t route_7_to_0_y[] =
{
  Y_SEGMENT(-300.0f, 9000U),
};

static const MotionGroup_t route_7_to_0[] =
{
  XY_GROUP(0.0f, 30.0f, route_7_to_0_y),
};

static const YMotionSegment_t route_8_to_0_y[] =
{
  Y_SEGMENT(-300.0f, 9000U),
};

static const MotionGroup_t route_8_to_0[] =
{
  XY_GROUP(0.0f, 30.0f, route_8_to_0_y),
};

static const RoutePlan_t route_table[] =
{
	{0U, 1U, route_0_to_1, (uint8_t)ARRAY_SIZE(route_0_to_1), 1U},
  {0U, 3U, route_0_to_3, (uint8_t)ARRAY_SIZE(route_0_to_3), 1U},
  {1U, 4U, route_1_to_4, (uint8_t)ARRAY_SIZE(route_1_to_4), 1U},
  {1U, 5U, route_1_to_5, (uint8_t)ARRAY_SIZE(route_1_to_5), 1U},
  {1U, 6U, route_1_to_6, (uint8_t)ARRAY_SIZE(route_1_to_6), 1U},
  {1U, 7U, route_1_to_7, (uint8_t)ARRAY_SIZE(route_1_to_7), 1U},
  {1U, 8U, route_1_to_8, (uint8_t)ARRAY_SIZE(route_1_to_8), 1U},

  {2U, 4U, route_2_to_4, (uint8_t)ARRAY_SIZE(route_2_to_4), 1U},
  {2U, 5U, route_2_to_5, (uint8_t)ARRAY_SIZE(route_2_to_5), 1U},
  {2U, 6U, route_2_to_6, (uint8_t)ARRAY_SIZE(route_2_to_6), 1U},
  {2U, 7U, route_2_to_7, (uint8_t)ARRAY_SIZE(route_2_to_7), 1U},
  {2U, 8U, route_2_to_8, (uint8_t)ARRAY_SIZE(route_2_to_8), 1U},

  {3U, 4U, route_3_to_4, (uint8_t)ARRAY_SIZE(route_3_to_4), 1U},
  {3U, 5U, route_3_to_5, (uint8_t)ARRAY_SIZE(route_3_to_5), 1U},
  {3U, 6U, route_3_to_6, (uint8_t)ARRAY_SIZE(route_3_to_6), 1U},
  {3U, 7U, route_3_to_7, (uint8_t)ARRAY_SIZE(route_3_to_7), 1U},
  {3U, 8U, route_3_to_8, (uint8_t)ARRAY_SIZE(route_3_to_8), 1U},

  {4U, 1U, route_4_to_1, (uint8_t)ARRAY_SIZE(route_4_to_1), 1U},
  {4U, 2U, route_4_to_2, (uint8_t)ARRAY_SIZE(route_4_to_2), 1U},
  {4U, 3U, route_4_to_3, (uint8_t)ARRAY_SIZE(route_4_to_3), 1U},
  {5U, 1U, route_5_to_1, (uint8_t)ARRAY_SIZE(route_5_to_1), 1U},
  {5U, 2U, route_5_to_2, (uint8_t)ARRAY_SIZE(route_5_to_2), 1U},
  {5U, 3U, route_5_to_3, (uint8_t)ARRAY_SIZE(route_5_to_3), 1U},
  {6U, 1U, route_6_to_1, (uint8_t)ARRAY_SIZE(route_6_to_1), 1U},
  {6U, 2U, route_6_to_2, (uint8_t)ARRAY_SIZE(route_6_to_2), 1U},
  {6U, 3U, route_6_to_3, (uint8_t)ARRAY_SIZE(route_6_to_3), 1U},
  {7U, 1U, route_7_to_1, (uint8_t)ARRAY_SIZE(route_7_to_1), 1U},
  {7U, 2U, route_7_to_2, (uint8_t)ARRAY_SIZE(route_7_to_2), 1U},
  {7U, 3U, route_7_to_3, (uint8_t)ARRAY_SIZE(route_7_to_3), 1U},
  {8U, 1U, route_8_to_1, (uint8_t)ARRAY_SIZE(route_8_to_1), 1U},
  {8U, 2U, route_8_to_2, (uint8_t)ARRAY_SIZE(route_8_to_2), 1U},
  {8U, 3U, route_8_to_3, (uint8_t)ARRAY_SIZE(route_8_to_3), 1U},

  {4U, 0U, route_4_to_0, (uint8_t)ARRAY_SIZE(route_4_to_0), 1U},
  {5U, 0U, route_5_to_0, (uint8_t)ARRAY_SIZE(route_5_to_0), 1U},
  {6U, 0U, route_6_to_0, (uint8_t)ARRAY_SIZE(route_6_to_0), 1U},
  {7U, 0U, route_7_to_0, (uint8_t)ARRAY_SIZE(route_7_to_0), 1U},
  {8U, 0U, route_8_to_0, (uint8_t)ARRAY_SIZE(route_8_to_0), 1U},
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

uint8_t RoutePlan_ValidateGroup(const MotionGroup_t *group)
{
  uint8_t y_index;

  if ((group == 0) ||
      ((group->x_enabled == 0U) && (group->y_segment_count == 0U)) ||
      ((group->y_segments == 0) != (group->y_segment_count == 0U)))
  {
    return 0U;
  }

  if ((group->x_enabled != 0U) &&
      ((RoutePlan_IsFinite(group->x_target_position_mm) == 0U) ||
       (RoutePlan_IsFinite(group->x_speed_rad_s) == 0U) ||
       (group->x_speed_rad_s <= 0.0f) ||
       (group->x_speed_rad_s > ROUTE_X_MAX_SPEED_RAD_S)))
  {
    return 0U;
  }

  for (y_index = 0U; y_index < group->y_segment_count; y_index++)
  {
    const YMotionSegment_t *segment = &group->y_segments[y_index];

    if ((RoutePlan_IsFinite(segment->y_target_position_mm) == 0U) ||
        (segment->y_target_position_mm < ROUTE_Y_POSITION_MIN_MM) ||
        (segment->y_target_position_mm > ROUTE_Y_POSITION_MAX_MM) ||
        (segment->y_speed_rpm == 0U) ||
        (segment->y_speed_rpm > ROUTE_Y_MAX_SPEED_RPM) ||
        ((segment->wait_for_x_position != 0U) &&
         ((group->x_enabled == 0U) ||
          (RoutePlan_IsFinite(segment->x_trigger_position_mm) == 0U))))
    {
      return 0U;
    }
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
