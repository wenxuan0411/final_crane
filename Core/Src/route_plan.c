#include "route_plan.h"

#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))

/*
 * Route template format:
 * { X speed (rad/s), X time (ms), Y speed (rpm), Y time (ms), pause (ms) }
 *
 * Keep configured = 0U in route_table until all segments of that route
 * have been measured and filled in.
 */

static const MotionSegment_t route_1_to_4[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_1_to_5[] =
{
  {-10.0f, 4203U, 3000, 4000U, 0U},
  {-15.0f, 6223U, -3600, 3800U, 0U},
};

static const MotionSegment_t route_1_to_6[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_1_to_7[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_1_to_8[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_2_to_4[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_2_to_5[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_2_to_6[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_2_to_7[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_2_to_8[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_3_to_4[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_3_to_5[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_3_to_6[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_3_to_7[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_3_to_8[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_4_to_1[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_4_to_2[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_4_to_3[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_5_to_1[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_5_to_2[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_5_to_3[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_6_to_1[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_6_to_2[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_6_to_3[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_7_to_1[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_7_to_2[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_7_to_3[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_8_to_1[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_8_to_2[] =
{
  {0.0f, 0U, 0, 0U, 0U},
};

static const MotionSegment_t route_8_to_3[] =
{
  {0.0f, 0U, 0, 0U, 0U},
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
