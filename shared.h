#ifndef SHARED_H
#define SHARED_H

typedef enum {
  SENSOR_0_REJECTED = 0U,
  SENSOR_1_REJECTED = 1U,
  SENSOR_2_REJECTED = 2U,
  SENSOR_ALL_VALID  = 3U,
  SENSOR_FAULT      = 4U,
  SENSOR_UNINITIALISED = 5U,
  SENSOR_COUNT_END // Do not add types beyond this
} sensor_vote_result_t;

#endif