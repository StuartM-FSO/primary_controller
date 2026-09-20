#include <sys/_stdint.h>
#ifndef SHARED_H
#define SHARED_H

constexpr uint8_t THREE_CELLS = 3U;

typedef enum {
  SENSOR_0_REJECTED = 0U,
  SENSOR_1_REJECTED = 1U,
  SENSOR_2_REJECTED = 2U,
  SENSOR_ALL_VALID  = 3U,
  SENSOR_FAULT      = 4U,
  SENSOR_UNINITIALISED = 5U,
  SENSOR_COUNT_END // Do not add types beyond this
} sensor_vote_result_t;

typedef enum{
  OPSTATE_ZERO,
  OPSTATE_DIVEMODE,
  OPSTATE_DATAMODE,
  OPSTATE_FAILURE,
  OPSTATE_TESTMODE,
  OPSTATE_END_COUNT
} operational_state_t;

typedef struct{
  uint16_t ppo2_x1000[THREE_CELLS];
  bool latest_crc_ok;
  uint32_t latest_packet_received_id;
  operational_state_t operational_state;
  sensor_vote_result_t voted_cell;
} eccr_state_t;

#endif