#include <sys/_stdint.h>
#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>
#include <stdint.h>
#include "shared.h"


typedef enum{
  STATE_OK,
  STATE_INVALID_PARAMETER,
  STATE_UNINITIALISED,
  STATE_FAILED_FUNCTION_CALL,
  STATE_INVALID_CONDITION,
  STATE_OVERFLOW,
  STATE_READ_TOO_OLD,
  STATE_ADC_OFFLINE,
  STATE_TASK_NOT_SCHEDULED
} system_state_t;

typedef enum{
  FSM_ZERO_COUNT = 0,   // Do not use or add states before
  FSM_START_UP,
  FSM_DIVE_MODE,
  FSM_DATA_MODE,
  FSM_CALIBRATION_WAIT,
  FSM_CALIBRATION_WRITE,
  FSM_FAILURE_RECOVERABLE,
  FSM_FAILURE_HARD,
  FSM_END_COUNT         // Do not use or add states beyond
} fsm_state_t;

typedef enum{
  SYSTEM_LOW_OUTPUT_CELL,
  SYSTEM_HIGH_OUTPUT_CELL,
  SYSTEM_CELL_END_COUNT   // Do not use or add states beyond
} system_cell_type_t;

typedef struct{
  bool initialised;
  bool main_led_on;
  bool adc_online;
  bool display_requires_update;
  fsm_state_t current_state;
  uint16_t reference_reading[3U];
  system_cell_type_t cell_type;
  uint16_t battery_mv;
} internal_state_t;

typedef struct{
  uint32_t cell_read_ms;
  uint32_t main_led_flash_ms;
  uint32_t adc_function_check_ms;
  uint32_t calibration_button_pushed_ms;
  uint32_t read_battery_ms;
} system_scheduling_t;

typedef struct{
  uint16_t ppo2_x1000[THREE_CELLS];
  uint32_t timestamp_ms;
  sensor_vote_result_t voted_sensor;
  uint16_t voted_ppo2;
} ppo2_t;

system_state_t system_init(system_cell_type_t cell_type);

system_state_t system_get_loop_state(internal_state_t *local_state);
system_state_t system_get_timer_state(system_scheduling_t *local_state);
system_state_t system_set_cell_read_time(const uint32_t now);
system_state_t system_set_main_led_timer(const uint32_t now);
system_state_t system_set_adc_function_check_time(const uint32_t now);
system_state_t system_set_adc_online(const bool adc_online);
system_state_t system_set_current_state(const fsm_state_t current_state);
system_state_t system_set_main_led_on(bool led_on);
system_state_t system_set_calibration_button_pushed(const uint32_t now);
system_state_t system_set_reference_reading(uint16_t * const reference_reading);
system_state_t system_get_reference_reading(uint16_t * const reference_reading);
system_state_t system_set_voted(const sensor_vote_result_t voted_sensor, const uint16_t voted_ppo2);
system_state_t system_set_battery_read_time(const uint32_t now);
system_state_t system_set_battery_mv(const uint16_t battery_mv);

system_state_t system_display_has_been_updated(void);
system_state_t system_display_requires_update(void);

system_state_t system_get_ppo2(ppo2_t * const ppo2);
system_state_t system_set_ppo2(uint16_t * const ppo2_x1000);

#endif