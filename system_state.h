#include <sys/_stdint.h>
#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>
#include <stdint.h>

typedef enum{
  STATE_OK,
  STATE_INVALID_PARAMETER,
  STATE_UNINITIALISED,
  STATE_FAILED_FUNCTION_CALL,
  STATE_INVALID_CONDITION
} system_state_t;

typedef enum{
  FSM_ZERO_COUNT = 0,   // Do not use or add states before
  FSM_START_UP,
  FSM_DIVE_MODE,
  FSM_READ_CELLS,
  FSM_DATA_MODE,
  FSM_CALIBRATION_MODE,
  FSM_END_COUNT         // Do not use or add states beyond
} fsm_state_t;

typedef struct{
  bool initialised;
  bool main_led_on;
  bool adc_online;
  bool display_changed;

  fsm_state_t current_state;

  uint32_t cell_read_time;
  uint32_t main_led_flash_time;
  uint32_t adc_function_check_time;
} internal_state_t;

system_state_t system_init(void);

system_state_t system_get_loop_state(internal_state_t *loop_state);
system_state_t system_set_cell_read_time(const uint32_t now);
system_state_t system_set_main_led_timer(const uint32_t now);
system_state_t system_set_adc_function_check_time(const uint32_t now);
system_state_t system_set_adc_online(const bool adc_online);
system_state_t system_set_current_state(const fsm_state_t current_state);
system_state_t system_set_display_changed(const bool changed);
system_state_t system_set_main_led_on(bool led_on);

#endif