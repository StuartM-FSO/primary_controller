#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>
#include <stdint.h>

typedef enum{
  STATE_OK,
  STATE_INVALID_PARAMETER,
  STATE_UNINITIALISED
} system_t;

typedef enum{
  FSM_ZERO_COUNT = 0,   // Do not use or add states before
  FSM_START_UP,
  FSM_DIVE_MODE,
  FSM_READ_CELLS,
  FSM_END_COUNT         // Do not use or add states beyond
} fsm_state_t;

typedef struct{
  bool initialised;
  bool main_led_om;

  fsm_state_t current_state;

  uint32_t cell_read_time;
  uint32_t main_led_flash_time;
} internal_state_t;

system_t system_init(void);

system_t system_get_timers(internal_state_t *timers);
system_t system_set_cell_read_time(const uint32_t now);
system_t system_set_main_led_timer(const uint32_t now);

system_t system_get_current_state(fsm_state_t *current_state);
system_t system_set_current_state(const fsm_state_t current_state);

// Non critical setters & getters

bool system_get_main_led_on(void);
void system_set_main_led_on(bool led_on);

#endif