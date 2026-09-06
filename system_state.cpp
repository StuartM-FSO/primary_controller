#include <cstddef>
#include <stdint.h>
#include "system_state.h"


static internal_state_t state = {};


// Public API

system_t system_init(void){
  if(state.initialised){
    return STATE_OK;
  }

  state.current_state = FSM_START_UP;
  state.main_led_flash_time = 0U;
  state.cell_read_time = 0U;
  state.main_led_om = false;
  state.initialised = true;
  return STATE_OK;
}

// Timers

system_t system_get_timers(internal_state_t *timers){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  timers->cell_read_time = state.cell_read_time;
  timers->main_led_flash_time = state.main_led_flash_time;
  return STATE_OK;
}

system_t system_set_cell_read_time(const uint32_t now){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  state.cell_read_time = now;
  return STATE_OK;
}

system_t system_set_main_led_timer(const uint32_t now){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  state.main_led_flash_time = now;
  return STATE_OK;
}

// State setters & getters

system_t system_get_current_state(fsm_state_t *current_state){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  if(current_state == NULL){
    return STATE_INVALID_PARAMETER;
  }

  *current_state = state.current_state;
  return STATE_OK;
}

system_t system_set_current_state(const fsm_state_t current_state){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  if((current_state == FSM_ZERO_COUNT) || (current_state >= FSM_END_COUNT)){
    return STATE_INVALID_PARAMETER;
  }
  state.current_state = current_state;
}

// Non-critical setters & getters

bool system_get_main_led_on(void){
  return state.main_led_om;
}

void system_set_main_led_on(bool led_on){
  state.main_led_om = led_on;
}



// Private