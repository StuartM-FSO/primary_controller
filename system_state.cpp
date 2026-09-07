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
  state.adc_function_check_time = 0U;
  state.main_led_on = false;
  state.adc_online = false;
  state.display_changed = true;
  state.initialised = true;
  return STATE_OK;
}

// Setters & getters

system_t system_get_loop_state(internal_state_t *loop_state){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  loop_state->cell_read_time = state.cell_read_time;
  loop_state->main_led_flash_time = state.main_led_flash_time;
  loop_state->adc_function_check_time = state.adc_function_check_time;
  loop_state->current_state = state.current_state;
  loop_state->main_led_on = state.main_led_on;
  loop_state->adc_online = state.adc_online;
  loop_state->initialised = state.initialised;
  loop_state->display_changed = state.display_changed;
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

system_t system_set_adc_function_check_time(const uint32_t now){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  state.adc_function_check_time = now;
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
  return STATE_OK;
}

system_t system_set_adc_online(const bool adc_online){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  state.adc_online = adc_online;
  return STATE_OK;
}

system_t system_set_display_changed(const bool changed){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  state.display_changed = changed;
  return STATE_OK;
}

// Non-critical setters & getters

void system_set_main_led_on(bool led_on){
  state.main_led_on = led_on;
}



// Private