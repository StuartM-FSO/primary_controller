#include <ctime>
#include "IRQManager.h"
#include <sys/_stdint.h>
#include <cstddef>
#include <stdint.h>
#include "system_state.h"


static internal_state_t state = {};

static system_scheduling_t timer_state = {};


// Public API

system_state_t system_init(void){
  if(state.initialised){
    return STATE_OK;
  }

  state.current_state = FSM_START_UP;
  state.main_led_on = false;
  state.adc_online = false;
  state.display_changed = true;

  timer_state.adc_function_check_ms = 0U;
  timer_state.calibration_button_pushed_ms;
  timer_state.cell_read_ms = 0U;
  timer_state.main_led_flash_ms = 0U;

  state.initialised = true;
  return STATE_OK;
}

// Setters & getters

system_state_t system_get_timer_state(system_scheduling_t *local_state){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  local_state->adc_function_check_ms = timer_state.adc_function_check_ms;
  local_state->calibration_button_pushed_ms = timer_state.calibration_button_pushed_ms;
  local_state->cell_read_ms = timer_state.cell_read_ms;
  local_state->main_led_flash_ms = timer_state.main_led_flash_ms;
  return STATE_OK;
}

system_state_t system_get_loop_state(internal_state_t *local_state){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  local_state->current_state = state.current_state;
  local_state->adc_online = state.adc_online;
  local_state->display_changed = state.display_changed;
  local_state->main_led_on = state.main_led_on;
  return STATE_OK;
}

system_state_t system_set_cell_read_time(const uint32_t now){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  timer_state.cell_read_ms = now;
  return STATE_OK;
}

system_state_t system_set_main_led_timer(const uint32_t now){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  timer_state.main_led_flash_ms = now;
  return STATE_OK;
}

system_state_t system_set_adc_function_check_time(const uint32_t now){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  timer_state.adc_function_check_ms = now;
  return STATE_OK;
}

system_state_t system_set_current_state(const fsm_state_t current_state){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  if((current_state == FSM_ZERO_COUNT) || (current_state >= FSM_END_COUNT)){
    return STATE_INVALID_PARAMETER;
  }
  state.current_state = current_state;
  return STATE_OK;
}

system_state_t system_set_adc_online(const bool adc_online){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  state.adc_online = adc_online;
  return STATE_OK;
}

system_state_t system_set_display_changed(const bool changed){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  state.display_changed = changed;
  return STATE_OK;
}

system_state_t system_set_main_led_on(bool led_on){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  state.main_led_on = led_on;
  return STATE_OK;
}

system_state_t system_set_calibration_button_pushed(const uint32_t now){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  timer_state.calibration_button_pushed_ms = now;
  return STATE_OK;
}

system_state_t system_set_reference_reading(uint16_t * const reference_reading){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  if(reference_reading == NULL){
    return STATE_INVALID_PARAMETER;
  }
  for(uint8_t channel = 0U; channel < 3U; channel++){
    state.reference_reading[channel] = reference_reading[channel];
  }
  return STATE_OK;
}

system_state_t system_get_reference_reading(uint16_t * const reference_reading){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  if(reference_reading == NULL){
    return STATE_INVALID_PARAMETER;
  }

  for(uint8_t channel = 0U; channel < 3U; channel++){
    reference_reading[channel] = state.reference_reading[channel];
  }
  return STATE_OK;
}



// Private