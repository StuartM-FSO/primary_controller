#include "api/Common.h"
#include <sys/_stdint.h>
#include <cstddef>
#include <stdint.h>
#include "system_state.h"



static internal_state_t state = {};
static system_scheduling_t timer_state = {};
static ppo2_t last_read_ppo2 = {};



// Public API

system_state_t system_init(system_cell_type_t cell_type){
  if(state.initialised){
    return STATE_OK;
  }
  if(cell_type >= SYSTEM_CELL_END_COUNT){
    return STATE_INVALID_PARAMETER;
  }

  state.current_state = FSM_START_UP;
  state.main_led_on = false;
  state.adc_online = false;
  state.display_requires_update = true;
  state.battery_mv = 0U;

  timer_state.adc_function_check_ms = 0U;
  timer_state.calibration_button_pushed_ms;
  timer_state.cell_read_ms = 0U;
  timer_state.main_led_flash_ms = 0U;
  timer_state.read_battery_ms = 0U;

  last_read_ppo2.timestamp_ms = 0U;
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    last_read_ppo2.ppo2_x1000[channel] = 0U;
  }
  last_read_ppo2.voted_sensor = SENSOR_UNINITIALISED;
  last_read_ppo2.voted_ppo2 = 0U;

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
  local_state->read_battery_ms = timer_state.read_battery_ms;
  return STATE_OK;
}

system_state_t system_get_loop_state(internal_state_t *local_state){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }

  local_state->current_state = state.current_state;
  local_state->adc_online = state.adc_online;
  local_state->display_requires_update = state.display_requires_update;
  local_state->main_led_on = state.main_led_on;
  local_state->cell_type = state.cell_type;
  local_state->battery_mv = state.battery_mv;
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

system_state_t system_set_voted(const sensor_vote_result_t voted_sensor, const uint16_t voted_ppo2){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  if(voted_sensor >= SENSOR_COUNT_END){
    return STATE_INVALID_PARAMETER;
  }

  last_read_ppo2.voted_ppo2 = voted_ppo2;
  last_read_ppo2.voted_sensor = voted_sensor;
  return STATE_OK;
}

system_state_t system_display_has_been_updated(void){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  state.display_requires_update = false;
  return STATE_OK;
}

system_state_t system_display_requires_update(void){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  state.display_requires_update = true;
  return STATE_OK;
}

system_state_t system_set_battery_read_time(const uint32_t now){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  timer_state.read_battery_ms = now;
  return STATE_OK;
}

system_state_t system_set_battery_mv(const uint16_t battery_mv){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  state.battery_mv = battery_mv;
  return STATE_OK;
}

system_state_t system_get_ppo2(ppo2_t * const ppo2){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  if(ppo2 == NULL){
    return STATE_INVALID_PARAMETER;
  }
  ppo2->timestamp_ms = last_read_ppo2.timestamp_ms;
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    ppo2->ppo2_x1000[channel] = last_read_ppo2.ppo2_x1000[channel];
  }
  ppo2->voted_sensor = last_read_ppo2.voted_sensor;
  ppo2->voted_ppo2 = last_read_ppo2.voted_ppo2;
  return STATE_OK;
}

system_state_t system_set_ppo2(uint16_t * const ppo2_x1000){
  if(!state.initialised){
    return STATE_UNINITIALISED;
  }
  last_read_ppo2.timestamp_ms = millis();
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    last_read_ppo2.ppo2_x1000[channel] = ppo2_x1000[channel];
  }
  return STATE_OK;
}


// Private