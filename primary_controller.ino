#include <Wire.h>
#include "system_state.h"
#include "time_helpers.h"
#include "adc_hal.h"
#include "display_hal.h"
#include "gpio_hal.h"
#include "format_for_print.h"
#include "eeprom_hal.h"
#include "shared.h"

constexpr uint32_t INTERVAL_CELL_READ_MS = 1000U;
constexpr uint32_t INTERVAL_MAIN_LED_FLASH = 1000U;
constexpr uint32_t INTERVAL_ADC_CHECK_MS = 1000U;
constexpr uint32_t INTERVAL_CAL_WAIT_BEFORE_WRITE_MS = 3000U;
constexpr uint8_t THREE_CELLS = 3U;
constexpr uint16_t CALIBRATION_PPO2x1000 = 970U;
constexpr uint8_t MAXIMUM_ALLOWED_FAILED_ATTEMPTS = 10U;
constexpr uint16_t MAX_DEVIATION_FROM_SETPOINT = 100U;
constexpr uint16_t LOW_OUTPUT_CALIBRATION_ACCEPTABLE_MIN_MV = 33U; // Limit to be established through testing
constexpr uint16_t LOW_OUTPUT_CALIBRATION_ACCEPTABLE_MAX_MV = 76U; // Limit to be established through testing
constexpr uint16_t HIGH_OUTPUT_CALIBRATION_ACCEPTABLE_MIN_MV = 71U; // Limit to be established through testing
constexpr uint16_t HIGH_OUTPUT_CALIBRATION_ACCEPTABLE_MAX_MV = 143U; // Limit to be established through testing

constexpr uint8_t SCREEN_LINE_PPO2 = 0U;
constexpr uint8_t SCREEN_LINE_MV = 8U;

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }
  Serial.println("Starting...");

  Wire.begin();

  bool proceed = true;

  if(system_init(SYSTEM_LOW_OUTPUT_CELL) != STATE_OK){
    proceed = false;
  } else if (adc_init() != ADC_STATUS_OK){
    proceed = false;
  } else if(display_init() != DISPLAY_STATUS_OK){
    proceed = false;
  } else if(gpio_init() != GPIO_STATUS_OK){
    proceed = false;
  } else if(eeprom_init() != MEM_OK){
    proceed = false;
  } else {}

  if(!proceed){
    Serial.println("Start up failed");
    for(;;);
  }
}

void loop() {
  uint32_t now = millis();
  system_state_t result = STATE_UNINITIALISED;
  fsm_state_t current_state = FSM_FAILURE_RECOVERABLE;

  current_state = run_scheduled_tasks(now);

  switch (current_state) {
    case FSM_START_UP:
      result = fsm_start_up();
      break;
    case FSM_DIVE_MODE:
      result = fsm_dive_mode();
      break;
    case FSM_DATA_MODE:
      result = fsm_data_mode(now);
      break;
    case FSM_CALIBRATION_WAIT:
      result = fsm_calibration_wait(now);
      break;
    case FSM_CALIBRATION_WRITE:
      result = fsm_calibration_write();
      break;
    case FSM_FAILURE_RECOVERABLE:
      result = fsm_failure_recoverable();
      break;
    case FSM_FAILURE_HARD:
      result = fsm_failure_hard();
      break;
    default:
      result = STATE_INVALID_CONDITION;
      break;
  }

  if(result != STATE_OK){
    Serial.println("Loop failed");
    fsm_failure_recoverable();
    // Handle error
  }
}

// 00 - WIP

bool are_all_cells_in_range_for_calibration(uint16_t cells[]){
  uint16_t low_limit_mv = 0U;
  uint16_t high_limit_mv = 0U;
  uint16_t converted_mv = 0U;
  internal_state_t local_state = {};

  if(system_get_loop_state(&local_state) != STATE_OK){
    return false;
  }

  if(local_state.cell_type == SYSTEM_LOW_OUTPUT_CELL){
    low_limit_mv = LOW_OUTPUT_CALIBRATION_ACCEPTABLE_MIN_MV;
    high_limit_mv = LOW_OUTPUT_CALIBRATION_ACCEPTABLE_MAX_MV;
  } else {
    low_limit_mv = HIGH_OUTPUT_CALIBRATION_ACCEPTABLE_MIN_MV;
    high_limit_mv = HIGH_OUTPUT_CALIBRATION_ACCEPTABLE_MAX_MV;
  }

  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    converted_mv = adc_convert_raw_to_mV(cells[channel]);
    if((converted_mv < low_limit_mv) || (converted_mv > high_limit_mv)){
      return false;
    }
  }
  return true;
}

sensor_vote_result_t get_voted_sensor(const uint16_t cells_raw[], uint16_t *const voted_ppo2){
  if((voted_ppo2 == NULL) || (cells_raw == NULL)){
    return SENSOR_FAULT;
  }

  uint16_t readings[THREE_CELLS] = {0U};
  const uint8_t SENSOR_0 = 0U;
  const uint8_t SENSOR_1 = 1U;
  const uint8_t SENSOR_2 = 2U;
  const uint8_t AVERAGE_OF_3_SENSORS = 3U;
  const uint8_t AVERAGE_OF_2_SENSORS = 2U;
  uint16_t d01;
  uint16_t d02;
  uint16_t d12;

  for (uint8_t channel = 0U; channel < THREE_CELLS; channel++){    
    if(convert_raw_to_ppo2(cells_raw[channel], channel, &readings[channel]) != STATE_OK){
      return SENSOR_FAULT;
    }
  }
  d01 = diff_u16(readings[SENSOR_0], readings[SENSOR_1]);
  d02 = diff_u16(readings[SENSOR_0], readings[SENSOR_2]);
  d12 = diff_u16(readings[SENSOR_1], readings[SENSOR_2]);
  if ((d01 <= MAX_DEVIATION_FROM_SETPOINT) && (d02 <= MAX_DEVIATION_FROM_SETPOINT) && (d12 <= MAX_DEVIATION_FROM_SETPOINT)){
    *voted_ppo2 = (uint16_t)((readings[SENSOR_0] + readings[SENSOR_1] + readings[SENSOR_2]) / AVERAGE_OF_3_SENSORS);
    return SENSOR_ALL_VALID;
  }

  uint8_t pair_a;
  uint8_t pair_b;
  sensor_vote_result_t rejected;
  uint16_t min_deviation;

  pair_a = SENSOR_0;
  pair_b = SENSOR_1;
  rejected = SENSOR_2_REJECTED;
  min_deviation = d01;
  if(d02 < min_deviation){
    pair_a = SENSOR_0;
    pair_b = SENSOR_2;
    rejected = SENSOR_1_REJECTED;
    min_deviation = d02;
  }
  if(d12 < min_deviation){
    pair_a = SENSOR_1;
    pair_b = SENSOR_2;
    rejected = SENSOR_0_REJECTED;
    min_deviation = d12;
  }
  *voted_ppo2 = (uint16_t)((readings[pair_a] + readings[pair_b]) / AVERAGE_OF_2_SENSORS);
  return rejected;
}

static uint16_t diff_u16(const uint16_t a, const uint16_t b){
  return (a > b) ? (a - b) : (b - a);
}





// 01 - FSM handlers

system_state_t fsm_start_up(void){
  // NOTE fsm_start_up is WIP and will be expanded later
  // Currently just a placeholder for future development

  if(assign_cell_reference_readings() != STATE_OK){
    Serial.println("Failed at assign cell ref in start up");
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
    Serial.println("Error at start up transition");
    return STATE_FAILED_FUNCTION_CALL;
  }
  return STATE_OK;
}

system_state_t fsm_dive_mode(void){
  if(gpio_slide_switch_on() == SWITCH_ON){
    if(system_set_current_state(FSM_DATA_MODE) != STATE_OK){
      Serial.println("Error changing state in dive mode");
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  }
  return STATE_OK;
}

system_state_t fsm_data_mode(const uint32_t now){
  internal_state_t local_state = {};
  uint32_t time_stamp = 0U;

  if(system_get_loop_state(&local_state) != STATE_OK){
    Serial.println("Failed getting state, fsm_data_mode");
    return STATE_FAILED_FUNCTION_CALL;
  }
  if(!local_state.adc_online){
    display_clear();
    display_set_cursor(0, 0);
    display_println("ADC FAILED");
    display_update();
    return STATE_OK;
  }

  if(gpio_slide_switch_on() == SWITCH_OFF){
    if(screen_off() != STATE_OK){
      Serial.println("Error turning off screen in data mode");
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
      Serial.println("Error state transition data mode");
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  }

  if(screen_data_mode() != STATE_OK){
    Serial.println("data mode screen write failed");
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(gpio_momentary_pushed() == SWITCH_ON){
    Serial.println("Switching to cal mode");
    if(system_set_calibration_button_pushed(now) != STATE_OK){
      Serial.println("Error writing cal button timer");
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_current_state(FSM_CALIBRATION_WAIT) != STATE_OK){
      Serial.println("Error switching to cal mode, fsm_data_mode");
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  }

  return STATE_OK;
}

system_state_t fsm_calibration_wait(const uint32_t now){
  switchstate_t button = gpio_momentary_pushed();
  switchstate_t slider = gpio_slide_switch_on();
  system_scheduling_t local_timers = {};
  bool timed_out = false;

  if(system_get_timer_state(&local_timers) != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(slider == SWITCH_OFF){
    if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(screen_off() != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  } else if(slider == SWITCH_ON){
    // Do nothing
  } else {
    return STATE_INVALID_CONDITION;
  }

  timed_out = has_timer_elapsed(now, local_timers.calibration_button_pushed_ms, INTERVAL_CAL_WAIT_BEFORE_WRITE_MS);

  if(button == SWITCH_ON){
    uint32_t elapsed = now - local_timers.calibration_button_pushed_ms;
    if(screen_hold_button(elapsed) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(timed_out){
      if(system_set_current_state(FSM_CALIBRATION_WRITE) != STATE_OK){
        return STATE_FAILED_FUNCTION_CALL;
      }
    }
    return STATE_OK;
  } else if(button == SWITCH_OFF){
    if(system_set_current_state(FSM_DATA_MODE) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_display_changed(true) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  } else {
    return STATE_INVALID_CONDITION;
  }
}

system_state_t fsm_calibration_write(void){
  switchstate_t button = gpio_momentary_pushed();
  switchstate_t slider = gpio_slide_switch_on();
  uint16_t reference_reading[THREE_CELLS] = {};

  if(slider == SWITCH_OFF){
    if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(screen_off() != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  } else if (slider == SWITCH_ON){
    // Do nothing
  } else {
    return STATE_INVALID_CONDITION;
  }

  if(button == SWITCH_ON){
    if(screen_release_button() != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  } else if(button == SWITCH_OFF){
    Serial.println("Writing calibration");
    
    if(adc_get_last_good_cell_read(reference_reading) != ADC_STATUS_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }    

    if(eeprom_write_calibration(reference_reading) != MEM_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }

    if(system_set_reference_reading(reference_reading) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }

    if(system_set_current_state(FSM_DATA_MODE) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    
    if(system_set_display_changed(true) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    
    return STATE_OK;
  } else {
    return STATE_INVALID_CONDITION;
  }
}

system_state_t fsm_failure_recoverable(void){
  /* In this state the system will force a reset and try to recover. If the reset does not bring all systems back
  online then it will transition to FSM_FAILURE_HARD where it will remain in a safe state */

  // Replace code below with reset routine
  Serial.println("IN FSM_FAILURE_RECOVERABLE MODE");
  for(;;);
}

system_state_t fsm_failure_hard(){
  /* Remain in a safe state */
  Serial.println("FAILED IN SAFE STATE");
  for(;;);
}


// 02 - Scheduler functions

fsm_state_t run_scheduled_tasks(const uint32_t now){
  internal_state_t local_state = {};
  system_scheduling_t local_timers = {};
  uint8_t failed_attempts = 0U;

  if(system_get_loop_state(&local_state) != STATE_OK){   // Create local copy of system state
    Serial.println("Error getting loop state");
    return FSM_FAILURE_RECOVERABLE;
  }
  
  if(system_get_timer_state(&local_timers) != STATE_OK){
    Serial.println("Error getting timer state");
    return FSM_FAILURE_RECOVERABLE;
  }

  if(scheduler_led_flash(now, local_timers.main_led_flash_ms, local_state.main_led_on) != STATE_OK){
    Serial.println("LED flash failure");
    return FSM_FAILURE_RECOVERABLE;
  }

  if(scheduler_adc_health_check(now, local_timers.adc_function_check_ms) != STATE_OK){
    Serial.println("ADC health check scheduler failed");
    return FSM_FAILURE_RECOVERABLE;
  }

  if(scheduler_new_cell_read(now) != STATE_OK){
    if(adc_get_failed_attempts(&failed_attempts) != ADC_STATUS_OK){
      Serial.println("Failure getting failed_attempts");
      return FSM_FAILURE_RECOVERABLE;
    }
    if(failed_attempts > MAXIMUM_ALLOWED_FAILED_ATTEMPTS){
      Serial.println("Max failed attempts reached");
      return FSM_FAILURE_RECOVERABLE;
    }
  }

  return local_state.current_state;
}

system_state_t scheduler_new_cell_read(const uint32_t now){
  system_scheduling_t local_timers = {};
  internal_state_t local_state = {};
  uint16_t filtered_reading[THREE_CELLS] = {};
  uint16_t conversion_result_ppo2 = 0U;
  uint16_t conversion_result_mv = 0U;

  if(system_get_timer_state(&local_timers) != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(system_get_loop_state(&local_state) != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(has_timer_elapsed(now, local_timers.cell_read_ms, INTERVAL_CELL_READ_MS)){
    uint16_t cell_read_filtered[THREE_CELLS] = {};
    uint16_t voted_ppo2 = 0U;
    sensor_vote_result_t voted_cell = SENSOR_UNINITIALISED;

    if(system_set_cell_read_time(now) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(!local_state.adc_online){
      return STATE_ADC_OFFLINE;
    }
    if(adc_read_cells() != ADC_STATUS_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(adc_get_last_good_cell_read(cell_read_filtered) != ADC_STATUS_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    voted_cell = get_voted_sensor(cell_read_filtered, &voted_ppo2);
    if(system_set_voted(voted_cell, voted_ppo2) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    
    
    // DELETE FOR PRODUCTION
    if(adc_get_last_good_cell_read(filtered_reading) != ADC_STATUS_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
      if(convert_raw_to_ppo2(filtered_reading[channel], channel, &conversion_result_ppo2) != STATE_OK){
        return STATE_FAILED_FUNCTION_CALL;
      }
      Serial.print(conversion_result_ppo2);
      Serial.print(" ");
    }
    Serial.println();
    for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
      conversion_result_mv = adc_convert_raw_to_mV(filtered_reading[channel]);
      Serial.print(conversion_result_mv);
      Serial.print("mV ");
    }
    Serial.println();
    Serial.print("Vote result: ");
    Serial.print(voted_cell);
    Serial.print(" : ");
    Serial.println(voted_ppo2);
    // END OF SECTION

    
  }
  return STATE_OK;
}

system_state_t scheduler_adc_health_check(const uint32_t now, const uint32_t last_time){
  if(has_timer_elapsed(now, last_time, INTERVAL_ADC_CHECK_MS)){   // Check if ADC is online once a second
    hal_adc_status_t current_adc_status = adc_health_check();
    bool adc_online = (current_adc_status == ADC_STATUS_OK);

    if(adc_online){
      Serial.println("ADC online");
    } else {
      Serial.println("ADC offline");
    }

    if(system_set_adc_online(adc_online) != STATE_OK){
      Serial.println("Error writing adc_online");
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_adc_function_check_time(now) != STATE_OK){
      Serial.println("Error setting adc check time, scheduler_adc_health_check");
      return STATE_FAILED_FUNCTION_CALL;
    }
  }
  return STATE_OK;
}

system_state_t scheduler_led_flash(const uint32_t now, const uint32_t last_time, const bool system_led_state){
  if(has_timer_elapsed(now, last_time, INTERVAL_MAIN_LED_FLASH)){ // Turn main led on & off every 1s
    bool led_on = !system_led_state;
    if(system_set_main_led_on(led_on) != STATE_OK){
      Serial.println("Error set main led on, scheduler led flash");
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_main_led_timer(now) != STATE_OK){
      Serial.println("Error writing main led timer");
      return STATE_FAILED_FUNCTION_CALL;
    }
    digitalWrite(LED_BUILTIN, led_on);
  }
  return STATE_OK;
}

// 03 - Display

system_state_t screen_data_mode(void){
  display_font_size(1);
  display_set_colour(DISPLAY_WHITE, DISPLAY_BLACK);
  display_clear();

  if(screen_print_ppo2() != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }
  
  if(screen_print_mv() != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }
  
  if(display_update() != DISPLAY_STATUS_OK){
    Serial.println("Display update failed");
    return STATE_FAILED_FUNCTION_CALL;
  }
  return STATE_OK;
}

system_state_t screen_off(void){
  display_clear();
  if(display_update() != DISPLAY_STATUS_OK){
    Serial.println("Error turning screen off");
    return STATE_FAILED_FUNCTION_CALL;
  }
  if(system_set_display_changed(true) != STATE_OK){
    Serial.println("Error writing state in screen off");
    return STATE_FAILED_FUNCTION_CALL;
  }
  return STATE_OK;
}

system_state_t screen_hold_button(const uint32_t elapsed){
  uint32_t remaining = 0U;
  uint16_t count = 0U;
  char buffer[FORMATTING_INTEGER_STR_LEN];

  if(elapsed < INTERVAL_CAL_WAIT_BEFORE_WRITE_MS){
    remaining = INTERVAL_CAL_WAIT_BEFORE_WRITE_MS - elapsed;
  }
  count = (remaining + ONE_SECOND_MS - 1U) / ONE_SECOND_MS;
  format_integer_for_display(count, buffer);

  display_clear();
  display_set_cursor(0U, 0U);
  display_println("HOLD BUTTON");
  display_println("TO CALIBRATE");
  display_print(buffer);
  if(display_update() != DISPLAY_STATUS_OK){
    Serial.println("Error button warning");
    return STATE_FAILED_FUNCTION_CALL;
  }
  return STATE_OK;
}

system_state_t screen_release_button(void){
  display_clear();
  display_set_cursor(0U, 0U);
  display_println("RELEASE BUTTON");
  display_println("TO WRITE");
  if(display_update() != DISPLAY_STATUS_OK){
    Serial.println("Error button warning");
    return STATE_FAILED_FUNCTION_CALL;
  }
  return STATE_OK;
}

system_state_t screen_print_mv(void){
  char buffer[FORMATTING_INTEGER_STR_LEN];
  uint16_t reading_raw[THREE_CELLS];
  internal_state_t local_state = {};

  if(system_get_loop_state(&local_state) != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(adc_get_last_good_cell_read(reading_raw) != ADC_STATUS_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  display_set_cursor(0U, SCREEN_LINE_MV);
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    uint16_t reading_mv = adc_convert_raw_to_mV(reading_raw[channel]);

    if(local_state.voted_sensor == channel){
      display_set_colour(DISPLAY_BLACK, DISPLAY_WHITE);
    }
    format_integer_for_display(reading_mv, buffer);
    display_print(buffer);
    display_print("mV");
    display_set_colour(DISPLAY_WHITE, DISPLAY_BLACK);
    display_print(" ");
  }
  return STATE_OK;
}

system_state_t screen_print_ppo2(void){
  uint16_t current_read[THREE_CELLS] = {};
  uint16_t current_ppo2 = 0U;
  char buffer[FORMATTING_PPO2_STR_LEN] = {};
  internal_state_t local_state = {};

  if(system_get_loop_state(&local_state) != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(adc_get_last_good_cell_read(current_read) != ADC_STATUS_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  display_set_cursor(0, SCREEN_LINE_PPO2);
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    if(convert_raw_to_ppo2(current_read[channel], channel, &current_ppo2) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if((uint8_t)local_state.voted_sensor == channel){
      display_set_colour(DISPLAY_BLACK, DISPLAY_WHITE);
    }
    format_ppo2_to_text(current_ppo2, buffer);
    display_print(buffer);
    display_set_colour(DISPLAY_WHITE, DISPLAY_BLACK);
    display_print(" ");
  }
  return STATE_OK;
}

// 04 - General helpers

system_state_t assign_cell_reference_readings(void){
  uint16_t temp_reference_readings[THREE_CELLS];

  if(eeprom_read_calibration(temp_reference_readings) != MEM_OK){
    Serial.println("Cal read failed");
    return STATE_FAILED_FUNCTION_CALL;
  }
  if(system_set_reference_reading(temp_reference_readings) != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }
  Serial.println("Ref readings read from EEPROM");
  return STATE_OK;
}

system_state_t convert_raw_to_ppo2(const uint16_t raw, const uint8_t channel, uint16_t *const raw_converted_to_ppo2){
  uint16_t reference_reading[THREE_CELLS] = {};
  uint32_t ppo2 = 0U;
  const uint32_t scale = CALIBRATION_PPO2x1000;
  uint32_t temp = 0U;

  if(raw_converted_to_ppo2 == NULL){
    return STATE_INVALID_PARAMETER;
  }

  if(channel >= THREE_CELLS){
    return STATE_INVALID_PARAMETER;
  }

  if(system_get_reference_reading(reference_reading) != STATE_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }


  /* if(reference_value == 0U){
    return STATE_REQUIRES_CALIBRATION;
  } */
  temp = ((uint32_t)raw) * scale;
  ppo2 = temp / reference_reading[channel];
  if (ppo2 > UINT16_MAX) return STATE_OVERFLOW;
  *raw_converted_to_ppo2 = (uint16_t)ppo2;
  return STATE_OK;
}