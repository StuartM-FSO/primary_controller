#include <Wire.h>
#include "system_state.h"
#include "time_helpers.h"
#include "adc_hal.h"
#include "display_hal.h"
#include "gpio_hal.h"
#include "format_for_print.h"
#include "eeprom_hal.h"

constexpr uint32_t INTERVAL_CELL_READ_MS = 1000U;
constexpr uint32_t INTERVAL_MAIN_LED_FLASH = 1000U;
constexpr uint32_t INTERVAL_ADC_CHECK_MS = 1000U;
constexpr uint32_t INTERVAL_CAL_WAIT_BEFORE_WRITE_MS = 7000U;
constexpr uint8_t THREE_CELLS = 3U;
constexpr uint16_t CALIBRATION_PPO2x1000 = 970U;

// To be moved to display protocol library
constexpr uint8_t SCREEN_LINE_PPO2 = 0U;
constexpr uint8_t SCREEN_LINE_MV = 8U;

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }
  Serial.println("Starting...");

  Wire.begin();

  //pinMode(LED_BUILTIN, OUTPUT);

  bool proceed = true;

  if(system_init() != STATE_OK){
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
  internal_state_t loop_state = {};
  system_state_t result = STATE_UNINITIALISED;

  if(system_get_loop_state(&loop_state) != STATE_OK){   // Create local copy of system state
    Serial.println("Error getting loop state");
    for(;;);
    // Handle error
  }

  if(scheduler_led_flash(now, loop_state.main_led_flash_time, loop_state.main_led_on) != STATE_OK){
    Serial.println("LED flash failure");
    for(;;);
    // Handle error
  }

  if(scheduler_adc_health_check(now, loop_state.adc_function_check_time) != STATE_OK){
    Serial.println("ADC health check scheduler failed");
    for(;;);
    // Handle error
  }

  switch (loop_state.current_state) {
    case FSM_START_UP:
      result = fsm_start_up();
      break;
    case FSM_DIVE_MODE:
      result = fsm_dive_mode(now);
      break;
    case FSM_READ_CELLS:
      result = fsm_read_cells();
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
    default:
      result = STATE_INVALID_CONDITION;
      break;
  }

  if(result != STATE_OK){
    Serial.println("Loop failed");
    for(;;);
    // Handle error
  }
}

// 00 - WIP

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

void debug_print_ppo2(void){
  uint16_t current_read[THREE_CELLS] = {};

  adc_get_last_good_cell_read(current_read);
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    uint16_t ppo2 = 0U;

    convert_raw_to_ppo2(current_read[channel], channel, &ppo2);
    Serial.print(ppo2);
    Serial.print(" : ");
  }
  Serial.println();
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

system_state_t fsm_dive_mode(const uint32_t now){
  internal_state_t local_state = {};
  bool cell_read_due = false;

  if(system_get_loop_state(&local_state) != STATE_OK){    // Create local copy of system state
    Serial.println("Error reading state, fsm_dive_mode");
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(scheduler_read_cells(now, local_state.cell_read_time, &cell_read_due) != STATE_OK){
    Serial.println("Error running read cells scheduler");
    return STATE_FAILED_FUNCTION_CALL;
  }
  if(cell_read_due){
    return STATE_OK;
  }

  if(gpio_slide_switch_on() == SWITCH_ON){
    if(system_set_current_state(FSM_DATA_MODE) != STATE_OK){
      Serial.println("Error changing state in dive mode");
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  }
  return STATE_OK;
}

system_state_t fsm_read_cells(void){
  switchstate_t slider = gpio_slide_switch_on();
  uint16_t filtered_reading[THREE_CELLS] = {};
  uint16_t reading_mv = 0U;

  Serial.println("fsm_read_cells");

  if(adc_read_cells() != ADC_STATUS_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(adc_get_last_good_cell_read(filtered_reading) != ADC_STATUS_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    reading_mv = adc_convert_raw_to_mV(filtered_reading[channel]);
    Serial.print(reading_mv);
    Serial.print("mV ");
  }
  Serial.println();

  if(slider == SWITCH_ON){
    if(system_set_current_state(FSM_DATA_MODE) != STATE_OK){
      Serial.println("Error changing state fsm_read_cells 1");
      return STATE_FAILED_FUNCTION_CALL;
    }
  } else if(slider == SWITCH_OFF){
    if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
      Serial.println("Error changing state fsm_read_cells 2");
      return STATE_FAILED_FUNCTION_CALL;
    }
  } else {
    return STATE_INVALID_CONDITION;
  }
  debug_print_ppo2();
  return STATE_OK;
}

system_state_t fsm_data_mode(const uint32_t now){
  internal_state_t local_state = {};
  bool cell_read_due = false;

  if(system_get_loop_state(&local_state) != STATE_OK){
    Serial.println("Error getting local state in data mode");
    return STATE_FAILED_FUNCTION_CALL;
  }

  if(scheduler_read_cells(now, local_state.cell_read_time, &cell_read_due) != STATE_OK){
    Serial.println("Error checking cell read time data mode");
    return STATE_FAILED_FUNCTION_CALL;
  }
  if(cell_read_due){
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
  internal_state_t local_state = {};
  bool timed_out = false;

  if(system_get_loop_state(&local_state) != STATE_OK){
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

  timed_out = has_timer_elapsed(now, local_state.calibration_button_pushed, INTERVAL_CAL_WAIT_BEFORE_WRITE_MS);

  if(button == SWITCH_ON){
    uint32_t elapsed = now - local_state.calibration_button_pushed;
    if(screen_hold_button(elapsed) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(timed_out){
      if(system_set_current_state(FSM_CALIBRATION_WRITE) != STATE_OK){
        return STATE_FAILED_FUNCTION_CALL;
      }
    }
    return STATE_OK;
  } else {
    if(system_set_current_state(FSM_DATA_MODE) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_display_changed(true) != STATE_OK){
      return STATE_FAILED_FUNCTION_CALL;
    }
    return STATE_OK;
  }
  return STATE_OK;
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

    system_set_reference_reading(reference_reading);

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


// 02 - Scheduler functions

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

system_state_t scheduler_read_cells(const uint32_t now, const uint32_t last_time, bool * const cell_read_due){
  if(cell_read_due == NULL){
    return STATE_INVALID_PARAMETER;
  }
  if(has_timer_elapsed(now, last_time, INTERVAL_CELL_READ_MS)){
    if(system_set_current_state(FSM_READ_CELLS) != STATE_OK){
      Serial.println("Error setting state, scheduler read cells");
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_cell_read_time(now) != STATE_OK){
      Serial.println("Error set cell read time, scheduler read cells");
      return STATE_FAILED_FUNCTION_CALL;
    }
    *cell_read_due = true;
  } else {
    *cell_read_due = false;
  }
  return STATE_OK;
}

// 03 - Display

system_state_t screen_data_mode(void){
  internal_state_t local_state = {};

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
  uint16_t count = (uint16_t)((INTERVAL_CAL_WAIT_BEFORE_WRITE_MS + ONE_SECOND_MS - elapsed) / ONE_SECOND_MS);
  char buffer[FORMATTING_INTEGER_STR_LEN];

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

  if(adc_get_last_good_cell_read(reading_raw) != ADC_STATUS_OK){
    return STATE_FAILED_FUNCTION_CALL;
  }

  display_set_cursor(0U, SCREEN_LINE_MV);
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    uint16_t reading_mv = adc_convert_raw_to_mV(reading_raw[channel]);

    format_integer_for_display(reading_mv, buffer);
    display_print(buffer);
    display_print("mV ");
  }
  return STATE_OK;
}

system_state_t screen_print_ppo2(void){
  display_set_cursor(0, SCREEN_LINE_PPO2);
  display_println("DISPLAY ON!!!");
  return STATE_OK;
}