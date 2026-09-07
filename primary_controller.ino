#include <Wire.h>
#include "system_state.h"
#include "time_helpers.h"
#include "adc_hal.h"
#include "display_hal.h"
#include "gpio_hal.h"

constexpr uint32_t FREQUENCY_CELL_READ_MS = 1000U;
constexpr uint32_t FREQUENCY_MAIN_LED_FLASH = 1000U;
constexpr uint32_t FREQUENCY_ADC_CHECK_MS = 1000U;

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
  }

  if(!proceed){
    Serial.println("Start up failed");
    for(;;);
  }
}

void loop() {
  uint32_t now = millis();
  internal_state_t loop_state = {};

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
      fsm_start_up();
      break;
    case FSM_DIVE_MODE:
      fsm_dive_mode(now);
      break;
    case FSM_READ_CELLS:
      fsm_read_cells();
      break;
    case FSM_DATA_MODE:
      fsm_data_mode(now);
      break;
    default:
      break;
  }
}

// 00 - WIP

system_t screen_data_mode(void){
  internal_state_t local_state = {};

  if(system_get_loop_state(&local_state) != STATE_OK){
    Serial.println("screen_data_mode error getting state");
    for(;;);
    // Handle error
  }

  if(local_state.display_changed){
    Serial.println("Screen printed once");
    display_font_size(1);
    display_set_colour(DISPLAY_WHITE, DISPLAY_BLACK);
    display_clear();
    display_set_cursor(0, 0);
    display_println("DISPLAY ON!!!");
    if(display_update() != DISPLAY_STATUS_OK){
      Serial.println("Display update failed");
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_display_changed(false) != STATE_OK){
      Serial.println("Error writing state, screen data mode");
      return STATE_FAILED_FUNCTION_CALL;
    }
  }
  return STATE_OK;
}

system_t screen_off(void){
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

// 01 - FSM handlers

void fsm_start_up(void){
  if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
    Serial.println("Error at start up transition");
    for(;;);
    // Handle error
  }
}

void fsm_dive_mode(const uint32_t now){
  internal_state_t local_state = {};
  bool cell_read_due = false;

  if(system_get_loop_state(&local_state) != STATE_OK){    // Create local copy of system state
    Serial.println("Error reading state, fsm_dive_mode");
    for(;;);
    // Handle error
  }

  if(scheduler_read_cells(now, local_state.cell_read_time, &cell_read_due) != STATE_OK){
    Serial.println("Error running read cells scheduler");
    for(;;);
    // Handle error
  }
  if(cell_read_due){
    return;
  }

  if(gpio_slide_switch_on() == SWITCH_ON){
    if(system_set_current_state(FSM_DATA_MODE) != STATE_OK){
      Serial.println("Error changing state in dive mode");
      for(;;);
      // Handle error
    }
    return;
  }
}

void fsm_read_cells(void){
  Serial.println("fsm_read_cells");
  if(gpio_slide_switch_on() == SWITCH_ON){
    if(system_set_current_state(FSM_DATA_MODE) != STATE_OK){
      Serial.println("Error changing state fsm_read_cells 1");
      for(;;);
      // Handle error
    }
  } else {
    if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
      Serial.println("Error changing state fsm_read_cells 2");
      for(;;);
      // Handle error
    }
  }
}

void fsm_data_mode(const uint32_t now){
  internal_state_t local_state = {};
  bool cell_read_due = false;

  if(system_get_loop_state(&local_state) != STATE_OK){
    Serial.println("Error getting local state in data mode");
    for(;;);
    // Handle error
  }

  if(scheduler_read_cells(now, local_state.cell_read_time, &cell_read_due) != STATE_OK){
    Serial.println("Error checking cell read time data mode");
    for(;;);
    // Handle error
  }
  if(cell_read_due){
    return;
  }

  if(gpio_slide_switch_on() == SWITCH_OFF){
    if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
      Serial.println("Error state transition data mode");
      for(;;);
      // Handle error
    }
    if(screen_off() != STATE_OK){
      Serial.println("Error turning off screen in data mode");
      for(;;);
      // Handle error
    }
    return;
  }

  if(screen_data_mode() != STATE_OK){
    Serial.println("data mode screen write failed");
    for(;;);
    // Handle error
  }
}


// 02 - Scheduler functions

system_t scheduler_adc_health_check(const uint32_t now, const uint32_t last_time){
  if(has_timer_elapsed(now, last_time, FREQUENCY_ADC_CHECK_MS)){   // Check if ADC is online once a second
    hal_adc_status_t current_adc_status = adc_health_check();
    bool adc_online = (current_adc_status == ADC_STATUS_OK);

    if(adc_online){
      Serial.println("ADC online");
    } else {
      Serial.println("ADC offline");
    }

    if(system_set_adc_online(adc_online) != STATE_OK){
      Serial.println("Error writing adc_onine");
      return STATE_FAILED_FUNCTION_CALL;
    }
    if(system_set_adc_function_check_time(now) != STATE_OK){
      Serial.println("Error setting adc check time, scheduler_adc_health_check");
      for(;;);
      // Handle error
    }
  }
  return STATE_OK;
}

system_t scheduler_led_flash(const uint32_t now, const uint32_t last_time, const bool system_led_state){
  if(has_timer_elapsed(now, last_time, FREQUENCY_MAIN_LED_FLASH)){ // Turn main led on & off every 1s
    bool led_on = !system_led_state;
    digitalWrite(LED_BUILTIN, led_on);
    if(system_set_main_led_on(led_on) != STATE_OK){
      Serial.println("Error set main led on, scheduler led flash");
      for(;;);
      // Handle error
    }
    if(system_set_main_led_timer(now) != STATE_OK){
      Serial.println("Error writing main led timer");
      return STATE_FAILED_FUNCTION_CALL;
      // Handle error
    }
  }
  return STATE_OK;
}

system_t scheduler_read_cells(const uint32_t now, const uint32_t last_time, bool * const cell_read_due){
  if(cell_read_due == NULL){
    return STATE_INVALID_PARAMETER;
  }
  if(has_timer_elapsed(now, last_time, FREQUENCY_CELL_READ_MS)){
    if(system_set_current_state(FSM_READ_CELLS) != STATE_OK){
      Serial.println("Error setting state, scheduler read cells");
      for(;;);
      // Handle error
    }
    if(system_set_cell_read_time(now) != STATE_OK){
      Serial.println("Error set cell read time, scheduler read cells");
      for(;;);
      // Handle error
    }
    *cell_read_due = true;
  } else {
    *cell_read_due = false;
  }
  return STATE_OK;
}

// 03 - Display

