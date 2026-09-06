#include <Wire.h>
#include "system_state.h"
#include "time_helpers.h"
#include "adc_hal.h"

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

  pinMode(LED_BUILTIN, OUTPUT);

  bool proceed = true;

  if(system_init() != STATE_OK){
    proceed = false;
  } else if (adc_init() != ADC_STATUS_OK){
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
  bool time_to_read_cell = false;

  if(system_get_loop_state(&loop_state) != STATE_OK){   // Create local copy of system state
    Serial.println("Error getting loop state");
    for(;;);
    // Handle error
  }

  if(!scheduler_led_flash(now, loop_state.main_led_flash_time, loop_state.main_led_on)){
    Serial.println("LED flash failure");
    for(;;);
    // Handle error
  }

  if(!scheduler_adc_health_check(now, loop_state.adc_function_check_time)){
    Serial.println("ADC health check scheduler failed");
    for(;;);
    // Handle error
  }

  switch (loop_state.current_state) {
    case FSM_START_UP:
      fsm_start_up();
      break;
    case FSM_DIVE_MODE:
      fsm_dive_mode();
      break;
    case FSM_READ_CELLS:
      fsm_read_cells();
      break;
    default:
      break;
  }
}

// 00 - WIP



// 01 - FSM handlers

void fsm_start_up(void){
  if(system_set_current_state(FSM_DIVE_MODE) != STATE_OK){
    Serial.println("Error at start up transition");
    for(;;);
    // Handle error
  }
}

void fsm_dive_mode(void){

}

void fsm_read_cells(void){

}


// 02 - Scheduler functions

bool scheduler_adc_health_check(const uint32_t now, const uint32_t elapsed_time){
  if(has_timer_elapsed(now, elapsed_time, FREQUENCY_ADC_CHECK_MS)){   // Check if ADC is online once a second
    hal_adc_status_t current_adc_status = adc_health_check();
    bool adc_online = (current_adc_status == ADC_STATUS_OK);

    if(adc_online){
      Serial.println("ADC online");
    } else {
      Serial.println("ADC offline");
    }

    if(system_set_adc_online(adc_online) != STATE_OK){
      Serial.println("Error writing adc_onine");
      return false;
    }
    system_set_adc_function_check_time(now);
  }
  return true;
}

bool scheduler_led_flash(const uint32_t now, const uint32_t elapsed_time, const bool system_led_state){
  if(has_timer_elapsed(now, elapsed_time, FREQUENCY_MAIN_LED_FLASH)){ // Turn main led on & off every 1s
    bool led_on = !system_led_state;
    digitalWrite(LED_BUILTIN, led_on);
    system_set_main_led_on(led_on);
    if(system_set_main_led_timer(now) != STATE_OK){
      Serial.println("Error writing main led timer");
      return false;
      // Handle error
    }
  }
  return true;
}