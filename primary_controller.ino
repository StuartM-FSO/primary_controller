#include "system_state.h"
#include "time_helpers.h"
#include "adc_hal.h"

constexpr uint32_t FREQUENCY_CELL_READ_MS = 250U;
constexpr uint32_t FREQUENCY_MAIN_LED_FLASH = 1000U;
constexpr uint32_t FREQUENCY_ADC_CHECK_MS = 1000U;

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }
  Serial.println("Starting...");

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
  internal_state_t timers = {};
  bool time_to_read_cell = false;
  fsm_state_t current_state;

  if(system_get_current_state(&current_state) != STATE_OK){
    Serial.println("Error getting current state");
    for(;;);
    // Handle error
  }

  if(system_get_timers(&timers) != STATE_OK){
    Serial.println("Error getting timers");
    for(;;);
    // Handle error
  }

  if(has_timer_elapsed(now, timers.main_led_flash_time, FREQUENCY_MAIN_LED_FLASH)){ // Turn main led on & off every 1s
    bool led_on = !system_get_main_led_on();
    digitalWrite(LED_BUILTIN, led_on);
    system_set_main_led_on(led_on);
    if(system_set_main_led_timer(now) != STATE_OK){
      Serial.println("Error writing main led timer");
      for(;;);
      // Handle error
    }
  }

  if(has_timer_elapsed(now, timers.adc_function_check_time, FREQUENCY_ADC_CHECK_MS)){
    hal_adc_status_t current_adc_status = adc_health_check();

    if(current_adc_status != ADC_STATUS_OK){
      Serial.println("ADC offline");
    } else {
      Serial.println("ADC online");
    }
    system_set_adc_function_check_time(now);
  }

  switch (current_state) {
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
