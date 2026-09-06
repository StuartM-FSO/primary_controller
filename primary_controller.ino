#include "system_state.h"
#include "time_helpers.h"

constexpr uint32_t FREQUENCY_CELL_READ_MS = 250U;
constexpr uint32_t FREQUENCY_MAIN_LED_FLASH = 1000U;

void setup() {
  Serial.begin(115200);
  while(!Serial){
    delay(1);
  }
  Serial.println("Starting...");

  pinMode(LED_BUILTIN, OUTPUT);

  bool proceed = true;

  if((system_init() != STATE_OK) && proceed){
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

  if(has_timer_elapsed(now, timers.cell_read_time, FREQUENCY_CELL_READ_MS)){  // Check if time to read cells
    Serial.println("+");
    time_to_read_cell = true;
    if(system_set_cell_read_time(now) != STATE_OK){
      Serial.println("Error setting cell read timer");
      for(;;);
      // Handle error
    }
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

}

void fsm_dive_mode(void){

}

void fsm_read_cells(void){

}
