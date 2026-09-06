#include "adc_hal.h"

typedef struct{
  bool initialised;
} internal_state_t;

static internal_state_t state = {};


static bool is_powered(void);
static bool is_connected(void);



// Public API

hal_adc_status_t adc_init(void){
  if(state.initialised){
    return ADC_STATUS_OK;
  }

  state.initialised = true;
  return ADC_STATUS_OK;
}

hal_adc_status_t adc_health_check(void){
  if(is_powered() && is_connected()){
    return ADC_STATUS_OK;
  } else {
    return ADC_STATUS_HW_ERROR;
  }
}


// Private

static bool is_powered(void){

}

static bool is_connected(void){

}

