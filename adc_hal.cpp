#include <stdint.h>
// NOTES
//  1 - HAL assumes that Wire.h has been included in main sketch and Wire.begin() has been called

#include <Adafruit_ADS1X15.h>
#include "adc_hal.h"

constexpr uint8_t TRANSMISSION_OK = 0U;

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
  return true;
}

static bool is_connected(void){
  uint8_t result = 99U;

  Wire.beginTransmission(0x48);
  result = Wire.endTransmission();
  return (result == TRANSMISSION_OK);
}

