#include <stdint.h>
// NOTES
//  1 - HAL assumes that Wire.h has been included in main sketch and Wire.begin() has been called

#include <Adafruit_ADS1X15.h>
#include "adc_hal.h"

static constexpr uint8_t TRANSMISSION_OK = 0U;
static constexpr uint8_t ADC_POWER_READ_PIN = D2;
static const uint16_t ADC_POWER_MAX_RAW = 1023U;
static const uint16_t ADC_POWER_MAX_MV = 5000U;
static const uint16_t ADC_MINIMUM_POWER_READING_MV = 2500; 


typedef struct{
  bool initialised;
} internal_state_t;

static internal_state_t state = {};


static bool is_powered(void);
static bool is_connected(void);
static bool map_non_arduino(const uint16_t base_value, uint16_t *scaled_value,
    const uint16_t in_min, const uint16_t in_max, const uint16_t out_min, const uint16_t out_max);




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

hal_adc_status_t adc_get_raw_reading(uint16_t * const raw_reading, const uint8_t channel){
    const uint8_t MAX_SAMPLES = 3;                            // System will never have anything other than 3 sensors connected to operate
    const uint8_t MEDIAN_SAMPLE_NUMBER = MAX_SAMPLES / 2;     // so the median will always be [1]
    uint16_t reading = 0;
    uint16_t sample[MAX_SAMPLES];
    
    if(!current_state.adc_initialised){
        return ADC_STATUS_NOT_INITIALIZED;
    }
    if(raw_reading == NULL){
        return ADC_STATUS_INVALID_PARAMETER;
    }
    if(channel >= THREE_CELLS){
        return ADC_STATUS_INVALID_PARAMETER;
    }
    if(!power_check_multiple()){
        current_state.adc_initialised = false;
        return ADC_STATUS_HW_ERROR;
    }
    if((millis() - current_state.last_function_check_time) > MAX_INTERVAL_FUNCTION_CHECK_MS){
        current_state.last_function_check_time = millis();
        if(!connected_check_multiple()){
            current_state.adc_initialised = false;
            return ADC_STATUS_HW_ERROR;
        }
    }
    for(uint8_t sample_number = 0U; sample_number < MAX_SAMPLES; sample_number++){
        if(read_sensor(&reading, channel) != ADC_STATUS_OK){
            current_state.adc_initialised = false;
            return ADC_STATUS_HW_ERROR;
        }
        // Add reading validity check here, if passes then write to array
        sample[sample_number] = reading;
    }
    if(sort_values(sample) != true){
        return ADC_STATUS_INVALID_PARAMETER;
    }
    *raw_reading = sample[MEDIAN_SAMPLE_NUMBER];
    return ADC_STATUS_OK;
}


// Private

static bool is_powered(void){
   uint16_t reading_raw = 0;
    uint16_t reading_mv = 0;

    reading_raw = analogRead(ADC_POWER_READ_PIN);
    if (map_non_arduino(reading_raw, &reading_mv, 0, ADC_POWER_MAX_RAW, 0, ADC_POWER_MAX_MV) == false){
        return false;
    }
    return (reading_mv >= ADC_MINIMUM_POWER_READING_MV);
}

static bool is_connected(void){
  uint8_t result = 99U;

  Wire.beginTransmission(0x48);
  result = Wire.endTransmission();
  return (result == TRANSMISSION_OK);
}

static bool map_non_arduino(const uint16_t base_value, uint16_t * const scaled_value,
    const uint16_t in_min, const uint16_t in_max, const uint16_t out_min, const uint16_t out_max)
{
    uint32_t numerator;
    uint32_t denominator;
    uint32_t result;
    uint16_t working_value = base_value;

    if (scaled_value == NULL) {
        return false;
    }
    if (in_min == in_max) {
        return false;
    }
    if (out_max < out_min) return false;
    if (base_value < in_min) {
        working_value = in_min;
    } else if (base_value > in_max) {
        working_value = in_max;
    }
    numerator = (uint32_t)(working_value - in_min) * (uint32_t)(out_max - out_min);
    denominator = (uint32_t)(in_max - in_min);
    result = numerator / denominator + out_min;
    *scaled_value = (uint16_t)result;
    return true;
}
