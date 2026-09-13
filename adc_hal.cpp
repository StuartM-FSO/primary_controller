#include <cstddef>
#include "api/Common.h"
#include <sys/_stdint.h>
#include <stdint.h>
// NOTES
//  1 - HAL assumes that Wire.h has been included in main sketch and Wire.begin() has been called

#include <Adafruit_ADS1X15.h>
#include "adc_hal.h"

static constexpr uint8_t TRANSMISSION_OK = 0U;
static constexpr uint8_t ADC_POWER_READ_PIN = D2;
static constexpr uint8_t THREE_CELLS = 3U;
static constexpr uint32_t MAX_INTERVAL_FUNCTION_CHECK_MS = 250;
static constexpr uint32_t MAX_WAIT_TIME_MS = 50;
static constexpr uint8_t ADC_ADDRESS = 0x48;
static constexpr uint8_t MAX_SAMPLES = 3U;
static constexpr uint8_t MEDIAN_SAMPLE_NUMBER = MAX_SAMPLES / 2;
static constexpr int32_t ADS1115_FULL_SCALE_MV = 256;
static constexpr int32_t ADS1115_RESOLUTION = 32768;
static constexpr int32_t MICROVOLTS_PER_MILLIVOLT = 1000;

// Configured for Xiao RA4M1 (3.3V and 10bit ADC)
static constexpr uint16_t ADC_POWER_MAX_RAW = 1023U;
static constexpr uint16_t ADC_POWER_MAX_MV = 3300U;

// Minimum acceptable mV being supllied to ADS1115 module
static constexpr uint16_t ADC_MINIMUM_POWER_READING_MV = 2500;


typedef struct {
  bool initialised;
  Adafruit_ADS1115 device;
  uint16_t cell_raw[THREE_CELLS];
  uint32_t cell_read_timestamp_ms;
  uint8_t failed_attempts;
} internal_state_t;

static internal_state_t state = {};


static bool is_powered(void);
static bool is_connected(void);
static bool map_non_arduino(const uint16_t base_value, uint16_t *scaled_value,
                            const uint16_t in_min, const uint16_t in_max, const uint16_t out_min, const uint16_t out_max);
static hal_adc_status_t read_sensor(uint16_t *const reading, const uint8_t channel);
static bool sort_values(uint16_t arr[3]);
static hal_adc_status_t get_filtered_reading(uint16_t *const filtered_reading, const uint8_t channel);




// Public API

hal_adc_status_t adc_init(void) {
  if (state.initialised) {
    return ADC_STATUS_OK;
  }

  if (state.device.begin() == false) {
    return ADC_STATUS_INIT_FAILED;
  }
  state.device.setGain(GAIN_SIXTEEN);
  state.device.setDataRate(RATE_ADS1115_128SPS);
  state.cell_read_timestamp_ms = 0U;
  state.failed_attempts = 0U;
  state.initialised = true;
  return ADC_STATUS_OK;
}

hal_adc_status_t adc_read_cells(void) {
  uint16_t filtered_reading[THREE_CELLS] = {};
  if (!state.initialised) {
    return ADC_STATUS_NOT_INITIALIZED;
  }
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    if(get_filtered_reading(&filtered_reading[channel], channel) != ADC_STATUS_OK){
      return ADC_STATUS_FAILED_READ;
    }
    // Add cell validation here
    // If validation passes then allow to move on
    // If all three pass then save to state
    // else
    // Reject and return an error without overwriting previous read
  }
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    state.cell_raw[channel] = filtered_reading[channel];
  }
  state.cell_read_timestamp_ms = millis();
  return ADC_STATUS_OK;
}

hal_adc_status_t adc_health_check(void) {
  if (!state.initialised) {
    return ADC_STATUS_NOT_INITIALIZED;
  }
  if (is_powered() && is_connected()) {
    state.failed_attempts = 0U;
    return ADC_STATUS_OK;
  } else {
    state.failed_attempts++;
    return ADC_STATUS_HW_ERROR;
  }
}

uint16_t adc_convert_raw_to_mV(const uint16_t raw_reading) {
  uint64_t microvolts = 0;

  if (raw_reading < 0) {
    return 0;
  }
  microvolts = ((uint64_t)raw_reading * (ADS1115_FULL_SCALE_MV * MICROVOLTS_PER_MILLIVOLT)) / ADS1115_RESOLUTION;
  //microvolts = ((int32_t)raw_reading * (ADS1115_FULL_SCALE_MV * MICROVOLTS_PER_MILLIVOLT)) / ADS1115_RESOLUTION;

  // Convert µV → mV
  return (uint16_t)(microvolts / MICROVOLTS_PER_MILLIVOLT);
}

hal_adc_status_t adc_get_last_good_cell_read(uint16_t * const reading){
  if(!state.initialised){
    return ADC_STATUS_NOT_INITIALIZED;
  }
  if(reading == NULL){
    return ADC_STATUS_INVALID_PARAMETER;
  }
  for(uint8_t channel = 0U; channel < THREE_CELLS; channel++){
    reading[channel] = state.cell_raw[channel];
  }
  return ADC_STATUS_OK;
}

hal_adc_status_t adc_get_timestamp(uint32_t * const timestamp_ms){
  if(!state.initialised){
    return ADC_STATUS_NOT_INITIALIZED;
  }
  *timestamp_ms = state.cell_read_timestamp_ms;
  return ADC_STATUS_OK;
}

hal_adc_status_t adc_get_failed_attempts(uint8_t * const failed_attempts){
  if(!state.initialised){
    return ADC_STATUS_NOT_INITIALIZED;
  }
  if(failed_attempts == NULL){
    return ADC_STATUS_INVALID_PARAMETER;
  }
  *failed_attempts = state.failed_attempts;
  return ADC_STATUS_OK;
}

// Private

static bool is_powered(void) {
  uint16_t reading_raw = 0;
  uint16_t reading_mv = 0;

  reading_raw = analogRead(ADC_POWER_READ_PIN);
  if (map_non_arduino(reading_raw, &reading_mv, 0, ADC_POWER_MAX_RAW, 0, ADC_POWER_MAX_MV) == false) {
    return false;
  }
  return (reading_mv >= ADC_MINIMUM_POWER_READING_MV);
}

static bool is_connected(void) {
  uint8_t result = 99U;

  Wire.beginTransmission(ADC_ADDRESS);
  result = Wire.endTransmission();
  return (result == TRANSMISSION_OK);
}

static bool map_non_arduino(const uint16_t base_value, uint16_t *const scaled_value,
                            const uint16_t in_min, const uint16_t in_max, const uint16_t out_min, const uint16_t out_max) {
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

static hal_adc_status_t read_sensor(uint16_t *const reading, const uint8_t channel) {
  bool wait_timeout_ok;
  uint32_t start_time_ms = 0;
  uint32_t elapsed_time_ms = 0;
  int32_t unfiltered_reading = 0;

  if (reading == NULL) {
    return ADC_STATUS_INVALID_PARAMETER;
  }
  if (channel >= THREE_CELLS) {
    return ADC_STATUS_INVALID_PARAMETER;
  }
  state.device.startADCReading(MUX_BY_CHANNEL[channel], false);
  wait_timeout_ok = true;
  start_time_ms = millis();
  while (!state.device.conversionComplete()) {
    elapsed_time_ms = millis() - start_time_ms;
    if (elapsed_time_ms >= MAX_WAIT_TIME_MS) {
      wait_timeout_ok = false;
      break;
    }
    delay(1);  // Reduces CPU cycles. Accepting the penalty of using delay() in this situation.
  }
  if (wait_timeout_ok) {
    unfiltered_reading = state.device.getLastConversionResults();
    *reading = (unfiltered_reading < 0) ? 0U : (uint16_t)unfiltered_reading;
    return ADC_STATUS_OK;
  } else {
    return ADC_STATUS_HW_ERROR;
  }
}

static bool sort_values(uint16_t arr[3]) {
  uint16_t temp;

  if (arr[0] > arr[1]) {
    temp = arr[0];
    arr[0] = arr[1];
    arr[1] = temp;
  }
  if (arr[0] > arr[2]) {
    temp = arr[0];
    arr[0] = arr[2];
    arr[2] = temp;
  }
  if (arr[1] > arr[2]) {
    temp = arr[1];
    arr[1] = arr[2];
    arr[2] = temp;
  }

  return true;
}

static hal_adc_status_t get_filtered_reading(uint16_t *const filtered_reading, const uint8_t channel) {
  uint16_t reading = 0;
  uint16_t sample[MAX_SAMPLES];

  if (!state.initialised) {
    return ADC_STATUS_NOT_INITIALIZED;
  }
  if (filtered_reading == NULL) {
    return ADC_STATUS_INVALID_PARAMETER;
  }
  if (channel >= THREE_CELLS) {
    return ADC_STATUS_INVALID_PARAMETER;
  }
  /* if((millis() - state.last_function_check_time) > MAX_INTERVAL_FUNCTION_CHECK_MS){
        state.last_function_check_time = millis();
        if(!connected_check_multiple()){
            current_state.adc_initialised = false;
            return ADC_STATUS_HW_ERROR;
        }
    } */
  for (uint8_t sample_number = 0U; sample_number < MAX_SAMPLES; sample_number++) {
    if (read_sensor(&reading, channel) != ADC_STATUS_OK) {
      state.initialised = false;
      return ADC_STATUS_HW_ERROR;
    }
    // Add reading validity check here, if passes then write to array
    sample[sample_number] = reading;
  }
  if (sort_values(sample) != true) {
    return ADC_STATUS_INVALID_PARAMETER;
  }
  *filtered_reading = sample[MEDIAN_SAMPLE_NUMBER];
  state.cell_raw[channel] = sample[MEDIAN_SAMPLE_NUMBER];
  return ADC_STATUS_OK;
}
