#ifndef ADC_HAL_H
#define ADC_HAL_H

#include <Arduino.h>
#include <stdint.h>

typedef enum{
  ADC_STATUS_OK = 0,
  ADC_STATUS_NOT_INITIALIZED,
  ADC_STATUS_INIT_FAILED,
  ADC_STATUS_INVALID_CHANNEL,
  ADC_STATUS_HW_ERROR,
  ADC_STATUS_INVALID_PARAMETER,
  ADC_STATUS_NOT_POWERED
} hal_adc_status_t;

hal_adc_status_t adc_init(void);
hal_adc_status_t adc_health_check(void);
hal_adc_status_t adc_get_raw_reading(uint16_t * const raw_reading, const uint8_t channel);



#endif