#include <sys/_stdint.h>
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
  ADC_STATUS_FAILED_READ,
  ADC_STATUS_NOT_POWERED
} hal_adc_status_t;

hal_adc_status_t adc_init(void);
hal_adc_status_t adc_health_check(void);
hal_adc_status_t adc_get_last_good_cell_read(uint16_t * const reading);
hal_adc_status_t adc_read_cells(void);
hal_adc_status_t adc_get_timestamp(uint32_t * const timestamp_ms);
hal_adc_status_t adc_get_failed_attempts(uint8_t * const failed_attempts);

uint16_t adc_convert_raw_to_mV(const uint16_t raw_reading);



#endif