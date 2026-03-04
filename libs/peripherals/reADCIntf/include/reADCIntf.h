/* 
   EN: Wrapper library for the ADC interface in ESP-IDF version 5.0.0 and higher
   RU: Библиотека-обёртка для интерфейса ADC в ESP-IDF версии от 5.0.0 и выше
   --------------------------
   (с) 2023 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_ADC_INTF_H__
#define __RE_ADC_INTF_H__

#include "esp_idf_version.h"

#if ESP_IDF_VERSION_MAJOR >= 5

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#ifdef __cplusplus
extern "C" {
#endif

adc_oneshot_unit_handle_t adcUnitCreate(const adc_unit_t unit, const adc_ulp_mode_t ulp_mode);
adc_oneshot_unit_handle_t adcUnitGet(const adc_unit_t unit);

bool adcCaliCreate(const adc_unit_t unit, const adc_atten_t atten, const adc_bitwidth_t bitwidth, adc_cali_handle_t * cali_handle);
void adcCaliFree(adc_cali_handle_t cali_handle);

#ifdef __cplusplus
}
#endif

#endif // ESP_IDF_VERSION_MAJOR

#endif // __RE_ADC_INTF_H__

