#include "reADCIntf.h"
#include "reEsp32.h"
#include "reEvents.h"
#include <string.h>

#if ESP_IDF_VERSION_MAJOR >= 5

static const char* logTAG = "ADC";

#define MAX_ADC_UNITS 2

static adc_oneshot_unit_handle_t _units[MAX_ADC_UNITS] = {nullptr};

adc_oneshot_unit_handle_t adcUnitCreate(const adc_unit_t unit, const adc_ulp_mode_t ulp_mode)
{
  if (_units[unit] == nullptr) {
    adc_oneshot_unit_init_cfg_t config_unit;
    memset(&config_unit, 0, sizeof(config_unit));
    config_unit.unit_id = unit;
    config_unit.ulp_mode = ulp_mode;
    RE_OK_CHECK(adc_oneshot_new_unit(&config_unit, &_units[unit]), return nullptr);
    RE_LINK_CHECK_EVENT(_units[unit], "unit_handle", return nullptr);
  };
  return _units[unit];
}

adc_oneshot_unit_handle_t adcUnitGet(const adc_unit_t unit)
{
  if (_units[unit] == nullptr) {
    adcUnitCreate(unit, ADC_ULP_MODE_DISABLE);
  };
  return _units[unit];
}

bool adcCaliCreate(const adc_unit_t unit, const adc_atten_t atten, const adc_bitwidth_t bitwidth, adc_cali_handle_t * cali_handle)
{
  adc_cali_scheme_ver_t scheme_mask;
  if (adc_cali_check_scheme(&scheme_mask) == ESP_OK) {
    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED      
      if (scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING) {
        adc_cali_curve_fitting_config_t config_cali;
        memset(&config_cali, 0, sizeof(config_cali));
        config_cali.unit_id = unit;
        config_cali.atten = atten;
        config_cali.bitwidth = bitwidth;
        RE_OK_CHECK(adc_cali_create_scheme_curve_fitting(&config_cali, cali_handle), return false);
        RE_LINK_CHECK_EVENT(cali_handle, "cali_handle", return false);
        return true;
      };
    #endif
    #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED      
      if (scheme_mask & ADC_CALI_SCHEME_VER_LINE_FITTING) {
        adc_cali_line_fitting_config_t config_cali;
        memset(&config_cali, 0, sizeof(config_cali));
        config_cali.unit_id = unit;
        config_cali.atten = atten;
        config_cali.bitwidth = bitwidth;
        RE_OK_CHECK(adc_cali_create_scheme_line_fitting(&config_cali, cali_handle), return false);
        RE_LINK_CHECK_EVENT(cali_handle, "cali_handle", return false);
        return true;
      };
    #endif
  };
  return false;
}

void adcCaliFree(adc_cali_handle_t cali_handle)
{
  #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED      
    if (cali_handle) {
      adc_cali_scheme_ver_t scheme_mask;
      adc_cali_check_scheme(&scheme_mask);
      if (scheme_mask & ADC_CALI_SCHEME_VER_CURVE_FITTING) {
        adc_cali_delete_scheme_curve_fitting(cali_handle);
      };
    );
  #endif
  #if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED    
    if (cali_handle) {
      adc_cali_delete_scheme_line_fitting(cali_handle);
    };
  #endif
}

#endif // ESP_IDF_VERSION_MAJOR