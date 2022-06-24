#include "my_adc_channel.h"

#include <stdlib.h>
#include "esp_log.h"

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

static const char* TAG = "MY_ADC_CHANNEL";

static bool adc_calibration_init(esp_adc_cal_characteristics_t* chars, adc_atten_t att)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, att, ADC_BITS, 0, chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

void my_adc_init()
{
    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_BITS));
}


my_adc_channel::my_adc_channel(adc1_channel_t ch, adc_atten_t att, const char* t, uint32_t averaging_width) 
    : av(averaging_width), channel(ch), tag(t), attenuation(att)
{
    
}
my_adc_channel::~my_adc_channel()
{
    av.~Average();
}

bool my_adc_channel::init()
{
    bool cali_enable = adc_calibration_init(&adc_chars, attenuation);
    if (!cali_enable)
    {
        ESP_LOGE(TAG, "Calibrate the ADC first!");
        return false;
    }
    ESP_ERROR_CHECK(adc1_config_channel_atten(channel, attenuation));
    return true;
}

float my_adc_channel::get_value()
{
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc1_get_raw(channel), &adc_chars);
    return av.rolling(voltage);
}

const char* my_adc_channel::get_tag()
{
    return tag;
}