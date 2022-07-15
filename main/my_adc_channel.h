#pragma once

#include "average.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <stdint.h>

#define ADC_BITS (static_cast<adc_bits_width_t>(ADC_WIDTH_BIT_DEFAULT))

namespace my_adc
{
    void init();
} // namespace my_adc

class my_adc_channel
{
private:
    Average<uint32_t> av;
    adc1_channel_t channel;
    const char* tag;
    esp_adc_cal_characteristics_t adc_chars;
    adc_atten_t attenuation;
    float gain_correction;
    float offset_correction;
public:
    my_adc_channel(adc1_channel_t ch, adc_atten_t att, const char* t, uint32_t averaging_width, float gain, float offset);
    ~my_adc_channel();
    float get_value();
    const char* get_tag();
    bool init();
};