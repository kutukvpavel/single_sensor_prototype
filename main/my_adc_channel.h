#pragma once

#include "average.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <stdint.h>

#define ADC_BITS (static_cast<adc_bits_width_t>(ADC_WIDTH_BIT_DEFAULT))

struct my_adc_cal_t
{
    float gain;
    float offset;
};

class my_adc_channel
{
private:
    Average<uint32_t>* av;
    adc1_channel_t channel;
    const char* tag;
    esp_adc_cal_characteristics_t adc_chars;
    adc_atten_t attenuation;
    const my_adc_cal_t* calibration;
public:
    my_adc_channel(adc1_channel_t ch, adc_atten_t att, const char* t);
    ~my_adc_channel();
    float get_value();
    const char* get_tag();
    bool init(const my_adc_cal_t* cal);
};