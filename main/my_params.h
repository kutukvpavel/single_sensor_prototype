#pragma once

#include "my_adc_channel.h"
#include "my_dac.h"
#include <inttypes.h>

struct my_timings_t
{
    size_t averaging_len;
    uint sampling_rate;
    uint oversampling_rate;
};

namespace my_params
{
    extern const my_dac_cal_t default_dac_cal;
    extern const my_adc_cal_t default_adc_cal;

    float get_r4();
    void set_r4(float val);
    float get_heater_coef();
    void set_heater_coef(float val);
    const my_adc_cal_t* get_adc_channel_cal(size_t index);
    const my_dac_cal_t* get_dac_cal();
    const my_timings_t* get_timings();
    void init();
}