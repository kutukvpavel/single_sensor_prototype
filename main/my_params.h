#pragma once

#include "my_adc_channel.h"
#include "my_dac.h"
#include "my_pid.h"
#include "my_hal.h"
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
    extern const float rt_temp;
    
    extern bool enable_pid_dbg;

    uint get_charge_pump_freq();
    uint get_buzzer_freq();
    float get_ref_resistance(); // For measurement
    void set_ref_resistance(float val);
    float get_heater_coef();
    float get_rt_resistance(); // For the heater, calculated at 273K
    void set_rt_resistance(float val, float temp); // Call aftet heater_coef (tempco) has been set!
    void set_heater_coef(float val);
    const my_adc_cal_t* get_adc_channel_cal(size_t index);
    void set_adc_channel_cal(size_t index, my_adc_cal_t* c);
    const my_dac_cal_t* get_dac_cal();
    void set_dac_cal(my_dac_cal_t* c);
    const my_timings_t* get_timings();
    const my_pid_params_t* get_pid_params();
    void set_pid_params(my_pid_params_t* p);
    esp_err_t init();
    esp_err_t save();
    uint8_t* get_nvs_dump(size_t* len);
    esp_err_t factory_reset();
}