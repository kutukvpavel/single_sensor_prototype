/**
 * @file my_hal.h
 * @author Kutukov Pavel
 * @brief Hardware abstraction layer: pin mapping, shift register IO
 * @version 0.1
 * @date 2022-10-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "my_adc_channel.h"
#include <inttypes.h>
#include <stdlib.h>
#include <driver/gpio.h>

#define MY_ADC_CHANNEL_NUM 6
#define MY_ADC_SENSING_CHANNEL_NUM 4

namespace my_hal
{
    typedef uint16_t adc_code_t;

    enum adc_channel_types
    {
        i_h,
        v_h_mon,
        v_r4,
        v_div,
        bat_v,
        bat_t
    };
    extern my_adc_channel adc_channels[MY_ADC_CHANNEL_NUM];

    enum sr_types
    {
        SR_DAC
    };

    bool init(uint buzzer_freq, uint shdn_pump_freq, const my_adc_cal_t* (*get_adc_cal)(size_t));

    float adc_read_channel(adc_channel_types t);
    void sr_write(sr_types t, const uint8_t* contents);
    bool get_btn_pressed();
    bool get_vent_closed();
    bool get_usb_detected();
    bool get_charger_emergency();
    bool get_p6v_mon();
    bool get_n6v_mon();

    void set_charge_enable(bool v);
    void set_output_enable(bool v);
    void set_led_enabled(bool v);
    void set_buzzer_enable(bool v);
    void set_shutdown_analog(bool v);
    void set_battery_monitor_enable(bool v);
}