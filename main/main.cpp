#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#include "my_adc_channel.h"
#include "my_dac.h"
#include "my_uart.h"
#include "macros.h"

#define OVERSAMPLING_LEN 32
#define SAMPLING_RATE 10
#define OVERSAMPLING_RATE 500
#define MY_DAC_MAX 6.0 //V
#define MY_DAC_RESOLUTION 1024.0 //Steps
#define MY_DAC_CAL (MY_DAC_RESOLUTION / MY_DAC_MAX)
#define HEATER_COEF 0.0025
#define R4 100000.0 //Ohms
#define V_H_MON_MULT 4.0
#define V_H_OFFSET 67.0
#define CURRENT_SHUNT 2.0 //Ohms
#define CURRENT_AMPLIFICATION 2.0 //Times
#define CURRENT_OFFSET -25.35
#define I_H_MULT (1.0/(CURRENT_SHUNT*CURRENT_AMPLIFICATION))

//static const char *TAG = "ADC";

enum my_adc_channels {
    i_h,
    v_h_mon,
    v_r4,
    v_div
};
static my_adc_channel channels[] = {
    my_adc_channel(ADC1_CHANNEL_3, ADC_ATTEN_DB_0, "I_h", OVERSAMPLING_LEN, I_H_MULT, CURRENT_OFFSET),
    my_adc_channel(ADC1_CHANNEL_4, ADC_ATTEN_DB_6, "V_h_mon", OVERSAMPLING_LEN, V_H_MON_MULT, V_H_OFFSET),
    my_adc_channel(ADC1_CHANNEL_8, ADC_ATTEN_DB_0, "V_r4", OVERSAMPLING_LEN, 1, 0),
    my_adc_channel(ADC1_CHANNEL_9, ADC_ATTEN_DB_0, "V_div", OVERSAMPLING_LEN, 1, 0)
};
static float rt_resistance = 0;

extern "C" {
    void app_main(void);
}

float calc_resistance(float vr4, float vdiv, float ref)
{
    return R4 * (vdiv - vr4) / vr4;
}

float calc_temperature(float voltage, float current)
{
    return (voltage / current - rt_resistance) * HEATER_COEF;
}

void app_main(void)
{
    uint counter = 0;
    float buffer[ARRAY_SIZE(channels)];

    //vTaskDelay(pdMS_TO_TICKS(1000)); //For the voltages to stabilize
    ets_delay_us(100000);

    my_uart::init();
    my_adc::init();
    bool init_ok = true;
    for (auto &&i : channels)
    {
        init_ok = init_ok && i.init();
    }
    if (!init_ok) my_uart::raise_error(my_error_codes::software_init);
    my_dac::init(MY_DAC_CAL);
    my_dac::set(my_uart::first());

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000 / OVERSAMPLING_RATE));
        for (size_t i = 0; i < ARRAY_SIZE(buffer); i++)
        {
            buffer[i] = channels[i].get_value();
        }
        if ((counter++ % (OVERSAMPLING_RATE / SAMPLING_RATE) == 0) && my_uart::get_operate()) 
        {
            printf("mV: %6.1f; %6.1f; %6.1f; %6.1f\n", 
                buffer[my_adc_channels::v_r4],
                buffer[my_adc_channels::v_div],
                buffer[my_adc_channels::v_h_mon],
                buffer[my_adc_channels::i_h]
                );
            my_dac::set(my_uart::next(
                calc_temperature(buffer[my_adc_channels::v_h_mon], buffer[my_adc_channels::i_h]), 
                calc_resistance(buffer[my_adc_channels::v_r4], buffer[my_adc_channels::v_div], R4)
                ));
        }
    }
}
